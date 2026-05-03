#include "tracker.h"
#include <string.h>
#include <math.h>
#include <float.h>

void tracker_init(MultiTracker *t)
{
    memset(t, 0, sizeof(*t));
    t->next_id = 1;
}

static void track_absorb(Track *tr, const BlobDetection *det)
{
    tr->centroid_row = det->centroid_row;
    tr->centroid_col = det->centroid_col;

    tr->traj[tr->traj_head][0] = det->centroid_row;
    tr->traj[tr->traj_head][1] = det->centroid_col;
    tr->traj_head = (tr->traj_head + 1) % MAX_TRAJ_LEN;
    if (tr->traj_len < MAX_TRAJ_LEN) tr->traj_len++;

    tr->absent_frames = 0;
}

int tracker_update(MultiTracker *t,
                   const BlobDetection *dets, int n_dets,
                   Track dead_out[MAX_TRACKS])
{
    bool matched_track[MAX_TRACKS] = { false };
    bool matched_det[MAX_BLOBS]    = { false };

    /* --- greedy nearest-neighbour matching --- */
    for (int di = 0; di < n_dets; di++) {
        float best_dist = MAX_MATCH_DIST;
        int   best_ti   = -1;

        for (int ti = 0; ti < MAX_TRACKS; ti++) {
            if (!t->tracks[ti].active) continue;
            if (matched_track[ti])     continue;

            float dr = dets[di].centroid_row - t->tracks[ti].centroid_row;
            float dc = dets[di].centroid_col - t->tracks[ti].centroid_col;
            float dist = sqrtf(dr * dr + dc * dc);
            if (dist < best_dist) {
                best_dist = dist;
                best_ti   = ti;
            }
        }

        if (best_ti >= 0) {
            track_absorb(&t->tracks[best_ti], &dets[di]);
            matched_track[best_ti] = true;
            matched_det[di]        = true;
        }
    }

    /* --- create new tracks for unmatched detections --- */
    for (int di = 0; di < n_dets; di++) {
        if (matched_det[di]) continue;

        /* find an inactive slot */
        for (int ti = 0; ti < MAX_TRACKS; ti++) {
            if (t->tracks[ti].active) continue;

            memset(&t->tracks[ti], 0, sizeof(Track));
            t->tracks[ti].id     = t->next_id++;
            if (t->next_id < 0) t->next_id = 1;  /* wrap guard */
            t->tracks[ti].active = true;
            track_absorb(&t->tracks[ti], &dets[di]);
            matched_track[ti] = true;
            break;
        }
        /* if no slot free, silently drop detection */
    }

    /* --- increment absent_frames for unmatched tracks --- */
    for (int ti = 0; ti < MAX_TRACKS; ti++) {
        if (!t->tracks[ti].active) continue;
        if (!matched_track[ti]) t->tracks[ti].absent_frames++;
    }

    /* --- collect dead tracks, deactivate them --- */
    int n_dead = 0;
    for (int ti = 0; ti < MAX_TRACKS; ti++) {
        if (!t->tracks[ti].active) continue;
        if (t->tracks[ti].absent_frames >= ABSENT_FRAMES_MAX) {
            dead_out[n_dead++] = t->tracks[ti];
            t->tracks[ti].active = false;
        }
    }
    return n_dead;
}
