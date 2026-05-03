#pragma once
#include <stdbool.h>
#include "config.h"
#include "detector.h"

typedef struct {
    int   id;

    /* ring buffer: traj[traj_head] is the slot for the NEXT write */
    float traj[MAX_TRAJ_LEN][2];   /* [i][0]=row, [i][1]=col */
    int   traj_head;               /* next write index (mod MAX_TRAJ_LEN) */
    int   traj_len;                /* valid entries, capped at MAX_TRAJ_LEN */

    float centroid_row;
    float centroid_col;
    int   absent_frames;
    bool  active;
} Track;

typedef struct {
    Track tracks[MAX_TRACKS];
    int   next_id;
} MultiTracker;

void tracker_init(MultiTracker *t);

/*
 * Match detections to existing tracks (greedy nearest-neighbour).
 * Dead tracks (absent_frames >= ABSENT_FRAMES_MAX) are removed from the
 * live set and written into dead_out[].
 * Returns the number of dead tracks written.
 */
int tracker_update(MultiTracker *t,
                   const BlobDetection *dets, int n_dets,
                   Track dead_out[MAX_TRACKS]);
