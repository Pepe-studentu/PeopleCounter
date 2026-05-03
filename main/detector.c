#include "detector.h"
#include <string.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>

/* ------------------------------------------------------------------ */
/* Helpers                                                            */
/* ------------------------------------------------------------------ */

static float compute_ambient(const float frame[64])
{
    float buf[64];
    memcpy(buf, frame, 64 * sizeof(float));

    /* partial selection sort: pull AMBIENT_SAMPLE_N minimums to the front */
    float sum = 0.0f;
    for (int i = 0; i < AMBIENT_SAMPLE_N; i++) {
        int min_j = i;
        for (int j = i + 1; j < 64; j++) {
            if (buf[j] < buf[min_j]) min_j = j;
        }
        float tmp = buf[i]; buf[i] = buf[min_j]; buf[min_j] = tmp;
        sum += buf[i];
    }
    return sum / AMBIENT_SAMPLE_N;
}

/* 4-connected flood-fill. Returns number of labels assigned (1-based). */
static int ccl_4conn(const bool hot[64], uint8_t labels[64])
{
    memset(labels, 0, 64);

    /* stack holds flat pixel indices; at most 64 entries */
    uint8_t stack[64];
    int next_label = 0;

    /* neighbor offsets for 4-connectivity */
    static const int8_t dr[4] = { -1,  1,  0,  0 };
    static const int8_t dc[4] = {  0,  0, -1,  1 };

    for (int seed = 0; seed < 64; seed++) {
        if (!hot[seed] || labels[seed] != 0) continue;

        next_label++;
        int top = 0;
        stack[top++] = (uint8_t)seed;
        labels[seed] = (uint8_t)next_label;

        while (top > 0) {
            uint8_t idx = stack[--top];
            int r = idx / 8, c = idx % 8;

            for (int d = 0; d < 4; d++) {
                int nr = r + dr[d], nc = c + dc[d];
                if (nr < 0 || nr > 7 || nc < 0 || nc > 7) continue;
                int ni = nr * 8 + nc;
                if (!hot[ni] || labels[ni] != 0) continue;
                labels[ni] = (uint8_t)next_label;
                stack[top++] = (uint8_t)ni;
            }
        }
    }
    return next_label;
}

/* Compute weighted centroid of pixels matching `label`. */
static BlobDetection weighted_centroid(uint8_t label,
                                        const uint8_t labels[64],
                                        const float frame[64],
                                        float threshold)
{
    float sum_w = 0.0f, sum_wr = 0.0f, sum_wc = 0.0f;
    int count = 0;
    for (int i = 0; i < 64; i++) {
        if (labels[i] != label) continue;
        float w = frame[i] - threshold;
        if (w < 0.0f) w = 0.0f;
        float r = (float)(i / 8), c = (float)(i % 8);
        sum_w  += w;
        sum_wr += w * r;
        sum_wc += w * c;
        count++;
    }
    BlobDetection det;
    if (sum_w > 0.0f) {
        det.centroid_row = sum_wr / sum_w;
        det.centroid_col = sum_wc / sum_w;
    } else {
        /* fallback: unweighted mean (shouldn't normally happen) */
        float sr = 0.0f, sc = 0.0f;
        for (int i = 0; i < 64; i++) {
            if (labels[i] != label) continue;
            sr += (float)(i / 8); sc += (float)(i % 8);
        }
        det.centroid_row = sr / count;
        det.centroid_col = sc / count;
    }
    return det;
}

/*
 * PCA-split an oversized blob into two detections along its principal axis.
 * Writes 1 or 2 detections into out[]. Returns number written.
 */
static int split_blob(uint8_t label,
                      const uint8_t labels[64],
                      const float frame[64],
                      float threshold,
                      BlobDetection *out,
                      int out_cap)
{
    /* collect blob pixels */
    float rs[64], cs[64], ws[64];
    int n = 0;
    float sum_w = 0.0f;

    for (int i = 0; i < 64; i++) {
        if (labels[i] != label) continue;
        float w = frame[i] - threshold;
        if (w < 0.0f) w = 0.0f;
        rs[n] = (float)(i / 8);
        cs[n] = (float)(i % 8);
        ws[n] = w;
        sum_w += w;
        n++;
    }

    /* if all weights are zero, fall back to uniform */
    if (sum_w <= 0.0f) {
        for (int i = 0; i < n; i++) ws[i] = 1.0f;
        sum_w = (float)n;
    }

    /* weighted centroid */
    float mr = 0.0f, mc = 0.0f;
    for (int i = 0; i < n; i++) {
        mr += ws[i] * rs[i];
        mc += ws[i] * cs[i];
    }
    mr /= sum_w; mc /= sum_w;

    /* 2×2 weighted covariance */
    float Crr = 0.0f, Ccc = 0.0f, Crc = 0.0f;
    for (int i = 0; i < n; i++) {
        float dr = rs[i] - mr, dc = cs[i] - mc;
        Crr += ws[i] * dr * dr;
        Ccc += ws[i] * dc * dc;
        Crc += ws[i] * dr * dc;
    }
    Crr /= sum_w; Ccc /= sum_w; Crc /= sum_w;

    /* largest eigenvalue */
    float half_trace = (Crr + Ccc) * 0.5f;
    float disc       = sqrtf((Crr - Ccc) * (Crr - Ccc) * 0.25f + Crc * Crc);
    float lam        = half_trace + disc;

    /* principal eigenvector */
    float ev_r, ev_c;
    if (fabsf(Crc) > 1e-6f) {
        ev_r = lam - Ccc;
        ev_c = Crc;
    } else if (fabsf(Crr) >= fabsf(Ccc)) {
        ev_r = 1.0f; ev_c = 0.0f;
    } else {
        ev_r = 0.0f; ev_c = 1.0f;
    }
    float ev_len = sqrtf(ev_r * ev_r + ev_c * ev_c);
    if (ev_len > 1e-9f) { ev_r /= ev_len; ev_c /= ev_len; }

    /* project pixels onto principal axis */
    float projs[64];
    for (int i = 0; i < n; i++) {
        projs[i] = (rs[i] - mr) * ev_r + (cs[i] - mc) * ev_c;
    }

    /* find median projection via insertion sort on a copy */
    float sorted[64];
    memcpy(sorted, projs, n * sizeof(float));
    for (int i = 1; i < n; i++) {
        float key = sorted[i];
        int j = i - 1;
        while (j >= 0 && sorted[j] > key) { sorted[j+1] = sorted[j]; j--; }
        sorted[j+1] = key;
    }
    float split_val = sorted[n / 2];  /* upper median for even n */

    /* split at median */
    float sr1 = 0, sc1 = 0, sw1 = 0;
    float sr2 = 0, sc2 = 0, sw2 = 0;
    int   cnt1 = 0, cnt2 = 0;

    for (int i = 0; i < n; i++) {
        if (projs[i] <= split_val) {
            sr1 += ws[i] * rs[i]; sc1 += ws[i] * cs[i]; sw1 += ws[i]; cnt1++;
        } else {
            sr2 += ws[i] * rs[i]; sc2 += ws[i] * cs[i]; sw2 += ws[i]; cnt2++;
        }
    }

    /* degenerate: all pixels on one side */
    if (cnt1 == 0 || cnt2 == 0) {
        if (out_cap < 1) return 0;
        out[0].centroid_row = mr;
        out[0].centroid_col = mc;
        return 1;
    }

    int written = 0;
    if (out_cap >= 1) {
        out[written].centroid_row = sr1 / sw1;
        out[written].centroid_col = sc1 / sw1;
        written++;
    }
    if (out_cap >= 2) {
        out[written].centroid_row = sr2 / sw2;
        out[written].centroid_col = sc2 / sw2;
        written++;
    }
    return written;
}

/* ------------------------------------------------------------------ */
/* Public API                                                         */
/* ------------------------------------------------------------------ */

int detector_run(const float frame[64], BlobDetection out[MAX_BLOBS])
{
    float threshold = compute_ambient(frame) + TEMP_OFFSET;

    bool hot[64];
    int  hot_total = 0;
    for (int i = 0; i < 64; i++) {
        hot[i] = (frame[i] > threshold);
        if (hot[i]) hot_total++;
    }
    if (hot_total < MIN_HOT_PIXELS) return 0;

    uint8_t labels[64];
    int n_labels = ccl_4conn(hot, labels);

    int n_blobs = 0;

    for (int lbl = 1; lbl <= n_labels && n_blobs < MAX_BLOBS; lbl++) {
        int count = 0;
        for (int i = 0; i < 64; i++) if (labels[i] == lbl) count++;
        if (count < MIN_HOT_PIXELS) continue;

        if (count > SPLIT_THRESHOLD) {
            int added = split_blob((uint8_t)lbl, labels, frame, threshold,
                                   out + n_blobs, MAX_BLOBS - n_blobs);
            n_blobs += added;
        } else {
            out[n_blobs++] = weighted_centroid((uint8_t)lbl, labels, frame, threshold);
        }
    }
    return n_blobs;
}
