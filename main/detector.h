#pragma once
#include "config.h"

typedef struct {
    float centroid_row;
    float centroid_col;
} BlobDetection;

/*
 * Run the full detection pipeline on one 8×8 thermal frame.
 *
 * Steps:
 *   1. Compute ambient = mean of AMBIENT_SAMPLE_N coldest pixels.
 *   2. Threshold = ambient + TEMP_OFFSET.
 *   3. Label 4-connected hot-pixel blobs.
 *   4. For each blob ≥ MIN_HOT_PIXELS pixels:
 *        - If pixels > SPLIT_THRESHOLD: split along PCA principal axis - 2 detections.
 *        - Otherwise: weighted centroid - 1 detection.
 *
 * Returns the number of detections written into `out` (0 … MAX_BLOBS).
 */
int detector_run(const float frame[64], BlobDetection out[MAX_BLOBS]);
