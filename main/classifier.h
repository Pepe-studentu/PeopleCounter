#pragma once
#include "tracker.h"

typedef enum {
    CLASS_NOISE = 0,
    CLASS_IN,
    CLASS_OUT,
} Classification;

/*
 * Classify a completed track trajectory as IN, OUT, or NOISE.
 *
 * Algorithm:
 *   1. Require at least MIN_TRAJ_FRAMES points.
 *   2. Extract positions along GOOD_AXIS (row or col).
 *   3. Linear regression: x_i = frame index, y_i = position.
 *   4. displacement = slope × (N − 1).
 *   5. Accept if abs(displacement) >= MIN_DISPLACEMENT and R² >= MIN_R_SQUARED.
 *   6. Sign of displacement × IN_DIRECTION decides IN vs OUT.
 */
Classification classifier_run(const Track *track);
