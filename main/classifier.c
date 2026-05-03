#include "classifier.h"
#include "config.h"
#include <math.h>

Classification classifier_run(const Track *track)
{
    int N = track->traj_len;
    if (N < MIN_TRAJ_FRAMES) return CLASS_NOISE;

    /*
     * Read trajectory in chronological order from the ring buffer.
     * oldest entry is at index (traj_head - traj_len + MAX_TRAJ_LEN) % MAX_TRAJ_LEN.
     */
    int start = (track->traj_head - N + MAX_TRAJ_LEN) % MAX_TRAJ_LEN;

    /*
     * Linear regression with integer x_i = i (0 … N-1).
     * Closed-form sums:
     *   Sx  = N*(N-1)/2
     *   Sx2 = N*(N-1)*(2N-1)/6
     */
    float Sx  = (float)(N * (N - 1)) * 0.5f;
    float Sx2 = (float)(N * (N - 1) * (2 * N - 1)) / 6.0f;
    float Sy  = 0.0f, Sxy = 0.0f;

    for (int i = 0; i < N; i++) {
        int idx = (start + i) % MAX_TRAJ_LEN;
        float y = track->traj[idx][GOOD_AXIS];
        Sy  += y;
        Sxy += (float)i * y;
    }

    float denom = (float)N * Sx2 - Sx * Sx;
    if (fabsf(denom) < 1e-9f) return CLASS_NOISE;

    float slope     = ((float)N * Sxy - Sx * Sy) / denom;
    float intercept = (Sy - slope * Sx) / (float)N;
    float y_mean    = Sy / (float)N;

    /* R² */
    float ss_res = 0.0f, ss_tot = 0.0f;
    for (int i = 0; i < N; i++) {
        int   idx    = (start + i) % MAX_TRAJ_LEN;
        float y      = track->traj[idx][GOOD_AXIS];
        float y_fit  = slope * (float)i + intercept;
        float dy_res = y - y_fit;
        float dy_tot = y - y_mean;
        ss_res += dy_res * dy_res;
        ss_tot += dy_tot * dy_tot;
    }

    float r_squared = (ss_tot > 1e-6f) ? (1.0f - ss_res / ss_tot) : 0.0f;

    float displacement = slope * (float)(N - 1);

    if (fabsf(displacement) < MIN_DISPLACEMENT) return CLASS_NOISE;
    if (r_squared < MIN_R_SQUARED)              return CLASS_NOISE;

    /* positive displacement in the IN_DIRECTION means IN */
    if (displacement * (float)IN_DIRECTION > 0.0f) return CLASS_IN;
    return CLASS_OUT;
}
