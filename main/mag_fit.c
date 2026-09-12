#include "mag_fit.h"

#include <math.h>
#include <string.h>

// Tunables, mirroring tools/mag_calibrate.py. CALIBRATION.md §2 and §3 explain them.
#define IRLS_PASSES 6
#define GATE_SIGMAS 3.0f
#define LM_ITERATIONS 20
#define RANSAC_TRIALS 100
#define RANSAC_TOLERANCE 0.1f
#define REINIT_INLIER_FRACTION 0.7f
#define NOISE_DEFAULT_UT 0.5f
#define MIN_FIT_INLIERS 20
#define R_MIN 20.0f
#define R_MAX 70.0f
#define OFFSET_MAX 500.0f
#define CAP_COS 0.70710678f
#define N_CELLS 100

#define LEVEL_DIAG_CELLS 35
#define LEVEL_DIAG_SCATTER 0.08f
#define LEVEL_DIAG_SAMPLES 100
#define LEVEL_FULL_CELLS 60
#define LEVEL_FULL_SCATTER 0.15f
#define LEVEL_FULL_CAP 2
#define LEVEL_FULL_SAMPLES 150

// The fit works with a unit-scale W, so |W (m - o)| is about 1 and the residual
// is already relative. The radius is recovered as det(W)^(-1/3) at the end.
typedef struct {
    float o[3];
    float w[6];  // w00, w11, w22, w01, w02, w12
    uint8_t level;
} model_t;

// Scratch. Not on the stack: the calibration task has a small one. It belongs
// to the caller now, and is reached through this pointer for the duration of
// one fit - only one runs at a time, so a file-static is enough and no inner
// function needs to carry it.
static const float (*g_pts)[3];
static int g_n;
static mag_fit_scratch_t *SC;
static uint32_t s_rng = 1u;  // fixed seed: the same readings must give the same fit

#ifdef COUNTING
static unsigned long cnt_res, cnt_normal, cnt_solve, cnt_lvl[3];
#define COUNT(x) (x)++
#define COUNT_LVL(l) cnt_lvl[l]++
#else
#define COUNT(x) ((void)0)
#define COUNT_LVL(l) ((void)0)
#endif

static const float *pt(int i)
{
    return g_pts[SC->idx[i]];
}

static uint32_t rnd(void)
{
    s_rng ^= s_rng << 13;
    s_rng ^= s_rng >> 17;
    s_rng ^= s_rng << 5;
    return s_rng;
}

// ---------------------------------------------------------------- small maths

static double det_sym(const float w[6])
{
    return (double)w[0] * ((double)w[1] * w[2] - (double)w[5] * w[5])
         - (double)w[3] * ((double)w[3] * w[2] - (double)w[5] * w[4])
         + (double)w[4] * ((double)w[3] * w[5] - (double)w[1] * w[4]);
}

// Eigenvalues of a symmetric 3x3, ascending: the closed form via the reduced
// cubic, which needs no iteration.
static void eig_sym(const float w[6], float out[3])
{
    double off = (double)w[3] * w[3] + (double)w[4] * w[4] + (double)w[5] * w[5];
    double q = ((double)w[0] + w[1] + w[2]) / 3.0;
    if (off <= 0.0) {  // already diagonal
        float d[3] = {w[0], w[1], w[2]};
        for (int i = 0; i < 2; i++) {
            for (int k = i + 1; k < 3; k++) {
                if (d[k] < d[i]) {
                    float t = d[i];
                    d[i] = d[k];
                    d[k] = t;
                }
            }
        }
        out[0] = d[0];
        out[1] = d[1];
        out[2] = d[2];
        return;
    }
    double p2 = ((double)w[0] - q) * ((double)w[0] - q) + ((double)w[1] - q) * ((double)w[1] - q)
              + ((double)w[2] - q) * ((double)w[2] - q) + 2.0 * off;
    double p = sqrt(p2 / 6.0);
    float b[6];
    for (int i = 0; i < 3; i++) {
        b[i] = (float)(((double)w[i] - q) / p);
    }
    for (int i = 3; i < 6; i++) {
        b[i] = (float)((double)w[i] / p);
    }
    double r = det_sym(b) / 2.0;
    double phi = r <= -1.0 ? M_PI / 3.0 : (r >= 1.0 ? 0.0 : acos(r) / 3.0);
    double e_max = q + 2.0 * p * cos(phi);
    double e_min = q + 2.0 * p * cos(phi + 2.0 * M_PI / 3.0);
    out[0] = (float)e_min;
    out[1] = (float)(3.0 * q - e_max - e_min);
    out[2] = (float)e_max;
}

static bool positive_definite(const float w[6])
{
    for (int i = 0; i < 6; i++) {
        if (!isfinite(w[i])) {
            return false;
        }
    }
    float e[3];
    eig_sym(w, e);
    return e[0] > 0.0f;
}

static float model_radius(const model_t *m)
{
    double d = det_sym(m->w);
    return d > 0.0 ? (float)pow(d, -1.0 / 3.0) : NAN;
}

// v = W d, W symmetric.
static void mul_sym(const float w[6], const float d[3], float v[3])
{
    v[0] = w[0] * d[0] + w[3] * d[1] + w[4] * d[2];
    v[1] = w[3] * d[0] + w[1] * d[1] + w[5] * d[2];
    v[2] = w[4] * d[0] + w[5] * d[1] + w[2] * d[2];
}

// |W (m - o)| - 1: the reading's distance from the unit sphere, relative.
static float residual_of(const model_t *m, const float p[3])
{
    float d[3] = {p[0] - m->o[0], p[1] - m->o[1], p[2] - m->o[2]};
    float v[3];
    mul_sym(m->w, d, v);
    return sqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]) - 1.0f;
}

static void residuals_all(const model_t *m, float *out)
{
    for (int i = 0; i < g_n; i++) {
        out[i] = residual_of(m, pt(i));
    }
}

// Solves an n x n system in SC->a (augmented) by Gaussian elimination with partial
// pivoting. False if singular.
static bool solve(int n, double *x)
{
    COUNT(cnt_solve);
    for (int col = 0; col < n; col++) {
        int piv = col;
        for (int r = col + 1; r < n; r++) {
            if (fabs(SC->a[r][col]) > fabs(SC->a[piv][col])) {
                piv = r;
            }
        }
        if (!(fabs(SC->a[piv][col]) > 1e-14)) {
            return false;
        }
        if (piv != col) {
            for (int k = col; k <= n; k++) {
                double t = SC->a[col][k];
                SC->a[col][k] = SC->a[piv][k];
                SC->a[piv][k] = t;
            }
        }
        for (int r = col + 1; r < n; r++) {
            double f = SC->a[r][col] / SC->a[col][col];
            if (f == 0.0) {
                continue;
            }
            for (int k = col; k <= n; k++) {
                SC->a[r][k] -= f * SC->a[col][k];
            }
        }
    }
    for (int i = n - 1; i >= 0; i--) {
        double acc = SC->a[i][n];
        for (int k = i + 1; k < n; k++) {
            acc -= SC->a[i][k] * x[k];
        }
        x[i] = acc / SC->a[i][i];
    }
    for (int i = 0; i < n; i++) {
        if (!isfinite(x[i])) {
            return false;
        }
    }
    return true;
}

// Median of n values, destroying the array: iterative quickselect, so no
// recursion and no allocation.
static float median_of(float *a, int n)
{
    if (n <= 0) {
        return 0.0f;
    }
    int k = n / 2, lo = 0, hi = n - 1;
    while (lo < hi) {
        float pivot = a[(lo + hi) / 2];
        int i = lo, j = hi;
        while (i <= j) {
            while (a[i] < pivot) {
                i++;
            }
            while (a[j] > pivot) {
                j--;
            }
            if (i <= j) {
                float t = a[i];
                a[i] = a[j];
                a[j] = t;
                i++;
                j--;
            }
        }
        if (k <= j) {
            hi = j;
        } else if (k >= i) {
            lo = i;
        } else {
            break;
        }
    }
    // Even counts average the two middle values, as the reference does: taking
    // just one shifts the inlier gate and with it the whole fit.
    if (n % 2 != 0) {
        return a[k];
    }
    float upper = a[k], lower = a[0];
    for (int i = 1; i < k; i++) {
        if (a[i] > lower) {
            lower = a[i];
        }
    }
    return 0.5f * (lower + upper);
}

// ------------------------------------------------------------- inlier gating

// Inliers within GATE_SIGMAS robust sigmas of the median residual. The gate
// never closes tighter than the sensor noise, so a good fit doesn't start
// rejecting honest readings.
static int gate_inliers(const float *res, float noise_rel, uint8_t *mask, float *gate_out)
{
    memcpy(SC->sort, res, (size_t)g_n * sizeof(float));
    float med = median_of(SC->sort, g_n);
    for (int i = 0; i < g_n; i++) {
        SC->sort[i] = fabsf(res[i] - med);
    }
    float spread = 1.4826f * median_of(SC->sort, g_n);
    float gate = GATE_SIGMAS * (spread > noise_rel ? spread : noise_rel);
    int count = 0;
    for (int i = 0; i < g_n; i++) {
        mask[i] = fabsf(res[i] - med) <= gate;
        count += mask[i];
    }
    if (gate_out) {
        *gate_out = gate;
    }
    return count;
}

// ------------------------------------------------------------------ Kasa fit

// Linear least-squares sphere through the masked readings, centred first so the
// normal equations stay well conditioned.
static bool kasa(const uint8_t *mask, model_t *out)
{
    double mean[3] = {0, 0, 0};
    int n = 0;
    for (int i = 0; i < g_n; i++) {
        if (mask && !mask[i]) {
            continue;
        }
        for (int k = 0; k < 3; k++) {
            mean[k] += pt(i)[k];
        }
        n++;
    }
    if (n < 4) {
        return false;
    }
    for (int k = 0; k < 3; k++) {
        mean[k] /= n;
    }
    memset(SC->a, 0, sizeof(SC->a));
    for (int i = 0; i < g_n; i++) {
        if (mask && !mask[i]) {
            continue;
        }
        double d[3];
        for (int k = 0; k < 3; k++) {
            d[k] = pt(i)[k] - mean[k];
        }
        double row[4] = {2.0 * d[0], 2.0 * d[1], 2.0 * d[2], 1.0};
        double y = d[0] * d[0] + d[1] * d[1] + d[2] * d[2];
        for (int a = 0; a < 4; a++) {
            for (int b = 0; b < 4; b++) {
                SC->a[a][b] += row[a] * row[b];
            }
            SC->a[a][4] += row[a] * y;
        }
    }
    double sol[4];
    if (!solve(4, sol)) {
        return false;
    }
    double r2 = sol[3] + sol[0] * sol[0] + sol[1] * sol[1] + sol[2] * sol[2];
    if (!(r2 > 1e-6) || !isfinite(r2)) {
        return false;
    }
    float scale = (float)(1.0 / sqrt(r2));
    memset(out, 0, sizeof(*out));
    for (int k = 0; k < 3; k++) {
        out->o[k] = (float)(sol[k] + mean[k]);
    }
    out->w[0] = out->w[1] = out->w[2] = scale;
    out->level = MAG_FIT_SPHERE;
    return isfinite(out->o[0]) && isfinite(out->o[1]) && isfinite(out->o[2]) && isfinite(scale);
}

// Best sphere through four random readings, by inlier count. Needs no starting
// guess, which is what makes it the fallback when the ordinary start fails.
static bool ransac(model_t *out)
{
    model_t best;
    int best_count = -1;
    bool have = false;
    if (g_n < 4) {
        return false;
    }
    for (int trial = 0; trial < RANSAC_TRIALS; trial++) {
        memset(SC->tmp, 0, (size_t)g_n);
        for (int k = 0; k < 4; k++) {  // four distinct readings
            int pick = (int)(rnd() % (uint32_t)g_n);
            for (int guard = 0; SC->tmp[pick] && guard < g_n; guard++) {
                pick = (pick + 1) % g_n;
            }
            SC->tmp[pick] = 1;
        }
        model_t cand;
        if (!kasa(SC->tmp, &cand)) {
            continue;
        }
        float r = model_radius(&cand);
        if (!(r >= R_MIN / 2 && r <= R_MAX * 2)) {
            continue;
        }
        int count = 0;
        for (int i = 0; i < g_n; i++) {
            count += fabsf(residual_of(&cand, pt(i))) < RANSAC_TOLERANCE;
        }
        if (count > best_count) {
            best_count = count;
            best = cand;
            have = true;
        }
    }
    if (!have) {
        return false;
    }
    for (int i = 0; i < g_n; i++) {  // refit on everything that agreed
        SC->tmp[i] = fabsf(residual_of(&best, pt(i))) < RANSAC_TOLERANCE;
    }
    model_t refined;
    *out = kasa(SC->tmp, &refined) ? refined : best;
    return true;
}

// ------------------------------------------------------- Levenberg-Marquardt

static int param_count(uint8_t level)
{
    return level == MAG_FIT_SPHERE ? 4 : (level == MAG_FIT_DIAGONAL ? 6 : 9);
}

static void pack(const model_t *m, uint8_t level, double p[9])
{
    p[0] = m->o[0];
    p[1] = m->o[1];
    p[2] = m->o[2];
    if (level == MAG_FIT_SPHERE) {
        double d = det_sym(m->w);
        p[3] = pow(fabs(d), 1.0 / 3.0);
    } else {
        p[3] = m->w[0];
        p[4] = m->w[1];
        p[5] = m->w[2];
        if (level == MAG_FIT_FULL) {
            p[6] = m->w[3];
            p[7] = m->w[4];
            p[8] = m->w[5];
        }
    }
}

static void unpack(const double p[9], uint8_t level, model_t *m)
{
    memset(m, 0, sizeof(*m));
    for (int k = 0; k < 3; k++) {
        m->o[k] = (float)p[k];
    }
    if (level == MAG_FIT_SPHERE) {
        m->w[0] = m->w[1] = m->w[2] = (float)p[3];
    } else {
        m->w[0] = (float)p[3];
        m->w[1] = (float)p[4];
        m->w[2] = (float)p[5];
        if (level == MAG_FIT_FULL) {
            m->w[3] = (float)p[6];
            m->w[4] = (float)p[7];
            m->w[5] = (float)p[8];
        }
    }
    m->level = level;
}

// Cost sum(r^2) over the masked readings, and optionally the normal equations
// JtJ and Jtr. The Jacobian is accumulated row by row, never stored.
static double lm_step_setup(const double p[9], uint8_t level, const uint8_t *mask, bool normal)
{
    model_t m;
    unpack(p, level, &m);
    int np = param_count(level);
    if (normal) {
        memset(SC->a, 0, sizeof(SC->a));
    }
    double cost = 0.0;
    for (int i = 0; i < g_n; i++) {
        if (!mask[i]) {
            continue;
        }
        const float *q = pt(i);
        float d[3] = {q[0] - m.o[0], q[1] - m.o[1], q[2] - m.o[2]};
        float v[3];
        mul_sym(m.w, d, v);
        float n = sqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
        if (n < 1e-9f) {
            n = 1e-9f;
        }
        double r = n - 1.0;
        cost += r * r;
        COUNT(cnt_res);
        if (normal) { COUNT(cnt_normal); COUNT_LVL(level); }
        if (!normal) {
            continue;
        }
        float j[9];
        float wv[3];
        mul_sym(m.w, v, wv);  // dn/do = -W v / n
        j[0] = -wv[0] / n;
        j[1] = -wv[1] / n;
        j[2] = -wv[2] / n;
        if (level == MAG_FIT_SPHERE) {
            j[3] = n / (float)p[3];  // W = s I, so dn/ds = |d|
        } else {
            j[3] = v[0] * d[0] / n;
            j[4] = v[1] * d[1] / n;
            j[5] = v[2] * d[2] / n;
            if (level == MAG_FIT_FULL) {
                j[6] = (v[0] * d[1] + v[1] * d[0]) / n;
                j[7] = (v[0] * d[2] + v[2] * d[0]) / n;
                j[8] = (v[1] * d[2] + v[2] * d[1]) / n;
            }
        }
        for (int a = 0; a < np; a++) {
            for (int b = a; b < np; b++) {
                SC->a[a][b] += j[a] * j[b];
            }
            SC->a[a][np] -= j[a] * r;  // right-hand side is -Jt r, in the column solve() reads
        }
    }
    if (normal) {
        for (int a = 0; a < np; a++) {
            for (int b = 0; b < a; b++) {
                SC->a[a][b] = SC->a[b][a];
            }
        }
    }
    return cost;
}

// Geometric fit of |W (m - o)| = 1 over the masked readings. False if it ends up
// not positive definite, which means the shape stopped being a real ellipsoid.
static bool lm_fit(const model_t *start, const uint8_t *mask, uint8_t level, model_t *out)
{
    int np = param_count(level);
    double p[9], cand[9], step[9], diag[9];
    pack(start, level, p);
    if (level == MAG_FIT_SPHERE && !(p[3] > 0.0)) {
        return false;
    }
    double cost = lm_step_setup(p, level, mask, false);
    if (!isfinite(cost)) {
        return false;
    }
    double lambda = 1e-3;
    for (int it = 0; it < LM_ITERATIONS; it++) {
        lm_step_setup(p, level, mask, true);
        for (int a = 0; a < np; a++) {
            diag[a] = SC->a[a][a] + 1e-12;
        }
        for (int a = 0; a < np; a++) {
            SC->a[a][a] += lambda * diag[a];
        }
        if (!solve(np, step)) {
            lambda *= 10.0;
            if (lambda > 1e6) {
                break;
            }
            continue;
        }
        for (int a = 0; a < np; a++) {
            cand[a] = p[a] + step[a];
        }
        double cost_try = lm_step_setup(cand, level, mask, false);
        if (isfinite(cost_try) && cost_try < cost) {
            bool done = cost - cost_try < 1e-7 * cost;
            memcpy(p, cand, sizeof(p));
            cost = cost_try;
            lambda = lambda / 3.0 > 1e-9 ? lambda / 3.0 : 1e-9;
            if (done) {
                break;
            }
        } else {
            lambda *= 4.0;
            if (lambda > 1e6) {
                break;
            }
        }
    }
    unpack(p, level, out);
    return positive_definite(out->w) && isfinite(model_radius(out));
}

// ----------------------------------------------------------------- coverage

uint8_t mag_fit_cell(const float u[3])
{
    static const float floor_z[5] = {0.98f, 0.68f, 0.0f, -0.68f, -0.98f};
    static const uint8_t per_band[6] = {1, 15, 34, 34, 15, 1};
    static const uint8_t first[6] = {0, 1, 16, 50, 84, 99};
    int band = 5;
    for (int b = 0; b < 5; b++) {
        if (u[2] >= floor_z[b]) {
            band = b;
            break;
        }
    }
    float lon = atan2f(u[1], u[0]);
    if (lon < 0.0f) {
        lon += 2.0f * (float)M_PI;
    }
    int n = per_band[band];
    int i = (int)(lon / (2.0f * (float)M_PI) * n);
    return (uint8_t)(first[band] + (i < n - 1 ? i : n - 1));
}

static bool direction_of(const model_t *m, const float p[3], float u[3])
{
    float d[3] = {p[0] - m->o[0], p[1] - m->o[1], p[2] - m->o[2]};
    float v[3];
    mul_sym(m->w, d, v);
    float n = sqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
    if (!(n > 1e-9f)) {
        return false;
    }
    for (int k = 0; k < 3; k++) {
        u[k] = v[k] / n;
    }
    return true;
}

// Occupied cells, the six cap counts, the scatter (smallest eigenvalue of the
// mean of u uT) and the wobble, all over the masked readings.
static void coverage(const model_t *m, const uint8_t *mask, int *cells, int caps[6],
                     float *scatter, float *wobble)
{
    memset(SC->cell_n, 0, sizeof(SC->cell_n));
    memset(SC->cell_v, 0, sizeof(SC->cell_v));
    float scat[6] = {0, 0, 0, 0, 0, 0};
    int n = 0;
    for (int k = 0; k < 6; k++) {
        caps[k] = 0;
    }
    for (int i = 0; i < g_n; i++) {
        float u[3];
        if (!mask[i] || !direction_of(m, pt(i), u)) {
            continue;
        }
        uint8_t c = mag_fit_cell(u);
        float r = residual_of(m, pt(i));
        SC->cell_n[c]++;
        for (int k = 0; k < 3; k++) {
            SC->cell_v[c][k] += r * u[k];
        }
        for (int k = 0; k < 3; k++) {
            caps[2 * k] += u[k] >= CAP_COS;
            caps[2 * k + 1] += -u[k] >= CAP_COS;
        }
        scat[0] += u[0] * u[0];
        scat[1] += u[1] * u[1];
        scat[2] += u[2] * u[2];
        scat[3] += u[0] * u[1];
        scat[4] += u[0] * u[2];
        scat[5] += u[1] * u[2];
        n++;
    }
    int occupied = 0;
    float total[3] = {0, 0, 0};
    for (int c = 0; c < N_CELLS; c++) {
        if (!SC->cell_n[c]) {
            continue;
        }
        occupied++;
        for (int k = 0; k < 3; k++) {
            total[k] += SC->cell_v[c][k] / SC->cell_n[c];
        }
    }
    *cells = occupied;
    if (!n) {
        *scatter = 0.0f;
        *wobble = INFINITY;
        return;
    }
    float sym[6];
    for (int k = 0; k < 6; k++) {
        sym[k] = scat[k] / n;
    }
    float e[3];
    eig_sym(sym, e);
    *scatter = e[0];
    *wobble = occupied ? sqrtf(total[0] * total[0] + total[1] * total[1] + total[2] * total[2]) / occupied
                       : INFINITY;
}

static uint8_t choose_level(int cells, const int caps[6], float scatter, int samples)
{
    int cap_min = caps[0];
    for (int k = 1; k < 6; k++) {
        if (caps[k] < cap_min) {
            cap_min = caps[k];
        }
    }
    if (cells >= LEVEL_FULL_CELLS && scatter >= LEVEL_FULL_SCATTER && cap_min >= LEVEL_FULL_CAP
        && samples >= LEVEL_FULL_SAMPLES) {
        return MAG_FIT_FULL;
    }
    if (cells >= LEVEL_DIAG_CELLS && scatter >= LEVEL_DIAG_SCATTER && samples >= LEVEL_DIAG_SAMPLES) {
        return MAG_FIT_DIAGONAL;
    }
    return MAG_FIT_SPHERE;
}

// --------------------------------------------------------------- robust fit

// Refit on the inliers until the inlier set stops changing. Every pass re-gates
// every reading, so one rejected under a poor early model comes back.
static bool robust_fit(const model_t *start, uint8_t level, float noise_ut, model_t *out,
                       uint8_t *mask, int *inliers)
{
    model_t model = *start;
    bool have_mask = false;
    for (int pass = 0; pass < IRLS_PASSES; pass++) {
        float radius = model_radius(&model);
        if (!isfinite(radius) || radius <= 0.0f) {
            return false;
        }
        residuals_all(&model, SC->res);
        int count = gate_inliers(SC->res, noise_ut / radius, SC->tmp, NULL);
        if (have_mask && memcmp(SC->tmp, mask, (size_t)g_n) == 0) {
            break;
        }
        memcpy(mask, SC->tmp, (size_t)g_n);
        have_mask = true;
        if (count < MIN_FIT_INLIERS) {
            return false;
        }
        model_t fitted;
        if (!lm_fit(&model, mask, level, &fitted)) {
            return false;
        }
        model = fitted;
    }
    float radius = model_radius(&model);
    if (!isfinite(radius) || radius <= 0.0f) {
        return false;
    }
    residuals_all(&model, SC->res);
    *inliers = gate_inliers(SC->res, noise_ut / radius, mask, NULL);
    *out = model;
    return true;
}

// One start: pick the level the coverage supports, drop a level whenever the fit
// refuses, then try once more for a higher level if the inliers grew into it.
static bool fit_from(const model_t *start, float noise_ut, model_t *out, uint8_t *mask, int *inliers)
{
    float radius = model_radius(start);
    if (!isfinite(radius) || radius <= 0.0f) {
        return false;
    }
    residuals_all(start, SC->res);
    int count = gate_inliers(SC->res, noise_ut / radius, SC->tmp, NULL);
    int cells, caps[6];
    float scatter, wob;
    coverage(start, SC->tmp, &cells, caps, &scatter, &wob);
    int level = choose_level(cells, caps, scatter, count);

    bool ok = false;
    for (; level >= MAG_FIT_SPHERE && !ok; level--) {
        ok = robust_fit(start, (uint8_t)level, noise_ut, out, mask, inliers);
    }
    if (!ok) {
        return false;
    }
    coverage(out, mask, &cells, caps, &scatter, &wob);
    uint8_t higher = choose_level(cells, caps, scatter, *inliers);
    if (higher > out->level) {
        model_t better;
        int better_inliers;
        if (robust_fit(out, higher, noise_ut, &better, SC->tmp, &better_inliers)
            && better_inliers >= (int)(0.98f * *inliers)) {
            *out = better;
            *inliers = better_inliers;
            memcpy(mask, SC->tmp, (size_t)g_n);
        }
    }
    return true;
}

// ------------------------------------------------------------------ the API

static float masked_rms(const model_t *m, const uint8_t *mask)
{
    double sum = 0.0;
    int n = 0;
    for (int i = 0; i < g_n; i++) {
        if (mask[i]) {
            float r = residual_of(m, pt(i));
            sum += (double)r * r;
            n++;
        }
    }
    return n ? (float)sqrt(sum / n) : INFINITY;
}

static void finish(const model_t *m, const uint8_t *mask, int inliers, mag_fit_t *out)
{
    memset(out, 0, sizeof(*out));
    float radius = model_radius(m);
    for (int k = 0; k < 3; k++) {
        out->offset[k] = m->o[k];
    }
    // Report W scaled to det 1, so the radius carries the sensor's gain.
    float norm[6];
    for (int k = 0; k < 6; k++) {
        norm[k] = m->w[k] * radius;
    }
    out->matrix[0] = norm[0];
    out->matrix[4] = norm[1];
    out->matrix[8] = norm[2];
    out->matrix[1] = out->matrix[3] = norm[3];
    out->matrix[2] = out->matrix[6] = norm[4];
    out->matrix[5] = out->matrix[7] = norm[5];
    out->radius = radius;
    out->level = m->level;
    out->points = (uint16_t)g_n;
    out->inliers = (uint16_t)inliers;

    out->rms = masked_rms(m, mask);

    int cells, caps[6];
    float scatter, wobble;
    coverage(m, mask, &cells, caps, &scatter, &wobble);
    int cap_min = caps[0];
    for (int k = 1; k < 6; k++) {
        if (caps[k] < cap_min) {
            cap_min = caps[k];
        }
    }
    out->cells = (uint16_t)cells;
    out->caps_min = (uint8_t)(cap_min > 255 ? 255 : cap_min);
    out->scatter = scatter;
    out->wobble = wobble;
    float e[3];
    eig_sym(norm, e);
    out->eig_min = e[0];
    out->eig_max = e[2];
}

// Is the candidate a better fit than what we have? More readings explained wins,
// and the residual breaks a tie.
static bool is_better(bool have, int inliers, float rms, int cand_inliers, float cand_rms)
{
    if (!have) {
        return true;
    }
    if (inliers != cand_inliers) {
        return cand_inliers > inliers;
    }
    return cand_rms < rms;
}

bool mag_fit(const float (*points)[3], size_t n, float noise_ut, const mag_fit_t *start,
             mag_fit_t *out, bool *inlier, mag_fit_scratch_t *scratch)
{
    if (!points || !out || !scratch) {
        return false;
    }
    SC = scratch;
    if (n > MAG_FIT_MAX_POINTS) {
        n = MAG_FIT_MAX_POINTS;
    }
    if (inlier) {
        memset(inlier, 0, n * sizeof(bool));
    }
    // Drop anything non-finite up front: one NaN would poison every sum below.
    g_pts = points;
    g_n = 0;
    for (size_t i = 0; i < n; i++) {
        if (isfinite(points[i][0]) && isfinite(points[i][1]) && isfinite(points[i][2])) {
            SC->idx[g_n++] = (uint16_t)i;
        }
    }
    if (g_n < MAG_FIT_MIN_POINTS) {
        return false;
    }
    if (!(noise_ut > 0.0f) || !isfinite(noise_ut)) {
        noise_ut = NOISE_DEFAULT_UT;
    }
    s_rng = 1u;

    model_t best;
    int best_inliers = 0;
    float best_rms = INFINITY;
    bool have = false;
    bool tried_kasa = false;

    model_t seed;
    if (start && isfinite(start->radius) && start->radius >= R_MIN / 2 && start->radius <= R_MAX * 2) {
        memset(&seed, 0, sizeof(seed));
        for (int k = 0; k < 3; k++) {
            seed.o[k] = start->offset[k];
        }
        float inv = 1.0f / start->radius;  // back to the unit scale the fit works in
        seed.w[0] = start->matrix[0] * inv;
        seed.w[1] = start->matrix[4] * inv;
        seed.w[2] = start->matrix[8] * inv;
        seed.w[3] = start->matrix[1] * inv;
        seed.w[4] = start->matrix[2] * inv;
        seed.w[5] = start->matrix[5] * inv;
        seed.level = start->level;
        if (positive_definite(seed.w)) {
            model_t got;
            int got_inliers;
            if (fit_from(&seed, noise_ut, &got, SC->mask, &got_inliers)) {
                best = got;
                best_inliers = got_inliers;
                best_rms = masked_rms(&got, SC->mask);
                memcpy(SC->best, SC->mask, (size_t)g_n);
                have = true;
            }
        }
    }
    if (!have) {
        tried_kasa = true;
        model_t k;
        if (kasa(NULL, &k)) {
            model_t got;
            int got_inliers;
            if (fit_from(&k, noise_ut, &got, SC->mask, &got_inliers)) {
                best = got;
                best_inliers = got_inliers;
                best_rms = masked_rms(&got, SC->mask);
                memcpy(SC->best, SC->mask, (size_t)g_n);
                have = true;
            }
        }
    }
    // A fit that explains less than REINIT_INLIER_FRACTION of the readings is
    // probably a local minimum: start again from scratch and keep the better one.
    if (!have || best_inliers < (int)(REINIT_INLIER_FRACTION * g_n)) {
        for (int attempt = 0; attempt < 2; attempt++) {
            model_t seed2;
            bool got_seed = attempt == 0 ? (!tried_kasa && kasa(NULL, &seed2)) : ransac(&seed2);
            if (!got_seed) {
                continue;
            }
            model_t got;
            int got_inliers;
            if (!fit_from(&seed2, noise_ut, &got, SC->mask, &got_inliers)) {
                continue;
            }
            float rms = masked_rms(&got, SC->mask);
            if (is_better(have, best_inliers, best_rms, got_inliers, rms)) {
                best = got;
                best_inliers = got_inliers;
                best_rms = rms;
                memcpy(SC->best, SC->mask, (size_t)g_n);
                have = true;
            }
        }
    }
    if (!have) {
        return false;
    }
    finish(&best, SC->best, best_inliers, out);
    if (inlier) {
        for (int i = 0; i < g_n; i++) {
            inlier[SC->idx[i]] = SC->best[i] != 0;
        }
    }
    return isfinite(out->radius) && isfinite(out->rms);
}

bool mag_fit_plausible_for_binning(const mag_fit_t *fit)
{
    if (!fit || !isfinite(fit->radius)) {
        return false;
    }
    float offset = sqrtf(fit->offset[0] * fit->offset[0] + fit->offset[1] * fit->offset[1]
                         + fit->offset[2] * fit->offset[2]);
    return fit->radius >= R_MIN / 2 && fit->radius <= R_MAX * 2 && offset <= OFFSET_MAX
           && fit->eig_min >= 0.5f && fit->eig_max <= 2.0f;
}

void mag_fit_apply(const mag_fit_t *fit, const float m[3], float b[3])
{
    float d[3] = {m[0] - fit->offset[0], m[1] - fit->offset[1], m[2] - fit->offset[2]};
    for (int i = 0; i < 3; i++) {
        b[i] = fit->matrix[3 * i] * d[0] + fit->matrix[3 * i + 1] * d[1] + fit->matrix[3 * i + 2] * d[2];
    }
}

float mag_fit_residual(const mag_fit_t *fit, const float m[3])
{
    float b[3];
    mag_fit_apply(fit, m, b);
    return sqrtf(b[0] * b[0] + b[1] * b[1] + b[2] * b[2]) / fit->radius - 1.0f;
}
