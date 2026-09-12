#include "bearing_cal.h"

#include <math.h>
#include <string.h>

#define DEG (float)(M_PI / 180.0)
#define SECTOR_DEG (360.0f / BEARING_SECTORS)

static float wrap180(float d)
{
    d = fmodf(d + 180.0f, 360.0f);
    if (d < 0.0f) {
        d += 360.0f;
    }
    return d - 180.0f;
}

static float wrap360(float d)
{
    d = fmodf(d, 360.0f);
    return d < 0.0f ? d + 360.0f : d;
}

void bearing_cal_defaults(bearing_cfg_t *cfg)
{
    cfg->window_s = 8.0f;
    cfg->min_speed_mps = 3.0f;
    cfg->speed_tol = 0.2f;
    cfg->max_slope_dps = 1.0f;
    cfg->max_change_deg = 5.0f;
    cfg->max_scatter_deg = 2.5f;
    cfg->max_slope_diff_dps = 0.5f;
    cfg->h_field_ut = 19.4f;  // Kyiv; the caller should pass the WMM's
    cfg->weight_cap = 200.0f;
    cfg->min_sectors = 5;
    cfg->seed_windows = 8;
    cfg->gate_deg = 15.0f;
    cfg->dissent_windows = 8;
    cfg->dissent_spread_deg = 12.0f;
}

void bearing_cal_init(bearing_cal_t *st, const bearing_cfg_t *cfg)
{
    memset(st, 0, sizeof(*st));
    st->cfg = *cfg;
    st->q_t0 = -1.0f;  // "no first window yet"; zero is a valid timestamp
}

const char *bearing_reject_name(bearing_reject_t why)
{
    switch (why) {
    case BEARING_OK:             return "accepted";
    case BEARING_UNUSABLE:       return "no usable fix";
    case BEARING_TOO_SLOW:       return "too slow";
    case BEARING_SPEED_CHANGED:  return "speed changed";
    case BEARING_GAP:            return "a gap in the fixes";
    case BEARING_SHORT:          return "too few fixes";
    case BEARING_TURNING:        return "turning";
    case BEARING_COURSE_ROUGH:   return "the course wandered";
    case BEARING_HEADING_ROUGH:  return "the heading wandered";
    case BEARING_NOT_TOGETHER:   return "the module turned on its own";
    case BEARING_DISAGREES:      return "disagrees with what is known";
    default:                     return "still filling";
    }
}

// ---- the window ----

static void window_reset(bearing_cal_t *st)
{
    st->n = 0;
    st->sum_t = st->sum_tt = 0.0f;
    st->sum_c = st->sum_tc = st->sum_cc = 0.0f;
    st->sum_h = st->sum_th = st->sum_hh = 0.0f;
    st->sum_d = st->sum_dd = 0.0f;
    st->head_sum_x = st->head_sum_y = 0.0f;
    st->window_dead = false;
    st->window_why = BEARING_OK;
}

// Least squares of y against t, from the running sums. Returns the slope and
// writes the scatter about the fitted line; both in the units of y.
static float regress(int n, float st_, float stt, float sy, float sty, float syy, float *scatter)
{
    float fn = (float)n;
    float den = fn * stt - st_ * st_;
    if (den <= 0.0f) {
        *scatter = 0.0f;
        return 0.0f;
    }
    float slope = (fn * sty - st_ * sy) / den;
    float inter = (sy - slope * st_) / fn;
    // Residual sum of squares, from the same sums.
    float sse = syy - inter * sy - slope * sty;
    *scatter = n > 2 && sse > 0.0f ? sqrtf(sse / (float)(n - 2)) : 0.0f;
    return slope;
}

// One accepted window becomes one unit vector in one sector.
static void accumulate(bearing_cal_t *st, float delta_deg, float heading_deg, float w)
{
    int k = (int)(wrap360(heading_deg) / SECTOR_DEG);
    if (k < 0 || k >= BEARING_SECTORS) {
        k = 0;
    }
    float r = delta_deg * DEG;
    st->sx[k] += w * cosf(r);
    st->sy[k] += w * sinf(r);
    st->sw[k] += w;

    // The same window also goes into one of two halves, alternately. They never
    // share a window, so how far apart they end up is an honest error bar.
    int h = st->next_half;
    st->next_half ^= 1;
    st->hx[h][k] += w * cosf(r);
    st->hy[h][k] += w * sinf(r);
    st->hw[h][k] += w;

    // Fading memory: hold the total weight at the cap so a long motorway run
    // cannot make the estimate too heavy to ever move again.
    float total = 0.0f;
    for (int i = 0; i < BEARING_SECTORS; i++) {
        total += st->sw[i];
    }
    if (total > st->cfg.weight_cap) {
        float f = st->cfg.weight_cap / total;
        for (int i = 0; i < BEARING_SECTORS; i++) {
            st->sx[i] *= f;
            st->sy[i] *= f;
            st->sw[i] *= f;
            for (int j = 0; j < 2; j++) {
                st->hx[j][i] *= f;
                st->hy[j][i] *= f;
                st->hw[j][i] *= f;
            }
        }
    }
}

// Circular mean of a set of sector accumulators, and how many are occupied.
static float sectors_mean(const float *sx, const float *sy, const float *sw, int *occupied)
{
    float gx = 0.0f, gy = 0.0f;
    int m = 0;
    for (int k = 0; k < BEARING_SECTORS; k++) {
        if (sw[k] > 0.0f) {
            gx += sx[k];
            gy += sy[k];
            m++;
        }
    }
    if (occupied) {
        *occupied = m;
    }
    return m ? wrap360(atan2f(gy, gx) / DEG) : 0.0f;
}

static float circular_median(const bearing_hold_t *a, int n)
{
    // n is at most BEARING_HOLD, so an insertion sort is the whole of it. The
    // values are angles, so sort them relative to one of their own.
    float ref = a[0].delta;
    float rel[BEARING_HOLD];
    for (int i = 0; i < n; i++) {
        rel[i] = wrap180(a[i].delta - ref);
    }
    for (int i = 1; i < n; i++) {
        float v = rel[i];
        int j = i - 1;
        while (j >= 0 && rel[j] > v) {
            rel[j + 1] = rel[j];
            j--;
        }
        rel[j + 1] = v;
    }
    float m = n & 1 ? rel[n / 2] : 0.5f * (rel[n / 2 - 1] + rel[n / 2]);
    return wrap360(ref + m);
}

// Spread of held windows about their own median: tight means they agree.
static float hold_spread(const bearing_hold_t *a, int n)
{
    float m = circular_median(a, n), s = 0.0f;
    for (int i = 0; i < n; i++) {
        float e = wrap180(a[i].delta - m);
        s += e * e;
    }
    return sqrtf(s / (float)n);
}

// Takes the median of the held windows, drops the ones that disagree with it,
// and lets the rest through. This is the only place a reversing car is
// filtered out, because until there is an estimate there is nothing to gate on.
static void release_hold(bearing_cal_t *st)
{
    float m = circular_median(st->seed, st->n_seed);
    for (int i = 0; i < st->n_seed; i++) {
        if (fabsf(wrap180(st->seed[i].delta - m)) <= 30.0f) {
            accumulate(st, st->seed[i].delta, st->seed[i].head, st->seed[i].w);
        }
    }
    st->n_seed = 0;
    st->converged = true;
}

static void forget_everything(bearing_cal_t *st)
{
    memset(st->sx, 0, sizeof(st->sx));
    memset(st->sy, 0, sizeof(st->sy));
    memset(st->sw, 0, sizeof(st->sw));
    memset(st->hx, 0, sizeof(st->hx));
    memset(st->hy, 0, sizeof(st->hy));
    memset(st->hw, 0, sizeof(st->hw));
    // The drift measurement restarts too: a remount is not drift, and folding
    // one into the other would make an honest estimator look like a wandering one.
    st->q_resid_n = 0;
    st->q_resid_sq = st->q_resid_sum = st->q_resid_max = 0.0f;
    st->q_n = st->q_st = st->q_stt = st->q_sa = st->q_sta = 0.0f;
    st->q_a_min = st->q_a_max = 0.0f;
    st->q_t0 = -1.0f;
    st->converged = false;
    st->n_seed = 0;
    st->windows = 0;
    st->remounts++;
}

bearing_reject_t bearing_cal_feed(bearing_cal_t *st, const bearing_sample_t *s)
{
    const bearing_cfg_t *c = &st->cfg;

    if (st->n == 0) {
        // A window can only start on a sample that already passes the gates.
        if (!s->usable || s->speed_mps < c->min_speed_mps) {
            return BEARING_REASONS;
        }
        st->t0 = st->t_last = s->t;
        st->c_prev = s->course_deg;
        st->h_prev = s->heading_deg;
        st->course_unwrapped = s->course_deg;
        st->head_unwrapped = s->heading_deg;
        st->d_first = wrap360(s->course_deg - s->decl_deg - s->heading_deg);
        st->speed_min = st->speed_max = s->speed_mps;
    } else {
        // A hole in the fixes makes this two short windows, not one long one.
        if (s->t - st->t_last > 1.0f) {
            st->window_dead = true;
            st->window_why = BEARING_GAP;
        }
        st->course_unwrapped += wrap180(s->course_deg - st->c_prev);
        st->head_unwrapped += wrap180(s->heading_deg - st->h_prev);
        st->c_prev = s->course_deg;
        st->h_prev = s->heading_deg;
        if (s->speed_mps < st->speed_min) {
            st->speed_min = s->speed_mps;
        }
        if (s->speed_mps > st->speed_max) {
            st->speed_max = s->speed_mps;
        }
    }

    if (!s->usable) {
        st->window_dead = true;
        st->window_why = BEARING_UNUSABLE;
    } else if (s->speed_mps < c->min_speed_mps) {
        st->window_dead = true;
        st->window_why = BEARING_TOO_SLOW;
    }

    float t = s->t - st->t0;
    st->t_last = s->t;
    st->n++;
    st->sum_t += t;
    st->sum_tt += t * t;

    float cu = st->course_unwrapped, hu = st->head_unwrapped;
    st->sum_c += cu;
    st->sum_tc += t * cu;
    st->sum_cc += cu * cu;
    st->sum_h += hu;
    st->sum_th += t * hu;
    st->sum_hh += hu * hu;

    // delta stays near constant inside a window, so carry it relative to the
    // first: no unwrapping to get wrong, and the scatter comes out directly.
    float d = wrap180(wrap360(s->course_deg - s->decl_deg - s->heading_deg) - st->d_first);
    st->sum_d += d;
    st->sum_dd += d * d;

    float hr = s->heading_deg * DEG;
    st->head_sum_x += cosf(hr);
    st->head_sum_y += sinf(hr);

    if (t < c->window_s) {
        return BEARING_REASONS;  // still filling
    }

    // ---- the window is full: judge it ----
    bearing_reject_t why = BEARING_OK;
    int n = st->n;
    float dur = t;

    if (st->window_dead) {
        why = st->window_why;
    } else if (n < 8) {
        why = BEARING_SHORT;
    } else if (st->speed_max - st->speed_min > c->speed_tol * st->speed_min) {
        why = BEARING_SPEED_CHANGED;
    }

    float c_scatter = 0.0f, h_scatter = 0.0f, c_slope = 0.0f, h_slope = 0.0f;
    if (why == BEARING_OK) {
        c_slope = regress(n, st->sum_t, st->sum_tt, st->sum_c, st->sum_tc, st->sum_cc, &c_scatter);
        h_slope = regress(n, st->sum_t, st->sum_tt, st->sum_h, st->sum_th, st->sum_hh, &h_scatter);

        if (fabsf(c_slope) > c->max_slope_dps || fabsf(c_slope * dur) > c->max_change_deg) {
            why = BEARING_TURNING;
        } else if (c_scatter > c->max_scatter_deg) {
            why = BEARING_COURSE_ROUGH;
        } else if (fabsf(h_slope) > c->max_slope_dps || fabsf(h_slope * dur) > c->max_change_deg) {
            why = BEARING_TURNING;
        } else if (h_scatter > c->max_scatter_deg) {
            why = BEARING_HEADING_ROUGH;
        } else if (fabsf(c_slope - h_slope) > c->max_slope_diff_dps) {
            // The vehicle turned and the module did not, or the other way
            // about: it is not rigidly attached to what GNSS is tracking.
            why = BEARING_NOT_TOGETHER;
        }
    }

    float delta = 0.0f, mean_head = 0.0f, w = 0.0f;
    if (why == BEARING_OK) {
        float mean_rel = st->sum_d / (float)n;
        delta = wrap360(st->d_first + mean_rel);
        mean_head = wrap360(atan2f(st->head_sum_y, st->head_sum_x) / DEG);

        // The window measured its own scatter, so use that rather than a model
        // of how good the fixes should have been. Course errors correlate over
        // a few seconds, so the window is worth dur/5 independent samples.
        float var = st->sum_dd / (float)n - mean_rel * mean_rel;
        float n_eff = dur / 5.0f;
        if (n_eff < 1.0f) {
            n_eff = 1.0f;
        }
        if (var < 0.01f) {
            var = 0.01f;  // 0.1 degrees: nothing is better than this
        }
        w = n_eff / var;

        if (st->converged) {
            bearing_result_t r;
            bearing_cal_result(st, &r);
            if (fabsf(wrap180(delta - r.offset_deg)) > 3.0f * r.sigma_deg + c->gate_deg) {
                why = BEARING_DISAGREES;
            }
        }
    }

    if (why == BEARING_OK) {
        st->windows++;
        st->n_dissent = 0;  // it agreed, so nothing is being held against it
        bearing_hold_t hold = {.delta = delta, .head = mean_head, .w = w};
        if (st->converged) {
            // Before folding it in: how far was it from what the estimate
            // already predicted for this heading? This window is data the
            // estimate has not seen, which is the only kind worth measuring on.
            bearing_result_t before;
            bearing_cal_result(st, &before);
            if (before.sectors >= 2) {
                float predicted = wrap180(bearing_cal_correct(&before, mean_head) - mean_head);
                float e = wrap180(delta - predicted);
                st->q_resid_n++;
                st->q_resid_sum += e;
                st->q_resid_sq += e * e;
                if (fabsf(e) > st->q_resid_max) {
                    st->q_resid_max = fabsf(e);
                }
            }
            accumulate(st, delta, mean_head, w);

            // And where the estimate stands now, against the clock.
            bearing_result_t after;
            bearing_cal_result(st, &after);
            if (st->q_t0 < 0.0f) {
                st->q_t0 = s->t;
                st->q_a0 = after.offset_deg;
            }
            float dt = s->t - st->q_t0;
            float da = wrap180(after.offset_deg - st->q_a0);
            st->q_n += 1.0f;
            st->q_st += dt;
            st->q_stt += dt * dt;
            st->q_sa += da;
            st->q_sta += dt * da;
            if (da < st->q_a_min) {
                st->q_a_min = da;
            }
            if (da > st->q_a_max) {
                st->q_a_max = da;
            }
            st->q_t_last = s->t;
        } else {
            if (st->n_seed < BEARING_HOLD) {
                st->seed[st->n_seed++] = hold;
            }
            if (st->n_seed >= st->cfg.seed_windows) {
                release_hold(st);
            }
        }
    } else {
        st->rejected[why]++;
        if (why == BEARING_DISAGREES) {
            // Windows that refuse the estimate, one after another. If enough of
            // them agree with each other, the estimate is what is wrong: the
            // module has been moved. Start again from them rather than holding
            // out for readings that will never come.
            if (st->n_dissent < BEARING_HOLD) {
                bearing_hold_t hold = {.delta = delta, .head = mean_head, .w = w};
                st->dissent[st->n_dissent++] = hold;
            }
            if (st->n_dissent >= st->cfg.dissent_windows &&
                hold_spread(st->dissent, st->n_dissent) <= st->cfg.dissent_spread_deg) {
                forget_everything(st);
                memcpy(st->seed, st->dissent, sizeof(bearing_hold_t) * (size_t)st->n_dissent);
                st->n_seed = st->n_dissent;
                st->n_dissent = 0;
                if (st->n_seed >= st->cfg.seed_windows) {
                    release_hold(st);
                }
            }
        } else {
            st->n_dissent = 0;
        }
    }

    window_reset(st);
    return why;
}

// ---- the fit: five coefficients on the sector means, section 3.5 ----

// Solves A x = b for a small symmetric system, and leaves A holding its
// inverse so the standard errors can be read off the diagonal.
static bool solve(float *a, float *b, int p)
{
    float inv[25];
    for (int i = 0; i < p * p; i++) {
        inv[i] = 0.0f;
    }
    for (int i = 0; i < p; i++) {
        inv[i * p + i] = 1.0f;
    }
    for (int col = 0; col < p; col++) {
        int piv = col;
        for (int r = col + 1; r < p; r++) {
            if (fabsf(a[r * p + col]) > fabsf(a[piv * p + col])) {
                piv = r;
            }
        }
        if (fabsf(a[piv * p + col]) < 1e-9f) {
            return false;
        }
        if (piv != col) {
            for (int k = 0; k < p; k++) {
                float t = a[col * p + k]; a[col * p + k] = a[piv * p + k]; a[piv * p + k] = t;
                t = inv[col * p + k]; inv[col * p + k] = inv[piv * p + k]; inv[piv * p + k] = t;
            }
            float t = b[col]; b[col] = b[piv]; b[piv] = t;
        }
        float d = a[col * p + col];
        for (int k = 0; k < p; k++) {
            a[col * p + k] /= d;
            inv[col * p + k] /= d;
        }
        b[col] /= d;
        for (int r = 0; r < p; r++) {
            if (r == col) {
                continue;
            }
            float f = a[r * p + col];
            if (f == 0.0f) {
                continue;
            }
            for (int k = 0; k < p; k++) {
                a[r * p + k] -= f * a[col * p + k];
                inv[r * p + k] -= f * inv[col * p + k];
            }
            b[r] -= f * b[col];
        }
    }
    memcpy(a, inv, sizeof(float) * (size_t)(p * p));
    return true;
}

// delta(H) = A + B sin H + C cos H + D sin 2H + E cos 2H, weighted, on the
// occupied sectors. p is 1, 3 or 5 terms.
static bool fit(const float *head, const float *resid, const float *w, int m, int p,
                float *coef, float *sigma)
{
    float a[25] = {0}, b[5] = {0};
    for (int i = 0; i < m; i++) {
        float h = head[i] * DEG;
        float x[5] = {1.0f, sinf(h), cosf(h), sinf(2.0f * h), cosf(2.0f * h)};
        for (int r = 0; r < p; r++) {
            for (int c = 0; c < p; c++) {
                a[r * p + c] += w[i] * x[r] * x[c];
            }
            b[r] += w[i] * x[r] * resid[i];
        }
    }
    if (!solve(a, b, p)) {
        return false;
    }
    // Residual variance, so the standard errors reflect how well it actually
    // fitted rather than how good the weights claimed the data was.
    float sse = 0.0f, sw = 0.0f;
    for (int i = 0; i < m; i++) {
        float h = head[i] * DEG;
        float x[5] = {1.0f, sinf(h), cosf(h), sinf(2.0f * h), cosf(2.0f * h)};
        float pred = 0.0f;
        for (int r = 0; r < p; r++) {
            pred += b[r] * x[r];
        }
        float e = resid[i] - pred;
        sse += w[i] * e * e;
        sw += w[i];
    }
    float s2 = m > p ? sse / (float)(m - p) : 1.0f / (sw > 0.0f ? sw : 1.0f);
    for (int r = 0; r < p; r++) {
        coef[r] = b[r];
        float v = s2 * a[r * p + r];
        sigma[r] = v > 0.0f ? sqrtf(v) : 0.0f;
    }
    return true;
}

void bearing_cal_result(const bearing_cal_t *st, bearing_result_t *out)
{
    memset(out, 0, sizeof(*out));
    out->windows = st->windows;
    out->remounts = st->remounts;
    memcpy(out->rejected, st->rejected, sizeof(out->rejected));

    float head[BEARING_SECTORS], delta[BEARING_SECTORS], w[BEARING_SECTORS];
    int m = 0;
    float gx = 0.0f, gy = 0.0f;
    for (int k = 0; k < BEARING_SECTORS; k++) {
        if (st->sw[k] <= 0.0f) {
            continue;
        }
        head[m] = (k + 0.5f) * SECTOR_DEG;
        delta[m] = atan2f(st->sy[k], st->sx[k]) / DEG;
        w[m] = st->sw[k];
        gx += st->sx[k];
        gy += st->sy[k];
        m++;
    }
    out->sectors = m;
    if (m == 0) {
        return;
    }

    float mu = wrap360(atan2f(gy, gx) / DEG);
    if (m < 2) {
        // One sector says nothing about the shape, only about the level.
        out->coef[0] = mu;
        out->offset_deg = mu;
        out->terms = 1;
        out->sigma_deg = 180.0f;
        return;
    }

    // Fit the residual about the overall mean: the deviations are small, so
    // ordinary least squares on the wrapped residual is enough, and it avoids
    // fitting a circular quantity with a linear model.
    float resid[BEARING_SECTORS];
    for (int i = 0; i < m; i++) {
        resid[i] = wrap180(delta[i] - mu);
    }

    // As many terms as the spread of sectors can carry, then drop the ones
    // that are not distinguishable from zero.
    int p = m >= 8 ? 5 : m >= 5 ? 3 : 1;
    float coef[5] = {0}, sigma[5] = {0};
    while (p >= 1) {
        if (!fit(head, resid, w, m, p, coef, sigma)) {
            p = p == 5 ? 3 : p == 3 ? 1 : 0;
            continue;
        }
        if (p == 5 && hypotf(coef[3], coef[4]) < 2.0f * hypotf(sigma[3], sigma[4])) {
            p = 3;
            continue;
        }
        if (p == 3 && hypotf(coef[1], coef[2]) < 2.0f * hypotf(sigma[1], sigma[2])) {
            p = 1;
            continue;
        }
        break;
    }
    if (p < 1) {
        out->coef[0] = mu;
        out->offset_deg = mu;
        out->terms = 1;
        out->sigma_deg = 180.0f;
        return;
    }

    out->terms = p;
    out->coef[0] = wrap360(mu + coef[0]);
    for (int r = 1; r < p; r++) {
        out->coef[r] = coef[r];
    }
    out->offset_deg = out->coef[0];
    out->sigma_deg = sigma[0];

    out->once_round_deg = hypotf(out->coef[1], out->coef[2]);
    out->twice_round_deg = hypotf(out->coef[3], out->coef[4]);
    // What those amplitudes say about the magnetometer calibration, in the
    // units the calibration is expressed in.
    out->residual_hard_ut = out->once_round_deg * DEG * st->cfg.h_field_ut;
    out->soft_asym_pct = 2.0f * out->twice_round_deg * DEG * 100.0f;

    out->have = m >= st->cfg.min_sectors && st->converged;

    bearing_quality_t *q = &out->quality;
    q->resid_n = st->q_resid_n;
    if (st->q_resid_n) {
        float n = (float)st->q_resid_n;
        q->resid_bias_deg = st->q_resid_sum / n;
        float ms = st->q_resid_sq / n;
        q->resid_rms_deg = ms > 0.0f ? sqrtf(ms) : 0.0f;
        q->resid_max_deg = st->q_resid_max;
    }
    float a0 = sectors_mean(st->hx[0], st->hy[0], st->hw[0], &q->split_sectors[0]);
    float a1 = sectors_mean(st->hx[1], st->hy[1], st->hw[1], &q->split_sectors[1]);
    q->split_deg = q->split_sectors[0] && q->split_sectors[1] ? fabsf(wrap180(a0 - a1)) : 0.0f;
    if (st->q_n >= 3.0f) {
        float den = st->q_n * st->q_stt - st->q_st * st->q_st;
        if (den > 0.0f) {
            q->drift_deg_per_hour = (st->q_n * st->q_sta - st->q_st * st->q_sa) / den * 3600.0f;
        }
        q->range_deg = st->q_a_max - st->q_a_min;
        q->hours = (st->q_t_last - st->q_t0) / 3600.0f;
    }
    uint32_t seen = (uint32_t)st->windows;
    for (int i = 1; i < BEARING_REASONS; i++) {
        seen += st->rejected[i];
    }
    q->windows_seen = seen;
    q->accept_pct = seen ? 100.0f * (float)st->windows / (float)seen : 0.0f;
}

float bearing_cal_correct(const bearing_result_t *r, float heading_deg)
{
    float h = heading_deg * DEG;
    float d = r->coef[0];
    if (r->terms >= 3) {
        d += r->coef[1] * sinf(h) + r->coef[2] * cosf(h);
    }
    if (r->terms >= 5) {
        d += r->coef[3] * sinf(2.0f * h) + r->coef[4] * cosf(2.0f * h);
    }
    return wrap360(heading_deg + d);
}

void bearing_cal_trace(const bearing_cal_t *st, uint32_t t_s, bearing_trace_t *out)
{
    bearing_result_t r;
    bearing_cal_result(st, &r);
    memset(out, 0, sizeof(*out));
    out->t_s = t_s;
    out->offset_cdeg = (int16_t)lrintf(r.offset_deg * 100.0f);
    out->resid_rms_cdeg = (int16_t)lrintf(fminf(r.quality.resid_rms_deg, 327.0f) * 100.0f);
    out->split_cdeg = (int16_t)lrintf(fminf(r.quality.split_deg, 327.0f) * 100.0f);
    out->sectors = (uint8_t)r.sectors;
    out->terms = (uint8_t)r.terms;
    out->accept_pct = (uint8_t)lrintf(fminf(r.quality.accept_pct, 255.0f));
    out->remounts = (uint8_t)(r.remounts > 255 ? 255 : r.remounts);
}
