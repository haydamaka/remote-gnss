#pragma once

#include <stdbool.h>
#include <stdint.h>

// Learning the compass bearing offset from GNSS, per BEARING-CALIBRATION.md.
//
// Feed it fixes as they arrive. It collects them into windows of measured-
// straight travel (§3.3), emits one sample per accepted window, sorts those
// into 12 sectors of heading (§3.4), and fits the five deviation coefficients
// A, B, C, D, E to the sector means (§3.5).
//
// No RTOS, no storage, no floating-point library beyond libm: this is the part
// that can be tested on a host. The sector accumulators are the only state
// worth saving between runs.

#define BEARING_SECTORS 12  // 30 degrees each, as a compass deviation card

typedef struct {
    float window_s;            // length of a straight run, §3.3.1
    float min_speed_mps;       // below this the course is noise, §3.2
    float speed_tol;           // fractional speed change allowed across a window
    float max_slope_dps;       // turning
    float max_change_deg;      // total course change across the window
    float max_scatter_deg;     // scatter about the fitted line
    float max_slope_diff_dps;  // course and heading must turn together
    float h_field_ut;          // WMM horizontal field, for the physical readouts
    float weight_cap;          // fading memory, §3.6
    int   min_sectors;         // before anything is applied, §3.4
    int   seed_windows;        // circular median over this many first, §3.7
    float gate_deg;            // outlier gate once converged, on top of 3 sigma
    int   dissent_windows;     // this many agreeing refusals mean it was remounted
    float dissent_spread_deg;  // ...and "agreeing" means this tight
} bearing_cfg_t;

// Sensible values; window_s and min_sectors are the two worth tuning.
void bearing_cal_defaults(bearing_cfg_t *cfg);

typedef struct {
    float t;            // seconds, monotonic
    float course_deg;   // GNSS course over ground, referenced to true north
    float speed_mps;
    float heading_deg;  // magnetic heading as measured, no offset applied
    float decl_deg;     // magnetic declination here, from the WMM
    bool  usable;       // fix good, not spoofed, field magnitude plausible
} bearing_sample_t;

// Why a window was thrown away. Counted, so the page can say what the drive
// was mostly doing instead of contributing.
typedef enum {
    BEARING_OK,
    BEARING_UNUSABLE,      // a gate outside this module said no
    BEARING_TOO_SLOW,
    BEARING_SPEED_CHANGED,
    BEARING_GAP,           // missing fixes: the window has a hole in it
    BEARING_SHORT,         // too few samples to judge
    BEARING_TURNING,       // course slope or total change
    BEARING_COURSE_ROUGH,  // scatter about the line
    BEARING_HEADING_ROUGH,
    BEARING_NOT_TOGETHER,  // the module yawed and the vehicle did not
    BEARING_DISAGREES,     // reversing, crabbing, a transient
    BEARING_REASONS
} bearing_reject_t;

// How much the learned adjustment can be trusted. None of this comes from the
// fit's own standard error, which only knows about the scatter it was shown and
// reads absurdly small; every number here is measured against data the estimate
// had not seen, or against an independent half of it.
typedef struct {
    // (1) Out-of-sample: how far each window was from what the estimate
    // predicted for its heading, before that window was folded in.
    uint32_t resid_n;
    float resid_rms_deg;    // the headline number
    float resid_bias_deg;   // a systematic lean says the model is wrong, not noisy
    float resid_max_deg;

    // (2) Split halves: alternate windows feed two independent accumulators.
    // They saw the same roads and the same weather but no shared data, so their
    // disagreement is an honest confidence interval.
    float split_deg;
    int   split_sectors[2];

    // (3) Drift: does it settle, or wander?
    float drift_deg_per_hour;   // least squares of the estimate against time
    float range_deg;            // how far it moved, first to last, at its widest
    float hours;

    // (4) Yield: a measure that never gets to run is no good either.
    float accept_pct;
    uint32_t windows_seen;
} bearing_quality_t;

// 16 bytes, one per minute of qualifying driving: enough to plot the whole
// trip afterwards and see whether the estimate settled.
typedef struct {
    uint32_t t_s;
    int16_t  offset_cdeg;
    int16_t  resid_rms_cdeg;
    int16_t  split_cdeg;
    uint8_t  sectors;
    uint8_t  terms;
    uint8_t  accept_pct;
    uint8_t  remounts;
} bearing_trace_t;

typedef struct {
    bool  have;            // enough, and spread enough, to be applied
    float offset_deg;      // A: what to add to the measured heading, 0..360
    float sigma_deg;       // its standard error
    float coef[5];         // A, B, C, D, E in degrees
    int   terms;           // how many of them survived the significance test
    float once_round_deg;  // hypot(B, C)
    float twice_round_deg; // hypot(D, E)
    float residual_hard_ut;  // what the once-round says is left of the hard iron
    float soft_asym_pct;     // what the twice-round says about axis asymmetry
    int   sectors;         // occupied
    int   windows;         // accepted
    uint32_t remounts;     // times the estimate was thrown away and rebuilt
    uint32_t rejected[BEARING_REASONS];
    bearing_quality_t quality;
} bearing_result_t;

#define BEARING_HOLD 16

typedef struct {
    float delta, head, w;
} bearing_hold_t;

typedef struct {
    bearing_cfg_t cfg;

    // The window being filled. Everything here is a running sum: a window at
    // 10 Hz is 80 fixes and none of them need keeping.
    int   n;
    float t0, t_last;
    float sum_t, sum_tt;
    float course_unwrapped, c_prev, sum_c, sum_tc, sum_cc;
    float head_unwrapped, h_prev, sum_h, sum_th, sum_hh;
    float d_first, sum_d, sum_dd;   // delta, relative to the window's first
    float speed_min, speed_max;
    float head_sum_x, head_sum_y;   // mean heading, for picking the sector
    float interval;                 // the receiver's own fix interval, learned
    bool  window_dead;              // a gate failed; run it out and drop it
    bearing_reject_t window_why;

    // Accepted samples, §3.1: one unit vector per window, per sector.
    float sx[BEARING_SECTORS], sy[BEARING_SECTORS], sw[BEARING_SECTORS];

    // Windows held back until there are enough to tell a good one from a
    // reversing one, §3.7. The same buffer collects windows that disagree with
    // a settled estimate: if enough of them agree with each *other*, the module
    // has been moved and the old estimate is the thing that is wrong.
    bearing_hold_t seed[BEARING_HOLD];
    int   n_seed;
    bearing_hold_t dissent[BEARING_HOLD];
    int   n_dissent;
    bool  converged;
    uint32_t remounts;

    int   windows;
    uint32_t rejected[BEARING_REASONS];

    // ---- quality, all of it measured rather than assumed ----
    uint32_t q_resid_n;
    float q_resid_sq, q_resid_sum, q_resid_max;
    float q_t0, q_a0;                       // where the drift regression starts
    float q_st, q_stt, q_sa, q_sta, q_n;    // its running sums
    float q_a_min, q_a_max;                 // relative to q_a0
    float q_t_last;
    float hx[2][BEARING_SECTORS], hy[2][BEARING_SECTORS], hw[2][BEARING_SECTORS];
    int   next_half;
} bearing_cal_t;

void bearing_cal_init(bearing_cal_t *st, const bearing_cfg_t *cfg);

// Feeds one fix. Returns the outcome of a window if one just ended, else
// BEARING_REASONS as "still filling".
bearing_reject_t bearing_cal_feed(bearing_cal_t *st, const bearing_sample_t *s);

// The current estimate. Cheap enough to call per page refresh.
void bearing_cal_result(const bearing_cal_t *st, bearing_result_t *out);

// Applies what has been learned to a measured heading, in degrees.
float bearing_cal_correct(const bearing_result_t *r, float heading_deg);

const char *bearing_reject_name(bearing_reject_t why);

// Fills one trace record from the current state. The caller decides when: once
// a minute of qualifying driving is the intent.
void bearing_cal_trace(const bearing_cal_t *st, uint32_t t_s, bearing_trace_t *out);
