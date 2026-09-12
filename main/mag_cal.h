#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#include "mag_fit.h"

// Magnetometer calibration: a session collects readings into 100 equal-area
// direction cells (up to 4 each) until they cover the sphere, fits it, saves the
// result to NVS and applies it from then on. The readings themselves are saved
// too (on completion, Stop, failure and every 30 s), so the next session
// continues from them instead of starting empty. The fitting itself is main/mag_fit.c.
// See CALIBRATION.md.

#define MAG_CAL_CELLS 100
#define MAG_CAL_PER_CELL 4
#define MAG_CAL_MAX_POINTS (MAG_CAL_CELLS * MAG_CAL_PER_CELL)

typedef enum {
    MAG_CAL_IDLE,     // no session since boot
    MAG_CAL_RUNNING,
    MAG_CAL_DONE,     // completed and saved
    MAG_CAL_STOPPED,  // interrupted; the previous calibration stays
    MAG_CAL_FAILED,   // see reason; the previous calibration stays
} mag_cal_state_t;

// b = matrix · (m − offset), in µT.
// How much to trust a saved calibration. Anything but NONE is applied: a
// compass that works with a warning beats one that refuses to work.
typedef enum {
    MAG_CAL_GOOD,   // met every criterion of CALIBRATION.md section 2
    MAG_CAL_ROUGH,  // usable, above the good limit
    MAG_CAL_POOR,   // plausible but well off; the page warns
} mag_cal_quality_t;

typedef struct {
    bool valid;
    mag_cal_quality_t quality;
    float offset[3];
    float matrix[9];  // row-major
    float radius;     // µT
    float fit_rms;    // RMS of |b|/radius − 1 over the readings the fit kept
    uint16_t points;
} mag_cal_params_t;

typedef struct {
    mag_cal_state_t state;
    char reason[64];            // why it failed, else empty
    char blocking[64];          // what a running session is still waiting for, else empty
    char message[200];          // how the last session ended, for the user
    float progress;             // 0..1, 1 exactly when complete
    float coverage;             // share of cells holding ≥ 2 readings
    uint8_t missing;            // bit k: cap k of +X −X +Y −Y +Z −Z not reached
    float centre[3];            // current centre estimate
    uint16_t points;            // readings in the buffer
    uint32_t points_seq;        // changes whenever those readings do
    uint32_t refused;           // readings this session turned away as disagreeing
    mag_cal_params_t params;    // the calibration in use
    bool can_accept;            // the fit is usable now, though below the quality bar
    bool have_fit;              // whether fit holds a result yet
    mag_fit_t fit;              // the latest fit, for showing how it's going
    uint32_t fit_ms;            // what that fit cost
} mag_cal_status_t;

// Loads the saved calibration and starts the calibration task. Call after
// magnetometer_start().
esp_err_t mag_cal_init(void);

// Starts a session that continues from the saved readings, or with fresh set,
// from none. ESP_ERR_INVALID_STATE without a magnetometer.
esp_err_t mag_cal_begin(bool fresh);

// Interrupts a running session.
void mag_cal_stop(void);

// Replaces the buffered readings with a set loaded from elsewhere (a file the
// user saved earlier). The session state is cleared; the readings are kept and
// re-binned, so a following session continues from them.
esp_err_t mag_cal_set_points(const float (*pts)[3], size_t n);

// Saves the current fit even though it hasn't met the quality bar, as long as
// it is usable (see ROUGH_* in the source). ESP_ERR_INVALID_STATE if it isn't.
esp_err_t mag_cal_accept(void);

void mag_cal_get_status(mag_cal_status_t *out);

// Copies up to max buffered readings (µT): the session's, or the saved ones.
size_t mag_cal_get_points(float (*out)[3], size_t max);

// One reading at a time, so a caller can stream them without a second copy of
// the buffer. False once i is past the end.
bool mag_cal_get_point(size_t i, float out[3]);

// Bytes of stack the calibration task has never used: the fit is the heaviest
// thing it does, so this is what to watch when the fit grows.
size_t mag_cal_stack_free(void);

// Applies the calibration in use; copies m when there is none. False if none.
bool mag_cal_apply(const float m[3], float b[3]);
