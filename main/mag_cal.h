#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

// Magnetometer calibration: a session collects readings into 100 equal-area
// direction cells (up to 4 each) until they cover the sphere, fits it, saves the
// result to NVS and applies it from then on. The readings themselves are saved
// too (on completion, Stop, failure and every 30 s), so the next session
// continues from them instead of starting empty. See CALIBRATION.md. The fit is
// a plain sphere (hard iron) until the robust 9-parameter pipeline is ported.

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
typedef struct {
    bool valid;
    float offset[3];
    float matrix[9];  // row-major
    float radius;     // µT
    float fit_rms;    // RMS of |b|/radius − 1
    uint16_t points;
} mag_cal_params_t;

typedef struct {
    mag_cal_state_t state;
    char reason[64];            // why it failed, else empty
    float progress;             // 0..1, 1 exactly when complete
    float coverage;             // share of cells holding ≥ 2 readings
    uint8_t missing;            // bit k: cap k of +X −X +Y −Y +Z −Z not reached
    float centre[3];            // current centre estimate
    uint16_t points;            // readings in the buffer
    mag_cal_params_t params;    // the calibration in use
} mag_cal_status_t;

// Loads the saved calibration and starts the calibration task. Call after
// magnetometer_start().
esp_err_t mag_cal_init(void);

// Starts a session that continues from the saved readings, or with fresh set,
// from none. ESP_ERR_INVALID_STATE without a magnetometer.
esp_err_t mag_cal_begin(bool fresh);

// Interrupts a running session.
void mag_cal_stop(void);

void mag_cal_get_status(mag_cal_status_t *out);

// Copies up to max buffered readings (µT): the session's, or the saved ones.
size_t mag_cal_get_points(float (*out)[3], size_t max);

// Applies the calibration in use; copies m when there is none. False if none.
bool mag_cal_apply(const float m[3], float b[3]);
