#pragma once

#include <stdbool.h>
#include <stddef.h>

#include "esp_err.h"

#include "bearing_cal.h"

// Learns the compass bearing offset from GNSS while the device moves, per
// BEARING-CALIBRATION.md. This is the device side: it reads the receiver and
// the magnetometer, feeds bearing_cal.c, keeps the state and a minute-by-minute
// trace in NVS, and serves both to the page.
//
// It starts in **shadow mode**: it measures, and says what it would set, but
// changes nothing until it is told to apply. The point of a drive is to find
// out whether the estimate holds still; see bearing_quality_t.

// One record a minute of qualifying driving. Two hours is a long drive, and
// RAM is the binding constraint on this board once WiFi and BLE have theirs.
#define BEARING_TRACE_MAX 120

// Straight runs kept, so a new magnetometer calibration can re-score the drive
// instead of discarding it. Each is 16 bytes. A trip yields a few dozen, and by
// the time one falls off the end its weight has long faded (§3.6), so this is
// the useful memory of the estimate, not a log.
#define BEARING_RUNS_MAX 256

// Why a fix never even got as far as a window. Without this, a drive that
// produced nothing looks the same as a drive that was never attempted.
typedef enum {
    BEARING_BLOCK_NO_FIX,
    BEARING_BLOCK_SPOOFED,
    BEARING_BLOCK_FEW_SATS,
    BEARING_BLOCK_HDOP,
    BEARING_BLOCK_NO_MAG,
    BEARING_BLOCK_UNCALIBRATED,
    BEARING_BLOCK_NO_WMM,
    BEARING_BLOCK_FIELD_OFF,
    BEARING_BLOCKS
} bearing_block_t;

const char *bearing_block_name(bearing_block_t b);

typedef struct {
    bool learning;   // collecting while moving
    bool applying;   // and correcting the heading with what it learned
    bool have;       // enough to apply, whether or not it is being applied
    float offset_deg;
    float total_hours;     // qualifying driving, across restarts
    uint32_t trace_used;
    uint32_t runs_kept;    // straight runs stored, up to BEARING_RUNS_MAX
    uint32_t stack_free;   // bytes the task has never used
    uint32_t fixes_seen, fixes_usable;
    uint32_t blocked[BEARING_BLOCKS];
    bearing_result_t result;
} bearing_status_t;

// Loads what was saved and starts the task. Call after gnss_start(),
// magnetometer_start() and mag_cal_init().
esp_err_t bearing_start(void);

void bearing_get(bearing_status_t *out);

// Switches collecting and correcting on and off; saves the choice.
esp_err_t bearing_set_learning(bool on);
esp_err_t bearing_set_applying(bool on);

// Throws away everything learned and starts again.
esp_err_t bearing_reset(void);

// Writes the current state to NVS now, rather than waiting for the timer.
esp_err_t bearing_save(void);

// The trace, oldest first, one record at a time: there is no spare RAM on this
// board for a second copy of it.
size_t bearing_trace_count(void);
bool bearing_trace_at(size_t i, bearing_trace_t *out);

// The correction itself is applied by whoever computes a heading: the page does
// it from the coefficients in /api/status, the same way it applies the manual
// offset. bearing_cal_correct() is the shared implementation.
