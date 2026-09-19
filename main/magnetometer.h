#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

// Latest magnetometer sample, in the sensor's own axes, uncalibrated. A sample
// comes from ten reads of the chip at 100 Hz, with the WiFi radio's disturbance
// taken out; see sample_task() in magnetometer.c.
typedef struct {
    const char *chip;     // detected chip, NULL if none was found
    int64_t updated_us;   // esp_timer time of the last sample, 0 if none yet
    bool failing;         // the last read of the chip failed
    int16_t raw[3];       // X, Y, Z counts
    float field_ut[3];    // X, Y, Z in microtesla
    uint32_t samples_clean;      // from reads with the radio off
    uint32_t samples_corrected;  // from reads with the radio on, less the offset
    uint32_t samples_skipped;    // blocks with nothing usable, not published
    float radio_offset_ut[3];    // what the powered radio adds to a reading
    bool radio_offset_learned;   // true once corrected samples are allowed
} magnetometer_data_t;

// One entry of the recent-samples history.
typedef struct {
    uint32_t seq;         // 1 for the first sample after boot, then +1 per sample
    uint32_t t_ms;        // esp_timer time
    int16_t raw[3];       // X, Y, Z counts
} magnetometer_sample_t;

// About 25 s at 10 Hz, so a recorder rides out a short WiFi dropout. It was
// 1024, four times longer, which cost 16 kB of the 166 kB this chip has for
// everything - the single largest buffer in the firmware, for a tolerance
// nothing had ever needed.
#define MAGNETOMETER_HISTORY 256

// Finds a QMC5883P, QMC5883L or IST8310 on the I2C pins from menuconfig
// "Remote GNSS → Magnetometer" and starts sampling it.
esp_err_t magnetometer_start(void);

void magnetometer_get(magnetometer_data_t *out);

// Copies up to max of the last MAGNETOMETER_HISTORY samples with seq > since,
// oldest first, and returns how many. latest gets the newest seq (0 if none).
size_t magnetometer_samples_since(uint32_t since, magnetometer_sample_t *out, size_t max, uint32_t *latest);

// Degrees added to the measured heading so the compass reads north where the
// user wants it. Kept in NVS.
float magnetometer_heading_offset(void);

esp_err_t magnetometer_set_heading_offset(float degrees);

// Microtesla per raw count for the detected chip; 0 if none was found.
float magnetometer_ut_per_count(void);
