#pragma once

#include <stdint.h>

#include "esp_err.h"

// Latest magnetometer sample, in the sensor's own axes, uncalibrated.
typedef struct {
    const char *chip;     // detected chip, NULL if none was found
    int64_t updated_us;   // esp_timer time of the last sample, 0 if none yet
    int16_t raw[3];       // X, Y, Z counts
    float field_ut[3];    // X, Y, Z in microtesla
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
