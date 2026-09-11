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

// Finds a QMC5883P, QMC5883L or IST8310 on the I2C pins from menuconfig
// "Remote GNSS → Magnetometer" and starts sampling it.
esp_err_t magnetometer_start(void);

void magnetometer_get(magnetometer_data_t *out);
