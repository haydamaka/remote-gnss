#pragma once

#include <stdbool.h>

// The World Magnetic Model: the Earth's own field at a place and date, from the
// coefficients NOAA publishes. With a GNSS fix the device can therefore know
// what the magnetometer ought to be reading, which gives two things:
//   - an absolute check on a calibration's field strength, instead of merely
//     "20-70 µT is plausible";
//   - the declination, so a heading can be shown against true north.
//
// Coefficients: WMM2025, valid from 2025.0 until 2030.0. Replace them with
// tools/wmm_gen.py when the model is renewed.

#define WMM_EPOCH 2025.0f
#define WMM_VALID_UNTIL 2030.0f

typedef struct {
    float declination;  // degrees, east positive: add to a magnetic heading for true north
    float inclination;  // degrees, down positive
    float intensity;    // total field, µT
    float horizontal;   // horizontal part of it, µT
} wmm_field_t;

// The field at a geodetic position. alt_m is height above the WGS-84 ellipsoid,
// date_years a decimal year such as 2026.7.
//
// False, leaving out untouched, when the date falls outside the model's validity
// or the position is not a number. Latitude is clamped just short of the poles,
// where the horizontal direction is undefined; the error from that is far below
// the model's own.
bool wmm_field(double lat_deg, double lon_deg, float alt_m, float date_years, wmm_field_t *out);

// Decimal year from a calendar date, for feeding the above from a GNSS date.
float wmm_decimal_year(int year, int month, int day);
