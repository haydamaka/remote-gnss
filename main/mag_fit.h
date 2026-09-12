#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// The magnetometer calibration fit: given readings that should lie on a sphere,
// find the offset and shape that put them there. Pure numerics, no RTOS, no
// storage; main/mag_cal.c owns the session, the buffer and the criteria.
// tools/mag_calibrate.py is the reference this was ported from, and
// CALIBRATION.md sections 2, 3.1 and 3.3 explain the algorithm.

#define MAG_FIT_MAX_POINTS 400
#define MAG_FIT_MIN_POINTS 20

// How many parameters the fit dares to use, chosen from the coverage.
enum {
    MAG_FIT_SPHERE = 0,  // offset + one scale
    MAG_FIT_DIAGONAL,    // offset + a scale per axis
    MAG_FIT_FULL,        // offset + symmetric matrix: the 9-parameter model
};

// Calibrated reading: b = matrix * (m - offset), with |b| close to radius.
typedef struct {
    float offset[3];    // o, µT
    float matrix[9];    // W, row-major, symmetric, scaled to det W = 1
    float radius;       // R, µT
    float rms;          // RMS of |b|/R - 1 over the inliers
    float wobble;       // mean per-cell residual vector, relative to R
    float scatter;      // smallest eigenvalue of mean(u*uT) over the inliers
    float eig_min;      // eigenvalues of the det-normalised matrix
    float eig_max;
    uint16_t points;    // readings the fit was given (valid ones)
    uint16_t inliers;
    uint16_t cells;     // direction cells the inliers occupy, of 100
    uint8_t caps_min;   // smallest of the six +-X/+-Y/+-Z cap counts
    uint8_t level;
} mag_fit_t;

// Fits n readings (µT, at most MAG_FIT_MAX_POINTS). noise_ut is the sensor noise
// estimate; 0 asks for the default. start may be NULL, or a previous fit to
// continue from, which is both faster and steadier. inlier may be NULL,
// otherwise it takes one flag per reading, in the order given.
//
// Returns false only when no usable fit exists at all (too few readings, or the
// numbers never settled). A fit that is merely not good enough yet is returned
// with its metrics, and the caller applies the criteria of CALIBRATION.md §2.
// Not reentrant: it works in static scratch buffers.
bool mag_fit(const float (*points)[3], size_t n, float noise_ut,
             const mag_fit_t *start, mag_fit_t *out, bool *inlier);

// Whether a fit is sane enough to define the direction cells. An arc of readings
// fits a huge sphere well, and binning by that sends every reading into one cell.
bool mag_fit_plausible_for_binning(const mag_fit_t *fit);

// Which of the 100 equal-area direction cells a unit vector falls in, the
// partition of CALIBRATION.md section 3.2. Shared so the buffer bins readings
// exactly as the fit counts coverage.
uint8_t mag_fit_cell(const float u[3]);

// b = matrix * (m - offset). in and out may be the same array.
void mag_fit_apply(const mag_fit_t *fit, const float m[3], float b[3]);

// |b|/R - 1 for one reading: how far off the sphere it sits.
float mag_fit_residual(const mag_fit_t *fit, const float m[3]);
