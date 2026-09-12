// Checks main/wmm.c against NOAA's published test values, on the host.
//
//   gcc -O2 -Wall -Wextra -I main -o /tmp/wmm_test tools/wmm_test.c main/wmm.c -lm
//   /tmp/wmm_test tools/wmm/WMM2025_TestValues.txt tools/wmm/WMM2025_TEST_VALUES.txt
//
// Both of NOAA's layouts are read, told apart by how many columns a row has:
//   18: year alt_km lat lon D I H X Y Z F  (+ rates)      -- bundled with the coefficients
//   19: year alt_km lat lon X Y Z H F I D GV (+ rates)     -- the standalone test values
// The two agree; having both is a check on the reader as much as on the model.

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "wmm.h"

// Worst deviation seen, and where.
typedef struct {
    double value;
    double lat, lon, alt_km, year;
} worst_t;

static void note(worst_t *w, double deviation, double lat, double lon, double alt_km, double year)
{
    if (!(fabs(deviation) > fabs(w->value))) {
        return;
    }
    w->value = deviation;
    w->lat = lat;
    w->lon = lon;
    w->alt_km = alt_km;
    w->year = year;
}

static void print_worst(const char *what, const worst_t *w, const char *unit)
{
    printf("  %-12s %8.4f %-3s at %6.1f N %7.1f E, %5.1f km, %.1f\n", what, w->value, unit, w->lat, w->lon,
           w->alt_km, w->year);
}

// Declination wraps: -179.9 and 180.0 are a tenth of a degree apart.
static double angle_difference(double a, double b)
{
    double d = fmod(a - b + 540.0, 360.0) - 180.0;
    return d;
}

int main(int argc, char **argv)
{
    if (argc < 2) {
        fprintf(stderr, "usage: %s <NOAA test value file> ...\n", argv[0]);
        return 2;
    }

    worst_t worst_d = {0}, worst_i = {0}, worst_f = {0}, worst_h = {0};
    int rows = 0, refused = 0;

    for (int a = 1; a < argc; a++) {
        FILE *fp = fopen(argv[a], "r");
        if (!fp) {
            perror(argv[a]);
            return 2;
        }
        char line[512];
        int file_rows = 0;
        while (fgets(line, sizeof(line), fp)) {
            if (line[0] == '#' || strspn(line, " \t\r\n") == strlen(line)) {
                continue;
            }
            double v[24];
            int n = 0;
            for (char *p = line, *end; n < 24; p = end) {
                double x = strtod(p, &end);
                if (end == p) {
                    break;
                }
                v[n++] = x;
            }
            double year = v[0], alt_km = v[1], lat = v[2], lon = v[3];
            double want_d, want_i, want_h, want_f;
            if (n >= 19) {
                want_h = v[7]; want_f = v[8]; want_i = v[9]; want_d = v[10];
            } else if (n >= 11) {
                want_d = v[4]; want_i = v[5]; want_h = v[6]; want_f = v[10];
            } else {
                continue;
            }

            wmm_field_t got;
            if (!wmm_field(lat, lon, (float)(alt_km * 1000.0), (float)year, &got)) {
                refused++;
                continue;
            }
            note(&worst_d, angle_difference(got.declination, want_d), lat, lon, alt_km, year);
            note(&worst_i, got.inclination - want_i, lat, lon, alt_km, year);
            note(&worst_f, got.intensity - want_f / 1000.0, lat, lon, alt_km, year);
            note(&worst_h, got.horizontal - want_h / 1000.0, lat, lon, alt_km, year);
            rows++;
            file_rows++;
        }
        fclose(fp);
        printf("%s: %d rows\n", argv[a], file_rows);
    }

    printf("\n%d rows checked, %d refused. Largest deviation from NOAA:\n", rows, refused);
    print_worst("declination", &worst_d, "deg");
    print_worst("inclination", &worst_i, "deg");
    print_worst("intensity", &worst_f, "uT");
    print_worst("horizontal", &worst_h, "uT");

    // Kyiv, where the parent measured a fitted field radius of about 50.2 uT.
    wmm_field_t kyiv;
    if (wmm_field(50.59, 30.62, 180.0f, wmm_decimal_year(2026, 9, 12), &kyiv)) {
        printf("\nKyiv 50.59 N 30.62 E, 180 m, 2026-09-12:\n");
        printf("  declination %.2f deg, inclination %.2f deg, F %.2f uT, H %.2f uT\n", kyiv.declination,
               kyiv.inclination, kyiv.intensity, kyiv.horizontal);
    }

    // What one call costs. The device is a 160 MHz ESP32-C3 with no FPU, so
    // scale accordingly; this is only the host figure.
    const int reps = 20000;
    wmm_field_t sink;
    clock_t t0 = clock();
    for (int i = 0; i < reps; i++) {
        wmm_field(50.59 + i % 90, 30.62, 180.0f, 2026.7f, &sink);
    }
    double us = (double)(clock() - t0) / CLOCKS_PER_SEC / reps * 1e6;
    printf("\n%.1f us per call on this host (%d calls)\n", us, reps);

    double tolerance_d = 0.1, tolerance_i = 0.1, tolerance_f = 0.05;
    bool ok = fabs(worst_d.value) < tolerance_d && fabs(worst_i.value) < tolerance_i &&
              fabs(worst_f.value) < tolerance_f;
    printf("\n%s\n", ok ? "PASS" : "FAIL");
    return ok ? 0 : 1;
}
