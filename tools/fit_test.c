// Host test for main/mag_fit.c: compares against the Python reference on real
// and synthetic recordings, checks degenerate input, and times a fit.
//
//   gcc -O2 -Wall -Wextra -I../main -o /tmp/fit_test fit_test.c ../main/mag_fit.c -lm
//   /tmp/fit_test fit RECORDING.csv | bench RECORDING.csv | degenerate
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "mag_fit.h"

static float points[MAG_FIT_MAX_POINTS][3];
static bool inlier[MAG_FIT_MAX_POINTS];

// Reads seq,t_ms,x_ut,y_ut,z_ut, keeping at most MAG_FIT_MAX_POINTS by an even
// stride. tools/fit_ref.py picks exactly the same readings.
static int load(const char *path)
{
    static float all[20000][3];
    char line[256];
    FILE *f = fopen(path, "r");
    if (!f) {
        perror(path);
        exit(2);
    }
    int total = 0;
    if (!fgets(line, sizeof(line), f)) {
        exit(2);
    }
    while (fgets(line, sizeof(line), f) && total < (int)(sizeof(all) / sizeof(all[0]))) {
        double seq, t, x, y, z;
        if (sscanf(line, "%lf,%lf,%lf,%lf,%lf", &seq, &t, &x, &y, &z) == 5) {
            all[total][0] = (float)x;
            all[total][1] = (float)y;
            all[total][2] = (float)z;
            total++;
        }
    }
    fclose(f);
    int stride = (total + MAG_FIT_MAX_POINTS - 1) / MAG_FIT_MAX_POINTS;
    if (stride < 1) {
        stride = 1;
    }
    int n = 0;
    for (int i = 0; i < total && n < MAG_FIT_MAX_POINTS; i += stride) {
        memcpy(points[n++], all[i], sizeof(all[0]));
    }
    return n;
}

static void print_fit(const mag_fit_t *f, int n)
{
    printf("points %d inliers %u level %u\n", n, f->inliers, f->level);
    printf("offset %.4f %.4f %.4f\n", f->offset[0], f->offset[1], f->offset[2]);
    printf("radius %.5f\n", f->radius);
    printf("rms %.6f\n", f->rms);
    printf("eig %.5f %.5f\n", f->eig_min, f->eig_max);
    printf("cells %u caps_min %u scatter %.4f wobble %.5f\n", f->cells, f->caps_min, f->scatter, f->wobble);
    printf("matrix %.5f %.5f %.5f %.5f %.5f %.5f\n", f->matrix[0], f->matrix[4], f->matrix[8],
           f->matrix[1], f->matrix[2], f->matrix[5]);
}

static void degenerate(void)
{
    mag_fit_t fit;
    int failures = 0;

    struct {
        const char *name;
        int n;
    } cases[] = {{"empty", 0}, {"one point", 1}, {"nine points", 9}, {"all identical", 200},
                 {"one arc", 200}, {"all NaN", 200}, {"some NaN", 200}, {"huge values", 200}};

    for (size_t c = 0; c < sizeof(cases) / sizeof(cases[0]); c++) {
        int n = cases[c].n;
        for (int i = 0; i < n; i++) {
            float a = (float)i / n * 2.0f * (float)M_PI;
            if (strcmp(cases[c].name, "all identical") == 0) {
                points[i][0] = 12.0f;
                points[i][1] = -7.0f;
                points[i][2] = 3.0f;
            } else if (strcmp(cases[c].name, "one arc") == 0) {
                points[i][0] = 16.0f + 50.0f * cosf(a * 0.08f);  // a short arc, no closure
                points[i][1] = 7.0f + 50.0f * sinf(a * 0.08f);
                points[i][2] = -10.0f;
            } else if (strcmp(cases[c].name, "all NaN") == 0) {
                points[i][0] = points[i][1] = points[i][2] = NAN;
            } else if (strcmp(cases[c].name, "some NaN") == 0) {
                points[i][0] = 16.0f + 50.0f * cosf(a) * sinf(a * 3.0f);
                points[i][1] = 7.0f + 50.0f * sinf(a) * sinf(a * 3.0f);
                points[i][2] = -10.0f + 50.0f * cosf(a * 3.0f);
                if (i % 7 == 0) {
                    points[i][1] = NAN;
                }
            } else if (strcmp(cases[c].name, "huge values") == 0) {
                points[i][0] = 1e30f * cosf(a);
                points[i][1] = 1e30f * sinf(a);
                points[i][2] = 1e30f;
            } else {
                points[i][0] = 16.0f + 50.0f * cosf(a);
                points[i][1] = 7.0f + 50.0f * sinf(a);
                points[i][2] = -10.0f;
            }
        }
        bool ok = mag_fit(points, n, 0.5f, NULL, &fit, inlier);
        bool sane = !ok || (isfinite(fit.radius) && isfinite(fit.rms) && isfinite(fit.offset[0])
                            && fit.inliers <= (uint16_t)n);
        printf("%-14s n=%3d -> %-5s %s", cases[c].name, n, ok ? "fit" : "none", sane ? "sane" : "INSANE");
        if (ok) {
            printf("  R=%.2f o=(%.1f,%.1f,%.1f) rms=%.4f", fit.radius, fit.offset[0], fit.offset[1],
                   fit.offset[2], fit.rms);
        }
        printf("\n");
        failures += !sane;
    }
    printf("%s\n", failures ? "DEGENERATE CASES FAILED" : "degenerate cases: all sane");
    exit(failures ? 1 : 0);
}

int main(int argc, char **argv)
{
    if (argc < 2) {
        fprintf(stderr, "usage: fit_test fit|bench FILE.csv | degenerate\n");
        return 2;
    }
    if (strcmp(argv[1], "degenerate") == 0) {
        degenerate();
    }
    if (argc < 3) {
        fprintf(stderr, "need a CSV\n");
        return 2;
    }
    int n = load(argv[2]);
    mag_fit_t fit;
    if (strcmp(argv[1], "bench") == 0) {
        // Cold start (no previous fit) is the worst case the device will meet.
        int runs = 20;
        struct timespec a, b;
        clock_gettime(CLOCK_MONOTONIC, &a);
        for (int i = 0; i < runs; i++) {
            mag_fit(points, n, 0.5f, NULL, &fit, NULL);
        }
        clock_gettime(CLOCK_MONOTONIC, &b);
        double cold = ((b.tv_sec - a.tv_sec) * 1e3 + (b.tv_nsec - a.tv_nsec) / 1e6) / runs;
        mag_fit_t warm = fit;
        clock_gettime(CLOCK_MONOTONIC, &a);
        for (int i = 0; i < runs; i++) {
            mag_fit(points, n, 0.5f, &warm, &fit, NULL);
        }
        clock_gettime(CLOCK_MONOTONIC, &b);
        double hot = ((b.tv_sec - a.tv_sec) * 1e3 + (b.tv_nsec - a.tv_nsec) / 1e6) / runs;
        printf("points %d  cold %.2f ms  continued %.2f ms  (host)\n", n, cold, hot);
        return 0;
    }
    if (!mag_fit(points, n, 0.5f, NULL, &fit, inlier)) {
        printf("no fit\n");
        return 1;
    }
    print_fit(&fit, n);
    return 0;
}
