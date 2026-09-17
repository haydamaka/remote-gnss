// What bearing_cal.c costs: the numbers quoted in BEARING-CALIBRATION.md 9.
// Memory is exact; the timings are host timings, to be scaled for a 160 MHz
// RV32 with no floating-point unit.
//
//   cc -O2 -Wall -I ../main -o bearing_cost bearing_cost.c ../main/bearing_cal.c -lm
//
// For the flash figure, compile it the way the firmware does:
//
//   riscv32-esp-elf-gcc -O2 -march=rv32imc_zicsr_zifencei -mabi=ilp32 \
//       -c -I ../main -o /tmp/bc.o ../main/bearing_cal.c && riscv32-esp-elf-size /tmp/bc.o

#include "bearing_cal.h"

#include <math.h>
#include <stdio.h>
#include <time.h>

static double since(struct timespec a, struct timespec b, int n)
{
    return ((b.tv_sec - a.tv_sec) + (b.tv_nsec - a.tv_nsec) / 1e9) / n * 1e6;
}

int main(void)
{
    bearing_cfg_t cfg;
    bearing_cal_defaults(&cfg);
    bearing_cal_t st;
    bearing_cal_init(&st, &cfg);

    printf("bearing_cal_t    %5zu bytes, of which %zu are the sector accumulators\n",
           sizeof(bearing_cal_t), sizeof(st.sx) + sizeof(st.sy) + sizeof(st.sw));
    printf("bearing_window_t %5zu bytes per straight run kept\n", sizeof(bearing_window_t));
    printf("bearing_result_t %5zu bytes\n", sizeof(bearing_result_t));
    printf("bearing_trace_t  %5zu bytes per minute of qualifying driving\n\n",
           sizeof(bearing_trace_t));

    // Straight running, round the compass, so every sector fills and the fit
    // has all five terms to work with.
    struct timespec a, b;
    const int N = 2000000;
    float t = 0.0f;
    clock_gettime(CLOCK_MONOTONIC, &a);
    for (int i = 0; i < N; i++) {
        float h = (float)((i / 100) % BEARING_SECTORS) * (360.0f / BEARING_SECTORS) + 15.0f;
        float hr = h * (float)M_PI / 180.0f;
        bearing_sample_t s = {.t = t, .course_deg = h + 137.0f + 8.6f, .speed_mps = 12.0f,
                              .heading_deg = h, .field_ut = {19.4f * cosf(hr), 19.4f * sinf(hr), 47.0f},
                              .decl_deg = 8.6f, .usable = true};
        bearing_window_t run;
        if (bearing_cal_feed(&st, &s, &run) == BEARING_OK) {
            float f[3];
            bearing_window_field(&run, f);
            bearing_cal_add(&st, &run, bearing_cal_heading(f));
        }
        t += 0.1f;
    }
    clock_gettime(CLOCK_MONOTONIC, &b);
    printf("per fix          %6.3f us\n", since(a, b, N));

    bearing_result_t r;
    bearing_cal_result(&st, &r);
    printf("  (%d sectors, %d windows, %d terms)\n", r.sectors, r.windows, r.terms);

    // The sink stops the whole loop being optimised away: the result depends
    // only on state the loop does not change.
    volatile float sink = 0.0f;
    const int M = 200000;
    clock_gettime(CLOCK_MONOTONIC, &a);
    for (int i = 0; i < M; i++) {
        bearing_cal_result(&st, &r);
        sink += r.offset_deg;
    }
    clock_gettime(CLOCK_MONOTONIC, &b);
    printf("one fit          %6.3f us   (once per window, so once every %.0f s)\n",
           since(a, b, M), cfg.window_s);
    return 0;
}
