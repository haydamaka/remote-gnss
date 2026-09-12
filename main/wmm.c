#include "wmm.h"

#include <math.h>

// Spherical-harmonic evaluation of the World Magnetic Model, as described in
// its technical report. The arithmetic runs in double: the series adds ninety
// terms that cancel against each other, and float would eat into the last
// digits that matter. Everything the caller sees is float.

#define WMM_DEGREE 12
#define WMM_COEFFS (WMM_DEGREE * (WMM_DEGREE + 3) / 2)  // 90: n = 1..12, m = 0..n

// Coefficients are in nT and nT/year; the caller gets µT.
// WMM-2025, epoch 2025.0, 90 coefficients to degree 12.
// Generated from WMM.COF by tools/wmm_gen.py; do not edit by hand.
static const float g[WMM_COEFFS] = {
      -29351.8f,    -1410.8f,    -2556.6f,     2951.1f,     1649.3f,     1361.0f,
       -2404.1f,     1243.8f,      453.6f,      895.0f,      799.5f,       55.7f,
        -281.1f,       12.1f,     -233.2f,      368.9f,      187.2f,     -138.7f,
        -142.0f,       20.9f,       64.4f,       63.8f,       76.9f,     -115.7f,
         -40.9f,       14.9f,      -60.7f,       79.5f,      -77.0f,       -8.8f,
          59.3f,       15.8f,        2.5f,      -11.1f,       14.2f,       23.2f,
          10.8f,      -17.5f,        2.0f,      -21.7f,       16.9f,       15.0f,
         -16.8f,        0.9f,        4.6f,        7.8f,        3.0f,       -0.2f,
          -2.5f,      -13.1f,        2.4f,        8.6f,       -8.7f,      -12.9f,
          -1.3f,       -6.4f,        0.2f,        2.0f,       -1.0f,       -0.6f,
          -0.9f,        1.5f,        0.9f,       -2.7f,       -3.9f,        2.9f,
          -1.5f,       -2.5f,        2.4f,       -0.6f,       -0.1f,       -0.6f,
          -0.1f,        1.1f,       -1.0f,       -0.2f,        2.6f,       -2.0f,
          -0.2f,        0.3f,        1.2f,       -1.3f,        0.6f,        0.6f,
           0.5f,       -0.1f,       -0.4f,       -0.2f,       -1.3f,       -0.7f,
};
static const float h[WMM_COEFFS] = {
           0.0f,     4545.4f,        0.0f,    -3133.6f,     -815.1f,        0.0f,
         -56.6f,      237.5f,     -549.5f,        0.0f,      278.6f,     -133.9f,
         212.0f,     -375.6f,        0.0f,       45.4f,      220.2f,     -122.9f,
          43.0f,      106.1f,        0.0f,      -18.4f,       16.8f,       48.8f,
         -59.8f,       10.9f,       72.7f,        0.0f,      -48.9f,      -14.4f,
          -1.0f,       23.4f,       -7.4f,      -25.1f,       -2.3f,        0.0f,
           7.1f,      -12.6f,       11.4f,       -9.7f,       12.7f,        0.7f,
          -5.2f,        3.9f,        0.0f,      -24.8f,       12.2f,        8.3f,
          -3.3f,       -5.2f,        7.2f,       -0.6f,        0.8f,       10.0f,
           0.0f,        3.3f,        0.0f,        2.4f,        5.3f,       -9.1f,
           0.4f,       -4.2f,       -3.8f,        0.9f,       -9.1f,        0.0f,
           0.0f,        2.9f,       -0.6f,        0.2f,        0.5f,       -0.3f,
          -1.2f,       -1.7f,       -2.9f,       -1.8f,       -2.3f,        0.0f,
          -1.3f,        0.7f,        1.0f,       -1.4f,       -0.0f,        0.6f,
          -0.1f,        0.8f,        0.1f,       -1.0f,        0.1f,        0.2f,
};
static const float g_dot[WMM_COEFFS] = {
          12.0f,        9.7f,      -11.6f,       -5.2f,       -8.0f,       -1.3f,
          -4.2f,        0.4f,      -15.6f,       -1.6f,       -2.4f,       -6.0f,
           5.6f,       -7.0f,        0.6f,        1.4f,        0.0f,        0.6f,
           2.2f,        0.9f,       -0.2f,       -0.4f,        0.9f,        1.2f,
          -0.9f,        0.3f,        0.9f,       -0.0f,       -0.1f,       -0.1f,
           0.5f,       -0.1f,       -0.8f,       -0.8f,        0.8f,       -0.1f,
           0.2f,        0.0f,        0.5f,       -0.1f,        0.3f,        0.2f,
          -0.0f,        0.2f,       -0.0f,       -0.1f,        0.1f,        0.3f,
          -0.3f,        0.0f,        0.3f,       -0.1f,        0.1f,       -0.1f,
           0.1f,        0.0f,        0.1f,        0.1f,       -0.0f,       -0.3f,
           0.0f,       -0.1f,       -0.1f,       -0.0f,       -0.0f,        0.0f,
          -0.0f,        0.0f,        0.0f,        0.0f,       -0.1f,        0.0f,
          -0.0f,       -0.1f,       -0.1f,       -0.1f,       -0.1f,        0.0f,
           0.0f,       -0.0f,       -0.0f,       -0.0f,       -0.0f,        0.1f,
          -0.0f,        0.0f,        0.0f,       -0.1f,       -0.0f,       -0.1f,
};
static const float h_dot[WMM_COEFFS] = {
           0.0f,      -21.5f,        0.0f,      -27.7f,      -12.1f,        0.0f,
           4.0f,       -0.3f,       -4.1f,        0.0f,       -1.1f,        4.1f,
           1.6f,       -4.4f,        0.0f,       -0.5f,        2.2f,        0.4f,
           1.7f,        1.9f,        0.0f,        0.3f,       -1.6f,       -0.4f,
           0.9f,        0.7f,        0.9f,        0.0f,        0.6f,        0.5f,
          -0.8f,        0.0f,       -1.0f,        0.6f,       -0.2f,        0.0f,
          -0.2f,        0.5f,       -0.4f,        0.4f,       -0.5f,       -0.6f,
           0.3f,        0.2f,        0.0f,       -0.3f,        0.3f,       -0.3f,
           0.3f,        0.2f,       -0.1f,       -0.2f,        0.4f,        0.1f,
           0.0f,        0.0f,       -0.0f,       -0.2f,        0.1f,       -0.1f,
           0.1f,        0.0f,       -0.1f,        0.2f,       -0.0f,        0.0f,
          -0.0f,        0.1f,       -0.0f,        0.1f,       -0.0f,       -0.0f,
           0.1f,       -0.0f,        0.0f,        0.0f,        0.0f,        0.0f,
          -0.0f,        0.0f,       -0.1f,        0.1f,       -0.0f,       -0.0f,
          -0.0f,        0.0f,       -0.0f,       -0.0f,        0.0f,       -0.1f,
};

// WGS-84, and the reference radius the model's coefficients are defined against.
#define WGS84_A_KM 6378.137
#define WGS84_F (1.0 / 298.257223563)
#define GEOMAG_R_KM 6371.2
#define POLE_LIMIT_DEG 89.9999  // the horizontal direction is undefined at the pole itself

// n = 1..WMM_DEGREE, m = 0..n, packed triangularly.
static inline int idx(int n, int m)
{
    return n * (n + 1) / 2 + m - 1;
}

// The same packing for the Legendre functions, which also need (0,0).
static inline int pidx(int n, int m)
{
    return n * (n + 1) / 2 + m;
}

float wmm_decimal_year(int year, int month, int day)
{
    static const int cumulative[12] = {0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334};
    if (month < 1 || month > 12 || day < 1 || day > 31) {
        return (float)year;
    }
    bool leap = (year % 4 == 0 && year % 100 != 0) || year % 400 == 0;
    int doy = cumulative[month - 1] + day - 1 + (leap && month > 2 ? 1 : 0);
    return (float)year + (float)doy / (leap ? 366.0f : 365.0f);
}

bool wmm_field(double lat_deg, double lon_deg, float alt_m, float date_years, wmm_field_t *out)
{
    if (!isfinite(lat_deg) || !isfinite(lon_deg) || !isfinite(alt_m) || !isfinite(date_years)) {
        return false;
    }
    if (date_years < WMM_EPOCH || date_years >= WMM_VALID_UNTIL) {
        return false;
    }
    if (lat_deg > POLE_LIMIT_DEG) {
        lat_deg = POLE_LIMIT_DEG;
    } else if (lat_deg < -POLE_LIMIT_DEG) {
        lat_deg = -POLE_LIMIT_DEG;
    }

    // Geodetic to geocentric: the model is spherical, the fix is on an ellipsoid.
    const double e2 = WGS84_F * (2.0 - WGS84_F);
    double lat = lat_deg * M_PI / 180.0;
    double lon = lon_deg * M_PI / 180.0;
    double sin_lat = sin(lat), cos_lat = cos(lat);
    double height_km = alt_m / 1000.0;
    double curvature = WGS84_A_KM / sqrt(1.0 - e2 * sin_lat * sin_lat);
    double p = (curvature + height_km) * cos_lat;
    double z = (curvature * (1.0 - e2) + height_km) * sin_lat;
    double r = sqrt(p * p + z * z);
    double sin_gc = z / r, cos_gc = p / r;  // geocentric latitude

    // Schmidt semi-normalised associated Legendre functions and their slopes,
    // by the usual recursion in latitude.
    // Stored as float to keep this off the stack: measured against the same
    // table in double, it costs 0.0002 degrees at worst, against a table
    // printed to 0.01 and a compass good to about a degree.
    float P[WMM_COEFFS + 1], dP[WMM_COEFFS + 1];
    P[pidx(0, 0)] = 1.0;
    dP[pidx(0, 0)] = 0.0;
    for (int n = 1; n <= WMM_DEGREE; n++) {
        for (int m = 0; m <= n; m++) {
            int i = pidx(n, m);
            if (m == n) {  // sectoral: straight from the one above
                // P(1,1) is cos(latitude) itself; Schmidt's factor starts at n = 2,
                // because m = 0 and m > 0 are normalised differently.
                double k = n == 1 ? 1.0 : sqrt((2.0 * n - 1.0) / (2.0 * n));
                double p1 = P[pidx(n - 1, n - 1)], dp1 = dP[pidx(n - 1, n - 1)];
                P[i] = k * cos_gc * p1;
                dP[i] = k * (cos_gc * dp1 - sin_gc * p1);
            } else {
                double a = sqrt((double)(n * n - m * m));
                double b = sqrt((double)((n - 1) * (n - 1) - m * m));
                double p1 = P[pidx(n - 1, m)], dp1 = dP[pidx(n - 1, m)];
                double p2 = (n - 2 >= m) ? P[pidx(n - 2, m)] : 0.0;
                double dp2 = (n - 2 >= m) ? dP[pidx(n - 2, m)] : 0.0;
                P[i] = ((2.0 * n - 1.0) * sin_gc * p1 - b * p2) / a;
                dP[i] = ((2.0 * n - 1.0) * (sin_gc * dp1 + cos_gc * p1) - b * dp2) / a;
            }
        }
    }

    // Sum the series. X' north, Y' east, Z' down, in geocentric coordinates.
    double years = date_years - WMM_EPOCH;
    double ratio = GEOMAG_R_KM / r;
    double power = ratio * ratio;  // (a/r)^(n+2) for n = 0, stepped up below
    double xp = 0.0, yp = 0.0, zp = 0.0;
    double cos_m[WMM_DEGREE + 1], sin_m[WMM_DEGREE + 1];
    double cos_lon = cos(lon), sin_lon = sin(lon);
    cos_m[0] = 1.0;
    sin_m[0] = 0.0;
    for (int m = 1; m <= WMM_DEGREE; m++) {  // cos/sin of m·longitude, by angle addition
        cos_m[m] = cos_m[m - 1] * cos_lon - sin_m[m - 1] * sin_lon;
        sin_m[m] = sin_m[m - 1] * cos_lon + cos_m[m - 1] * sin_lon;
    }
    for (int n = 1; n <= WMM_DEGREE; n++) {
        power *= ratio;  // now (a/r)^(n+2)
        for (int m = 0; m <= n; m++) {
            int i = idx(n, m);
            int pi = pidx(n, m);
            double gt = g[i] + years * g_dot[i];
            double ht = h[i] + years * h_dot[i];
            double along = gt * cos_m[m] + ht * sin_m[m];
            double across = gt * sin_m[m] - ht * cos_m[m];
            xp -= power * along * dP[pi];
            yp += power * m * across * P[pi] / cos_gc;
            zp -= power * (n + 1) * along * P[pi];
        }
    }

    // Back to geodetic: the two latitudes differ by up to about 0.19 degrees.
    double tilt = asin(sin_gc) - lat;
    double cos_tilt = cos(tilt), sin_tilt = sin(tilt);
    double north = xp * cos_tilt - zp * sin_tilt;
    double east = yp;
    double down = xp * sin_tilt + zp * cos_tilt;

    double horizontal = sqrt(north * north + east * east);
    out->horizontal = (float)(horizontal / 1000.0);
    out->intensity = (float)(sqrt(horizontal * horizontal + down * down) / 1000.0);
    out->declination = (float)(atan2(east, north) * 180.0 / M_PI);
    out->inclination = (float)(atan2(down, horizontal) * 180.0 / M_PI);
    return true;
}
