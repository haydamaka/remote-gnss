#include "waypoint.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "bearing.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "nmea_hub.h"
#include "nvs.h"

#define NVS_NAMESPACE "waypoint"
#define NVS_KEY_DEST "dest"
#define EARTH_RADIUS_M 6371008.8  // mean radius: the great-circle error is far below a fix's
#define METRES_PER_NM 1852.0
#define KMH_PER_KNOT 1.852

static const char *TAG = "waypoint";

// Set from the BLE command task, read by the GNSS reader.
static portMUX_TYPE lock = portMUX_INITIALIZER_UNLOCKED;
static waypoint_t dest;
static bool have_dest;

static bool valid(const waypoint_t *wp)
{
    if (!isfinite(wp->latitude) || !isfinite(wp->longitude) || fabs(wp->latitude) > 90.0 ||
        fabs(wp->longitude) > 180.0) {
        return false;
    }
    size_t n = strnlen(wp->id, sizeof(wp->id));
    if (n == sizeof(wp->id)) {
        return false;  // not terminated: too long
    }
    // NMEA's reserved characters would break the sentence the name goes out in.
    for (size_t i = 0; i < n; i++) {
        char c = wp->id[i];
        if (c < 0x20 || c > 0x7e || strchr(",*$!\\^~", c) != NULL) {
            return false;
        }
    }
    return true;
}

void waypoint_load(void)
{
    nvs_handle_t nvs;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs) != ESP_OK) {
        return;  // never saved
    }
    waypoint_t wp;
    size_t len = sizeof(wp);
    if (nvs_get_blob(nvs, NVS_KEY_DEST, &wp, &len) == ESP_OK && len == sizeof(wp) && valid(&wp)) {
        taskENTER_CRITICAL(&lock);
        dest = wp;
        have_dest = true;
        taskEXIT_CRITICAL(&lock);
        ESP_LOGI(TAG, "navigating to \"%s\" %.7f, %.7f", wp.id, wp.latitude, wp.longitude);
    }
    nvs_close(nvs);
}

bool waypoint_get(waypoint_t *out)
{
    taskENTER_CRITICAL(&lock);
    bool have = have_dest;
    *out = dest;
    taskEXIT_CRITICAL(&lock);
    return have;
}

esp_err_t waypoint_set(const waypoint_t *wp)
{
    ESP_RETURN_ON_FALSE(valid(wp), ESP_ERR_INVALID_ARG, TAG, "not a waypoint NMEA can carry");
    nvs_handle_t nvs;
    ESP_RETURN_ON_ERROR(nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs), TAG, "open NVS");
    esp_err_t err = nvs_set_blob(nvs, NVS_KEY_DEST, wp, sizeof(*wp));
    if (err == ESP_OK) {
        err = nvs_commit(nvs);
    }
    nvs_close(nvs);
    ESP_RETURN_ON_ERROR(err, TAG, "save the waypoint");
    taskENTER_CRITICAL(&lock);
    dest = *wp;
    have_dest = true;
    taskEXIT_CRITICAL(&lock);
    ESP_LOGI(TAG, "navigating to \"%s\" %.7f, %.7f", wp->id, wp->latitude, wp->longitude);
    return ESP_OK;
}

esp_err_t waypoint_clear(void)
{
    nvs_handle_t nvs;
    ESP_RETURN_ON_ERROR(nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs), TAG, "open NVS");
    esp_err_t err = nvs_erase_key(nvs, NVS_KEY_DEST);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        err = ESP_OK;
    }
    if (err == ESP_OK) {
        err = nvs_commit(nvs);
    }
    nvs_close(nvs);
    ESP_RETURN_ON_ERROR(err, TAG, "forget the waypoint");
    taskENTER_CRITICAL(&lock);
    have_dest = false;
    taskEXIT_CRITICAL(&lock);
    ESP_LOGI(TAG, "no destination");
    return ESP_OK;
}

// "ddmm.mmmm" (or "dddmm.mmmm") and its hemisphere letter to signed degrees.
static bool parse_coordinate(const char *value, const char *hemisphere, char positive, char negative,
                             double *out)
{
    char *end;
    double raw = strtod(value, &end);
    if (end == value || *end != '\0' || raw < 0) {
        return false;
    }
    double degrees = floor(raw / 100.0);
    double minutes = raw - degrees * 100.0;
    if (minutes >= 60.0) {
        return false;
    }
    if (hemisphere[0] == '\0' || hemisphere[1] != '\0' ||
        (hemisphere[0] != positive && hemisphere[0] != negative)) {
        return false;
    }
    double v = degrees + minutes / 60.0;
    *out = hemisphere[0] == positive ? v : -v;
    return true;
}

bool waypoint_parse_wpl(const char *fields, waypoint_t *out)
{
    char buf[64];
    if (strlen(fields) >= sizeof(buf)) {
        return false;
    }
    strcpy(buf, fields);
    char *f[5];
    int n = 0;
    for (char *p = buf; n < 5; n++) {
        f[n] = p;
        p = strchr(p, ',');
        if (p == NULL) {
            n++;
            break;
        }
        *p++ = '\0';
    }
    if (n != 5 || strchr(f[4], ',') != NULL) {
        return false;
    }
    waypoint_t wp = {0};
    if (!parse_coordinate(f[0], f[1], 'N', 'S', &wp.latitude) ||
        !parse_coordinate(f[2], f[3], 'E', 'W', &wp.longitude) || strlen(f[4]) > WAYPOINT_ID_MAX) {
        return false;
    }
    strcpy(wp.id, f[4]);
    *out = wp;
    return true;
}

// Signed degrees to "ddmm.mmmm" and a hemisphere letter. Rounded as a whole, so
// 59.99999 minutes carries into the degrees rather than printing as 60.0000.
static void format_coordinate(char *out, size_t size, double deg, int deg_digits, char positive, char negative,
                              char *hemisphere)
{
    int64_t total = llround(fabs(deg) * 60.0 * 10000.0);  // in 1/10000 of a minute
    snprintf(out, size, "%0*d%02d.%04d", deg_digits, (int)(total / 600000), (int)(total / 10000 % 60),
             (int)(total % 10000));
    hemisphere[0] = deg < 0 ? negative : positive;
    hemisphere[1] = '\0';
}

static double radians(double deg)
{
    return deg * M_PI / 180.0;
}

// Great-circle distance in metres and initial bearing in degrees true, 0-360.
static void distance_bearing(double lat1, double lon1, double lat2, double lon2, double *m, double *deg)
{
    double p1 = radians(lat1), p2 = radians(lat2), dl = radians(lon2 - lon1);
    double a = sin((p2 - p1) / 2) * sin((p2 - p1) / 2) + cos(p1) * cos(p2) * sin(dl / 2) * sin(dl / 2);
    *m = 2 * EARTH_RADIUS_M * atan2(sqrt(a), sqrt(1 - a));
    double y = sin(dl) * cos(p2), x = cos(p1) * sin(p2) - sin(p1) * cos(p2) * cos(dl);
    *deg = fmod(atan2(y, x) * 180.0 / M_PI + 360.0, 360.0);
}

// One decimal, and 359.96 as 0.0 rather than 360.0.
static void format_angle(char *out, size_t size, double deg)
{
    snprintf(out, size, "%.1f", fmod(round(fmod(deg + 360.0, 360.0) * 10.0) / 10.0, 360.0));
}

static void publish(char *const *fields, int n)
{
    char out[NMEA_HUB_MAX_SENTENCE + 1];
    size_t len = 0;
    for (int i = 0; i < n; i++) {
        int w = snprintf(out + len, sizeof(out) - len, "%s%s", i ? "," : "", fields[i]);
        if (w < 0 || (size_t)w >= sizeof(out) - len) {
            return;
        }
        len += (size_t)w;
    }
    uint8_t sum = 0;
    for (size_t k = 1; k < len; k++) {
        sum ^= (uint8_t)out[k];
    }
    int w = snprintf(out + len, sizeof(out) - len, "*%02X", sum);
    if (w == 3) {
        nmea_hub_publish(out, len + 3);
    }
}

bool waypoint_nav(const gnss_data_t *fix, waypoint_t *wp, waypoint_nav_t *out)
{
    if (!waypoint_get(wp) || !fix->fix || !isfinite(fix->latitude) || !isfinite(fix->longitude)) {
        return false;
    }
    distance_bearing(fix->latitude, fix->longitude, wp->latitude, wp->longitude, &out->distance_m,
                     &out->bearing_true);
    // The declination is the bearing task's WMM result for here, not a WMM run
    // of our own: that takes 2 kB of stack, which the GNSS reader task can't spare.
    float decl;
    out->bearing_mag = bearing_declination(&decl) ? fmod(out->bearing_true - decl + 360.0, 360.0) : NAN;
    // The part of the speed that is toward the waypoint: negative moving away.
    // The receiver leaves the course empty when it can't tell, standing still.
    // The speed as it goes out, smoothed and 0 at rest when that is on.
    out->closing_kn = isfinite(fix->speed_out_kmh) && isfinite(fix->course_out_deg)
                          ? fix->speed_out_kmh / KMH_PER_KNOT * cos(radians(fix->course_out_deg - out->bearing_true))
                          : fix->speed_out_kmh == 0.0f ? 0.0 : NAN;
    out->arrived = out->distance_m <= WAYPOINT_ARRIVAL_M;
    return true;
}

void waypoint_publish(const gnss_data_t *fix)
{
    waypoint_t wp;
    waypoint_nav_t nav;
    if (!waypoint_nav(fix, &wp, &nav)) {
        return;
    }
    double m = nav.distance_m, bearing = nav.bearing_true;

    char lat[12], ns[2], lon[12], ew[2], range[12], true_deg[8], mag_deg[8] = "", closing[12] = "";
    format_coordinate(lat, sizeof(lat), wp.latitude, 2, 'N', 'S', ns);
    format_coordinate(lon, sizeof(lon), wp.longitude, 3, 'E', 'W', ew);
    double nm = m / METRES_PER_NM;
    snprintf(range, sizeof(range), nm < 100.0 ? "%.3f" : "%.1f", nm);
    format_angle(true_deg, sizeof(true_deg), bearing);
    if (isfinite(nav.bearing_mag)) {
        format_angle(mag_deg, sizeof(mag_deg), nav.bearing_mag);
    }
    if (isfinite(nav.closing_kn)) {
        snprintf(closing, sizeof(closing), "%.1f", nav.closing_kn);
    }

    // RMB: no route, so no origin and no cross-track error. Range is capped at
    // 999.9 nm as the sentence defines it.
    char rmb_range[12];
    snprintf(rmb_range, sizeof(rmb_range), nm < 100.0 ? "%.3f" : "%.1f", nm < 999.9 ? nm : 999.9);
    char *rmb[] = {"$GPRMB", "A", "", "", "", wp.id, lat, ns, lon, ew, rmb_range, true_deg, closing,
                   m <= WAYPOINT_ARRIVAL_M ? "A" : "V", "A"};
    publish(rmb, sizeof(rmb) / sizeof(rmb[0]));

    // BWC wants the time of the fix; left empty before the receiver has one.
    char utc[16] = "";
    int y, mo, d, h, mi, s;
    if (sscanf(fix->utc, "%d-%d-%d %d:%d:%d", &y, &mo, &d, &h, &mi, &s) == 6) {
        snprintf(utc, sizeof(utc), "%02d%02d%02d.00", h % 24, mi % 60, s % 61);
    }
    char *bwc[] = {"$GPBWC", utc, lat, ns, lon, ew, true_deg, "T", mag_deg, "M", range, "N", wp.id, "A"};
    publish(bwc, sizeof(bwc) / sizeof(bwc[0]));
}
