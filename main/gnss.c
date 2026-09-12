#include "gnss.h"

#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "bearing.h"
#include "gnss_receiver.h"
#include "minmea.h"
#include "nmea_hub.h"
#include "nvs.h"
#include "waypoint.h"

#define GNSS_UART UART_NUM_1
#define UART_RX_BUF_SIZE 2048
#define NO_DATA_WARN_US (5 * 1000 * 1000)
#define STATUS_POLL_US (2 * 1000 * 1000)
#define PVT_POLL_US (1000 * 1000)      // UBX-NAV-PVT, for the receiver's speed accuracy
#define SACC_STALE_US (3 * 1000 * 1000)
#define CFG_ANSWER_US (2 * 1000 * 1000)  // a VALSET unanswered this long is sent again
#define CFG_TRIES 3
// Speed smoothing (smooth_speed() below). Doppler speed above SMOOTH_BELOW_MPS
// is left as it is; below it the velocity vector is smoothed with a 1 s time
// constant. Stopped below 2 x the receiver's speed accuracy, held between the
// two limits, and moving again 0.1 m/s above that.
#define SMOOTH_BELOW_MPS 2.0f
#define SMOOTH_TAU_S 1.0f
#define STOP_MIN_MPS 0.1f
#define STOP_MAX_MPS 0.4f
#define STOP_HYSTERESIS_MPS 0.1f
#define HEADING_PERIOD_US (500 * 1000)  // $HCHDG/$HCHDT at 2 Hz
// $GPRMB/$GPBWC about once a second, whatever the receiver's rate: a little
// under, so a 1 Hz receiver's jitter doesn't halve it.
#define WAYPOINT_PERIOD_US (900 * 1000)
#define BAUD_TRY_US (3 * 1000 * 1000)   // long enough for a sentence at any of these rates
#define RELOCK_US (60 * 1000 * 1000)    // silence this long means the rate was wrong
#define IMPLAUSIBLE_SPEED_KMH 900.0f  // faster than an airliner: the fix jumped
#define IMPLAUSIBLE_MIN_KM 0.5        // below this, a "jump" is just multipath
#define STALE_US (10 * 1000 * 1000)   // a warning older than this says nothing
// A spoofer's fix can look plausible for a moment, and the checks only run once
// per solution, so withholding outlasts the last sign by this much.
#define WITHHOLD_HOLD_US (60 * 1000 * 1000)
#define NVS_NAMESPACE "gnss"
#define NVS_KEY_WITHHOLD "no_spoofed"
#define NVS_KEY_FILTER "filter_spd"
#define NVS_KEY_MOTION "motion"

static const char *TAG = "gnss";

// Receivers disagree about the default: the WS-M181's M10 talks at 115200, a
// ZED-F9P at 38400 out of the box. Rather than make it a setting nobody
// remembers, try rates in turn until a sentence passes its checksum. The
// compiled-in receiver's own default comes first, because that is where one
// that has just been powered on will be whatever the link was set to last
// time; then the rate we mean to settle at, then the usual suspects.
#define BAUD_COUNT 8

static int baud_at(size_t i)
{
    static const int BAUDS[] = {38400, 115200, 9600, 57600, 230400, 460800};
    if (i == 0) {
        return gnss_receiver.default_baud;
    }
    if (i == 1) {
        return CONFIG_RGNSS_GNSS_BAUD;
    }
    return BAUDS[i - 2];
}

// The next rate to try after i, skipping any an earlier index already covers:
// the first two entries are usually in the list as well.
static size_t next_baud(size_t i)
{
    for (size_t n = 1; n <= BAUD_COUNT; n++) {
        size_t j = (i + n) % BAUD_COUNT;
        bool dup = false;
        for (size_t k = 0; k < j; k++) {
            dup = dup || baud_at(k) == baud_at(j);
        }
        if (!dup) {
            return j;
        }
    }
    return 0;
}

// Written only by the reader task, which may therefore read it unlocked;
// everyone else copies it out under the lock.
static portMUX_TYPE lock = portMUX_INITIALIZER_UNLOCKED;
static gnss_data_t data;

static volatile bool withhold_spoofed = true;

// Written by the web server, read by the reader task; each field is one word.
static gnss_settings_t settings = {.filter_speed = true, .motion = GNSS_MOTION_NONE};
static volatile bool cfg_dirty = true;  // the receiver hasn't been sent the settings since they changed
static volatile gnss_cfg_state_t cfg_state = GNSS_CFG_UNSENT;
static bool cfg_awaiting;               // the motion settings went out and nothing has answered them
static int64_t sacc_us;                 // when speed_acc_kmh was last set

// NMEA ddmm.mmmm to signed decimal degrees. Done in double: float would round
// away the last metre or so.
static double to_degrees(const struct minmea_float *f)
{
    if (f->scale == 0) {
        return NAN;
    }
    int64_t per_degree = (int64_t)f->scale * 100;
    int64_t degrees = f->value / per_degree;
    double minutes = (double)(f->value - degrees * per_degree) / f->scale;
    return degrees + minutes / 60.0;
}

static void handle_gga(const char *line)
{
    struct minmea_sentence_gga f;
    if (!minmea_parse_gga(&f, line)) {
        return;
    }
    double lat = to_degrees(&f.latitude);
    double lon = to_degrees(&f.longitude);
    float alt = minmea_tofloat(&f.altitude);
    float hdop = minmea_tofloat(&f.hdop);
    bool fix = f.fix_quality > 0;

    if (fix != data.fix) {
        if (fix) {
            ESP_LOGI(TAG, "fix acquired: %.7f, %.7f, %d satellites", lat, lon, f.satellites_tracked);
        } else {
            ESP_LOGW(TAG, "fix lost");
        }
    }

    taskENTER_CRITICAL(&lock);
    data.fix = fix;
    data.fix_quality = f.fix_quality;
    data.satellites = f.satellites_tracked;
    data.latitude = lat;
    data.longitude = lon;
    data.altitude_m = alt;
    data.hdop = hdop;
    taskEXIT_CRITICAL(&lock);
}

// GST is the receiver's own estimate of how wrong the position is, and the one
// number that separates a real RTK fix from a receiver claiming one. Most
// receivers have to be asked for it; see gnss_f9p.c.
static void handle_gst(const char *line)
{
    struct minmea_sentence_gst f;
    if (!minmea_parse_gst(&f, line)) {
        return;
    }
    float lat = minmea_tofloat(&f.latitude_error_deviation);
    float lon = minmea_tofloat(&f.longitude_error_deviation);
    float acc = isfinite(lat) && isfinite(lon) ? sqrtf(lat * lat + lon * lon) : NAN;
    int64_t now = esp_timer_get_time();

    taskENTER_CRITICAL(&lock);
    data.accuracy_m = acc;
    data.accuracy_us = now;
    taskEXIT_CRITICAL(&lock);
}

// Days from 1970-01-01 to a civil date. newlib has no timegm.
static int64_t to_epoch(int y, int m, int d)
{
    y -= m <= 2;
    int64_t era = (y >= 0 ? y : y - 399) / 400;
    unsigned yoe = (unsigned)(y - era * 400);
    unsigned doy = (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1;
    unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    return era * 146097 + (int64_t)doe - 719468;
}

// Seconds since the epoch of the last valid RMC, 0 before the first one, and
// when it was taken: GGA keeps arriving after the fix is lost, and a frozen
// clock drifting against network time would look exactly like a spoof.
static int64_t utc_epoch, utc_epoch_us;

static float rad(float deg)
{
    return deg * (float)M_PI / 180.0f;
}

// The receiver measures velocity from Doppler, so it is good while moving; at
// rest its noise is a small vector in a random direction, and the speed, being
// its length, never averages to 0. So this smooths the vector rather than the
// speed, which does, and calls it stopped once that is within twice the
// receiver's own speed accuracy estimate (clamped to STOP_MIN/MAX_MPS).
static void smooth_speed(float speed_kmh, float course, int64_t now, float *out_kmh, float *out_course)
{
    static float vn, ve;
    static int64_t last_us;
    static bool stopped;
    float v = isfinite(speed_kmh) ? speed_kmh / 3.6f : 0.0f;
    float rn, re;
    if (isfinite(course)) {
        rn = v * cosf(rad(course));
        re = v * sinf(rad(course));
    } else {
        // No course: the receiver couldn't tell a direction. Keep the last one.
        float h = hypotf(vn, ve);
        rn = h > 0.01f ? vn / h * v : 0.0f;
        re = h > 0.01f ? ve / h * v : 0.0f;
    }
    float dt = last_us ? (now - last_us) / 1e6f : 10.0f;
    last_us = now;
    if (v >= SMOOTH_BELOW_MPS || dt > 3.0f) {
        vn = rn;
        ve = re;
    } else {
        float a = 1.0f - expf(-dt / SMOOTH_TAU_S);
        vn += a * (rn - vn);
        ve += a * (re - ve);
    }
    float s = hypotf(vn, ve);

    float acc = sacc_us && now - sacc_us < SACC_STALE_US ? data.speed_acc_kmh / 3.6f : NAN;
    float enter = isfinite(acc) ? fminf(fmaxf(2.0f * acc, STOP_MIN_MPS), STOP_MAX_MPS) : STOP_MIN_MPS;
    if (stopped ? s > enter + STOP_HYSTERESIS_MPS : s < enter) {
        stopped = !stopped;
    }
    *out_kmh = stopped ? 0.0f : s * 3.6f;
    *out_course = stopped ? NAN : fmodf(atan2f(ve, vn) * 180.0f / (float)M_PI + 360.0f, 360.0f);
}

static void handle_rmc(const char *line)
{
    struct minmea_sentence_rmc f;
    if (!minmea_parse_rmc(&f, line)) {
        return;
    }
    // An invalid RMC can still carry stale time and speed, so ignore it - but
    // the motion from the last fix is no longer true either.
    if (!f.valid) {
        taskENTER_CRITICAL(&lock);
        data.speed_kmh = data.course_deg = NAN;
        data.speed_out_kmh = data.course_out_deg = NAN;
        taskEXIT_CRITICAL(&lock);
        return;
    }
    float speed_kmh = minmea_tofloat(&f.speed) * 1.852f;
    float course = minmea_tofloat(&f.course);
    char utc[sizeof(data.utc)] = "";
    // minmea gives -1 for empty fields, and the year as two digits. The narrowing
    // casts bound the field widths for the compiler's truncation check.
    if (f.date.year >= 0 && f.time.hours >= 0) {
        snprintf(utc, sizeof(utc), "20%02u-%02u-%02u %02u:%02u:%02u", (uint8_t)f.date.year, (uint8_t)f.date.month,
                 (uint8_t)f.date.day, (uint8_t)f.time.hours, (uint8_t)f.time.minutes, (uint8_t)f.time.seconds);
    }

    if (f.date.year >= 0 && f.time.hours >= 0) {
        utc_epoch = to_epoch(2000 + f.date.year, f.date.month, f.date.day) * 86400LL
                    + f.time.hours * 3600 + f.time.minutes * 60 + f.time.seconds;
        utc_epoch_us = esp_timer_get_time();
    }

    int64_t now = esp_timer_get_time();
    taskENTER_CRITICAL(&lock);
    // One per navigation solution, stamped here rather than on the GGA: the
    // receiver sends GGA first, so a reader woken between the two would pair a
    // fresh position with the previous solution's course.
    data.epoch_us = now;
    data.speed_kmh = speed_kmh;
    data.course_deg = course;
    memcpy(data.utc, utc, sizeof(utc));
    taskEXIT_CRITICAL(&lock);

    float out_kmh = speed_kmh, out_course = course;
    if (settings.filter_speed) {
        // The course isn't filtered: below the compass-course speed it is
        // replaced anyway. It goes when the filter says stopped.
        float smoothed_course;
        smooth_speed(speed_kmh, course, now, &out_kmh, &smoothed_course);
        out_course = out_kmh == 0.0f ? NAN : course;
    }
    taskENTER_CRITICAL(&lock);
    data.speed_out_kmh = out_kmh;
    data.course_out_deg = out_course;
    taskEXIT_CRITICAL(&lock);
}

// The receiver answers a poll for UBX-NAV-STATUS with its spoofing verdict.
// Polling needs no configuration, so nothing about the receiver is changed.
static const uint8_t POLL_NAV_STATUS[] = {0xB5, 0x62, 0x01, 0x03, 0x00, 0x00, 0x04, 0x0D};
// UBX-NAV-PVT, polled for sAcc, the receiver's estimate of its speed error.
static const uint8_t POLL_NAV_PVT[] = {0xB5, 0x62, 0x01, 0x07, 0x00, 0x00, 0x08, 0x19};

// UBX-CFG-VALSET keys (u-blox M10 interface description).
#define KEY_NAVSPG_DYNMODEL 0x20110021u   // E1: 0 portable, 3 pedestrian, 4 automotive
#define KEY_MOT_GNSSSPEED_THRS 0x20250038u  // U1, cm/s: static hold below this speed; 0 off
#define KEY_MOT_GNSSDIST_THRS 0x3025003Bu   // U2, m: static hold left beyond this distance

static size_t put_key(uint8_t *p, uint32_t key, uint32_t value, int bytes)
{
    for (int i = 0; i < 4; i++) {
        p[i] = (uint8_t)(key >> (8 * i));
    }
    for (int i = 0; i < bytes; i++) {
        p[4 + i] = (uint8_t)(value >> (8 * i));
    }
    return 4 + (size_t)bytes;
}

// The dynamic model and the static hold thresholds for the settings, to the
// receiver's RAM layer: it forgets them when its power goes, and they are sent
// again every time the reader finds it.
static void send_receiver_config(void)
{
    static bool model_sent;  // since this boot; the receiver keeps RAM across ours
    gnss_settings_t s = settings;
    static const uint8_t models[] = {[GNSS_MOTION_WALKING] = 3, [GNSS_MOTION_VEHICLE] = 4, [GNSS_MOTION_BOTH] = 0};
    static const uint8_t speed_cms[] = {[GNSS_MOTION_NONE] = 30, [GNSS_MOTION_WALKING] = 25,
                                        [GNSS_MOTION_VEHICLE] = 50, [GNSS_MOTION_BOTH] = 30};
    static const uint16_t dist_m[] = {[GNSS_MOTION_NONE] = 5, [GNSS_MOTION_WALKING] = 3,
                                      [GNSS_MOTION_VEHICLE] = 10, [GNSS_MOTION_BOTH] = 5};

    uint8_t msg[6 + 4 + 3 * 8 + 2] = {0xB5, 0x62, 0x06, 0x8A};
    size_t n = 6;
    msg[n++] = 0x00;  // version
    msg[n++] = 0x01;  // RAM layer
    msg[n++] = 0x00;
    msg[n++] = 0x00;
    if (s.motion != GNSS_MOTION_NONE) {
        n += put_key(msg + n, KEY_NAVSPG_DYNMODEL, models[s.motion], 1);
        model_sent = true;
    } else if (model_sent) {
        n += put_key(msg + n, KEY_NAVSPG_DYNMODEL, 0, 1);  // back to the receiver's default, portable
    }
    n += put_key(msg + n, KEY_MOT_GNSSSPEED_THRS, s.filter_speed ? speed_cms[s.motion] : 0, 1);
    n += put_key(msg + n, KEY_MOT_GNSSDIST_THRS, s.filter_speed ? dist_m[s.motion] : 0, 2);
    msg[4] = (uint8_t)(n - 6);
    msg[5] = 0;
    uint8_t a = 0, b = 0;
    for (size_t i = 2; i < n; i++) {
        a += msg[i];
        b += a;
    }
    msg[n++] = a;
    msg[n++] = b;
    uart_write_bytes(GNSS_UART, msg, n);
}

// Feeds one byte to the UBX side of the stream. NMEA and UBX share the wire.
static void feed_ubx(uint8_t b)
{
    // NAV-PVT's sAcc ends at byte 72; the receiver profile sees the first GNSS_UBX_PAYLOAD.
    static uint8_t buf[72 > GNSS_UBX_PAYLOAD ? 72 : GNSS_UBX_PAYLOAD];
    static uint16_t len, got;
    static uint8_t step, cls, id, ck_a, ck_b;

    switch (step) {
    case 0: step = b == 0xB5 ? 1 : 0; return;
    case 1: step = b == 0x62 ? 2 : (b == 0xB5 ? 1 : 0); return;
    case 2: cls = b; ck_a = b; ck_b = b; step = 3; return;
    case 3: id = b; break;
    case 4: len = b; break;
    case 5: len |= (uint16_t)b << 8; got = 0; step = len ? 6 : 7; ck_a += b; ck_b += ck_a; return;
    case 6:
        if (got < sizeof(buf)) {
            buf[got] = b;
        }
        got++;
        ck_a += b;
        ck_b += ck_a;
        if (got == len) {
            step = 7;
        }
        return;
    case 7:  // first checksum byte
        step = b == ck_a ? 8 : 0;
        return;
    case 8:
        step = 0;
        if (b != ck_b) {
            return;
        }
        if (cls == 0x01 && id == 0x07 && len >= 72) {
            // flags (byte 21) bit 0 is gnssFixOK; sAcc is bytes 68-71, mm/s.
            uint32_t sacc = buf[68] | buf[69] << 8 | buf[70] << 16 | (uint32_t)buf[71] << 24;
            float kmh = (buf[21] & 0x01) ? sacc * 0.0036f : NAN;
            taskENTER_CRITICAL(&lock);
            data.speed_acc_kmh = kmh;
            taskEXIT_CRITICAL(&lock);
            sacc_us = esp_timer_get_time();
        } else if (cls == 0x01 && id == 0x03 && len >= 8) {
            // flags2 is byte 7; bits 3-4 hold the spoofing state.
            uint8_t spoof = (buf[7] >> 3) & 0x03;
            int64_t now = esp_timer_get_time();
            taskENTER_CRITICAL(&lock);
            data.spoof = spoof;
            data.spoof_us = now;
            taskEXIT_CRITICAL(&lock);
        } else if (cls == 0x05 && len >= 2 && buf[0] == 0x06 && buf[1] == 0x8A && cfg_awaiting) {
            // Only while the motion settings are the VALSET in flight: the
            // receiver profile's own configuration is acknowledged the same way.
            cfg_awaiting = false;
            cfg_state = id == 0x01 ? GNSS_CFG_OK : GNSS_CFG_REJECTED;
            if (id == 0x01) {
                ESP_LOGI(TAG, "receiver took the motion settings");
            } else {
                ESP_LOGW(TAG, "receiver refused the motion settings");
            }
        } else if (gnss_receiver.ubx) {
            gnss_receiver.ubx(cls, id, buf, len < GNSS_UBX_PAYLOAD ? len : GNSS_UBX_PAYLOAD);
        }
        return;
    default: step = 0; return;
    }
    ck_a += b;
    ck_b += ck_a;
    step++;
}

// The receiver can be lied to, so check the fix against what is possible: a
// date far from this firmware's own, or a jump no vehicle could make.
static void check_plausible(double lat, double lon, const char *utc)
{
    static double last_lat, last_lon;
    static int64_t last_us;
    char why[sizeof(data.implausible_why)] = "";

    int64_t now = esp_timer_get_time();

    // Network time comes from the WiFi side, which the spoofer does not control.
    time_t net = time(NULL);
    if (utc_epoch && now - utc_epoch_us < STALE_US && net > 1767225600) {  // set, i.e. after 2026-01-01
        int64_t off = utc_epoch - (int64_t)net;
        if (off < -120 || off > 120) {
            const char *dir = off > 0 ? "ahead of" : "behind";
            int64_t a = off > 0 ? off : -off;
            const char *unit = "days";
            if (a < 7200) {
                a /= 60;
                unit = "minutes";
            } else if (a < 172800) {
                a /= 3600;
                unit = "hours";
            } else {
                a /= 86400;
            }
            snprintf(why, sizeof(why), "the clock is %lld %s %s network time", a, unit, dir);
        }
    }

    // Without network time - no WiFi, or not synced yet - the year is what is
    // left. A spoofer that sets the date years ahead fails this; one that keeps
    // it close gets past, and only network time catches that.
    int year = atoi(utc);  // "YYYY-..."
    int built = atoi(__DATE__ + 7);
    if (!why[0] && year && (year < built || year > built + 1)) {
        snprintf(why, sizeof(why), "the receiver says it is %d, this firmware is from %d", year, built);
    }
    if (!why[0] && last_us && isfinite(lat) && isfinite(lon)) {
        double dlon_deg = lon - last_lon;
        if (dlon_deg > 180) {
            dlon_deg -= 360;  // the antimeridian is not a 40 000 km jump
        } else if (dlon_deg < -180) {
            dlon_deg += 360;
        }
        double dlat = (lat - last_lat) * 111.32;                     // km
        double dlon = dlon_deg * 111.32 * cos(lat * M_PI / 180);
        double km = sqrt(dlat * dlat + dlon * dlon);
        double hours = (now - last_us) / 3.6e9;
        // A short interval divides into a huge speed, and buffered sentences can
        // be handled microseconds apart, so require a real gap and a real distance.
        if (now - last_us > 500000 && km > IMPLAUSIBLE_MIN_KM && km / hours > IMPLAUSIBLE_SPEED_KMH) {
            snprintf(why, sizeof(why), "the fix jumped %.0f km in %.0f s", km, hours * 3600);
        }
    }
    if (isfinite(lat) && isfinite(lon)) {
        last_lat = lat;
        last_lon = lon;
        last_us = now;
    }
    taskENTER_CRITICAL(&lock);
    data.implausible = why[0] != '\0';
    memcpy(data.implausible_why, why, sizeof(why));
    taskEXIT_CRITICAL(&lock);
}

// Sentences that carried a checksum and passed it. The baud hunt locks on this
// rather than on data.sentences, because minmea's lax mode accepts a sentence
// with no checksum at all - so seven plausible bytes of framing garbage at the
// wrong rate would otherwise end the search there for good.
static uint32_t checksummed;

static void handle_sentence(const char *line)
{
    enum minmea_sentence_id id = minmea_sentence_id(line, false);
    if (id == MINMEA_INVALID) {
        return;
    }
    if (minmea_check(line, true)) {
        checksummed++;
    }
    int64_t now = esp_timer_get_time();
    taskENTER_CRITICAL(&lock);
    data.sentences++;
    data.updated_us = now;
    taskEXIT_CRITICAL(&lock);

    if (id == MINMEA_SENTENCE_GGA) {
        handle_gga(line);
        check_plausible(data.latitude, data.longitude, data.utc);
    } else if (id == MINMEA_SENTENCE_RMC) {
        handle_rmc(line);
    } else if (id == MINMEA_SENTENCE_GST) {
        handle_gst(line);
    }
}

// A sign of spoofing from either detector. Reader task only.
static bool looks_spoofed(int64_t now)
{
    bool receiver = data.spoof_us && now - data.spoof_us <= STALE_US && data.spoof >= GNSS_SPOOF_INDICATED;
    return receiver || data.implausible;
}

// What goes out in place of a sentence while a fix is withheld. The ones that
// say where or when become the receiver's own "no fix" form, so a client shows
// no fix rather than timing out on missing sentences; the satellites in view
// and text messages say nothing about either and pass; the rest, $PUBX
// included, are dropped.
static void publish_withheld(const char *line, size_t len)
{
    static const struct {
        const char *type, *empty;
    } NO_FIX[] = {
        {"GGA", ",,,,,,0,00,99.99,,,,,,"},
        {"RMC", ",,V,,,,,,,,,,N,V"},
        {"GLL", ",,,,,,V,N"},
        {"VTG", ",,,,,,,,,N"},
    };
    if (len >= 7 && line[1] == 'G' && (strncmp(line + 3, "GSV,", 4) == 0 || strncmp(line + 3, "TXT,", 4) == 0)) {
        nmea_hub_publish(line, len);
        return;
    }
    for (size_t i = 0; i < sizeof(NO_FIX) / sizeof(NO_FIX[0]); i++) {
        if (line[1] == 'G' && strncmp(line + 3, NO_FIX[i].type, 3) == 0 && line[6] == ',') {
            char out[48];
            int n = snprintf(out, sizeof(out), "$%c%c%s%s", line[1], line[2], NO_FIX[i].type, NO_FIX[i].empty);
            uint8_t sum = 0;
            for (int k = 1; k < n; k++) {
                sum ^= (uint8_t)out[k];
            }
            n += snprintf(out + n, sizeof(out) - (size_t)n, "*%02X", sum);
            nmea_hub_publish(out, (size_t)n);
            return;
        }
    }
}

#define MAX_FIELDS 24

// Rewrites a sentence with fields[] (0 is the address), and a new checksum.
static size_t rebuild(char *out, size_t size, char *const *fields, int n)
{
    size_t len = 0;
    uint8_t sum = 0;
    for (int i = 0; i < n; i++) {
        int w = snprintf(out + len, size - len, "%s%s", i ? "," : "", fields[i]);
        if (w < 0 || (size_t)w >= size - len) {
            return 0;
        }
        len += (size_t)w;
    }
    for (size_t k = 1; k < len; k++) {
        sum ^= (uint8_t)out[k];
    }
    int w = snprintf(out + len, size - len, "*%02X", sum);
    return w == 3 ? len + 3 : 0;
}

// RMC and VTG go out carrying the motion as the settings want it. With speed
// smoothing on, the smoothed speed and course (no course when stopped). Below
// CONFIG_RGNSS_NMEA_COMPASS_BELOW_KMH the course over ground is noise, so the
// compass heading goes in its place. Only with a fix: a sentence saying "no
// fix" stays as it is. Publishes the sentence either way.
static void publish_motion(const char *line, size_t len)
{
    bool rmc = strncmp(line + 3, "RMC,", 4) == 0, vtg = strncmp(line + 3, "VTG,", 4) == 0;
    if (!(rmc || vtg)) {
        nmea_hub_publish(line, len);
        return;
    }
    taskENTER_CRITICAL(&lock);
    float speed = data.speed_out_kmh, course = data.course_out_deg;
    taskEXIT_CRITICAL(&lock);
    bool smooth = settings.filter_speed;
    float true_deg, mag_deg;
    bool compass = CONFIG_RGNSS_NMEA_COMPASS_BELOW_KMH != 0 &&
                   !(isfinite(speed) && speed >= CONFIG_RGNSS_NMEA_COMPASS_BELOW_KMH) &&
                   bearing_heading(&true_deg, &mag_deg);
    if (!smooth && !compass) {
        nmea_hub_publish(line, len);
        return;
    }

    char body[128];
    const char *star = memchr(line, '*', len);
    size_t body_len = star ? (size_t)(star - line) : len;
    if (body_len >= sizeof(body)) {
        nmea_hub_publish(line, len);
        return;
    }
    memcpy(body, line, body_len);
    body[body_len] = '\0';
    char *fields[MAX_FIELDS];
    int n = 0;
    for (char *p = body; n < MAX_FIELDS; n++) {
        fields[n] = p;
        p = strchr(p, ',');
        if (p == NULL) {
            n++;
            break;
        }
        *p++ = '\0';
    }

    // Rounded here, so 359.97 goes out as 0.0 rather than 360.0.
    char t[8] = "", m[8] = "", kn[12] = "", kmh[12] = "";
    if (compass) {
        snprintf(t, sizeof(t), "%.1f", fmodf(roundf(true_deg * 10.0f) / 10.0f, 360.0f));
        snprintf(m, sizeof(m), "%.1f", fmodf(roundf(mag_deg * 10.0f) / 10.0f, 360.0f));
    } else if (isfinite(course)) {
        snprintf(t, sizeof(t), "%.1f", fmodf(roundf(course * 10.0f) / 10.0f, 360.0f));
    }
    if (isfinite(speed)) {
        snprintf(kn, sizeof(kn), "%.3f", speed / 1.852f);
        snprintf(kmh, sizeof(kmh), "%.3f", speed);
    }
    if (rmc && n >= 9 && strcmp(fields[2], "A") == 0) {
        fields[8] = t;
        if (smooth) {
            fields[7] = kn;
        }
    } else if (vtg && n >= 10 && fields[9][0] && strcmp(fields[9], "N") != 0) {
        fields[1] = t;
        fields[2] = "T";
        if (compass) {
            fields[3] = m;
            fields[4] = "M";
        }
        if (smooth) {
            fields[5] = kn;
            fields[6] = "N";
            fields[7] = kmh;
            fields[8] = "K";
        }
    } else {
        nmea_hub_publish(line, len);
        return;
    }
    char out[128];
    size_t out_len = rebuild(out, sizeof(out), fields, n);
    if (out_len) {
        nmea_hub_publish(out, out_len);
    } else {
        nmea_hub_publish(line, len);
    }
}

// The compass as sentences of its own, for clients that show a heading
// whatever the speed or fix: $HCHDG, magnetic, with the declination once a fix
// has given one, and $HCHDT, true, only then. While the fix is withheld HDT is
// held back and HDG goes without its declination, since a spoofed position
// gives a wrong one; HDG's heading is the magnetometer alone and goes out
// regardless. Nothing without a calibrated
// reading, so a client can tell a silent compass from one pointing north.
static void publish_heading(bool withholding)
{
    float mag_deg, decl;
    if (!bearing_compass(&mag_deg, &decl)) {
        return;
    }
    if (withholding) {
        decl = NAN;
    }
    char h[8], v[8] = "", out[48];
    snprintf(h, sizeof(h), "%.1f", fmodf(roundf(mag_deg * 10.0f) / 10.0f, 360.0f));
    if (!isnan(decl)) {
        snprintf(v, sizeof(v), "%.1f", fabsf(decl));
    }
    char *hdg[] = {"$HCHDG", h, "", "", v, isnan(decl) ? "" : decl < 0 ? "W" : "E"};
    size_t n = rebuild(out, sizeof(out), hdg, 6);
    if (n) {
        nmea_hub_publish(out, n);
    }
    if (isnan(decl)) {
        return;
    }
    snprintf(h, sizeof(h), "%.1f", fmodf(roundf(fmodf(mag_deg + decl + 360.0f, 360.0f) * 10.0f) / 10.0f, 360.0f));
    char *hdt[] = {"$HCHDT", h, "T"};
    n = rebuild(out, sizeof(out), hdt, 3);
    if (n) {
        nmea_hub_publish(out, n);
    }
}

// Splits the byte stream into lines. The receiver also sends binary UBX
// frames; any garbage they leave in a line fails minmea's checksum.
static void reader_task(void *arg)
{
    char line[128];  // NMEA allows 82; minmea takes up to MINMEA_MAX_SENTENCE_LENGTH
    size_t len = 0;
    uint8_t buf[128];
    int64_t last_rx = esp_timer_get_time();
    int64_t last_poll = 0;
    int64_t last_pvt = 0;
    int64_t cfg_sent_us = 0;
    int cfg_tries = 0;
    int64_t last_heading = 0;
    int64_t last_waypoint = 0;
    bool warned = false;

    size_t baud_idx = 0;
    int64_t baud_since = esp_timer_get_time();
    int64_t last_good = baud_since;
    bool locked = false;
    int64_t suspect_us = 0;  // when a sign of spoofing was last seen
    uint32_t seen = 0;   // checksummed as of the last time anything parsed

    for (;;) {
        // Hunt for the rate until something checksums, then stay there.
        if (!locked) {
            if (checksummed > 0) {
                locked = true;
                cfg_dirty = true;  // a receiver found afresh may have been power cycled
                ESP_LOGI(TAG, "%s found at %d baud", gnss_receiver.name, data.baud);
                // Now that the receiver has proved it is there, tell it how we
                // want it. A receiver that has been power-cycled has forgotten,
                // so this runs after every re-lock, not once per boot.
                int baud = gnss_receiver.configure ? gnss_receiver.configure() : 0;
                if (baud && baud != data.baud) {
                    ESP_LOGI(TAG, "moving the link to %d baud", baud);
                    uart_wait_tx_done(GNSS_UART, pdMS_TO_TICKS(1000));
                    taskENTER_CRITICAL(&lock);
                    data.baud = baud;
                    taskEXIT_CRITICAL(&lock);
                    uart_set_baudrate(GNSS_UART, baud);
                    uart_flush_input(GNSS_UART);
                    len = 0;
                    // If it did not follow, nothing will parse and the re-lock
                    // below hunts it down again; start that clock now.
                    last_good = esp_timer_get_time();
                }
                if (gnss_receiver.configure) {
                    // The motion settings wait their turn, so the profile's
                    // answers are in before theirs is being listened for.
                    cfg_sent_us = esp_timer_get_time();
                }
                seen = checksummed;
            } else if (esp_timer_get_time() - baud_since > BAUD_TRY_US) {
                baud_idx = next_baud(baud_idx);
                int baud = baud_at(baud_idx);
                taskENTER_CRITICAL(&lock);
                data.baud = baud;
                taskEXIT_CRITICAL(&lock);
                uart_set_baudrate(GNSS_UART, baud);
                uart_flush_input(GNSS_UART);
                baud_since = esp_timer_get_time();
                len = 0;
                ESP_LOGI(TAG, "nothing at the last rate; trying %d baud", baud);
            }
        }

        // Before the read, so the compass keeps going while the receiver is silent.
        if (esp_timer_get_time() - last_heading > HEADING_PERIOD_US) {
            last_heading = esp_timer_get_time();
            publish_heading(data.withholding);
        }
        if (locked && esp_timer_get_time() - last_pvt > PVT_POLL_US) {
            last_pvt = esp_timer_get_time();
            uart_write_bytes(GNSS_UART, POLL_NAV_PVT, sizeof(POLL_NAV_PVT));
        }
        // The motion settings, until the receiver answers; a few tries.
        if (locked && cfg_dirty) {
            cfg_dirty = false;
            cfg_tries = 0;
            cfg_state = GNSS_CFG_PENDING;
        }
        if (locked && cfg_state == GNSS_CFG_PENDING && esp_timer_get_time() - cfg_sent_us > CFG_ANSWER_US) {
            if (cfg_tries++ < CFG_TRIES) {
                cfg_sent_us = esp_timer_get_time();
                cfg_awaiting = true;
                send_receiver_config();
            } else {
                cfg_state = GNSS_CFG_UNSENT;
                cfg_awaiting = false;
                ESP_LOGW(TAG, "receiver never answered the motion settings");
            }
        }
        if (esp_timer_get_time() - last_poll > STATUS_POLL_US) {
            last_poll = esp_timer_get_time();
            uart_write_bytes(GNSS_UART, POLL_NAV_STATUS, sizeof(POLL_NAV_STATUS));
        }
        int n = uart_read_bytes(GNSS_UART, buf, sizeof(buf), pdMS_TO_TICKS(100));
        // Nothing has parsed for a long time: the rate we settled on may have
        // been wrong, or the receiver was swapped. Hunt again rather than sit
        // silent for ever.
        if (locked && esp_timer_get_time() - last_good > RELOCK_US) {
            ESP_LOGW(TAG, "nothing has parsed for %d s; hunting for the rate again",
                     (int)(RELOCK_US / 1000000));
            locked = false;
            checksummed = 0;
            seen = 0;
            baud_since = esp_timer_get_time();
        }
        if (n <= 0) {
            if (!warned && esp_timer_get_time() - last_rx > NO_DATA_WARN_US) {
                // The UART driver pulls RX up, so a low line is being held down by something.
                ESP_LOGW(TAG, "no data on GPIO%d at %d baud; the line is %s", CONFIG_RGNSS_GNSS_RX_GPIO,
                         data.baud,
                         gpio_get_level(CONFIG_RGNSS_GNSS_RX_GPIO) ? "high (receiver silent or not connected)"
                                                                    : "held low (receiver unpowered?)");
                warned = true;
            }
            continue;
        }
        last_rx = esp_timer_get_time();
        warned = false;
        // Only a sentence that parsed counts as the rate still being right:
        // at the wrong rate there is no shortage of bytes to read, so taking
        // any traffic as proof would leave a re-configured receiver stranded.
        if (checksummed != seen) {
            seen = checksummed;
            last_good = last_rx;
        }

        for (int i = 0; i < n; i++) {
            char c = (char)buf[i];
            feed_ubx(buf[i]);
            if (c == '$') {
                len = 0;
            }
            if (c == '\n') {
                while (len > 0 && line[len - 1] == '\r') {
                    len--;
                }
                line[len] = '\0';
                // Parsed first, so the plausibility checks have seen this
                // solution before deciding whether it may go out.
                handle_sentence(line);

                int64_t now = esp_timer_get_time();
                if (looks_spoofed(now)) {
                    suspect_us = now;
                }
                bool withhold = withhold_spoofed && suspect_us && now - suspect_us < WITHHOLD_HOLD_US;
                if (withhold != data.withholding) {
                    ESP_LOGW(TAG, "%s", withhold ? "the fix looks spoofed; TCP and BLE get no fix until it stops"
                                                 : "no sign of spoofing for a minute; sending the fix again");
                    taskENTER_CRITICAL(&lock);
                    data.withholding = withhold;
                    taskEXIT_CRITICAL(&lock);
                }

                // Out to the network and BLE, all of them: subscribers want the
                // sentence types minmea has no struct for (GSV, GLL, ...) as
                // well. minmea_check screens out the wreckage the binary UBX
                // frames leave behind.
                // The shortest real sentence is "$" + a five-character address
                // + ",". Without the length test a lone 0x24 byte inside a
                // binary UBX frame, followed by a newline, passes minmea_check
                // and goes out as a stub sentence.
                if (len >= 7 && line[0] == '$' && minmea_check(line, false)) {
                    if (withhold) {
                        publish_withheld(line, len);
                    } else {
                        publish_motion(line, len);
                    }
                }
                // Toward the destination, after the RMC that completes a
                // solution. Not while the fix is withheld: the distance and
                // bearing would give the spoofed position away.
                if (!withhold && len >= 7 && strncmp(line + 3, "RMC,", 4) == 0 &&
                    now - last_waypoint >= WAYPOINT_PERIOD_US) {
                    last_waypoint = now;
                    waypoint_publish(&data);
                }
                len = 0;
            } else if (len < sizeof(line) - 1) {
                line[len++] = c;
            } else {
                len = 0;  // no NMEA sentence is this long
            }
        }
    }
}

bool gnss_withhold_spoofed(void)
{
    return withhold_spoofed;
}

esp_err_t gnss_set_withhold_spoofed(bool on)
{
    nvs_handle_t nvs;
    ESP_RETURN_ON_ERROR(nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs), TAG, "open NVS");
    esp_err_t err = nvs_set_u8(nvs, NVS_KEY_WITHHOLD, on);
    if (err == ESP_OK) {
        err = nvs_commit(nvs);
    }
    nvs_close(nvs);
    ESP_RETURN_ON_ERROR(err, TAG, "save the spoofing setting");
    withhold_spoofed = on;
    ESP_LOGI(TAG, "a fix that looks spoofed is %s", on ? "withheld from TCP and BLE" : "sent as it is");
    return ESP_OK;
}

void gnss_get_settings(gnss_settings_t *out, gnss_cfg_state_t *state)
{
    *out = settings;
    *state = cfg_state;
}

esp_err_t gnss_set_settings(const gnss_settings_t *s)
{
    ESP_RETURN_ON_FALSE(s->motion <= GNSS_MOTION_BOTH, ESP_ERR_INVALID_ARG, TAG, "motion");
    nvs_handle_t nvs;
    ESP_RETURN_ON_ERROR(nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs), TAG, "open NVS");
    esp_err_t err = nvs_set_u8(nvs, NVS_KEY_FILTER, s->filter_speed);
    if (err == ESP_OK) {
        err = nvs_set_u8(nvs, NVS_KEY_MOTION, (uint8_t)s->motion);
    }
    if (err == ESP_OK) {
        err = nvs_commit(nvs);
    }
    nvs_close(nvs);
    ESP_RETURN_ON_ERROR(err, TAG, "save the motion settings");
    settings = *s;
    cfg_dirty = true;
    ESP_LOGI(TAG, "speed filter %s, profile %d", s->filter_speed ? "on" : "off", s->motion);
    return ESP_OK;
}

esp_err_t gnss_start(void)
{
    nvs_handle_t nvs;
    uint8_t on;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs) == ESP_OK) {
        if (nvs_get_u8(nvs, NVS_KEY_WITHHOLD, &on) == ESP_OK) {
            withhold_spoofed = on != 0;
        }
        if (nvs_get_u8(nvs, NVS_KEY_FILTER, &on) == ESP_OK) {
            settings.filter_speed = on != 0;
        }
        if (nvs_get_u8(nvs, NVS_KEY_MOTION, &on) == ESP_OK && on <= GNSS_MOTION_BOTH) {
            settings.motion = (gnss_motion_t)on;
        }
        nvs_close(nvs);
    }

    waypoint_load();

    data.latitude = NAN;
    data.longitude = NAN;
    data.altitude_m = NAN;
    data.hdop = NAN;
    data.speed_kmh = NAN;
    data.course_deg = NAN;
    data.speed_out_kmh = NAN;
    data.course_out_deg = NAN;
    data.speed_acc_kmh = NAN;
    data.accuracy_m = NAN;
    data.receiver = gnss_receiver.name;
    data.baud = baud_at(0);

    const uart_config_t cfg = {
        .baud_rate = data.baud,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_RETURN_ON_ERROR(uart_driver_install(GNSS_UART, UART_RX_BUF_SIZE, 0, 0, NULL, 0), TAG, "install driver");
    ESP_RETURN_ON_ERROR(uart_param_config(GNSS_UART, &cfg), TAG, "configure");
    ESP_RETURN_ON_ERROR(uart_set_pin(GNSS_UART, CONFIG_RGNSS_GNSS_TX_GPIO, CONFIG_RGNSS_GNSS_RX_GPIO,
                                     UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE), TAG, "set pins");
    ESP_RETURN_ON_FALSE(xTaskCreate(reader_task, "gnss", 4096, NULL, 5, NULL) == pdPASS, ESP_ERR_NO_MEM,
                        TAG, "create task");
    ESP_LOGI(TAG, "%s on UART1 (RX GPIO%d, TX GPIO%d), starting at %d baud", gnss_receiver.name,
             CONFIG_RGNSS_GNSS_RX_GPIO, CONFIG_RGNSS_GNSS_TX_GPIO, data.baud);
    return ESP_OK;
}

void gnss_get(gnss_data_t *out)
{
    taskENTER_CRITICAL(&lock);
    *out = data;
    taskEXIT_CRITICAL(&lock);

    // A warning outlives nothing: a receiver that has been unplugged, or one
    // that never answers the poll, must not leave a banner up until reboot.
    int64_t now = esp_timer_get_time();
    if (!out->spoof_us || now - out->spoof_us > STALE_US) {
        out->spoof = GNSS_SPOOF_UNKNOWN;
    }
    if (!out->updated_us || now - out->updated_us > STALE_US) {
        out->implausible = false;
        out->implausible_why[0] = '\0';
    }
    // Likewise an accuracy figure: GST can stop while GGA carries on, and a
    // centimetre left on the page would be the most misleading number on it.
    if (!out->accuracy_us || now - out->accuracy_us > STALE_US) {
        out->accuracy_m = NAN;
    }
}

void gnss_write(const void *buf, size_t len)
{
    uart_write_bytes(GNSS_UART, buf, len);
}

void gnss_ubx_send(uint8_t cls, uint8_t id, const void *payload, uint16_t len)
{
    uint8_t head[6] = {0xB5, 0x62, cls, id, (uint8_t)len, (uint8_t)(len >> 8)};
    uint8_t ck_a = 0, ck_b = 0;
    // The checksum covers everything from the class byte to the end of the
    // payload, i.e. all but the two sync characters.
    for (size_t i = 2; i < sizeof(head); i++) {
        ck_a += head[i];
        ck_b += ck_a;
    }
    const uint8_t *p = payload;
    for (uint16_t i = 0; i < len; i++) {
        ck_a += p[i];
        ck_b += ck_a;
    }
    uint8_t tail[2] = {ck_a, ck_b};

    gnss_write(head, sizeof(head));
    if (len) {
        gnss_write(payload, len);
    }
    gnss_write(tail, sizeof(tail));
}

// Bits 28-30 of a configuration key give the value's width: 1 means a single
// bit, 2 one byte, 3 two, 4 four, 5 eight. 0 for a key that is malformed.
static size_t cfg_width(uint32_t key)
{
    switch ((key >> 28) & 0x07) {
    case 1: case 2: return 1;
    case 3: return 2;
    case 4: return 4;
    case 5: return 8;
    default: return 0;
    }
}

void gnss_ubx_valset(const gnss_cfg_t *items, size_t n)
{
    uint8_t buf[4 + GNSS_CFG_MAX * 12];
    buf[0] = 0x00;  // version
    buf[1] = 0x01;  // layers: RAM only, so the receiver's own flash is untouched
    buf[2] = 0x00;
    buf[3] = 0x00;
    size_t len = 4;

    for (size_t i = 0; i < n && i < GNSS_CFG_MAX; i++) {
        size_t width = cfg_width(items[i].key);
        if (!width) {
            ESP_LOGW(TAG, "configuration key 0x%08lx has no width; skipped", (unsigned long)items[i].key);
            continue;
        }
        for (size_t b = 0; b < 4; b++) {
            buf[len++] = (uint8_t)(items[i].key >> (8 * b));
        }
        // Keys wider than the 32-bit value are zero-filled; none used so far is.
        for (size_t b = 0; b < width; b++) {
            buf[len++] = b < 4 ? (uint8_t)(items[i].value >> (8 * b)) : 0;
        }
    }
    if (len > 4) {
        gnss_ubx_send(0x06, 0x8A, buf, len);
    }
}
