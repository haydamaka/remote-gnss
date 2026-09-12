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
#include "minmea.h"
#include "nmea_hub.h"

#define GNSS_UART UART_NUM_1
#define UART_RX_BUF_SIZE 2048
#define NO_DATA_WARN_US (5 * 1000 * 1000)
#define STATUS_POLL_US (2 * 1000 * 1000)
#define BAUD_TRY_US (3 * 1000 * 1000)  // long enough for a sentence at any of these rates
#define IMPLAUSIBLE_SPEED_KMH 900.0f  // faster than an airliner: the fix jumped
#define IMPLAUSIBLE_MIN_KM 0.5        // below this, a "jump" is just multipath
#define STALE_US (10 * 1000 * 1000)   // a warning older than this says nothing

static const char *TAG = "gnss";

// Receivers disagree about the default: the WS-M181's M10 talks at 115200,
// a ZED-F9P at 38400 out of the box. Rather than make it a setting nobody
// remembers, try the configured rate first and then the usual suspects until
// a sentence passes its checksum.
static const int BAUDS[] = {CONFIG_RGNSS_GNSS_BAUD, 38400, 115200, 9600, 57600, 230400, 460800};

// Written only by the reader task, which may therefore read it unlocked;
// everyone else copies it out under the lock.
static portMUX_TYPE lock = portMUX_INITIALIZER_UNLOCKED;
static gnss_data_t data;

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

    int64_t now = esp_timer_get_time();
    taskENTER_CRITICAL(&lock);
    data.epoch_us = now;  // one GGA per navigation solution, whatever the rate
    data.fix = fix;
    data.fix_quality = f.fix_quality;
    data.satellites = f.satellites_tracked;
    data.latitude = lat;
    data.longitude = lon;
    data.altitude_m = alt;
    data.hdop = hdop;
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

static void handle_rmc(const char *line)
{
    struct minmea_sentence_rmc f;
    // An invalid RMC can still carry stale time and speed, so ignore it.
    if (!minmea_parse_rmc(&f, line) || !f.valid) {
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

    taskENTER_CRITICAL(&lock);
    data.speed_kmh = speed_kmh;
    data.course_deg = course;
    memcpy(data.utc, utc, sizeof(utc));
    taskEXIT_CRITICAL(&lock);
}

// The receiver answers a poll for UBX-NAV-STATUS with its spoofing verdict.
// Polling needs no configuration, so nothing about the receiver is changed.
static const uint8_t POLL_NAV_STATUS[] = {0xB5, 0x62, 0x01, 0x03, 0x00, 0x00, 0x04, 0x0D};

// Feeds one byte to the UBX side of the stream. NMEA and UBX share the wire.
static void feed_ubx(uint8_t b)
{
    static uint8_t buf[24];
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
        if (b == ck_b && cls == 0x01 && id == 0x03 && len >= 8) {
            // flags2 is byte 7; bits 3-4 hold the spoofing state.
            uint8_t spoof = (buf[7] >> 3) & 0x03;
            int64_t now = esp_timer_get_time();
            taskENTER_CRITICAL(&lock);
            data.spoof = spoof;
            data.spoof_us = now;
            taskEXIT_CRITICAL(&lock);
        }
        step = 0;
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

    int year = atoi(utc);  // "YYYY-..."
    int built = atoi(__DATE__ + 7);
    if (!why[0] && year && (year < built || year > built + 5)) {
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

static void handle_sentence(const char *line)
{
    enum minmea_sentence_id id = minmea_sentence_id(line, false);
    if (id == MINMEA_INVALID) {
        return;
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
    bool warned = false;

    size_t baud_idx = 0;
    int64_t baud_since = esp_timer_get_time();
    bool locked = false;

    for (;;) {
        // Hunt for the rate until something checksums, then stay there.
        if (!locked) {
            if (data.sentences > 0) {
                locked = true;
                ESP_LOGI(TAG, "receiver found at %d baud", data.baud);
            } else if (esp_timer_get_time() - baud_since > BAUD_TRY_US) {
                baud_idx = (baud_idx + 1) % (sizeof(BAUDS) / sizeof(BAUDS[0]));
                if (BAUDS[baud_idx] == CONFIG_RGNSS_GNSS_BAUD && baud_idx != 0) {
                    baud_idx = (baud_idx + 1) % (sizeof(BAUDS) / sizeof(BAUDS[0]));  // already tried
                }
                int baud = BAUDS[baud_idx];
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

        if (esp_timer_get_time() - last_poll > STATUS_POLL_US) {
            last_poll = esp_timer_get_time();
            uart_write_bytes(GNSS_UART, POLL_NAV_STATUS, sizeof(POLL_NAV_STATUS));
        }
        int n = uart_read_bytes(GNSS_UART, buf, sizeof(buf), pdMS_TO_TICKS(100));
        if (n <= 0) {
            if (!warned && esp_timer_get_time() - last_rx > NO_DATA_WARN_US) {
                // The UART driver pulls RX up, so a low line is being held down by something.
                ESP_LOGW(TAG, "no data on GPIO%d at %d baud; the line is %s", CONFIG_RGNSS_GNSS_RX_GPIO,
                         CONFIG_RGNSS_GNSS_BAUD,
                         gpio_get_level(CONFIG_RGNSS_GNSS_RX_GPIO) ? "high (receiver silent or not connected)"
                                                                    : "held low (receiver unpowered?)");
                warned = true;
            }
            continue;
        }
        last_rx = esp_timer_get_time();
        warned = false;

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
                // Out to the network and BLE before parsing: subscribers want
                // the sentence types minmea has no struct for (GSV, GLL, ...)
                // as well. minmea_check screens out the wreckage the binary
                // UBX frames leave behind.
                // The shortest real sentence is "$" + a five-character address
                // + ",". Without the length test a lone 0x24 byte inside a
                // binary UBX frame, followed by a newline, passes minmea_check
                // and goes out as a stub sentence.
                if (len >= 7 && line[0] == '$' && minmea_check(line, false)) {
                    nmea_hub_publish(line, len);
                }
                handle_sentence(line);
                len = 0;
            } else if (len < sizeof(line) - 1) {
                line[len++] = c;
            } else {
                len = 0;  // no NMEA sentence is this long
            }
        }
    }
}

esp_err_t gnss_start(void)
{
    data.latitude = NAN;
    data.longitude = NAN;
    data.altitude_m = NAN;
    data.hdop = NAN;
    data.speed_kmh = NAN;
    data.course_deg = NAN;
    data.baud = CONFIG_RGNSS_GNSS_BAUD;

    const uart_config_t cfg = {
        .baud_rate = CONFIG_RGNSS_GNSS_BAUD,
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
    ESP_LOGI(TAG, "reading UART1 (RX GPIO%d, TX GPIO%d), starting at %d baud", CONFIG_RGNSS_GNSS_RX_GPIO,
             CONFIG_RGNSS_GNSS_TX_GPIO, CONFIG_RGNSS_GNSS_BAUD);
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
}
