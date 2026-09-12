#include "gnss.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

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

static const char *TAG = "gnss";

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

    taskENTER_CRITICAL(&lock);
    data.speed_kmh = speed_kmh;
    data.course_deg = course;
    memcpy(data.utc, utc, sizeof(utc));
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
    bool warned = false;

    for (;;) {
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
                if (len > 0 && line[0] == '$' && minmea_check(line, false)) {
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
    ESP_LOGI(TAG, "reading UART1 (RX GPIO%d, TX GPIO%d) at %d baud", CONFIG_RGNSS_GNSS_RX_GPIO,
             CONFIG_RGNSS_GNSS_TX_GPIO, CONFIG_RGNSS_GNSS_BAUD);
    return ESP_OK;
}

void gnss_get(gnss_data_t *out)
{
    taskENTER_CRITICAL(&lock);
    *out = data;
    taskEXIT_CRITICAL(&lock);
}
