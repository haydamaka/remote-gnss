#include "led.h"

#include "driver/gpio.h"
#include "esp_check.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "gnss.h"
#include "magnetometer.h"
#include "nmea_ble.h"
#include "wifi.h"

#define TICK_MS 50
#define CYCLE_TICKS 40  // 2 s
// A receiver silent this long counts as an error. It sends 10 sentences a
// second, so this is far past any gap in a healthy stream.
#define GNSS_SILENT_US (3 * 1000000)

#if CONFIG_RGNSS_LED_ACTIVE_LOW
#define LED_ON 0
#else
#define LED_ON 1
#endif

static const char *TAG = "led";

static bool ble;

static bool hardware_error(const gnss_data_t *g)
{
    magnetometer_data_t m;
    magnetometer_get(&m);
    return m.chip == NULL || m.failing || g->updated_us == 0 ||
           esp_timer_get_time() - g->updated_us > GNSS_SILENT_US;
}

static bool spoofed(const gnss_data_t *g)
{
    return g->spoof == GNSS_SPOOF_INDICATED || g->spoof == GNSS_SPOOF_MULTIPLE || g->implausible;
}

#define SHORT_TICKS 2  // 100 ms: no link
#define LONG_TICKS 8   // 400 ms: link up
#define GAP_TICKS 4    // 200 ms between the two BLE flashes

// Something can reach the device: on WiFi, the station has joined a network or
// a phone has joined the access point; on BLE, a client is connected.
static bool link_up(void)
{
    if (ble) {
#if CONFIG_RGNSS_BLE_ENABLED
        nmea_ble_status_t b;
        nmea_ble_status(&b);
        return b.connected;
#else
        return false;
#endif
    }
    return wifi_link_up();
}

// One flash for WiFi, two for BLE; each 100 ms without a link, 400 ms with one.
static bool flashing(unsigned tick, bool link)
{
    unsigned t = tick % CYCLE_TICKS, on = link ? LONG_TICKS : SHORT_TICKS;
    return t < on || (ble && t >= on + GAP_TICKS && t < 2 * on + GAP_TICKS);
}

// A hardware error comes first: it outlasts spoofing, and a fix that looks
// spoofed means little from a receiver that has gone quiet.
static void led_task(void *arg)
{
    TickType_t wake = xTaskGetTickCount();
    bool link = false;
    for (unsigned tick = 0;; tick++) {
        // Looked at once a cycle, at its start, so no flash is cut short or doubled.
        if (tick % CYCLE_TICKS == 0) {
            link = link_up();
        }
        gnss_data_t g;
        gnss_get(&g);
        bool lit;
        if (hardware_error(&g)) {
            lit = tick % 2 == 0;  // 10 Hz
        } else if (spoofed(&g)) {
            lit = tick / 2 % 2 == 0;  // 5 Hz
        } else {
            lit = g.fix != flashing(tick, link);
        }
        gpio_set_level(CONFIG_RGNSS_LED_GPIO, lit ? LED_ON : !LED_ON);
        xTaskDelayUntil(&wake, pdMS_TO_TICKS(TICK_MS));
    }
}

esp_err_t led_start(bool ble_mode)
{
    ble = ble_mode;
    gpio_config_t io = {
        .pin_bit_mask = 1ULL << CONFIG_RGNSS_LED_GPIO,
        .mode = GPIO_MODE_OUTPUT,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&io), TAG, "configure GPIO%d", CONFIG_RGNSS_LED_GPIO);
    gpio_set_level(CONFIG_RGNSS_LED_GPIO, !LED_ON);
    // Low priority: the LED can wait for anything else.
    ESP_RETURN_ON_FALSE(xTaskCreate(led_task, "led", 2048, NULL, 1, NULL) == pdPASS, ESP_ERR_NO_MEM, TAG,
                        "create task");
    return ESP_OK;
}
