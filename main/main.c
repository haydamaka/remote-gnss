#include "esp_chip_info.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "nvs_flash.h"

#include "gnss.h"
#include "led.h"
#include "bearing.h"
#include "mag_cal.h"
#include "magnetometer.h"
#include "mdns_service.h"
#include "nmea_ble.h"
#include "nmea_tcp.h"
#include "webserver.h"
#include "wifi.h"

static const char *TAG = "remote-gnss";

static void nvs_init(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
}

void app_main(void)
{
    esp_chip_info_t chip;
    esp_chip_info(&chip);
    ESP_LOGI(TAG, "%s rev v%d.%d, %d core(s)", CONFIG_IDF_TARGET,
             chip.revision / 100, chip.revision % 100, chip.cores);

    nvs_init();
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // One radio, so one of the two, as the switch says.
    bool ble_mode = false;
#if CONFIG_RGNSS_BLE_ENABLED
    ble_mode = nmea_ble_boot_mode();
#endif
    if (!ble_mode) {
        ESP_ERROR_CHECK(wifi_start());
    } else {
        // Networks can still be added over BLE, for the next boot on WiFi.
        ESP_ERROR_CHECK_WITHOUT_ABORT(wifi_load());
    }

    // The web page is still useful with a sensor missing, so these aren't fatal.
    // They come first: the server must not serve a request before the modules it
    // asks for are ready, and probing a stuck I2C bus can take a while.
    ESP_ERROR_CHECK_WITHOUT_ABORT(gnss_start());
    ESP_ERROR_CHECK_WITHOUT_ABORT(magnetometer_start());
    ESP_ERROR_CHECK_WITHOUT_ABORT(mag_cal_init());
    ESP_ERROR_CHECK_WITHOUT_ABORT(bearing_start());

    // The web server comes before the optional services, because it is how the
    // device is controlled and updated. Started last, it was the one that
    // failed when Bluetooth had taken the last of the heap - leaving a device
    // that could only be recovered with a USB cable. Whatever runs short now,
    // the page still answers and can turn things off.
    //
    // Not fatal either: an abort here costs a USB cable, since the device is
    // updated over WiFi and a firmware that panics in app_main cannot be
    // replaced by the very server that would have replaced it.
    if (!ble_mode) {
        ESP_ERROR_CHECK_WITHOUT_ABORT(mdns_service_start());
        ESP_ERROR_CHECK_WITHOUT_ABORT(webserver_start());
    }

    // Wireless GNSS out to a PC or a phone, over whichever radio is running.
#if CONFIG_RGNSS_TCP_ENABLED
    if (!ble_mode) {
        ESP_ERROR_CHECK_WITHOUT_ABORT(nmea_tcp_start());
    }
#endif
#if CONFIG_RGNSS_BLE_ENABLED
    if (ble_mode) {
        // If this fails there is no web page to say so, so fall back rather
        // than sit there with neither radio: restart into one boot on WiFi,
        // whatever the switch, since a half-started BLE stack can't be trusted
        // to let go of the radio or its heap.
        if (ESP_ERROR_CHECK_WITHOUT_ABORT(nmea_ble_start()) != ESP_OK) {
            ESP_LOGE(TAG, "BLE would not start; restarting into WiFi");
            nmea_ble_restart_on_wifi();
        }
    }
#endif

    ESP_ERROR_CHECK_WITHOUT_ABORT(led_start(ble_mode));
}
