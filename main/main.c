#include "esp_chip_info.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "nvs_flash.h"

#include "gnss.h"
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

    ESP_ERROR_CHECK(wifi_start());

    // The web page is still useful with a sensor missing, so these aren't fatal.
    // They come first: the server must not serve a request before the modules it
    // asks for are ready, and probing a stuck I2C bus can take a while.
    ESP_ERROR_CHECK_WITHOUT_ABORT(gnss_start());
    ESP_ERROR_CHECK_WITHOUT_ABORT(magnetometer_start());
    ESP_ERROR_CHECK_WITHOUT_ABORT(mag_cal_init());

    // Wireless GNSS out to a PC or a phone. Neither is fatal: the web page and
    // the receiver work without them.
#if CONFIG_RGNSS_TCP_ENABLED
    ESP_ERROR_CHECK_WITHOUT_ABORT(nmea_tcp_start());
#endif
#if CONFIG_RGNSS_BLE_ENABLED
    ESP_ERROR_CHECK_WITHOUT_ABORT(nmea_ble_start());
#endif

    ESP_ERROR_CHECK(mdns_service_start());
    ESP_ERROR_CHECK(webserver_start());
}
