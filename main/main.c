#include "esp_chip_info.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "nvs_flash.h"

#include "gnss.h"
#include "magnetometer.h"
#include "mdns_service.h"
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
    ESP_ERROR_CHECK(mdns_service_start());
    ESP_ERROR_CHECK(webserver_start());

    // The web page is still useful with a sensor missing, so these aren't fatal.
    ESP_ERROR_CHECK_WITHOUT_ABORT(gnss_start());
    ESP_ERROR_CHECK_WITHOUT_ABORT(magnetometer_start());
}
