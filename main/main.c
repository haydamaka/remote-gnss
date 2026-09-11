#include "esp_chip_info.h"
#include "esp_log.h"

static const char *TAG = "remote-gnss";

void app_main(void)
{
    esp_chip_info_t chip;
    esp_chip_info(&chip);
    ESP_LOGI(TAG, "%s rev v%d.%d, %d core(s)", CONFIG_IDF_TARGET,
             chip.revision / 100, chip.revision % 100, chip.cores);
}
