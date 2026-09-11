#include "mdns_service.h"

#include "esp_check.h"
#include "esp_log.h"
#include "mdns.h"

static const char *TAG = "mdns";

esp_err_t mdns_service_start(void)
{
    ESP_RETURN_ON_ERROR(mdns_init(), TAG, "init");
    ESP_RETURN_ON_ERROR(mdns_hostname_set(CONFIG_RGNSS_HOSTNAME), TAG, "set hostname");
    ESP_RETURN_ON_ERROR(mdns_instance_name_set("remote-gnss"), TAG, "set instance name");
    ESP_RETURN_ON_ERROR(mdns_service_add(NULL, "_http", "_tcp", CONFIG_RGNSS_HTTP_PORT, NULL, 0),
                        TAG, "add HTTP service");
    ESP_LOGI(TAG, "hostname %s.local", CONFIG_RGNSS_HOSTNAME);
    return ESP_OK;
}
