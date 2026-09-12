#include "ota.h"

#include "esp_check.h"
#include "esp_log.h"
#include "esp_ota_ops.h"

static const char *TAG = "ota";

// Only the HTTP server task touches these, one update at a time.
static esp_ota_handle_t handle;
static const esp_partition_t *target;

esp_err_t ota_begin(size_t total)
{
    ota_abort();  // a previous attempt may have been cut off
    target = esp_ota_get_next_update_partition(NULL);
    ESP_RETURN_ON_FALSE(target, ESP_ERR_NOT_FOUND, TAG, "no spare app partition");
    ESP_RETURN_ON_FALSE(total > 0 && total <= target->size, ESP_ERR_INVALID_SIZE, TAG,
                        "image is %u bytes, %s holds %u", (unsigned)total, target->label, (unsigned)target->size);
    esp_err_t err = esp_ota_begin(target, total, &handle);
    if (err != ESP_OK) {
        target = NULL;
        ESP_RETURN_ON_ERROR(err, TAG, "begin");
    }
    ESP_LOGI(TAG, "receiving %u bytes into %s", (unsigned)total, target->label);
    return ESP_OK;
}

esp_err_t ota_write(const void *data, size_t len)
{
    ESP_RETURN_ON_FALSE(target, ESP_ERR_INVALID_STATE, TAG, "no update in progress");
    return esp_ota_write(handle, data, len);
}

// esp_ota_end checks the image; only then does the bootloader get pointed at it.
esp_err_t ota_finish(void)
{
    ESP_RETURN_ON_FALSE(target, ESP_ERR_INVALID_STATE, TAG, "no update in progress");
    esp_err_t err = esp_ota_end(handle);
    if (err == ESP_OK) {
        err = esp_ota_set_boot_partition(target);
    }
    ESP_LOGI(TAG, "update %s: %s", err == ESP_OK ? "accepted" : "rejected", esp_err_to_name(err));
    target = NULL;
    return err;
}

void ota_abort(void)
{
    if (target) {
        esp_ota_abort(handle);
        target = NULL;
    }
}
