#pragma once

#include <stddef.h>

#include "esp_err.h"

// Firmware update over WiFi: begin, write the image in order, then finish,
// which points the bootloader at it. abort throws a partial image away. The
// caller restarts the device.
esp_err_t ota_begin(size_t total);
esp_err_t ota_write(const void *data, size_t len);
esp_err_t ota_finish(void);
void ota_abort(void);
