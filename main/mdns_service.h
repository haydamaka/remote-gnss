#pragma once

#include "esp_err.h"

// Answers mDNS queries for CONFIG_RGNSS_HOSTNAME.local on every WiFi interface,
// and advertises the web server as an _http._tcp service.
esp_err_t mdns_service_start(void);
