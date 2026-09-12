#include "nmea_hub.h"

#include <string.h>

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/message_buffer.h"
#include "freertos/semphr.h"

// Close to a second of the filtered stream: enough to ride out a WiFi
// retransmit or a BLE connection event. Every slot holds one, and they are
// allocated together at start-up -- the BLE controller takes a large bite out
// of the heap when it starts, and a buffer that could not be allocated
// afterwards would silently leave an output with nothing to read.
#define SUB_BUF_SIZE 1536

static const char *TAG = "nmea_hub";

// A message buffer keeps the sentence boundaries and, unlike a stream buffer,
// never writes a message partially: a send that doesn't fit fails outright,
// which is exactly the "whole sentences or nothing" the subscribers need.
struct nmea_sub {
    MessageBufferHandle_t buf;  // created at init, then reused by every subscriber
    const char *name;
    bool active;
    uint32_t sentences;
    uint32_t bytes;
    uint32_t dropped;
};

static struct nmea_sub subs[CONFIG_RGNSS_NMEA_SUBSCRIBERS];
static SemaphoreHandle_t lock;

esp_err_t nmea_hub_init(void)
{
    if (lock != NULL) {
        return ESP_OK;  // whichever output started first did this
    }
    lock = xSemaphoreCreateMutex();
    if (lock == NULL) {
        return ESP_ERR_NO_MEM;
    }
    for (size_t i = 0; i < sizeof(subs) / sizeof(subs[0]); i++) {
        subs[i].buf = xMessageBufferCreate(SUB_BUF_SIZE);
        if (subs[i].buf == NULL) {
            ESP_LOGE(TAG, "only %u of %d buffers fit; %u bytes of heap left", (unsigned)i,
                     CONFIG_RGNSS_NMEA_SUBSCRIBERS, (unsigned)esp_get_free_heap_size());
            return ESP_ERR_NO_MEM;
        }
    }
    ESP_LOGI(TAG, "%d subscriber buffers of %d bytes; %u bytes of heap left", CONFIG_RGNSS_NMEA_SUBSCRIBERS,
             SUB_BUF_SIZE, (unsigned)esp_get_free_heap_size());
    return ESP_OK;
}

// True when the sentence's three-character type is one the outputs asked for.
// The talker (GP, GN, GA, GB, ...) is deliberately not matched: a client wants
// GGA whichever constellation produced it.
static bool wanted(const char *sentence, size_t len)
{
    static const char types[] = CONFIG_RGNSS_NMEA_TYPES;
    if (types[0] == '\0') {
        return true;
    }
    if (len < 6) {
        return false;
    }
    for (const char *p = types; *p != '\0';) {
        if (strncmp(p, sentence + 3, 3) == 0 && (p[3] == ',' || p[3] == '\0')) {
            return true;
        }
        const char *comma = strchr(p, ',');
        if (comma == NULL) {
            break;
        }
        p = comma + 1;
    }
    return false;
}

void nmea_hub_publish(const char *sentence, size_t len)
{
    if (len == 0 || len > NMEA_HUB_MAX_SENTENCE || lock == NULL) {
        return;
    }
    if (!wanted(sentence, len)) {
        return;
    }
    // Zero timeout: the GNSS task must not wait on a subscriber coming or
    // going. Those are rare and brief, and losing a sentence to one is
    // cheaper than stalling the UART reader.
    if (xSemaphoreTake(lock, 0) != pdTRUE) {
        return;
    }
    for (size_t i = 0; i < sizeof(subs) / sizeof(subs[0]); i++) {
        struct nmea_sub *s = &subs[i];
        if (!s->active) {
            continue;
        }
        if (xMessageBufferSend(s->buf, sentence, len, 0) == len) {
            s->sentences++;
            s->bytes += len;
        } else {
            s->dropped++;
        }
    }
    xSemaphoreGive(lock);
}

nmea_sub_t nmea_hub_subscribe(const char *name)
{
    if (lock == NULL) {
        return NULL;
    }
    struct nmea_sub *found = NULL;
    xSemaphoreTake(lock, portMAX_DELAY);
    for (size_t i = 0; i < sizeof(subs) / sizeof(subs[0]); i++) {
        struct nmea_sub *s = &subs[i];
        if (s->active) {
            continue;
        }
        // Reset here rather than on unsubscribe: at this point nothing is
        // publishing into the buffer, so there is no race with a send.
        xMessageBufferReset(s->buf);
        s->name = name;
        s->sentences = 0;
        s->bytes = 0;
        s->dropped = 0;
        s->active = true;
        found = s;
        break;
    }
    xSemaphoreGive(lock);

    if (found == NULL) {
        ESP_LOGW(TAG, "no free slot for %s (%d subscribers)", name, CONFIG_RGNSS_NMEA_SUBSCRIBERS);
    }
    return found;
}

void nmea_hub_unsubscribe(nmea_sub_t sub)
{
    if (sub == NULL) {
        return;
    }
    xSemaphoreTake(lock, portMAX_DELAY);
    sub->active = false;
    xSemaphoreGive(lock);
}

size_t nmea_sub_receive(nmea_sub_t sub, char *dst, size_t dst_size, uint32_t timeout_ms)
{
    if (sub == NULL || dst_size == 0) {
        return 0;
    }
    size_t n = xMessageBufferReceive(sub->buf, dst, dst_size - 1, pdMS_TO_TICKS(timeout_ms));
    dst[n] = '\0';
    return n;
}

void nmea_sub_stats(nmea_sub_t sub, nmea_sub_stats_t *out)
{
    if (sub == NULL) {
        memset(out, 0, sizeof(*out));
        return;
    }
    xSemaphoreTake(lock, portMAX_DELAY);
    out->sentences = sub->sentences;
    out->bytes = sub->bytes;
    out->dropped = sub->dropped;
    xSemaphoreGive(lock);
}
