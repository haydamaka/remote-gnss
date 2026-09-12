#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

// Fans the receiver's NMEA sentences out to the outputs that carry them off
// the device: TCP clients, the BLE link. The GNSS reader task publishes; each
// output subscribes and reads at its own pace.
//
// NMEA-0183 allows 82 characters including the "$" and the line ending. u-blox
// exceeds that with some sentences, so the limit here is generous; anything
// longer is not a sentence and is dropped.
#define NMEA_HUB_MAX_SENTENCE 120

typedef struct nmea_sub *nmea_sub_t;

typedef struct {
    uint32_t sentences;  // delivered to this subscriber
    uint32_t bytes;      // sentence text, not counting the line endings
    uint32_t dropped;    // lost because the subscriber wasn't reading fast enough
} nmea_sub_stats_t;

esp_err_t nmea_hub_init(void);

// Hands one sentence, without its line ending, to every subscriber.
//
// Called from the GNSS reader task, which must keep draining the UART, so it
// never blocks: a subscriber whose buffer is full loses the sentence and
// counts it. Sentences arrive whole or not at all — a subscriber never sees
// half a line.
void nmea_hub_publish(const char *sentence, size_t len);

// Claims a subscriber slot, or returns NULL when they are all taken. `name`
// is for logging and must outlive the subscription.
nmea_sub_t nmea_hub_subscribe(const char *name);

// Releases the slot. The subscriber's own task must call this, and must not
// touch the handle afterwards.
void nmea_hub_unsubscribe(nmea_sub_t sub);

// Takes the next sentence into dst, without its line ending and
// NUL-terminated. Returns its length, or 0 if none arrived within
// timeout_ms (0 polls).
//
// dst_size must be at least NMEA_HUB_MAX_SENTENCE + 1: a message that doesn't
// fit is left in the buffer, which would wedge the subscriber for good.
size_t nmea_sub_receive(nmea_sub_t sub, char *dst, size_t dst_size, uint32_t timeout_ms);

void nmea_sub_stats(nmea_sub_t sub, nmea_sub_stats_t *out);
