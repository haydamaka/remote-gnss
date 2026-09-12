#pragma once

#include <stddef.h>
#include <stdint.h>

// What one receiver model needs done differently. gnss.c owns the UART, the
// line splitting and the NMEA parsing, which are the same whatever is on the
// other end of the wire; this is the rest of it. Exactly one implementation is
// compiled in, chosen by menuconfig "Remote GNSS -> GNSS -> Receiver".
typedef struct {
    // Shown on the web page and in the log.
    const char *name;

    // The rate it talks at out of the box. Tried first when hunting for the
    // receiver, because a receiver that has just been powered on is there
    // whatever the link was set to last time.
    int default_baud;

    // Called from the reader task once a sentence has checksummed, so the
    // receiver is known to be present and listening. Sends whatever
    // configuration it needs, and returns the rate the link should run at
    // afterwards, or 0 to leave it where it is. Called again after every
    // re-lock: a receiver that lost power lost its settings with it. May be
    // NULL, for one that arrives set up the way this firmware wants it.
    int (*configure)(void);

    // Called for every UBX frame that passes its checksum, apart from the
    // NAV-STATUS reply gnss.c reads itself. The payload is truncated to the
    // first GNSS_UBX_PAYLOAD bytes; len says how many of them there are. May
    // be NULL.
    void (*ubx)(uint8_t cls, uint8_t id, const uint8_t *payload, uint16_t len);
} gnss_receiver_t;

#define GNSS_UBX_PAYLOAD 24

// The one that was compiled in.
extern const gnss_receiver_t gnss_receiver;

// Puts bytes on the receiver's RX line. For configure() and ubx().
void gnss_write(const void *buf, size_t len);

// Wraps a payload in a UBX frame, checksum and all, and sends it.
void gnss_ubx_send(uint8_t cls, uint8_t id, const void *payload, uint16_t len);

// One setting of the u-blox configuration interface. The key carries the
// value's width in bits 28-30, so the value is always given as a 32-bit number
// and truncated to fit.
typedef struct {
    uint32_t key;
    uint32_t value;
} gnss_cfg_t;

#define GNSS_CFG_MAX 16

// Sends the settings as one UBX-CFG-VALSET addressed to the RAM layer, which
// is what makes this safe to do to somebody else's receiver: nothing is
// written to its flash, so a power cycle puts back whatever it had, and
// configure() simply runs again. At most GNSS_CFG_MAX settings per call.
void gnss_ubx_valset(const gnss_cfg_t *items, size_t n);
