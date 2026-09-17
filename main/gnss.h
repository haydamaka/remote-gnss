#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

// What the receiver's own spoofing detector reports (UBX-NAV-STATUS flags2).
typedef enum {
    GNSS_SPOOF_UNKNOWN,    // no answer yet, or the detector is off
    GNSS_SPOOF_NONE,       // it sees nothing wrong
    GNSS_SPOOF_INDICATED,
    GNSS_SPOOF_MULTIPLE,
} gnss_spoof_t;

// Latest state from the GNSS receiver. Unknown numbers are NaN.
typedef struct {
    uint32_t sentences;   // valid NMEA sentences received since boot
    int baud;             // the rate it was actually found at
    int64_t updated_us;   // esp_timer time of the last valid sentence, 0 if none yet
    int64_t epoch_us;     // ...of the last GGA, i.e. one per navigation solution
    bool fix;
    int fix_quality;      // GGA: 0 none, 1 GPS, 2 DGPS, 4 RTK fixed, 5 RTK float, 6 dead reckoning
    int satellites;       // used in the fix
    double latitude;      // degrees, north positive
    double longitude;     // degrees, east positive
    float altitude_m;     // above mean sea level
    float hdop;
    float speed_kmh;
    float course_deg;     // over ground, true north
    char utc[32];         // "YYYY-MM-DD HH:MM:SS", empty until the receiver knows the time
    uint8_t spoof;        // gnss_spoof_t: the receiver's own verdict, UNKNOWN once stale
    int64_t spoof_us;     // esp_timer time of the last NAV-STATUS reply, 0 if none
    bool implausible;     // our own checks say the fix cannot be true
    char implausible_why[64];
    bool withholding;     // TCP and BLE are being sent "no fix" instead of this
} gnss_data_t;

// Starts reading NMEA from the receiver on UART1, per menuconfig "Remote GNSS → GNSS".
esp_err_t gnss_start(void);

void gnss_get(gnss_data_t *out);

// Whether a fix that looks spoofed is kept from the NMEA outputs (TCP, BLE).
// While it is, and for a minute after the last sign of spoofing, they get
// "no fix" sentences instead of the position and time. On unless turned off;
// kept in NVS.
bool gnss_withhold_spoofed(void);
esp_err_t gnss_set_withhold_spoofed(bool on);
