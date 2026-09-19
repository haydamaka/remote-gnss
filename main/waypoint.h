#pragma once

#include <stdbool.h>

#include "esp_err.h"
#include "gnss.h"

// The one destination the device navigates to. A client sets it with the
// standard waypoint sentence, $--WPL, on the BLE command channel; it is kept in
// NVS, so it outlasts a restart and serves either radio. While there is one and
// a fix, the NMEA outputs get $GPRMB and $GPBWC toward it once a second:
// bearing, distance, closing speed and arrival, for any client that navigates.
//
// The name is WPL's waypoint identifier. Eight characters keeps both
// sentences inside NMEA's 82, and is what receivers traditionally allowed.
#define WAYPOINT_ID_MAX 8

// Within this many metres counts as arrived (RMB's arrival status).
#define WAYPOINT_ARRIVAL_M 20.0

typedef struct {
    double latitude;   // degrees, north positive
    double longitude;  // degrees, east positive
    char id[WAYPOINT_ID_MAX + 1];
} waypoint_t;

// Reads the saved waypoint, if any. Before the GNSS task starts publishing.
void waypoint_load(void);

// False when there is no destination.
bool waypoint_get(waypoint_t *out);

// ESP_ERR_INVALID_ARG for a position off the globe or an identifier NMEA can't
// carry (a comma, an asterisk, a '$', anything unprintable, or too long).
esp_err_t waypoint_set(const waypoint_t *wp);
esp_err_t waypoint_clear(void);

// Parses the fields of a WPL sentence, the part after "$--WPL,", without the
// checksum: "ddmm.mmmm,N,dddmm.mmmm,E,ID". Does not validate beyond the format;
// waypoint_set does that.
bool waypoint_parse_wpl(const char *fields, waypoint_t *out);

// The way from a fix to the waypoint, as RMB, BWC and the page give it.
typedef struct {
    double distance_m;    // great circle
    double bearing_true;  // initial bearing, degrees 0-360
    double bearing_mag;   // the same less the declination here; NAN before a fix has given one
    double closing_kn;    // speed toward it, negative moving away; NAN without a course
    bool arrived;         // within WAYPOINT_ARRIVAL_M
} waypoint_nav_t;

// False without a waypoint or a fix.
bool waypoint_nav(const gnss_data_t *fix, waypoint_t *wp, waypoint_nav_t *out);

// Publishes $GPRMB and $GPBWC toward the waypoint from one navigation
// solution. Nothing without a waypoint or a fix. GNSS reader task only.
void waypoint_publish(const gnss_data_t *fix);
