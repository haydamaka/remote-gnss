// Walksnail WS-M181: a u-blox M10 (UBX-M10050) with a QMC5883 or IST8310
// compass on the same board, which is why this firmware has a magnetometer at
// all.
//
// Nothing is sent to it. Walksnail ships it streaming NMEA at 115200 baud and
// 10 Hz across GPS, GLONASS and BeiDou, which is what this firmware wants, and
// those settings live in the module's own flash rather than in RAM - so a
// firmware that "corrected" them would be altering the module permanently, for
// no gain. The M10 speaks the same configuration interface as the F9P
// (gnss_ubx_valset) if a reason to use it ever turns up.

#include "gnss_receiver.h"

const gnss_receiver_t gnss_receiver = {
    .name = "Walksnail WS-M181 (u-blox M10)",
    .default_baud = 115200,
};
