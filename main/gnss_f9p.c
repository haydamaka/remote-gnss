// u-blox ZED-F9P: the RTK receiver, a bare module on a breakout board with no
// compass of its own.
//
// Unlike the WS-M181 it arrives set up for somebody else's application: 38400
// baud, 1 Hz, and NMEA rounded to five decimals of a minute - about 2 cm, which
// throws away most of what an RTK fix is worth. So this module configures it,
// every time it is found:
//
//   - NMEA output on the port, which a receiver set up for UBX logging may
//     have turned off;
//   - the link rate, if it is slower than ours, because 38400 baud carries
//     about 3.8 kB/s and a multi-constellation stream above 3 Hz is more;
//   - the navigation rate;
//   - high-precision NMEA, which adds the decimal digits the centimetres need;
//   - the sentence set: what gnss.c and the clients actually read, with GSV
//     slowed to once a second because it is three quarters of the bytes.
//
// All of it is addressed to the RAM layer only (see gnss_ubx_valset), so the
// receiver is never permanently altered: pull its power and it is back to
// whatever its owner had. Re-sending everything after each re-lock is what
// makes that cost nothing.
//
// Nothing here asks for RTK corrections - without RTCM3 on the receiver's RX
// line this is a 1 m receiver with a very good antenna. The corrections would
// come from an NTRIP client, which is not written yet; gnss_write() is the hole
// it will pour them into, and the F9P takes RTCM3 on UART1 by default.

#include "gnss_receiver.h"

#include "esp_log.h"

static const char *TAG = "gnss-f9p";

// Configuration keys, from the ZED-F9P interface description. Bits 28-30 of
// each carry the value's width, which gnss_ubx_valset reads back out.
#define CFG_RATE_MEAS              0x30210001  // U2, milliseconds between measurements
#define CFG_RATE_NAV               0x30210002  // U2, measurements per navigation solution
#define CFG_NMEA_HIGHPREC          0x10930006  // L
#define CFG_UART1_BAUDRATE         0x40520001  // U4
#define CFG_UART1OUTPROT_NMEA      0x10740002  // L
#define CFG_MSGOUT_NMEA_GGA_UART1  0x209100BB  // U1, solutions per sentence; 0 is off
#define CFG_MSGOUT_NMEA_GLL_UART1  0x209100CA
#define CFG_MSGOUT_NMEA_GSA_UART1  0x209100C0
#define CFG_MSGOUT_NMEA_GST_UART1  0x209100D4
#define CFG_MSGOUT_NMEA_GSV_UART1  0x209100C5
#define CFG_MSGOUT_NMEA_RMC_UART1  0x209100AC
#define CFG_MSGOUT_NMEA_VTG_UART1  0x209100B1
#define CFG_MSGOUT_NMEA_ZDA_UART1  0x209100D9

#define RATE_HZ CONFIG_RGNSS_GNSS_RATE_HZ

static int configure(int baud)
{
    // GST carries the position's own error estimate, which is the one number
    // that says whether the RTK fix is real; it is off by default. ZDA is on
    // for clients that want the date without parsing RMC.
    const gnss_cfg_t items[] = {
        // A receiver set up for somebody's logger may have NMEA off on the
        // port altogether; UBX is left as its owner had it.
        {CFG_UART1OUTPROT_NMEA, 1},
        {CFG_RATE_MEAS, 1000 / RATE_HZ},
        {CFG_RATE_NAV, 1},
        {CFG_NMEA_HIGHPREC, 1},
        {CFG_MSGOUT_NMEA_GGA_UART1, 1},
        {CFG_MSGOUT_NMEA_RMC_UART1, 1},
        {CFG_MSGOUT_NMEA_GSA_UART1, 1},
        {CFG_MSGOUT_NMEA_GST_UART1, 1},
        {CFG_MSGOUT_NMEA_VTG_UART1, 1},
        {CFG_MSGOUT_NMEA_GLL_UART1, 1},
        {CFG_MSGOUT_NMEA_ZDA_UART1, 1},
        {CFG_MSGOUT_NMEA_GSV_UART1, RATE_HZ},  // i.e. once a second
    };
    gnss_ubx_valset(items, sizeof(items) / sizeof(items[0]));
    ESP_LOGI(TAG, "configured for %d Hz with high-precision NMEA", RATE_HZ);

    // A faster link than ours was chosen by the receiver's owner, likely for
    // the UBX they have it sending; slowing it down could choke that.
    if (baud >= CONFIG_RGNSS_GNSS_BAUD) {
        return 0;
    }
    // Last, and in a frame of its own: the receiver switches its port the
    // moment this one is applied, so anything sharing the frame would be
    // acknowledged - or not - at a rate nobody is listening at yet.
    const gnss_cfg_t rate[] = {{CFG_UART1_BAUDRATE, CONFIG_RGNSS_GNSS_BAUD}};
    gnss_ubx_valset(rate, 1);
    return CONFIG_RGNSS_GNSS_BAUD;
}

// UBX-ACK-NAK (class 0x05, id 0x00) names the message it is refusing. A silent
// VALSET is a working one, so only the refusals are worth a line: this is how a
// key that is wrong for the firmware version in the module shows up, rather
// than as a receiver that mysteriously ignores its rate.
static void on_ubx(uint8_t cls, uint8_t id, const uint8_t *payload, uint16_t len)
{
    if (cls == 0x05 && id == 0x00 && len >= 2) {
        ESP_LOGW(TAG, "the receiver rejected configuration message 0x%02x 0x%02x", payload[0], payload[1]);
    }
}

const gnss_receiver_t gnss_receiver = {
    .name = "u-blox ZED-F9P",
    .default_baud = 38400,
    .configure = configure,
    .ubx = on_ubx,
};
