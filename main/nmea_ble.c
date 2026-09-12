#include "nmea_ble.h"

#include <string.h>

#include "esp_check.h"
#include "esp_log.h"
#include "esp_attr.h"
#include "esp_heap_caps.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "nvs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "nmea_hub.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

// Sentences waiting to go out. A notification carries at most ATT MTU - 3
// bytes, so this is drained over several of them; NUS is a byte stream and the
// client reassembles the lines.
#define TX_SIZE 512

// 60-120 ms, in units of 1.25 ms. Faster than this starves the WiFi station:
// the chip has one radio, and a connection event every 15 ms left too little
// airtime to hold a link to a distant access point. The controller sends
// several notifications per event, so this still clears the filtered stream's
// 1.9 KB/s with room to spare.
#define CONN_ITVL_MIN 48
#define CONN_ITVL_MAX 96

// 500 ms to 1 s, in units of 0.625 ms. NimBLE's default advertises every
// 30-60 ms, which on a one-radio chip is a constant tax on WiFi even when
// nobody is listening -- enough on its own to stop the station associating.
// Nothing here needs to be found in a hurry.
#define ADV_ITVL_MIN 800
#define ADV_ITVL_MAX 1600

#define NVS_NAMESPACE "ble"
#define NVS_KEY_PREFERRED "prefer"

// Survives a restart but not a power cycle: set when BLE hands the radio over,
// so the restart that follows comes up on WiFi even though the saved
// preference still says BLE. Power it off and on and it starts on BLE again.
#define WIFI_THIS_BOOT_MAGIC 0x57494649u
static RTC_DATA_ATTR uint32_t wifi_this_boot;

static const char *TAG = "nmea_ble";

// NimBLE takes 128-bit UUIDs least significant byte first.
static const ble_uuid128_t svc_uuid =
    BLE_UUID128_INIT(0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0, 0x93, 0xf3, 0xa3, 0xb5, 0x01, 0x00, 0x40, 0x6e);
static const ble_uuid128_t chr_rx_uuid =
    BLE_UUID128_INIT(0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0, 0x93, 0xf3, 0xa3, 0xb5, 0x02, 0x00, 0x40, 0x6e);
static const ble_uuid128_t chr_tx_uuid =
    BLE_UUID128_INIT(0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0, 0x93, 0xf3, 0xa3, 0xb5, 0x03, 0x00, 0x40, 0x6e);

static uint8_t own_addr_type;
static uint16_t tx_handle;
static volatile uint16_t conn_handle = BLE_HS_CONN_HANDLE_NONE;
static volatile bool subscribed;
static char peer[18];
static nmea_sub_t sub;
static uint32_t dropped;
static bool advertising;
static bool synced;         // the host has finished starting; GAP calls work
static bool enabled;        // BLE has the radio now
static bool preferred;      // and it is what the device starts with (NVS)
static int64_t idle_since;  // when the last client left, or BLE was switched on

static int gap_event(struct ble_gap_event *event, void *arg);

static int rx_access(uint16_t conn, uint16_t attr, struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    // Nothing reads or writes anything meaningful yet. The RX characteristic
    // exists because clients expect the pair, and it is where UBX commands
    // from the client would arrive in a later phase.
    return ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR ? 0 : BLE_ATT_ERR_UNLIKELY;
}

static int tx_access(uint16_t conn, uint16_t attr, struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    return BLE_ATT_ERR_UNLIKELY;  // notify only; there is nothing to read back
}

static const struct ble_gatt_svc_def gatt_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &svc_uuid.u,
        .characteristics =
            (struct ble_gatt_chr_def[]){
                {
                    .uuid = &chr_rx_uuid.u,
                    .access_cb = rx_access,
                    .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP,
                },
                {
                    .uuid = &chr_tx_uuid.u,
                    .access_cb = tx_access,
                    .flags = BLE_GATT_CHR_F_NOTIFY,
                    .val_handle = &tx_handle,
                },
                {0},
            },
    },
    {0},
};

static void advertise(void)
{
    const char *name = ble_svc_gap_device_name();
    struct ble_hs_adv_fields fields = {
        .flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP,
        .name = (uint8_t *)name,
        .name_len = strlen(name),
        .name_is_complete = 1,
        .tx_pwr_lvl = BLE_HS_ADV_TX_PWR_LVL_AUTO,
        .tx_pwr_lvl_is_present = 1,
    };
    int rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "adv_set_fields: %d", rc);
        return;
    }

    // The 128-bit service UUID is 16 bytes and doesn't fit alongside the name
    // in the 31-byte advertisement, so it goes in the scan response. Clients
    // that filter on the NUS UUID still find us.
    struct ble_hs_adv_fields rsp = {
        .uuids128 = (ble_uuid128_t *)&svc_uuid,
        .num_uuids128 = 1,
        .uuids128_is_complete = 1,
    };
    rc = ble_gap_adv_rsp_set_fields(&rsp);
    if (rc != 0) {
        ESP_LOGE(TAG, "adv_rsp_set_fields: %d", rc);
        return;
    }

    struct ble_gap_adv_params params = {
        .conn_mode = BLE_GAP_CONN_MODE_UND,
        .disc_mode = BLE_GAP_DISC_MODE_GEN,
        .itvl_min = ADV_ITVL_MIN,
        .itvl_max = ADV_ITVL_MAX,
    };
    rc = ble_gap_adv_start(own_addr_type, NULL, BLE_HS_FOREVER, &params, gap_event, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "adv_start: %d", rc);
        return;
    }
    advertising = true;
    ESP_LOGI(TAG, "advertising as %s", name);
}

// Advertising is cheap but not free, and on a one-radio chip "not free" is the
// difference between a station that completes a WPA2 handshake and one that
// does not. It runs only while BLE has the radio.
static void sync_advertising(void)
{
    // GAP calls before the host has synced with the controller fail with
    // BLE_HS_EDISABLED, which is what the start-up race used to look like.
    if (!synced || conn_handle != BLE_HS_CONN_HANDLE_NONE) {
        return;
    }
    if (enabled && !advertising) {
        advertise();
    } else if (!enabled && advertising) {
        if (ble_gap_adv_stop() == 0) {
            advertising = false;
        }
    }
}

static int gap_event(struct ble_gap_event *event, void *arg)
{
    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status != 0) {
            advertising = false;
            break;  // sync_advertising() puts it back on the air
        }
        conn_handle = event->connect.conn_handle;
        advertising = false;
        struct ble_gap_conn_desc desc;
        if (ble_gap_conn_find(conn_handle, &desc) == 0) {
            const uint8_t *a = desc.peer_id_addr.val;  // stored little endian
            snprintf(peer, sizeof(peer), "%02x:%02x:%02x:%02x:%02x:%02x", a[5], a[4], a[3], a[2], a[1], a[0]);
        }
        ESP_LOGI(TAG, "%s connected", peer);
        // Ask for a faster interval. The client may refuse, which is why the
        // sentence rate still has to be sane.
        struct ble_gap_upd_params upd = {
            .itvl_min = CONN_ITVL_MIN,
            .itvl_max = CONN_ITVL_MAX,
            .latency = 0,
            .supervision_timeout = 400,  // 4 s
        };
        ble_gap_update_params(conn_handle, &upd);
        break;

    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "%s disconnected (reason %d)", peer, event->disconnect.reason);
        conn_handle = BLE_HS_CONN_HANDLE_NONE;
        subscribed = false;
        peer[0] = '\0';
        idle_since = esp_timer_get_time();
        sync_advertising();
        // Not advertise(): the station has very likely been knocked off the
        // air by this connection, and it gets the radio back first.
        break;

    case BLE_GAP_EVENT_SUBSCRIBE:
        if (event->subscribe.attr_handle == tx_handle) {
            subscribed = event->subscribe.cur_notify;
            ESP_LOGI(TAG, "notifications %s", subscribed ? "on" : "off");
        }
        break;

    case BLE_GAP_EVENT_MTU:
        ESP_LOGI(TAG, "MTU %d", event->mtu.value);
        break;

    case BLE_GAP_EVENT_ADV_COMPLETE:
        advertising = false;
        break;

    default:
        break;
    }
    return 0;
}

static void on_sync(void)
{
    int rc = ble_hs_util_ensure_addr(0);
    if (rc != 0) {
        ESP_LOGE(TAG, "ensure_addr: %d", rc);
        return;
    }
    rc = ble_hs_id_infer_auto(0, &own_addr_type);
    if (rc != 0) {
        ESP_LOGE(TAG, "infer_auto: %d", rc);
        return;
    }
    synced = true;
    ESP_LOGI(TAG, "host synced");
    sync_advertising();
}

static void on_reset(int reason)
{
    ESP_LOGW(TAG, "host reset: %d", reason);
}

static void host_task(void *arg)
{
    nimble_port_run();  // returns only when the stack is stopped
    nimble_port_freertos_deinit();
}

// Fills buf with whole sentences, each followed by CRLF, and returns how many
// bytes it put there.
static size_t fill(char *buf, size_t size)
{
    char sentence[NMEA_HUB_MAX_SENTENCE + 1];
    size_t used = 0;

    // Wait for the first one; take any others that are already queued.
    size_t n = nmea_sub_receive(sub, sentence, sizeof(sentence), 500);
    while (n > 0) {
        memcpy(buf + used, sentence, n);
        used += n;
        buf[used++] = '\r';
        buf[used++] = '\n';
        if (used + NMEA_HUB_MAX_SENTENCE + 2 > size) {
            break;
        }
        n = nmea_sub_receive(sub, sentence, sizeof(sentence), 0);
    }
    return used;
}

static void pump_task(void *arg)
{
    char tx[TX_SIZE];
    size_t len = 0;
    size_t off = 0;

    for (;;) {
        uint16_t conn = conn_handle;
        if (conn == BLE_HS_CONN_HANDLE_NONE || !subscribed) {
            if (sub != NULL) {
                nmea_hub_unsubscribe(sub);
                sub = NULL;
            }
            len = off = 0;
            // Hand the radio back if nobody has used BLE for a while. Without
            // this, switching to BLE and walking away would leave the device
            // off the network until someone power-cycled it.
            sync_advertising();
            if (enabled && conn_handle == BLE_HS_CONN_HANDLE_NONE &&
                esp_timer_get_time() - idle_since > (int64_t)CONFIG_RGNSS_BLE_IDLE_S * 1000000) {
                ESP_LOGI(TAG, "no client for %d s; restarting so WiFi gets the radio",
                         CONFIG_RGNSS_BLE_IDLE_S);
                wifi_this_boot = WIFI_THIS_BOOT_MAGIC;  // the saved preference stands
                esp_restart();
            }
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }
        if (sub == NULL) {
            sub = nmea_hub_subscribe("ble");
            if (sub == NULL) {
                vTaskDelay(pdMS_TO_TICKS(1000));
                continue;
            }
        }

        if (off == len) {
            len = fill(tx, sizeof(tx));
            off = 0;
            if (len == 0) {
                continue;
            }
        }

        // A notification carries ATT MTU - 3 bytes at most. Whatever is left
        // over goes in the next one.
        size_t chunk = ble_att_mtu(conn) - 3;
        if (chunk > len - off) {
            chunk = len - off;
        }

        struct os_mbuf *om = ble_hs_mbuf_from_flat(tx + off, chunk);
        if (om == NULL) {
            dropped++;  // out of mbufs; drop the batch rather than stall
            len = off = 0;
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }
        // The stack takes the mbuf either way, so it is never leaked here.
        int rc = ble_gatts_notify_custom(conn, tx_handle, om);
        if (rc == 0) {
            off += chunk;
        } else {
            // Usually the controller's queue is full: back off and retry the
            // same bytes rather than lose them.
            vTaskDelay(pdMS_TO_TICKS(20));
        }
    }
}

// Brings the BLE stack up. Everything is registered on each start, because
// stopping tears the host down completely.
// NimBLE asserts and reboots when it cannot allocate, and with the preference
// saved in NVS that turns into a boot loop the device cannot be talked out of.
// So the heap is checked first and BLE simply declines to start if it is thin.
#define BLE_HEAP_NEEDED 45000

static esp_err_t ble_stack_start(void)
{
    size_t heap = esp_get_free_heap_size();
    if (heap < BLE_HEAP_NEEDED) {
        ESP_LOGE(TAG, "only %u bytes of heap; BLE needs about %d, staying on WiFi", (unsigned)heap,
                 BLE_HEAP_NEEDED);
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(TAG, "starting the BLE stack with %u bytes of heap", (unsigned)heap);
    ESP_RETURN_ON_ERROR(nimble_port_init(), TAG, "nimble_port_init");

    ble_hs_cfg.sync_cb = on_sync;
    ble_hs_cfg.reset_cb = on_reset;

    ble_svc_gap_init();
    ble_svc_gatt_init();
    ESP_RETURN_ON_FALSE(ble_gatts_count_cfg(gatt_svcs) == 0, ESP_FAIL, TAG, "count_cfg");
    ESP_RETURN_ON_FALSE(ble_gatts_add_svcs(gatt_svcs) == 0, ESP_FAIL, TAG, "add_svcs");
    ESP_RETURN_ON_FALSE(ble_svc_gap_device_name_set(CONFIG_RGNSS_HOSTNAME) == 0, ESP_FAIL, TAG, "set name");

    // 247 leaves 244 bytes of payload per notification, the most a 251-byte
    // link layer packet can carry without fragmenting.
    ble_att_set_preferred_mtu(247);

    nimble_port_freertos_init(host_task);
    return ESP_OK;
}

// True when this boot should run BLE rather than WiFi. Called before either
// radio is started, so the decision costs nothing to act on.
bool nmea_ble_boot_mode(void)
{
    nvs_handle_t nvs;
    uint8_t stored = 0;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs) == ESP_OK) {
        nvs_get_u8(nvs, NVS_KEY_PREFERRED, &stored);
        nvs_close(nvs);
    }
    preferred = stored != 0;

    if (wifi_this_boot == WIFI_THIS_BOOT_MAGIC) {
        wifi_this_boot = 0;  // this boot only
        ESP_LOGI(TAG, "set to start on BLE, but nothing used it last time: WiFi gets this boot");
        return false;
    }
    return preferred;
}

// Saves the choice and restarts into it, rather than moving the radio while
// running: tearing the BLE stack down mid-flight hung inside nimble_port_stop()
// and left the device with neither radio.
esp_err_t nmea_ble_set_enabled(bool on)
{
    nvs_handle_t nvs;
    ESP_RETURN_ON_ERROR(nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs), TAG, "open NVS");
    esp_err_t err = nvs_set_u8(nvs, NVS_KEY_PREFERRED, on ? 1 : 0);
    if (err == ESP_OK) {
        err = nvs_commit(nvs);
    }
    nvs_close(nvs);
    ESP_RETURN_ON_ERROR(err, TAG, "save the preference");

    preferred = on;
    wifi_this_boot = on ? 0 : WIFI_THIS_BOOT_MAGIC;
    ESP_LOGI(TAG, "restarting to come up on %s", on ? "BLE" : "WiFi");
    vTaskDelay(pdMS_TO_TICKS(250));  // let the reply reach the browser first
    esp_restart();
    return ESP_OK;
}

// Only called when nmea_ble_boot_mode() said so, so WiFi was never started and
// the radio is already free.
esp_err_t nmea_ble_start(void)
{
    ESP_RETURN_ON_ERROR(nmea_hub_init(), TAG, "hub");
    enabled = true;
    idle_since = esp_timer_get_time();
    ESP_RETURN_ON_ERROR(ble_stack_start(), TAG, "start the BLE stack");
    ESP_RETURN_ON_FALSE(xTaskCreate(pump_task, "nmea_ble", 3072, NULL, 5, NULL) == pdPASS, ESP_ERR_NO_MEM,
                        TAG, "create task");
    ESP_LOGI(TAG, "BLE has the radio; it goes back to WiFi after %d s with no client",
             CONFIG_RGNSS_BLE_IDLE_S);
    return ESP_OK;
}

void nmea_ble_status(nmea_ble_status_t *out)
{
    uint16_t conn = conn_handle;
    memset(out, 0, sizeof(*out));
    out->enabled = enabled;
    out->preferred = preferred;
    out->connected = conn != BLE_HS_CONN_HANDLE_NONE;
    out->subscribed = subscribed;
    out->mtu = out->connected ? ble_att_mtu(conn) : 0;
    strlcpy(out->peer, peer, sizeof(out->peer));
    if (sub != NULL) {
        nmea_sub_stats_t stats;
        nmea_sub_stats(sub, &stats);
        out->sentences = stats.sentences;
        out->bytes = stats.bytes;
        out->dropped = stats.dropped + dropped;
    }
}
