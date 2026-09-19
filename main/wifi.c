#include "wifi.h"

#include <stdatomic.h>
#include <stdio.h>
#include <string.h>

#include "esp_check.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_netif_sntp.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"
#include "nvs.h"

#if __has_include("wifi_credentials.h")
#include "wifi_credentials.h"
#endif
#ifndef WIFI_STA_SSID
#define WIFI_STA_SSID ""
#endif
#ifndef WIFI_STA_PASSWORD
#define WIFI_STA_PASSWORD ""
#endif
_Static_assert(sizeof(WIFI_STA_SSID) <= 33, "WIFI_STA_SSID is longer than 32 characters");
_Static_assert(sizeof(WIFI_STA_PASSWORD) == 1 || (sizeof(WIFI_STA_PASSWORD) >= 9 && sizeof(WIFI_STA_PASSWORD) <= 64),
               "WIFI_STA_PASSWORD must be empty or 8-63 characters");

#define NVS_NAMESPACE "wifi"
#define NVS_KEY_NETS "nets"
#define RECONNECT_MS 3000
// Reconnect attempts scan other channels and disturb the access point, so
// they're spaced out while it's up.
#define RECONNECT_WITH_AP_MS 15000
// A scan of every channel takes about two seconds, three while connected.
#define SCAN_WAIT_MS 8000
#define KNOWN_MAX (WIFI_SAVED_MAX + 1)  // the saved ones and the built-in one

ESP_EVENT_DEFINE_BASE(RGNSS_WIFI_EVENT);
enum { EV_FALLBACK, EV_RETRY, EV_CHANGED };

static const char *TAG = "wifi";

typedef struct {
    char ssid[33];
    char pass[64];
} net_t;

// The saved networks, newest first, exactly as they sit in NVS. Written by the
// web server's task and read by the event loop's, so both go through the lock.
static SemaphoreHandle_t list_lock;
static net_t saved[WIFI_SAVED_MAX];
static int n_saved;
static char sta_ssid[33];  // the network being joined or joined, under the lock too
static bool sta_builtin;

// Set once in wifi_start().
static esp_netif_t *sta_netif;
static TimerHandle_t retry_timer;
static TimerHandle_t fallback_timer;

// Event loop task only. Joining goes: scan, then try the networks it knows,
// strongest first, and only when every one has failed wait and scan again.
static enum { ST_IDLE, ST_SCANNING, ST_CONNECTING, ST_CONNECTED } state;
static atomic_int ap_clients;  // devices joined to the access point, from its events
static bool sta_up;  // the station interface exists: STA or AP+STA mode
static char cands[KNOWN_MAX][33];
static int8_t cand_rssi[KNOWN_MAX];  // INT8_MIN: not seen in the scan
static int n_cands, next_cand;

// The radio runs one scan at a time, and a scan's result belongs to whoever
// started it: the joining logic, or the page asking what is nearby. The event
// loop collects the page's into its buffer and wakes it.
enum { SCAN_NONE, SCAN_JOIN, SCAN_PAGE };
static atomic_int scan_owner;
static SemaphoreHandle_t scan_lock;  // guards the page's buffer pointer
static SemaphoreHandle_t scan_done;
static wifi_scan_entry_t *scan_out;
static size_t scan_max, scan_n;

// ---- the list ----

// Reads the saved networks. A device that last ran with a single saved network
// has it under "ssid" and "pass"; that one becomes the first of the list, so an
// update never loses the network the device is reachable through.
static void load_saved(void)
{
    nvs_handle_t nvs;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs) != ESP_OK) {
        return;
    }
    size_t len = sizeof(saved);
    esp_err_t err = nvs_get_blob(nvs, NVS_KEY_NETS, saved, &len);
    if (err == ESP_OK && len % sizeof(net_t) == 0) {
        n_saved = (int)(len / sizeof(net_t));
    } else if (err == ESP_ERR_NVS_NOT_FOUND) {
        size_t ssid_len = sizeof(saved[0].ssid), pass_len = sizeof(saved[0].pass);
        if (nvs_get_str(nvs, "ssid", saved[0].ssid, &ssid_len) == ESP_OK &&
            nvs_get_str(nvs, "pass", saved[0].pass, &pass_len) == ESP_OK && saved[0].ssid[0]) {
            n_saved = 1;
            // The old keys go only once the list is safely written.
            if (nvs_set_blob(nvs, NVS_KEY_NETS, saved, sizeof(net_t)) == ESP_OK && nvs_commit(nvs) == ESP_OK) {
                nvs_erase_key(nvs, "ssid");
                nvs_erase_key(nvs, "pass");
                nvs_commit(nvs);
                ESP_LOGI(TAG, "moved \"%s\" into the list of saved networks", saved[0].ssid);
            }
        } else {
            memset(saved, 0, sizeof(saved));
        }
    } else {
        ESP_LOGW(TAG, "saved networks unreadable (%s), starting with none", esp_err_to_name(err));
        memset(saved, 0, sizeof(saved));
    }
    nvs_close(nvs);
}

// Call with list_lock held.
static esp_err_t store_saved(void)
{
    nvs_handle_t nvs;
    ESP_RETURN_ON_ERROR(nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs), TAG, "open NVS");
    esp_err_t err = n_saved ? nvs_set_blob(nvs, NVS_KEY_NETS, saved, n_saved * sizeof(net_t))
                            : nvs_erase_key(nvs, NVS_KEY_NETS);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        err = ESP_OK;  // erasing a list that was never written
    }
    if (err == ESP_OK) {
        err = nvs_commit(nvs);
    }
    nvs_close(nvs);
    return err;
}

// Every network it may join, saved ones first, then the built-in one unless it
// was saved as well. Names only.
static int known_names(char names[][33])
{
    int n = 0;
    xSemaphoreTake(list_lock, portMAX_DELAY);
    bool builtin_saved = false;
    for (int i = 0; i < n_saved; i++) {
        strlcpy(names[n++], saved[i].ssid, 33);
        builtin_saved |= strcmp(saved[i].ssid, WIFI_STA_SSID) == 0;
    }
    xSemaphoreGive(list_lock);
    if (WIFI_STA_SSID[0] && !builtin_saved) {
        strlcpy(names[n++], WIFI_STA_SSID, 33);
    }
    return n;
}

static int known_count(void)
{
    char names[KNOWN_MAX][33];
    return known_names(names);
}

// The password for a network it knows; false if it no longer knows it.
static bool lookup(const char *ssid, char pass[64], bool *builtin)
{
    bool found = false;
    xSemaphoreTake(list_lock, portMAX_DELAY);
    for (int i = 0; i < n_saved && !found; i++) {
        if (strcmp(saved[i].ssid, ssid) == 0) {
            strlcpy(pass, saved[i].pass, 64);
            *builtin = false;
            found = true;
        }
    }
    xSemaphoreGive(list_lock);
    if (!found && WIFI_STA_SSID[0] && strcmp(ssid, WIFI_STA_SSID) == 0) {
        strlcpy(pass, WIFI_STA_PASSWORD, 64);
        *builtin = true;
        found = true;
    }
    return found;
}

// ---- the access point ----

static bool ap_active(void)
{
    wifi_mode_t mode;
    return esp_wifi_get_mode(&mode) == ESP_OK && (mode & WIFI_MODE_AP);
}

static esp_err_t ap_enable(void)
{
    wifi_config_t cfg = {
        .ap = {
            .ssid = CONFIG_RGNSS_AP_SSID,
            .ssid_len = strlen(CONFIG_RGNSS_AP_SSID),
            .channel = CONFIG_RGNSS_AP_CHANNEL,
            .max_connection = CONFIG_RGNSS_AP_MAX_CONN,
            .authmode = WIFI_AUTH_OPEN,
        },
    };
    wifi_mode_t mode = known_count() ? WIFI_MODE_APSTA : WIFI_MODE_AP;
    ESP_RETURN_ON_ERROR(esp_wifi_set_mode(mode), TAG, "set mode");
    ESP_RETURN_ON_ERROR(esp_wifi_set_config(WIFI_IF_AP, &cfg), TAG, "set AP config");
    ESP_LOGI(TAG, "open access point \"%s\" up", CONFIG_RGNSS_AP_SSID);
    return ESP_OK;
}

// ---- joining, all of it on the event loop task ----

static void retry_later(void)
{
    int delay_ms = ap_active() ? RECONNECT_WITH_AP_MS : RECONNECT_MS;
    if (xTimerChangePeriod(retry_timer, pdMS_TO_TICKS(delay_ms), 0) != pdPASS) {
        ESP_LOGE(TAG, "could not schedule the retry");
    }
}

// Tries the next candidate, or gives up on this round and waits.
static void connect_next(void)
{
    while (next_cand < n_cands) {
        const char *ssid = cands[next_cand];
        int8_t rssi = cand_rssi[next_cand];
        next_cand++;

        char pass[64];
        bool builtin;
        if (!lookup(ssid, pass, &builtin)) {
            continue;  // forgotten since the scan
        }
        // A 32-character SSID fills the field without a terminator.
        wifi_config_t cfg = {0};
        memcpy(cfg.sta.ssid, ssid, strlen(ssid));
        memcpy(cfg.sta.password, pass, strlen(pass));
        cfg.sta.threshold.authmode = pass[0] ? WIFI_AUTH_WPA2_PSK : WIFI_AUTH_OPEN;

        xSemaphoreTake(list_lock, portMAX_DELAY);
        strlcpy(sta_ssid, ssid, sizeof(sta_ssid));
        sta_builtin = builtin;
        xSemaphoreGive(list_lock);

        esp_err_t err = esp_wifi_set_config(WIFI_IF_STA, &cfg);
        if (err == ESP_OK) {
            err = esp_wifi_connect();
        }
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "could not start joining \"%s\": %s", ssid, esp_err_to_name(err));
            continue;
        }
        const char *source = builtin ? "built in" : "saved";
        if (rssi == INT8_MIN) {
            ESP_LOGI(TAG, "joining \"%s\" (%s)", ssid, source);
        } else {
            ESP_LOGI(TAG, "joining \"%s\" (%s, %d dBm)", ssid, source, rssi);
        }
        state = ST_CONNECTING;
        return;
    }
    state = ST_IDLE;
    retry_later();
}

static void start_round(void)
{
    char names[KNOWN_MAX][33];
    int known = known_names(names);
    n_cands = next_cand = 0;
    if (known == 0) {
        state = ST_IDLE;
        return;
    }
    if (known == 1) {
        // Nothing to choose between, so no scan: join it straight away, which
        // also finds a hidden network.
        strlcpy(cands[0], names[0], sizeof(cands[0]));
        cand_rssi[0] = INT8_MIN;
        n_cands = 1;
        connect_next();
        return;
    }
    int none = SCAN_NONE;
    if (!atomic_compare_exchange_strong(&scan_owner, &none, SCAN_JOIN)) {
        state = ST_IDLE;  // the page is scanning; its result is not ours to use
        retry_later();
        return;
    }
    esp_err_t err = esp_wifi_scan_start(NULL, false);
    if (err != ESP_OK) {
        atomic_store(&scan_owner, SCAN_NONE);
        ESP_LOGW(TAG, "could not scan: %s", esp_err_to_name(err));
        state = ST_IDLE;
        retry_later();
        return;
    }
    state = ST_SCANNING;
    // A scan that never reports back would otherwise leave it waiting for ever.
    xTimerChangePeriod(retry_timer, pdMS_TO_TICKS(SCAN_WAIT_MS), 0);
}

// Orders what it knows: the ones in range, strongest first, then the rest in
// case they are hidden or were missed.
static void on_join_scan_done(void)
{
    static char names[KNOWN_MAX][33];
    int known = known_names(names);
    int8_t best[KNOWN_MAX];
    bool seen[KNOWN_MAX] = {false};
    wifi_ap_record_t ap;
    while (esp_wifi_scan_get_ap_record(&ap) == ESP_OK) {
        for (int k = 0; k < known; k++) {
            if (strncmp((const char *)ap.ssid, names[k], sizeof(ap.ssid)) == 0 && (!seen[k] || ap.rssi > best[k])) {
                seen[k] = true;
                best[k] = ap.rssi;
            }
        }
    }
    if (state != ST_SCANNING) {
        return;  // the station went away mid-scan
    }

    n_cands = next_cand = 0;
    bool used[KNOWN_MAX] = {false};
    for (;;) {
        int pick = -1;
        for (int k = 0; k < known; k++) {
            if (seen[k] && !used[k] && (pick < 0 || best[k] > best[pick])) {
                pick = k;
            }
        }
        if (pick < 0) {
            break;
        }
        used[pick] = true;
        strlcpy(cands[n_cands], names[pick], sizeof(cands[0]));
        cand_rssi[n_cands++] = best[pick];
    }
    for (int k = 0; k < known; k++) {
        if (!used[k]) {
            strlcpy(cands[n_cands], names[k], sizeof(cands[0]));
            cand_rssi[n_cands++] = INT8_MIN;
        }
    }
    connect_next();
}

// The page's scan: one entry per name, strongest signal kept.
static void on_page_scan_done(void)
{
    xSemaphoreTake(scan_lock, portMAX_DELAY);
    size_t n = 0;
    wifi_ap_record_t ap;
    while (esp_wifi_scan_get_ap_record(&ap) == ESP_OK) {
        if (!scan_out || ap.ssid[0] == '\0') {
            continue;  // nobody waiting any more, or a hidden network with no name to show
        }
        const char *name = (const char *)ap.ssid;
        size_t i = 0;
        while (i < n && strncmp(scan_out[i].ssid, name, sizeof(ap.ssid)) != 0) {
            i++;
        }
        if (i == n) {
            if (n == scan_max) {
                continue;
            }
            n++;
            strlcpy(scan_out[i].ssid, name, sizeof(scan_out[i].ssid));
            scan_out[i].rssi = INT8_MIN;
        }
        if (ap.rssi > scan_out[i].rssi) {
            scan_out[i].rssi = ap.rssi;
            scan_out[i].channel = ap.primary;
            scan_out[i].secure = ap.authmode != WIFI_AUTH_OPEN && ap.authmode != WIFI_AUTH_OWE;
        }
    }
    scan_n = n;
    xSemaphoreGive(scan_lock);
}

static void on_wifi_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    switch (id) {
    case WIFI_EVENT_STA_START:
        sta_up = true;
        if (known_count() == 0) {
            break;  // brought up only for the page to scan with
        }
        start_round();
        if (!ap_active() && xTimerStart(fallback_timer, 0) != pdPASS) {
            ESP_LOGW(TAG, "could not arm the access point fallback");
        }
        break;
    case WIFI_EVENT_STA_STOP:
        sta_up = false;
        state = ST_IDLE;
        xTimerStop(retry_timer, 0);
        break;
    case WIFI_EVENT_SCAN_DONE:
        if (atomic_load(&scan_owner) == SCAN_JOIN) {
            atomic_store(&scan_owner, SCAN_NONE);
            on_join_scan_done();
        } else if (atomic_load(&scan_owner) == SCAN_PAGE) {
            on_page_scan_done();
            atomic_store(&scan_owner, SCAN_NONE);
            xSemaphoreGive(scan_done);
        }
        break;
    case WIFI_EVENT_STA_DISCONNECTED: {
        if (!sta_up) {
            break;
        }
        wifi_event_sta_disconnected_t *e = data;
        ESP_LOGW(TAG, "not connected to \"%.*s\" (reason %d)", e->ssid_len, (const char *)e->ssid, e->reason);
        if (!ap_active() && !xTimerIsTimerActive(fallback_timer) && xTimerStart(fallback_timer, 0) != pdPASS) {
            ESP_LOGW(TAG, "could not arm the access point fallback");
        }
        if (state == ST_CONNECTING && next_cand < n_cands) {
            connect_next();  // the next one it knows, without waiting
        } else if (state == ST_CONNECTING || state == ST_CONNECTED) {
            state = ST_IDLE;
            retry_later();
        }
        break;
    }
    case WIFI_EVENT_AP_STACONNECTED: {
        wifi_event_ap_staconnected_t *e = data;
        ESP_LOGI(TAG, "client " MACSTR " joined the access point", MAC2STR(e->mac));
        atomic_fetch_add(&ap_clients, 1);
        break;
    }
    case WIFI_EVENT_AP_STADISCONNECTED: {
        wifi_event_ap_stadisconnected_t *e = data;
        ESP_LOGI(TAG, "client " MACSTR " left the access point", MAC2STR(e->mac));
        if (atomic_fetch_sub(&ap_clients, 1) <= 0) {
            atomic_store(&ap_clients, 0);
        }
        break;
    }
    }
}

// Network time, once there is a network. It is the one clock the GNSS
// receiver cannot lie about, so gnss.c checks the receiver's time against it.
static void start_time_sync(void)
{
    static bool started;
    if (started) {
        return;
    }
    esp_sntp_config_t cfg = ESP_NETIF_SNTP_DEFAULT_CONFIG("pool.ntp.org");
    esp_err_t err = esp_netif_sntp_init(&cfg);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "no network time: %s", esp_err_to_name(err));
        return;
    }
    started = true;
}

static void on_got_ip(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    ip_event_got_ip_t *e = data;
    char ssid[33];
    xSemaphoreTake(list_lock, portMAX_DELAY);
    strlcpy(ssid, sta_ssid, sizeof(ssid));
    xSemaphoreGive(list_lock);
    ESP_LOGI(TAG, "connected to \"%s\", IP " IPSTR, ssid, IP2STR(&e->ip_info.ip));
    state = ST_CONNECTED;
    xTimerStop(fallback_timer, 0);
    xTimerStop(retry_timer, 0);
    start_time_sync();
    if (ap_active()) {
        ESP_LOGI(TAG, "access point down");
        esp_wifi_set_mode(WIFI_MODE_STA);
    }
}

static void on_own_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    switch (id) {
    case EV_FALLBACK:
        if (state != ST_CONNECTED && !ap_active()) {
            ESP_LOGW(TAG, "no connection for %d s", CONFIG_RGNSS_STA_FALLBACK_S);
            ap_enable();
        }
        break;
    case EV_RETRY:
        if (sta_up && state == ST_SCANNING) {
            ESP_LOGW(TAG, "the scan never finished; trying again");
            esp_wifi_scan_stop();
            atomic_store(&scan_owner, SCAN_NONE);
            state = ST_IDLE;
            retry_later();
        } else if (sta_up && state == ST_IDLE) {
            start_round();
        }
        break;
    case EV_CHANGED: {
        // A network was added or forgotten on the page. No restart: an access
        // point with nothing to join gets its station back, a device waiting to
        // retry tries now, and one on a network that was just forgotten leaves it.
        if (!sta_up) {
            if (known_count() > 0) {
                esp_wifi_set_mode(WIFI_MODE_APSTA);  // STA_START follows and joins
            }
            break;
        }
        char ssid[33], pass[64];
        bool builtin;
        xSemaphoreTake(list_lock, portMAX_DELAY);
        strlcpy(ssid, sta_ssid, sizeof(ssid));
        xSemaphoreGive(list_lock);
        if (state == ST_IDLE) {
            xTimerStop(retry_timer, 0);
            start_round();
        } else if ((state == ST_CONNECTING || state == ST_CONNECTED) && !lookup(ssid, pass, &builtin)) {
            ESP_LOGI(TAG, "\"%s\" was forgotten, leaving it", ssid);
            esp_wifi_disconnect();  // DISCONNECTED follows and moves on
        }
        break;
    }
    }
}

static void retry_timer_cb(TimerHandle_t timer)
{
    esp_event_post(RGNSS_WIFI_EVENT, EV_RETRY, NULL, 0, 0);
}

// Hands over to the event loop task, which owns the connection state.
static void fallback_timer_cb(TimerHandle_t timer)
{
    esp_event_post(RGNSS_WIFI_EVENT, EV_FALLBACK, NULL, 0, 0);
}

esp_err_t wifi_load(void)
{
    if (list_lock) {
        return ESP_OK;
    }
    list_lock = xSemaphoreCreateMutex();
    ESP_RETURN_ON_FALSE(list_lock, ESP_ERR_NO_MEM, TAG, "create list lock");
    load_saved();
    return ESP_OK;
}

esp_err_t wifi_start(void)
{
    ESP_RETURN_ON_ERROR(wifi_load(), TAG, "load saved networks");
    scan_lock = xSemaphoreCreateMutex();
    scan_done = xSemaphoreCreateBinary();
    ESP_RETURN_ON_FALSE(scan_lock && scan_done, ESP_ERR_NO_MEM, TAG, "create locks");

    esp_netif_create_default_wifi_ap();
    sta_netif = esp_netif_create_default_wifi_sta();

    wifi_init_config_t init = WIFI_INIT_CONFIG_DEFAULT();
    ESP_RETURN_ON_ERROR(esp_wifi_init(&init), TAG, "init");
    // Credentials live in our own NVS namespace, not the driver's.
    ESP_RETURN_ON_ERROR(esp_wifi_set_storage(WIFI_STORAGE_RAM), TAG, "set storage");
    ESP_RETURN_ON_ERROR(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, on_wifi_event, NULL),
                        TAG, "register WiFi handler");
    ESP_RETURN_ON_ERROR(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, on_got_ip, NULL),
                        TAG, "register IP handler");
    ESP_RETURN_ON_ERROR(esp_event_handler_register(RGNSS_WIFI_EVENT, ESP_EVENT_ANY_ID, on_own_event, NULL),
                        TAG, "register own handler");
    // The timers exist whether or not there is a network yet: one can be added
    // from the page at any time.
    retry_timer = xTimerCreate("wifi_retry", pdMS_TO_TICKS(RECONNECT_MS), pdFALSE, NULL, retry_timer_cb);
    fallback_timer = xTimerCreate("wifi_fallback", pdMS_TO_TICKS(CONFIG_RGNSS_STA_FALLBACK_S * 1000), pdFALSE,
                                  NULL, fallback_timer_cb);
    ESP_RETURN_ON_FALSE(retry_timer && fallback_timer, ESP_ERR_NO_MEM, TAG, "create timers");

    int known = known_count();
    if (known == 0) {
        ESP_LOGI(TAG, "no station network configured");
        ESP_RETURN_ON_ERROR(ap_enable(), TAG, "enable AP");
        return esp_wifi_start();
    }
    ESP_LOGI(TAG, "%d saved network%s%s", n_saved, n_saved == 1 ? "" : "s",
             known > n_saved ? ", plus the built-in one" : "");
    ESP_RETURN_ON_ERROR(esp_wifi_set_mode(WIFI_MODE_STA), TAG, "set mode");
    return esp_wifi_start();
}

esp_err_t wifi_save_station(const char *ssid, const char *password)
{
    size_t ssid_len = strlen(ssid);
    size_t pass_len = strlen(password);
    ESP_RETURN_ON_FALSE(ssid_len >= 1 && ssid_len <= 32, ESP_ERR_INVALID_ARG, TAG, "SSID must be 1-32 characters");
    ESP_RETURN_ON_FALSE(pass_len == 0 || (pass_len >= 8 && pass_len <= 63), ESP_ERR_INVALID_ARG,
                        TAG, "password must be empty or 8-63 characters");

    xSemaphoreTake(list_lock, portMAX_DELAY);
    net_t before[WIFI_SAVED_MAX];
    int n_before = n_saved;
    memcpy(before, saved, sizeof(saved));

    // Saving a network it already has replaces the password; either way the
    // network goes to the top, as the one most recently asked for.
    int at = 0;
    while (at < n_saved && strcmp(saved[at].ssid, ssid) != 0) {
        at++;
    }
    if (at == n_saved && n_saved == WIFI_SAVED_MAX) {
        xSemaphoreGive(list_lock);
        return ESP_ERR_NO_MEM;
    }
    if (at == n_saved) {
        n_saved++;
    }
    memmove(&saved[1], &saved[0], at * sizeof(net_t));
    memset(&saved[0], 0, sizeof(net_t));
    strlcpy(saved[0].ssid, ssid, sizeof(saved[0].ssid));
    strlcpy(saved[0].pass, password, sizeof(saved[0].pass));

    esp_err_t err = store_saved();
    if (err != ESP_OK) {
        memcpy(saved, before, sizeof(saved));
        n_saved = n_before;
    }
    xSemaphoreGive(list_lock);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "saved \"%s\"", ssid);
        esp_event_post(RGNSS_WIFI_EVENT, EV_CHANGED, NULL, 0, pdMS_TO_TICKS(100));
    }
    return err;
}

esp_err_t wifi_forget_station(const char *ssid)
{
    xSemaphoreTake(list_lock, portMAX_DELAY);
    net_t before[WIFI_SAVED_MAX];
    int n_before = n_saved;
    memcpy(before, saved, sizeof(saved));

    if (!ssid) {
        n_saved = 0;
    } else {
        int at = 0;
        while (at < n_saved && strcmp(saved[at].ssid, ssid) != 0) {
            at++;
        }
        if (at == n_saved) {
            xSemaphoreGive(list_lock);
            return ESP_ERR_NOT_FOUND;
        }
        memmove(&saved[at], &saved[at + 1], (n_saved - at - 1) * sizeof(net_t));
        n_saved--;
    }
    memset(&saved[n_saved], 0, (WIFI_SAVED_MAX - n_saved) * sizeof(net_t));

    esp_err_t err = store_saved();
    if (err != ESP_OK) {
        memcpy(saved, before, sizeof(saved));
        n_saved = n_before;
    }
    xSemaphoreGive(list_lock);
    if (err == ESP_OK) {
        if (ssid) {
            ESP_LOGI(TAG, "forgot \"%s\"", ssid);
        } else {
            ESP_LOGI(TAG, "forgot every saved network");
        }
        esp_event_post(RGNSS_WIFI_EVENT, EV_CHANGED, NULL, 0, pdMS_TO_TICKS(100));
    }
    return err;
}

size_t wifi_get_saved(char (*names)[33], size_t max)
{
    xSemaphoreTake(list_lock, portMAX_DELAY);
    size_t n = 0;
    for (; n < (size_t)n_saved && n < max; n++) {
        strlcpy(names[n], saved[n].ssid, 33);
    }
    xSemaphoreGive(list_lock);
    return n;
}

const char *wifi_builtin_ssid(void)
{
    return WIFI_STA_SSID[0] ? WIFI_STA_SSID : NULL;
}

esp_err_t wifi_scan(wifi_scan_entry_t *out, size_t max, size_t *n)
{
    *n = 0;
    wifi_mode_t mode;
    ESP_RETURN_ON_ERROR(esp_wifi_get_mode(&mode), TAG, "get mode");
    int none = SCAN_NONE;
    if (!atomic_compare_exchange_strong(&scan_owner, &none, SCAN_PAGE)) {
        return ESP_ERR_INVALID_STATE;  // it is scanning to join a network
    }

    // The access point alone cannot scan, so a station interface comes up for
    // the length of it. Its clients may miss a beacon or two meanwhile.
    bool added_sta = false;
    esp_err_t err = ESP_OK;
    if (mode == WIFI_MODE_AP) {
        err = esp_wifi_set_mode(WIFI_MODE_APSTA);
        added_sta = err == ESP_OK;
    }

    xSemaphoreTake(scan_lock, portMAX_DELAY);
    scan_out = out;
    scan_max = max;
    scan_n = 0;
    xSemaphoreGive(scan_lock);
    xSemaphoreTake(scan_done, 0);  // a give left over from a scan that timed out

    if (err == ESP_OK) {
        err = esp_wifi_scan_start(NULL, false);
    }
    if (err != ESP_OK) {
        atomic_store(&scan_owner, SCAN_NONE);
        if (err == ESP_ERR_WIFI_STATE) {
            err = ESP_ERR_INVALID_STATE;  // busy joining a network
        }
    } else if (xSemaphoreTake(scan_done, pdMS_TO_TICKS(SCAN_WAIT_MS)) != pdTRUE) {
        // Handed back here rather than waiting for a SCAN_DONE that may never
        // come: joining a network must not be locked out by the page.
        esp_wifi_scan_stop();
        atomic_store(&scan_owner, SCAN_NONE);
        err = ESP_ERR_TIMEOUT;
    }

    // Whatever happened, the event loop must not write here once this returns.
    xSemaphoreTake(scan_lock, portMAX_DELAY);
    scan_out = NULL;
    size_t got = err == ESP_OK ? scan_n : 0;
    xSemaphoreGive(scan_lock);

    if (added_sta && known_count() == 0) {
        esp_wifi_set_mode(WIFI_MODE_AP);
    }

    // Strongest first; and say which ones it already knows.
    for (size_t i = 1; i < got; i++) {
        wifi_scan_entry_t e = out[i];
        size_t j = i;
        for (; j > 0 && out[j - 1].rssi < e.rssi; j--) {
            out[j] = out[j - 1];
        }
        out[j] = e;
    }
    for (size_t i = 0; i < got; i++) {
        char pass[64];
        bool builtin;
        out[i].known = lookup(out[i].ssid, pass, &builtin);
    }
    *n = got;
    return err;
}

bool wifi_link_up(void)
{
    return state == ST_CONNECTED || (ap_active() && atomic_load(&ap_clients) > 0);
}

void wifi_get_status(wifi_status_t *out)
{
    memset(out, 0, sizeof(*out));
    out->ap_active = ap_active();
    strlcpy(out->ap_ssid, CONFIG_RGNSS_AP_SSID, sizeof(out->ap_ssid));
    out->sta_configured = known_count() > 0;
    xSemaphoreTake(list_lock, portMAX_DELAY);
    out->sta_saved = sta_ssid[0] && !sta_builtin;
    strlcpy(out->sta_ssid, sta_ssid, sizeof(out->sta_ssid));
    xSemaphoreGive(list_lock);

    wifi_ap_record_t ap;
    esp_netif_ip_info_t ip;
    if (sta_netif && esp_wifi_sta_get_ap_info(&ap) == ESP_OK &&
        esp_netif_get_ip_info(sta_netif, &ip) == ESP_OK && ip.ip.addr != 0) {
        out->sta_connected = true;
        out->sta_rssi = ap.rssi;
        snprintf(out->sta_ip, sizeof(out->sta_ip), IPSTR, IP2STR(&ip.ip));
    }
}
