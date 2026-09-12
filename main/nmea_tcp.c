#include "nmea_tcp.h"

#include <errno.h>
#include <string.h>

#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "lwip/netdb.h"
#include "lwip/sockets.h"
#include "nmea_hub.h"

// Holds several sentences per packet. Sized so a maximal sentence always fits
// after the last one that was appended.
#define OUT_SIZE 512

// A client that can't take data for this long is treated as gone. It is far
// longer than any transient stall and far shorter than the keepalive probe.
#define SEND_TIMEOUT_S 5

// A listener that only reads sends nothing back, so a client that vanished
// without a FIN (a phone out of range) is invisible until TCP probes for it.
#define KEEPALIVE_IDLE_S 30
#define KEEPALIVE_INTERVAL_S 10
#define KEEPALIVE_COUNT 3

static const char *TAG = "nmea_tcp";

typedef struct {
    bool in_use;
    int sock;
    nmea_sub_t sub;
    char peer[24];
    int64_t connected_us;
} client_t;

static client_t clients[CONFIG_RGNSS_TCP_MAX_CLIENTS];
static SemaphoreHandle_t lock;

static void configure_socket(int sock)
{
    int one = 1;
    int idle = KEEPALIVE_IDLE_S;
    int interval = KEEPALIVE_INTERVAL_S;
    int count = KEEPALIVE_COUNT;
    struct timeval timeout = {.tv_sec = SEND_TIMEOUT_S};

    // Sentences are small and latency matters more than packet count here.
    setsockopt(sock, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
    setsockopt(sock, SOL_SOCKET, SO_KEEPALIVE, &one, sizeof(one));
    setsockopt(sock, IPPROTO_TCP, TCP_KEEPIDLE, &idle, sizeof(idle));
    setsockopt(sock, IPPROTO_TCP, TCP_KEEPINTVL, &interval, sizeof(interval));
    setsockopt(sock, IPPROTO_TCP, TCP_KEEPCNT, &count, sizeof(count));
    setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));
}

static bool send_all(int sock, const char *buf, size_t len)
{
    size_t sent = 0;
    while (sent < len) {
        int n = send(sock, buf + sent, len - sent, 0);
        if (n < 0) {
            if (errno == EINTR) {
                continue;
            }
            return false;  // includes the send timeout: a client this slow is dropped
        }
        sent += (size_t)n;
    }
    return true;
}

// True when the peer has closed. Anything the client sends is discarded for
// now; it is the path a later phase would use for UBX commands.
static bool peer_closed(int sock)
{
    char scratch[64];
    for (;;) {
        int n = recv(sock, scratch, sizeof(scratch), MSG_DONTWAIT);
        if (n == 0) {
            return true;
        }
        if (n < 0) {
            return errno != EWOULDBLOCK && errno != EAGAIN && errno != EINTR;
        }
    }
}

static void release(client_t *c)
{
    ESP_LOGI(TAG, "%s disconnected", c->peer);
    nmea_hub_unsubscribe(c->sub);
    close(c->sock);
    xSemaphoreTake(lock, portMAX_DELAY);
    c->in_use = false;
    xSemaphoreGive(lock);
}

static void client_task(void *arg)
{
    client_t *c = arg;
    char sentence[NMEA_HUB_MAX_SENTENCE + 1];
    char out[OUT_SIZE];

    for (;;) {
        // Wake regularly even with no sentences, so a dead peer is noticed and
        // a receiver that went quiet doesn't hold the slot open forever.
        size_t n = nmea_sub_receive(c->sub, sentence, sizeof(sentence), 500);
        size_t used = 0;
        while (n > 0) {
            memcpy(out + used, sentence, n);
            used += n;
            out[used++] = '\r';
            out[used++] = '\n';
            if (used + NMEA_HUB_MAX_SENTENCE + 2 > sizeof(out)) {
                break;  // send what we have; the rest waits in the buffer
            }
            n = nmea_sub_receive(c->sub, sentence, sizeof(sentence), 0);
        }
        if (used > 0 && !send_all(c->sock, out, used)) {
            break;
        }
        if (peer_closed(c->sock)) {
            break;
        }
    }

    release(c);
    vTaskDelete(NULL);
}

// Takes a slot for the accepted socket and starts its task. Returns false when
// the server is full or out of memory, and the caller closes the socket.
static bool start_client(int sock, const struct sockaddr_in *addr)
{
    client_t *c = NULL;
    xSemaphoreTake(lock, portMAX_DELAY);
    for (size_t i = 0; i < sizeof(clients) / sizeof(clients[0]); i++) {
        if (!clients[i].in_use) {
            c = &clients[i];
            c->in_use = true;
            break;
        }
    }
    xSemaphoreGive(lock);

    if (c == NULL) {
        ESP_LOGW(TAG, "refused a client: all %d slots are taken", CONFIG_RGNSS_TCP_MAX_CLIENTS);
        return false;
    }

    char ip[16];
    inet_ntoa_r(addr->sin_addr, ip, sizeof(ip));
    snprintf(c->peer, sizeof(c->peer), "%s:%u", ip, ntohs(addr->sin_port));
    c->sock = sock;
    c->connected_us = esp_timer_get_time();

    c->sub = nmea_hub_subscribe(c->peer);
    if (c->sub == NULL) {
        xSemaphoreTake(lock, portMAX_DELAY);
        c->in_use = false;
        xSemaphoreGive(lock);
        return false;
    }

    configure_socket(sock);
    if (xTaskCreate(client_task, "nmea_tcp_cl", 3072, c, 5, NULL) != pdPASS) {
        ESP_LOGE(TAG, "no memory for a task for %s", c->peer);
        nmea_hub_unsubscribe(c->sub);
        xSemaphoreTake(lock, portMAX_DELAY);
        c->in_use = false;
        xSemaphoreGive(lock);
        return false;
    }

    ESP_LOGI(TAG, "%s connected", c->peer);
    return true;
}

static void listener_task(void *arg)
{
    int lsock = -1;

    for (;;) {
        if (lsock < 0) {
            lsock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
            if (lsock < 0) {
                ESP_LOGE(TAG, "socket: errno %d", errno);
                vTaskDelay(pdMS_TO_TICKS(1000));
                continue;
            }
            int one = 1;
            setsockopt(lsock, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));

            struct sockaddr_in bind_addr = {
                .sin_family = AF_INET,
                .sin_port = htons(CONFIG_RGNSS_TCP_PORT),
                .sin_addr.s_addr = htonl(INADDR_ANY),
            };
            if (bind(lsock, (struct sockaddr *)&bind_addr, sizeof(bind_addr)) != 0 ||
                listen(lsock, CONFIG_RGNSS_TCP_MAX_CLIENTS) != 0) {
                ESP_LOGE(TAG, "bind/listen on port %d: errno %d", CONFIG_RGNSS_TCP_PORT, errno);
                close(lsock);
                lsock = -1;
                vTaskDelay(pdMS_TO_TICKS(1000));
                continue;
            }
            ESP_LOGI(TAG, "listening on port %d", CONFIG_RGNSS_TCP_PORT);
        }

        struct sockaddr_in src;
        socklen_t len = sizeof(src);
        int sock = accept(lsock, (struct sockaddr *)&src, &len);
        if (sock < 0) {
            ESP_LOGW(TAG, "accept: errno %d", errno);
            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
        }
        if (!start_client(sock, &src)) {
            close(sock);
        }
    }
}

esp_err_t nmea_tcp_start(void)
{
    ESP_RETURN_ON_ERROR(nmea_hub_init(), TAG, "hub");
    if (lock == NULL) {
        lock = xSemaphoreCreateMutex();
        ESP_RETURN_ON_FALSE(lock != NULL, ESP_ERR_NO_MEM, TAG, "create lock");
    }
    ESP_RETURN_ON_FALSE(xTaskCreate(listener_task, "nmea_tcp", 4096, NULL, 5, NULL) == pdPASS, ESP_ERR_NO_MEM,
                        TAG, "create task");
    return ESP_OK;
}

size_t nmea_tcp_clients(nmea_tcp_client_t *out, size_t max)
{
    size_t n = 0;
    if (lock == NULL) {
        return 0;
    }
    xSemaphoreTake(lock, portMAX_DELAY);
    for (size_t i = 0; i < sizeof(clients) / sizeof(clients[0]) && n < max; i++) {
        if (!clients[i].in_use) {
            continue;
        }
        nmea_sub_stats_t stats;
        nmea_sub_stats(clients[i].sub, &stats);
        strlcpy(out[n].peer, clients[i].peer, sizeof(out[n].peer));
        out[n].sentences = stats.sentences;
        out[n].bytes = stats.bytes;
        out[n].dropped = stats.dropped;
        out[n].connected_us = clients[i].connected_us;
        n++;
    }
    xSemaphoreGive(lock);
    return n;
}
