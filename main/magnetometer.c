#include "magnetometer.h"

#include <math.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs.h"

#define I2C_HZ 100000
#define I2C_TIMEOUT_MS 50
#define SAMPLE_MS 100
#define RESET_MS 20
#define NVS_NAMESPACE "mag"
#define NVS_KEY_OFFSET "hdg_off"

typedef struct {
    const char *name;
    uint8_t addrs[4];          // candidate addresses; unused entries are 0
    uint8_t id_reg;
    uint8_t id;
    uint8_t data_reg;          // X LSB; X, Y, Z follow as little-endian int16
    float ut_per_count;        // at the range set in init
    uint8_t reset[2];          // {register, value} soft reset before the ID check; {0, 0} for none
    const uint8_t (*init)[2];  // {register, value} writes
    size_t init_len;
    uint8_t trigger[2];        // {register, value} starting a single measurement; {0, 0} if free-running
} chip_t;

// Axis signs, 8 G range with set/reset on, normal mode at 50 Hz (OSR 8, DSR 8).
static const uint8_t qmc5883p_init[][2] = {
    {0x29, 0x06},
    {0x0B, 0x08},
    {0x0A, 0xC5},
};

// Set/reset period, then continuous mode at 50 Hz, 8 G range, OSR 512.
static const uint8_t qmc5883l_init[][2] = {
    {0x0B, 0x01},
    {0x09, 0x15},
};

// 16x averaging on all axes, normal set/reset pulse duration.
static const uint8_t ist8310_init[][2] = {
    {0x41, 0x24},
    {0x42, 0xC0},
};

// The IST8310's address range overlaps the QMC5883L's 0x0D, so every candidate
// is confirmed by its ID, QMC5883L first.
static const chip_t chips[] = {
    {
        .name = "QMC5883P", .addrs = {0x2C}, .id_reg = 0x00, .id = 0x80, .data_reg = 0x01,
        .ut_per_count = 100.0f / 3750.0f, .init = qmc5883p_init, .init_len = 3,
    },
    {
        .name = "QMC5883L", .addrs = {0x0D}, .id_reg = 0x0D, .id = 0xFF, .data_reg = 0x00,
        .ut_per_count = 100.0f / 3000.0f, .init = qmc5883l_init, .init_len = 2,
    },
    {
        // No continuous mode: each sample is a triggered single measurement.
        // Its ID register is unreliable until after a soft reset.
        .name = "IST8310", .addrs = {0x0C, 0x0D, 0x0E, 0x0F}, .id_reg = 0x00, .id = 0x10, .data_reg = 0x03,
        .ut_per_count = 0.3f, .reset = {0x0B, 0x01}, .init = ist8310_init, .init_len = 2, .trigger = {0x0A, 0x01},
    },
};

static const char *TAG = "mag";

static i2c_master_bus_handle_t bus;
static i2c_master_dev_handle_t dev;
static const chip_t *chip;
static uint8_t chip_addr;

static portMUX_TYPE lock = portMUX_INITIALIZER_UNLOCKED;
static magnetometer_data_t data;
static magnetometer_sample_t history[MAGNETOMETER_HISTORY];
static uint32_t next_seq = 1;

static esp_err_t read_regs(uint8_t reg, uint8_t *buf, size_t len)
{
    return i2c_master_transmit_receive(dev, &reg, 1, buf, len, I2C_TIMEOUT_MS);
}

static esp_err_t write_reg(uint8_t reg, uint8_t value)
{
    uint8_t buf[2] = {reg, value};
    return i2c_master_transmit(dev, buf, sizeof(buf), I2C_TIMEOUT_MS);
}

static esp_err_t read_at(uint8_t addr, uint8_t reg, uint8_t *buf, size_t len)
{
    i2c_device_config_t cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = addr,
        .scl_speed_hz = I2C_HZ,
    };
    i2c_master_dev_handle_t d;
    ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(bus, &cfg, &d), TAG, "add device");
    esp_err_t err = i2c_master_transmit_receive(d, &reg, 1, buf, len, I2C_TIMEOUT_MS);
    i2c_master_bus_rm_device(d);
    return err;
}

// Registers 0x00-0x01 identify most magnetometers: IST8310 has 0x10 at 0x00,
// AKM chips have 0x48 at 0x00 and their device ID at 0x01.
static void log_bus_scan(void)
{
    bool any = false;
    for (uint8_t addr = 0x08; addr < 0x78; addr++) {
        esp_err_t err = i2c_master_probe(bus, addr, I2C_TIMEOUT_MS);
        if (err == ESP_OK) {
            uint8_t id[2] = {0};
            esp_err_t id_err = read_at(addr, 0x00, id, sizeof(id));
            ESP_LOGW(TAG, "I2C device at 0x%02x, registers 0x00-0x01: %02x %02x (%s)", addr, id[0], id[1],
                     esp_err_to_name(id_err));
            any = true;
        } else if (err == ESP_ERR_TIMEOUT) {
            return;  // bus stuck mid-scan; the driver has already logged it
        }
    }
    if (!any) {
        ESP_LOGW(TAG, "no I2C devices on SDA GPIO%d / SCL GPIO%d", CONFIG_RGNSS_MAG_SDA_GPIO, CONFIG_RGNSS_MAG_SCL_GPIO);
    }
}

// Leaves dev, chip and chip_addr set on success.
// ESP_ERR_TIMEOUT means the bus itself is stuck rather than the chip absent.
static void load_heading_offset(void);

static esp_err_t detect(void)
{
    for (size_t i = 0; i < sizeof(chips) / sizeof(chips[0]); i++) {
        const chip_t *c = &chips[i];
        for (size_t a = 0; a < sizeof(c->addrs) && c->addrs[a]; a++) {
            esp_err_t err = i2c_master_probe(bus, c->addrs[a], I2C_TIMEOUT_MS);
            if (err == ESP_ERR_TIMEOUT) {
                return err;
            }
            if (err != ESP_OK) {
                continue;
            }
            i2c_device_config_t cfg = {
                .dev_addr_length = I2C_ADDR_BIT_LEN_7,
                .device_address = c->addrs[a],
                .scl_speed_hz = I2C_HZ,
            };
            ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(bus, &cfg, &dev), TAG, "add device");

            if (c->reset[0]) {
                write_reg(c->reset[0], c->reset[1]);
                vTaskDelay(pdMS_TO_TICKS(RESET_MS));
            }
            // The QMC5883L reports its ID reliably only after a first read (per ArduPilot).
            uint8_t id = 0;
            read_regs(0x00, &id, 1);
            err = read_regs(c->id_reg, &id, 1);
            if (err == ESP_OK && id == c->id) {
                chip = c;
                chip_addr = c->addrs[a];
                return ESP_OK;
            }
            ESP_LOGD(TAG, "0x%02x is not a %s: ID 0x%02x (%s)", c->addrs[a], c->name, id, esp_err_to_name(err));
            i2c_master_bus_rm_device(dev);
            dev = NULL;
        }
    }
    return ESP_ERR_NOT_FOUND;
}

static void sample_task(void *arg)
{
    bool failing = false;
    TickType_t wake = xTaskGetTickCount();
    for (;;) {
        xTaskDelayUntil(&wake, pdMS_TO_TICKS(SAMPLE_MS));

        uint8_t buf[6];
        esp_err_t err = read_regs(chip->data_reg, buf, sizeof(buf));
        if (chip->trigger[0]) {
            write_reg(chip->trigger[0], chip->trigger[1]);  // next sample, ready well before the next read
        }
        if (err != ESP_OK) {
            if (!failing) {
                ESP_LOGW(TAG, "read failed: %s", esp_err_to_name(err));
                failing = true;
            }
            continue;
        }
        if (failing) {
            ESP_LOGI(TAG, "reading again");
            failing = false;
        }

        int16_t raw[3];
        float field_ut[3];
        for (int i = 0; i < 3; i++) {
            raw[i] = (int16_t)(buf[2 * i] | buf[2 * i + 1] << 8);
            field_ut[i] = raw[i] * chip->ut_per_count;
        }
        int64_t now = esp_timer_get_time();

        taskENTER_CRITICAL(&lock);
        memcpy(data.raw, raw, sizeof(raw));
        memcpy(data.field_ut, field_ut, sizeof(field_ut));
        data.updated_us = now;
        magnetometer_sample_t *h = &history[next_seq % MAGNETOMETER_HISTORY];
        h->seq = next_seq++;
        h->t_ms = (uint32_t)(now / 1000);
        memcpy(h->raw, raw, sizeof(raw));
        taskEXIT_CRITICAL(&lock);
    }
}

esp_err_t magnetometer_start(void)
{
    i2c_master_bus_config_t cfg = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = CONFIG_RGNSS_MAG_SDA_GPIO,
        .scl_io_num = CONFIG_RGNSS_MAG_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_RETURN_ON_ERROR(i2c_new_master_bus(&cfg, &bus), TAG, "create I2C bus");

    esp_err_t err = detect();
    if (err == ESP_ERR_TIMEOUT) {
        // Both lines are pulled up, so a low one is being held down, e.g. by an unpowered module.
        ESP_LOGE(TAG, "I2C bus stuck: SDA GPIO%d is %s, SCL GPIO%d is %s. Check the module's power and wiring.",
                 CONFIG_RGNSS_MAG_SDA_GPIO, gpio_get_level(CONFIG_RGNSS_MAG_SDA_GPIO) ? "high" : "low",
                 CONFIG_RGNSS_MAG_SCL_GPIO, gpio_get_level(CONFIG_RGNSS_MAG_SCL_GPIO) ? "high" : "low");
        return err;
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "no QMC5883P, QMC5883L or IST8310 found");
        log_bus_scan();
        return ESP_ERR_NOT_FOUND;
    }

    // On any failure below, chip goes back to NULL: a half-initialised sensor
    // must not look present to the rest of the firmware.
    for (size_t i = 0; i < chip->init_len; i++) {
        err = write_reg(chip->init[i][0], chip->init[i][1]);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "write 0x%02x to register 0x%02x: %s", chip->init[i][1], chip->init[i][0],
                     esp_err_to_name(err));
            goto fail;
        }
    }
    if (chip->trigger[0] && (err = write_reg(chip->trigger[0], chip->trigger[1])) != ESP_OK) {
        ESP_LOGE(TAG, "trigger first sample: %s", esp_err_to_name(err));
        goto fail;
    }
    data.chip = chip->name;

    if (xTaskCreate(sample_task, "mag", 3072, NULL, 5, NULL) != pdPASS) {
        ESP_LOGE(TAG, "create task");
        data.chip = NULL;
        err = ESP_ERR_NO_MEM;
        goto fail;
    }
    load_heading_offset();
    ESP_LOGI(TAG, "%s at 0x%02x, sampling every %d ms", chip->name, chip_addr, SAMPLE_MS);
    return ESP_OK;

fail:
    i2c_master_bus_rm_device(dev);
    dev = NULL;
    chip = NULL;
    return err;
}

void magnetometer_get(magnetometer_data_t *out)
{
    taskENTER_CRITICAL(&lock);
    *out = data;
    taskEXIT_CRITICAL(&lock);
}

size_t magnetometer_samples_since(uint32_t since, magnetometer_sample_t *out, size_t max, uint32_t *latest)
{
    size_t n = 0;
    taskENTER_CRITICAL(&lock);
    uint32_t first = since + 1;
    if (next_seq > MAGNETOMETER_HISTORY && first < next_seq - MAGNETOMETER_HISTORY) {
        first = next_seq - MAGNETOMETER_HISTORY;  // older ones are overwritten
    }
    for (uint32_t s = first; s < next_seq && n < max; s++) {
        out[n++] = history[s % MAGNETOMETER_HISTORY];
    }
    *latest = next_seq - 1;
    taskEXIT_CRITICAL(&lock);
    return n;
}

// Where the user wants the compass to read north: added to the measured
// heading. Set from the page, since only the user knows how the board is
// mounted, and kept in NVS so it survives a restart.
static float heading_offset;

static void load_heading_offset(void)
{
    nvs_handle_t nvs;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs) != ESP_OK) {
        return;
    }
    size_t len = sizeof(heading_offset);
    if (nvs_get_blob(nvs, NVS_KEY_OFFSET, &heading_offset, &len) != ESP_OK || len != sizeof(heading_offset)) {
        heading_offset = 0.0f;
    }
    nvs_close(nvs);
    if (heading_offset != 0.0f) {
        ESP_LOGI(TAG, "heading offset %.1f degrees", heading_offset);
    }
}

float magnetometer_heading_offset(void)
{
    return heading_offset;
}

esp_err_t magnetometer_set_heading_offset(float degrees)
{
    degrees = fmodf(degrees, 360.0f);
    if (degrees < 0.0f) {
        degrees += 360.0f;
    }
    nvs_handle_t nvs;
    ESP_RETURN_ON_ERROR(nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs), TAG, "open NVS");
    esp_err_t err = nvs_set_blob(nvs, NVS_KEY_OFFSET, &degrees, sizeof(degrees));
    if (err == ESP_OK) {
        err = nvs_commit(nvs);
    }
    nvs_close(nvs);
    ESP_RETURN_ON_ERROR(err, TAG, "save heading offset");
    heading_offset = degrees;
    ESP_LOGI(TAG, "heading offset set to %.1f degrees", degrees);
    return ESP_OK;
}

float magnetometer_ut_per_count(void)
{
    return chip ? chip->ut_per_count : 0.0f;
}
