#include <stdio.h>

#include "driver/i2c_master.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define I2C_SDA_GPIO        21
#define I2C_SCL_GPIO        22
#define I2C_FREQUENCY_HZ    100000
#define SHT31_ADDRESS_A     0x44
#define SHT31_ADDRESS_B     0x45
#define SHT31_MEASURE_CMD   0x2400
#define BH1750_ADDRESS      0x23
#define BH1750_POWER_ON_CMD 0x01
#define BH1750_MEASURE_CMD  0x20

static const char *TAG = "sht31";

static uint8_t sht31_crc8(const uint8_t *data, size_t length)
{
    uint8_t crc = 0xFF;

    for (size_t i = 0; i < length; ++i) {
        crc ^= data[i];
        for (int bit = 0; bit < 8; ++bit) {
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x31) : (uint8_t)(crc << 1);
        }
    }
    return crc;
}

static esp_err_t sht31_read(i2c_master_dev_handle_t device, float *temperature, float *humidity)
{
    const uint8_t command[] = {
        (uint8_t)(SHT31_MEASURE_CMD >> 8),
        (uint8_t)(SHT31_MEASURE_CMD & 0xFF),
    };
    uint8_t data[6] = {0};

    ESP_RETURN_ON_ERROR(i2c_master_transmit(device, command, sizeof(command), 100), TAG,
                        "failed to start measurement");

    // High-repeatability, no-clock-stretching measurement needs up to about 15 ms.
    vTaskDelay(pdMS_TO_TICKS(20));

    ESP_RETURN_ON_ERROR(i2c_master_receive(device, data, sizeof(data), 100), TAG,
                        "failed to read measurement");

    if (sht31_crc8(data, 2) != data[2] || sht31_crc8(data + 3, 2) != data[5]) {
        return ESP_ERR_INVALID_CRC;
    }

    uint16_t raw_temperature = ((uint16_t)data[0] << 8) | data[1];
    uint16_t raw_humidity = ((uint16_t)data[3] << 8) | data[4];
    *temperature = -45.0f + 175.0f * ((float)raw_temperature / 65535.0f);
    *humidity = 100.0f * ((float)raw_humidity / 65535.0f);
    return ESP_OK;
}

static esp_err_t bh1750_read(i2c_master_dev_handle_t device, float *illuminance)
{
    const uint8_t power_on_command = BH1750_POWER_ON_CMD;
    const uint8_t measure_command = BH1750_MEASURE_CMD;
    uint8_t data[2] = {0};

    ESP_RETURN_ON_ERROR(i2c_master_transmit(device, &power_on_command, 1, 100), TAG,
                        "failed to power on BH1750");
    ESP_RETURN_ON_ERROR(i2c_master_transmit(device, &measure_command, 1, 100), TAG,
                        "failed to start BH1750 measurement");

    // One-time high-resolution measurement takes up to about 180 ms.
    vTaskDelay(pdMS_TO_TICKS(180));

    ESP_RETURN_ON_ERROR(i2c_master_receive(device, data, sizeof(data), 100), TAG,
                        "failed to read BH1750 measurement");

    uint16_t raw_illuminance = ((uint16_t)data[0] << 8) | data[1];
    *illuminance = (float)raw_illuminance / 1.2f;
    return ESP_OK;
}

void app_main(void)
{
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = I2C_SDA_GPIO,
        .scl_io_num = I2C_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        // The SHT31 module already has I2C pull-up resistors fitted.
        .flags.enable_internal_pullup = false,
    };
    i2c_master_bus_handle_t bus = NULL;
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &bus));

    uint8_t address = SHT31_ADDRESS_A;
    esp_err_t probe_error = i2c_master_probe(bus, SHT31_ADDRESS_A, 100);
    if (probe_error != ESP_OK) {
        probe_error = i2c_master_probe(bus, SHT31_ADDRESS_B, 100);
        address = SHT31_ADDRESS_B;
    }
    if (probe_error != ESP_OK) {
        ESP_LOGE(TAG, "SHT31 not found at 0x44 or 0x45: %s", esp_err_to_name(probe_error));
        return;
    }

    i2c_device_config_t device_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = address,
        .scl_speed_hz = I2C_FREQUENCY_HZ,
    };
    i2c_master_dev_handle_t device = NULL;
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus, &device_config, &device));
    ESP_LOGI(TAG, "SHT31 detected at I2C address 0x%02X", address);

    i2c_master_dev_handle_t bh1750_device = NULL;
    esp_err_t bh1750_error = i2c_master_probe(bus, BH1750_ADDRESS, 100);
    if (bh1750_error == ESP_OK) {
        i2c_device_config_t bh1750_config = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = BH1750_ADDRESS,
            .scl_speed_hz = I2C_FREQUENCY_HZ,
        };
        bh1750_error = i2c_master_bus_add_device(bus, &bh1750_config, &bh1750_device);
        if (bh1750_error == ESP_OK) {
            ESP_LOGI(TAG, "BH1750 detected at I2C address 0x%02X", BH1750_ADDRESS);
        }
    }
    if (bh1750_error != ESP_OK) {
        ESP_LOGW(TAG, "BH1750 unavailable at 0x%02X: %s", BH1750_ADDRESS,
                 esp_err_to_name(bh1750_error));
    }

    while (true) {
        float temperature = 0.0f;
        float humidity = 0.0f;
        esp_err_t error = sht31_read(device, &temperature, &humidity);
        if (error == ESP_OK) {
            if (bh1750_device != NULL) {
                float illuminance = 0.0f;
                esp_err_t bh1750_read_error = bh1750_read(bh1750_device, &illuminance);
                if (bh1750_read_error == ESP_OK) {
                    ESP_LOGI(TAG, "temperature: %.2f C, humidity: %.2f %%RH, illuminance: %.2f lux",
                             temperature, humidity, illuminance);
                } else {
                    ESP_LOGE(TAG, "BH1750 measurement failed: %s", esp_err_to_name(bh1750_read_error));
                    ESP_LOGI(TAG, "temperature: %.2f C, humidity: %.2f %%RH, illuminance: unavailable",
                             temperature, humidity);
                }
            } else {
                ESP_LOGI(TAG, "temperature: %.2f C, humidity: %.2f %%RH, illuminance: unavailable",
                         temperature, humidity);
            }
        } else {
            ESP_LOGE(TAG, "measurement failed: %s", esp_err_to_name(error));
        }
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}
