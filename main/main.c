#include <stdio.h>
#include <string.h>

#include "driver/i2c_master.h"
#include "driver/uart.h"
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
#define PMS5003_UART_NUM    UART_NUM_2
#define PMS5003_RX_GPIO     16
#define PMS5003_TX_GPIO     17
#define PMS5003_BAUD_RATE   9600
#define PMS5003_FRAME_SIZE  32
#define PMS5003_FRAME_LENGTH 28
#define SSD1315_ADDRESS_A   0x3C
#define SSD1315_ADDRESS_B   0x3D
#define SSD1315_WIDTH       128
#define SSD1315_HEIGHT      64
#define SSD1315_COLUMN_OFFSET 0

static const char *TAG = "sensors";

typedef struct {
    uint16_t pm1_0;
    uint16_t pm2_5;
    uint16_t pm10;
    TickType_t last_update;
    bool valid;
} pms5003_data_t;

static pms5003_data_t pms5003_data;
static portMUX_TYPE pms5003_data_lock = portMUX_INITIALIZER_UNLOCKED;

// 5x7 glyphs are stored by column, with one extra column between characters.
static const uint8_t font5x7[][5] = {
    [' '] = {0x00, 0x00, 0x00, 0x00, 0x00},
    ['%'] = {0x62, 0x64, 0x08, 0x13, 0x23},
    ['.'] = {0x00, 0x60, 0x60, 0x00, 0x00},
    [':'] = {0x00, 0x36, 0x36, 0x00, 0x00},
    ['-'] = {0x08, 0x08, 0x08, 0x08, 0x08},
    ['0'] = {0x3E, 0x51, 0x49, 0x45, 0x3E},
    ['1'] = {0x00, 0x42, 0x7F, 0x40, 0x00},
    ['2'] = {0x42, 0x61, 0x51, 0x49, 0x46},
    ['3'] = {0x21, 0x41, 0x45, 0x4B, 0x31},
    ['4'] = {0x18, 0x14, 0x12, 0x7F, 0x10},
    ['5'] = {0x27, 0x45, 0x45, 0x45, 0x39},
    ['6'] = {0x3C, 0x4A, 0x49, 0x49, 0x30},
    ['7'] = {0x01, 0x71, 0x09, 0x05, 0x03},
    ['8'] = {0x36, 0x49, 0x49, 0x49, 0x36},
    ['9'] = {0x06, 0x49, 0x49, 0x29, 0x1E},
    ['A'] = {0x7E, 0x11, 0x11, 0x11, 0x7E},
    ['B'] = {0x7F, 0x49, 0x49, 0x49, 0x36},
    ['C'] = {0x3E, 0x41, 0x41, 0x41, 0x22},
    ['D'] = {0x7F, 0x41, 0x41, 0x22, 0x1C},
    ['E'] = {0x7F, 0x49, 0x49, 0x49, 0x41},
    ['G'] = {0x3E, 0x41, 0x49, 0x49, 0x7A},
    ['H'] = {0x7F, 0x08, 0x08, 0x08, 0x7F},
    ['I'] = {0x00, 0x41, 0x7F, 0x41, 0x00},
    ['L'] = {0x7F, 0x40, 0x40, 0x40, 0x40},
    ['M'] = {0x7F, 0x02, 0x0C, 0x02, 0x7F},
    ['P'] = {0x7F, 0x09, 0x09, 0x09, 0x06},
    ['R'] = {0x7F, 0x09, 0x19, 0x29, 0x46},
    ['T'] = {0x01, 0x01, 0x7F, 0x01, 0x01},
    ['U'] = {0x3F, 0x40, 0x40, 0x40, 0x3F},
    ['X'] = {0x63, 0x14, 0x08, 0x14, 0x63},
    ['Y'] = {0x03, 0x04, 0x78, 0x04, 0x03},
};

// Each page spans 8 pixel rows; each byte holds one column of 8 pixels.
static uint8_t ssd1315_framebuffer[SSD1315_HEIGHT / 8][SSD1315_WIDTH];

static esp_err_t ssd1315_command(i2c_master_dev_handle_t device, const uint8_t *commands,
                                 size_t length)
{
    uint8_t buffer[32];
    if (length + 1 > sizeof(buffer)) {
        return ESP_ERR_INVALID_SIZE;
    }
    buffer[0] = 0x00; // I2C control byte: following bytes are commands.
    memcpy(buffer + 1, commands, length);
    return i2c_master_transmit(device, buffer, length + 1, 100);
}

static esp_err_t ssd1315_init(i2c_master_dev_handle_t device)
{
    static const uint8_t commands[] = {
        0xAE,       // display off
        0xD5, 0x80, // clock divide ratio / oscillator
        0xA8, 0x3F, // 64 multiplex
        0xD3, 0x00, // display offset
        0x40,       // display start line 0
        0x8D, 0x14, // enable internal charge pump, 7.5 V mode
        0xAD, 0x30, // enable internal IREF, 30 uA setting
        0xA1,       // segment remap
        0xC8,       // COM scan direction remap
        0xDA, 0x12, // COM pins configuration
        0x81, 0x80, // contrast
        0xD9, 0xF1, // pre-charge
        0xDB, 0x20, // VCOMH deselect level (~0.77 x VCC)
        0xA4,       // display follows RAM
        0xA6,       // normal display
        0x20, 0x02, // page addressing mode
        0xAF        // display on
    };
    memset(ssd1315_framebuffer, 0, sizeof(ssd1315_framebuffer));
    return ssd1315_command(device, commands, sizeof(commands));
}

static void ssd1315_clear(void)
{
    memset(ssd1315_framebuffer, 0, sizeof(ssd1315_framebuffer));
}

static void ssd1315_draw_text(uint8_t x, uint8_t page, const char *text)
{
    while (*text != '\0' && x + 5 < SSD1315_WIDTH) {
        unsigned char character = (unsigned char)*text++;
        if (character < sizeof(font5x7) / sizeof(font5x7[0])) {
            memcpy(&ssd1315_framebuffer[page][x], font5x7[character], 5);
        }
        x += 6;
    }
}

static void ssd1315_draw_right_text(uint8_t page, const char *text)
{
    // Calculate the starting column from the text width for right alignment.
    size_t length = strlen(text);
    uint8_t width = (uint8_t)(length * 6);
    ssd1315_draw_text(width < SSD1315_WIDTH ? SSD1315_WIDTH - width : 0, page, text);
}

static esp_err_t ssd1315_refresh(i2c_master_dev_handle_t device)
{
    uint8_t commands[4];
    uint8_t data[SSD1315_WIDTH + 1];
    data[0] = 0x40; // I2C control byte: following bytes are display RAM data.

    for (uint8_t page = 0; page < SSD1315_HEIGHT / 8; ++page) {
        commands[0] = (uint8_t)(0xB0 | page);
        commands[1] = (uint8_t)(0x00 | (SSD1315_COLUMN_OFFSET & 0x0F));
        commands[2] = (uint8_t)(0x10 | (SSD1315_COLUMN_OFFSET >> 4));
        ESP_RETURN_ON_ERROR(ssd1315_command(device, commands, 3), TAG,
                            "failed to set SSD1315 page");
        memcpy(data + 1, ssd1315_framebuffer[page], SSD1315_WIDTH);
        ESP_RETURN_ON_ERROR(i2c_master_transmit(device, data, sizeof(data), 100), TAG,
                            "failed to write SSD1315 data");
    }
    return ESP_OK;
}

static esp_err_t ssd1315_update(i2c_master_dev_handle_t device, bool temperature_valid,
                                float temperature, bool humidity_valid, float humidity,
                                bool illuminance_valid, float illuminance,
                                const pms5003_data_t *pms_data)
{
    char value[24];
    // Redraw in memory before sending to the display; use -- for invalid readings.
    ssd1315_clear();
    ssd1315_draw_text(0, 0, "TEMPERATURE");
    ssd1315_draw_text(0, 1, "HUMIDITY");
    ssd1315_draw_text(0, 2, "LIGHT");
    ssd1315_draw_text(0, 3, "PM1.0");
    ssd1315_draw_text(0, 4, "PM2.5");
    ssd1315_draw_text(0, 5, "PM10");

    snprintf(value, sizeof(value), temperature_valid ? "%.1f C" : "--", temperature);
    ssd1315_draw_right_text(0, value);
    snprintf(value, sizeof(value), humidity_valid ? "%.1f %%" : "--", humidity);
    ssd1315_draw_right_text(1, value);
    snprintf(value, sizeof(value), illuminance_valid ? "%.0f LUX" : "--", illuminance);
    ssd1315_draw_right_text(2, value);
    if (pms_data->valid) {
        snprintf(value, sizeof(value), "%u", pms_data->pm1_0);
        ssd1315_draw_right_text(3, value);
        snprintf(value, sizeof(value), "%u", pms_data->pm2_5);
        ssd1315_draw_right_text(4, value);
        snprintf(value, sizeof(value), "%u", pms_data->pm10);
        ssd1315_draw_right_text(5, value);
    } else {
        ssd1315_draw_right_text(3, "--");
        ssd1315_draw_right_text(4, "--");
        ssd1315_draw_right_text(5, "--");
    }
    return ssd1315_refresh(device);
}

static uint8_t sht31_crc8(const uint8_t *data, size_t length)
{
    // SHT31 CRC-8: initial value 0xFF, polynomial 0x31.
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

    // High-repeatability mode without clock stretching takes up to about 15 ms.
    vTaskDelay(pdMS_TO_TICKS(20));

    ESP_RETURN_ON_ERROR(i2c_master_receive(device, data, sizeof(data), 100), TAG,
                        "failed to read measurement");

    // Temperature and humidity each use two bytes followed by a CRC byte.
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

    // Wait up to 180 ms for a one-time high-resolution measurement.
    vTaskDelay(pdMS_TO_TICKS(180));

    ESP_RETURN_ON_ERROR(i2c_master_receive(device, data, sizeof(data), 100), TAG,
                        "failed to read BH1750 measurement");

    uint16_t raw_illuminance = ((uint16_t)data[0] << 8) | data[1];
    *illuminance = (float)raw_illuminance / 1.2f; // Convert to lux at the default measurement time.
    return ESP_OK;
}

static void pms5003_parser_reset(uint8_t *frame, size_t *position)
{
    (void)frame;
    *position = 0;
}

static void pms5003_process_frame(const uint8_t *frame)
{
    uint16_t checksum = 0;
    for (size_t i = 0; i < PMS5003_FRAME_SIZE - 2; ++i) {
        checksum += frame[i];
    }

    // Sum the first 30 bytes and compare with the final two checksum bytes.
    uint16_t received_checksum = ((uint16_t)frame[30] << 8) | frame[31];
    if (checksum != received_checksum) {
        ESP_LOGW(TAG, "PMS5003 checksum error");
        return;
    }

    // Atmospheric PM concentrations (Data 4/5/6) in ug/m3; fields are big-endian.
    uint16_t pm1_0 = ((uint16_t)frame[10] << 8) | frame[11];
    uint16_t pm2_5 = ((uint16_t)frame[12] << 8) | frame[13];
    uint16_t pm10 = ((uint16_t)frame[14] << 8) | frame[15];

    portENTER_CRITICAL(&pms5003_data_lock);
    pms5003_data.pm1_0 = pm1_0;
    pms5003_data.pm2_5 = pm2_5;
    pms5003_data.pm10 = pm10;
    pms5003_data.last_update = xTaskGetTickCount();
    pms5003_data.valid = true;
    portEXIT_CRITICAL(&pms5003_data_lock);
}

static void pms5003_parser_feed(uint8_t byte, uint8_t *frame, size_t *position)
{
    // Find header 0x42 0x4D, check length, then validate and parse a complete frame.
    if (*position == 0) {
        if (byte == 0x42) {
            frame[(*position)++] = byte;
        }
        return;
    }

    if (*position == 1) {
        if (byte == 0x4D) {
            frame[(*position)++] = byte;
        } else {
            pms5003_parser_reset(frame, position);
            if (byte == 0x42) {
                frame[(*position)++] = byte;
            }
        }
        return;
    }

    frame[(*position)++] = byte;
    if (*position == 4) {
        uint16_t frame_length = ((uint16_t)frame[2] << 8) | frame[3];
        if (frame_length != PMS5003_FRAME_LENGTH) {
            pms5003_parser_reset(frame, position);
        }
    } else if (*position == PMS5003_FRAME_SIZE) {
        pms5003_process_frame(frame);
        pms5003_parser_reset(frame, position);
    }
}

static void pms5003_task(void *arg)
{
    (void)arg;
    uint8_t receive_buffer[128];
    uint8_t frame[PMS5003_FRAME_SIZE] = {0};
    size_t position = 0; // Keep parser state across reads to handle split and consecutive frames.

    while (true) {
        int received = uart_read_bytes(PMS5003_UART_NUM, receive_buffer, sizeof(receive_buffer),
                                       pdMS_TO_TICKS(100));
        for (int i = 0; i < received; ++i) {
            pms5003_parser_feed(receive_buffer[i], frame, &position);
        }
    }
}

static esp_err_t pms5003_uart_init(void)
{
    const uart_config_t uart_config = {
        .baud_rate = PMS5003_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_RETURN_ON_ERROR(uart_param_config(PMS5003_UART_NUM, &uart_config), TAG,
                        "failed to configure PMS5003 UART");
    ESP_RETURN_ON_ERROR(uart_set_pin(PMS5003_UART_NUM, PMS5003_TX_GPIO, PMS5003_RX_GPIO,
                                     UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE), TAG,
                        "failed to set PMS5003 UART pins");
    ESP_RETURN_ON_ERROR(uart_driver_install(PMS5003_UART_NUM, 1024, 0, 0, NULL, 0), TAG,
                        "failed to install PMS5003 UART driver");
    return ESP_OK;
}

void app_main(void)
{
    // SHT31, BH1750 and OLED share one I2C bus, accessed sequentially by this task.
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = I2C_SDA_GPIO,
        .scl_io_num = I2C_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        // The SHT31 module has external pull-ups, so internal pull-ups are disabled.
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

    i2c_master_dev_handle_t ssd1315_device = NULL;
    uint8_t ssd1315_address = SSD1315_ADDRESS_A;
    esp_err_t ssd1315_error = i2c_master_probe(bus, SSD1315_ADDRESS_A, 100);
    if (ssd1315_error != ESP_OK) {
        ssd1315_address = SSD1315_ADDRESS_B;
        ssd1315_error = i2c_master_probe(bus, SSD1315_ADDRESS_B, 100);
    }
    if (ssd1315_error == ESP_OK) {
        i2c_device_config_t ssd1315_config = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = ssd1315_address,
            .scl_speed_hz = I2C_FREQUENCY_HZ,
        };
        ssd1315_error = i2c_master_bus_add_device(bus, &ssd1315_config, &ssd1315_device);
        if (ssd1315_error == ESP_OK) {
            ssd1315_error = ssd1315_init(ssd1315_device);
        }
        if (ssd1315_error == ESP_OK) {
            ESP_LOGI(TAG, "SSD1315 detected at I2C address 0x%02X", ssd1315_address);
        }
    }
    if (ssd1315_error != ESP_OK) {
        ESP_LOGW(TAG, "SSD1315 unavailable at 0x%02X/0x%02X: %s", SSD1315_ADDRESS_A,
                 SSD1315_ADDRESS_B, esp_err_to_name(ssd1315_error));
    }

    esp_err_t pms5003_error = pms5003_uart_init();
    if (pms5003_error == ESP_OK) {
        // Receive UART data in a separate task to avoid blocking the main task.
        xTaskCreate(pms5003_task, "pms5003_task", 3072, NULL, 5, NULL);
        ESP_LOGI(TAG, "PMS5003 UART2 ready: RX GPIO%d, TX GPIO%d", PMS5003_RX_GPIO,
                 PMS5003_TX_GPIO);
    } else {
        ESP_LOGW(TAG, "PMS5003 unavailable: %s", esp_err_to_name(pms5003_error));
    }

    while (true) {
        float temperature = 0.0f;
        float humidity = 0.0f;
        float illuminance = 0.0f;
        esp_err_t sht31_error = sht31_read(device, &temperature, &humidity);
        esp_err_t bh1750_read_error = ESP_ERR_NOT_FOUND;
        if (bh1750_device != NULL) {
            bh1750_read_error = bh1750_read(bh1750_device, &illuminance);
        }

        if (sht31_error != ESP_OK) {
            ESP_LOGE(TAG, "SHT31 measurement failed: %s", esp_err_to_name(sht31_error));
        }
        if (bh1750_read_error != ESP_OK) {
            ESP_LOGE(TAG, "BH1750 measurement failed: %s", esp_err_to_name(bh1750_read_error));
        }

        // Copy a complete snapshot under the lock to avoid partially updated readings.
        pms5003_data_t pms_data;
        portENTER_CRITICAL(&pms5003_data_lock);
        pms_data = pms5003_data;
        portEXIT_CRITICAL(&pms5003_data_lock);
        // Treat readings as stale after 5 seconds without a valid frame.
        bool pms_available = pms_data.valid &&
                             (xTaskGetTickCount() - pms_data.last_update <= pdMS_TO_TICKS(5000));
        pms_data.valid = pms_available;

        char temperature_text[32];
        char humidity_text[32];
        char illuminance_text[32];
        char pms_text[80];
        snprintf(temperature_text, sizeof(temperature_text),
                 sht31_error == ESP_OK ? "%.2f C" : "unavailable", temperature);
        snprintf(humidity_text, sizeof(humidity_text),
                 sht31_error == ESP_OK ? "%.2f %%RH" : "unavailable", humidity);
        snprintf(illuminance_text, sizeof(illuminance_text),
                 bh1750_read_error == ESP_OK ? "%.2f lux" : "unavailable", illuminance);
        if (pms_available) {
            snprintf(pms_text, sizeof(pms_text), "PM1.0: %u, PM2.5: %u, PM10: %u ug/m3",
                     pms_data.pm1_0, pms_data.pm2_5, pms_data.pm10);
        } else {
            snprintf(pms_text, sizeof(pms_text), "PMS5003: waiting for valid data");
        }
        ESP_LOGI(TAG, "temperature: %s, humidity: %s, illuminance: %s, ambient %s",
                 temperature_text, humidity_text, illuminance_text, pms_text);

        if (ssd1315_device != NULL) {
            esp_err_t display_error = ssd1315_update(ssd1315_device, sht31_error == ESP_OK,
                                                     temperature, sht31_error == ESP_OK, humidity,
                                                     bh1750_read_error == ESP_OK, illuminance,
                                                     &pms_data);
            if (display_error != ESP_OK) {
                ESP_LOGE(TAG, "SSD1315 refresh failed: %s", esp_err_to_name(display_error));
            }
        }

        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}
