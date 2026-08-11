#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "mqtt_client.h"
#include "esp_task_wdt.h"

// --- ТЕГИ ЛОГИРОВАНИЯ ---
static const char *TAG_CAN = "CAN_BUS";
static const char *TAG_RS485 = "RS485_PROXY";
static const char *TAG_WIFI = "WIFI_STA";
static const char *TAG_MQTT = "MQTT_PUB";

// --- НАСТРОЙКИ WI-FI И MQTT ---
#define WIFI_SSID      "SSID"
#define WIFI_PASS      "password"
#define WIFI_MAX_RETRY 10

#define MQTT_BROKER_URI "mqtt://192.168.1.100:1883"
#define MQTT_TOPIC "solar/battery/state"
#define MQTT_USER "" // Оставьте пустым, если авторизация не нужна
#define MQTT_PASS ""

// Настройки Home Assistant Discovery
#define HA_DISCOVERY_PREFIX "homeassistant"
#define HA_NODE_ID "solar_battery"
// Единый блок описания устройства для всех сенсоров (чтобы сгруппировать их в HA)
#define HA_DEVICE_DICT "\"device\":{\"identifiers\":[\"esp32_solar_bat\"],\"name\":\"Solar Battery\",\"manufacturer\":\"ESP32 Proxy\"}"

static int s_retry_num = 0;

// --- НАСТРОЙКИ ОБОРУДОВАНИЯ ---

// RS485 (UART)
#define SERIAL_PORT UART_NUM_2
#define BAUDRATE 9600
#define TX_PIN 17
#define RX_PIN 16

// CAN (SPI MCP2515)
#define PIN_NUM_MISO 19
#define PIN_NUM_MOSI 23
#define PIN_NUM_CLK  18
#define PIN_NUM_CS   5

// --- ПРОТОКОЛ PYLONTECH (RS485) ---
#define SOI '~'
#define EOI 0x0D
#define VER "20"
#define ADR "02"
#define CID1 "46"
#define RTN_NORMAL "00"

// --- НАСТРОЙКИ CAN ---
#define CAN_SFF_MASK 0x000007FFU

// Регистры MCP2515
#define MCP_RXF0SIDH    0x00
#define MCP_CANINTF     0x2C
#define MCP_CANCTRL     0x0F
#define MCP_CANSTAT     0x0E
#define MCP_CNF1        0x2A
#define MCP_CNF2        0x29
#define MCP_CNF3        0x28
#define MCP_READ        0x03
#define MCP_WRITE       0x02
#define MCP_BITMOD      0x05
#define MCP_RESET       0xC0
#define MCP_RTS_TX0     0x81
#define MCP_TXB0CTRL    0x30
#define MCP_TXB0SIDH    0x31

// --- СТРУКТУРЫ ДАННЫХ ---

typedef struct {
    // 61H Data
    uint16_t voltage;       // 0.01V unit (5000 = 50.00V)
    int16_t  current;       // 0.1A unit (Signed)
    uint8_t  soc;           // %
    uint8_t  avg_soh;       // %
    uint8_t  min_soh;       // %
    uint16_t cycle_count;

    // Cell Info
    uint16_t cell_max;      // mV
    uint16_t cell_min;      // mV
    uint16_t cell_max_module;
    uint16_t cell_min_module;

    // Temp Info
    uint16_t temp_cell_avg;
    uint16_t temp_cell_max;
    uint16_t temp_cell_min;
    uint16_t temp_cell_max_module;
    uint16_t temp_cell_min_module;

    // BMS Temp
    uint16_t temp_bms_avg;
    uint16_t temp_bms_max;
    uint16_t temp_bms_min;
    uint16_t temp_bms_max_module;
    uint16_t temp_bms_min_module;

    // 63H Data (Limits)
    uint32_t charge_vol_limit;    // mV
    uint32_t discharge_vol_limit; // mV
    int16_t  max_chg_current;     // 0.1A
    int16_t  max_dis_current;     // 0.1A (signed)
    uint8_t  status;              // Status Flags
} BatteryState;

// CAN Frame
typedef struct {
    uint32_t can_id;
    uint8_t  dlc;
    uint8_t  data[8];
} can_frame_t;

// --- ГЛОБАЛЬНЫЕ ПЕРЕМЕННЫЕ ---
BatteryState bat;
SemaphoreHandle_t batMutex;
spi_device_handle_t spi;

static uint8_t *spi_tx_buf_dma;
static uint8_t *spi_rx_buf_dma;

// ============================================================================
//                                   ЧАСТЬ 1: ДРАЙВЕР CAN (MCP2515)
// ============================================================================

void mcp2515_reset() {
    spi_transaction_t t = { .length = 8, .flags = SPI_TRANS_USE_TXDATA };
    t.tx_data[0] = MCP_RESET;
    ESP_ERROR_CHECK(spi_device_transmit(spi, &t));
    vTaskDelay(pdMS_TO_TICKS(10));
}

void mcp2515_write_reg(uint8_t reg, uint8_t value) {
    spi_transaction_t t = { .length = 24, .flags = SPI_TRANS_USE_TXDATA };
    t.tx_data[0] = MCP_WRITE;
    t.tx_data[1] = reg;
    t.tx_data[2] = value;
    ESP_ERROR_CHECK(spi_device_transmit(spi, &t));
}

void mcp2515_modify_reg(uint8_t reg, uint8_t mask, uint8_t data) {
    spi_transaction_t t = { .length = 32, .flags = SPI_TRANS_USE_TXDATA };
    t.tx_data[0] = MCP_BITMOD;
    t.tx_data[1] = reg;
    t.tx_data[2] = mask;
    t.tx_data[3] = data;
    ESP_ERROR_CHECK(spi_device_transmit(spi, &t));
}

uint8_t mcp2515_read_reg(uint8_t reg) {
    spi_transaction_t t = { .length = 24, .flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA };
    t.tx_data[0] = MCP_READ;
    t.tx_data[1] = reg;
    t.tx_data[2] = 0x00;
    ESP_ERROR_CHECK(spi_device_transmit(spi, &t));
    return t.rx_data[2];
}

void mcp2515_send_frame(uint16_t id, const uint8_t *data, uint8_t len) {
    if (len > 8) len = 8;

    memset(spi_tx_buf_dma, 0, 16);
    spi_tx_buf_dma[0] = MCP_WRITE;
    spi_tx_buf_dma[1] = MCP_TXB0SIDH;
    spi_tx_buf_dma[2] = (uint8_t)(id >> 3);
    spi_tx_buf_dma[3] = (uint8_t)((id & 0x07) << 5);
    spi_tx_buf_dma[6] = len;
    memcpy(&spi_tx_buf_dma[7], data, len);

    spi_transaction_t t = {
        .length = (7 + len) * 8,
        .tx_buffer = spi_tx_buf_dma
    };
    ESP_ERROR_CHECK(spi_device_transmit(spi, &t));

    spi_transaction_t t_rts = { .length = 8, .flags = SPI_TRANS_USE_TXDATA };
    t_rts.tx_data[0] = MCP_RTS_TX0;
    ESP_ERROR_CHECK(spi_device_transmit(spi, &t_rts));
}


void update_battery_from_can(const can_frame_t *frame) {
    if (xSemaphoreTake(batMutex, portMAX_DELAY) == pdTRUE) {
        switch (frame->can_id) {
            case 0x351:
                bat.charge_vol_limit = ((frame->data[1] << 8) | frame->data[0]) * 100;
                bat.max_chg_current  = (int16_t)((frame->data[3] << 8) | frame->data[2]);
                bat.max_dis_current  = (int16_t)((frame->data[5] << 8) | frame->data[4]);
                if (bat.discharge_vol_limit == 0) bat.discharge_vol_limit = 45000;
                break;

            case 0x355:
                bat.soc     = (frame->data[1] << 8) | frame->data[0];
                bat.avg_soh = (frame->data[3] << 8) | frame->data[2];
                bat.min_soh = bat.avg_soh;
                break;

            case 0x356:
                bat.voltage = ((frame->data[1] << 8) | frame->data[0]);
                // Ток разряда положительный, заряд отрицательный (инверсия для RS485)
                bat.current = -(int16_t)((frame->data[3] << 8) | frame->data[2]);
                bat.temp_cell_avg = (int16_t)((frame->data[5] << 8) | frame->data[4]);

                uint16_t avg_cell_mv = (bat.voltage * 10) / 15;
                bat.cell_max = avg_cell_mv;
                bat.cell_min = avg_cell_mv;
                bat.temp_cell_max = bat.temp_cell_avg;
                bat.temp_cell_min = bat.temp_cell_avg;
                bat.temp_bms_avg = bat.temp_cell_avg;
                break;

            case 0x35C:
                bat.status = frame->data[0];
                break;
        }
        xSemaphoreGive(batMutex);
    }
}


void mcp2515_read_and_process() {
    memset(spi_tx_buf_dma, 0, 15);
    memset(spi_rx_buf_dma, 0, 15);
    spi_tx_buf_dma[0] = MCP_READ;
    spi_tx_buf_dma[1] = 0x61;

    spi_transaction_t t = {
        .length = 15 * 8,
        .tx_buffer = spi_tx_buf_dma,
        .rx_buffer = spi_rx_buf_dma
    };
    ESP_ERROR_CHECK(spi_device_transmit(spi, &t));

    uint8_t sidh = spi_rx_buf_dma[2];
    uint8_t sidl = spi_rx_buf_dma[3];

    // Бит 3 в sidl указывает, что кадр 29-битный. игнорируем.
    if ((sidl & 0x08) != 0) {
        mcp2515_modify_reg(MCP_CANINTF, 0x01, 0x00);
        return;
    }

    can_frame_t frame;
    frame.dlc = spi_rx_buf_dma[6] & 0x0F;

    // Игнорируем короткие кадры без полных данных
    if (frame.dlc < 6) {
        mcp2515_modify_reg(MCP_CANINTF, 0x01, 0x00);
        return;
    }

    frame.can_id = (sidh << 3) | (sidl >> 5);
    memcpy(frame.data, &spi_rx_buf_dma[7], 8);

    update_battery_from_can(&frame);
    mcp2515_modify_reg(MCP_CANINTF, 0x01, 0x00);
}


void task_can_bus(void *pvParameters) {
    ESP_LOGI(TAG_CAN, "CAN Task Started");

    spi_tx_buf_dma = heap_caps_malloc(32, MALLOC_CAP_DMA);
    spi_rx_buf_dma = heap_caps_malloc(32, MALLOC_CAP_DMA);

    spi_bus_config_t buscfg = {
        .miso_io_num = PIN_NUM_MISO,
        .mosi_io_num = PIN_NUM_MOSI,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 32
    };
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 2 * 1000 * 1000, // Снижено до 2 МГц для стабильности
        .mode = 0,
        .spics_io_num = PIN_NUM_CS,
        .queue_size = 7,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO));
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &devcfg, &spi));

    mcp2515_reset();
    mcp2515_write_reg(MCP_CANCTRL, 0x80);
    mcp2515_write_reg(MCP_CNF1, 0x00);
    mcp2515_write_reg(MCP_CNF2, 0x90);
    mcp2515_write_reg(MCP_CNF3, 0x02);
    mcp2515_write_reg(0x60, 0x60);
    mcp2515_modify_reg(MCP_CANCTRL, 0xE0, 0x00);

    TickType_t last_tx_time = 0;
    const TickType_t tx_interval = pdMS_TO_TICKS(1000);
    const uint8_t keep_alive_data[8] = {0};
//Подписка на Watchdog
    esp_task_wdt_add(NULL);

    while (1) {
        esp_task_wdt_reset();
        uint8_t intf = mcp2515_read_reg(MCP_CANINTF);
        while ((intf & 0x01) != 0) {
            mcp2515_read_and_process();
            intf = mcp2515_read_reg(MCP_CANINTF);
        }

        TickType_t now = xTaskGetTickCount();
        if ((now - last_tx_time) >= tx_interval) {
            mcp2515_send_frame(0x305, keep_alive_data, 8);
            last_tx_time = now;
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// ============================================================================
//                                   ЧАСТЬ 2: RS485 PROXY
// ============================================================================

uint16_t calculate_length_field(int len) {
    uint16_t lenid = len & 0x0FFF;
    uint8_t sum = ((lenid >> 8) & 0x0F) + ((lenid >> 4) & 0x0F) + (lenid & 0x0F);
    uint8_t remainder = sum % 16;
    uint8_t lchksum = (~remainder) & 0x0F;
    lchksum += 1;
    return ((lchksum & 0x0F) << 12) | lenid;
}

void calculate_and_append_chksum(char* buffer) {
    long sum = 0;
    char* ptr = buffer + 1;
    while (*ptr) {
        sum += *ptr;
        ptr++;
    }
    uint16_t checksum = ((sum % 65536) ^ 0xFFFF) + 1;
    char chksum_str[6];
    snprintf(chksum_str, sizeof(chksum_str), "%04X", checksum);
    strcat(buffer, chksum_str);
}

void get_response_63H(char* output_buffer, size_t buf_size, const BatteryState* state) {
    char info_data[128];
    snprintf(info_data, sizeof(info_data), "%04lX%04lX%04X%04X%02X",
             state->charge_vol_limit,
             state->discharge_vol_limit,
             (uint16_t)state->max_chg_current,
             (uint16_t)state->max_dis_current,
             state->status);

    int info_len = strlen(info_data);
    uint16_t length_field = calculate_length_field(info_len);
    snprintf(output_buffer, buf_size, "%c%s%s%s%s%04X%s",
             SOI, VER, ADR, CID1, RTN_NORMAL, length_field, info_data);
    calculate_and_append_chksum(output_buffer);

    int len = strlen(output_buffer);
    if (len < buf_size - 2) {
        output_buffer[len] = EOI;
        output_buffer[len+1] = '\0';
    }
}

void get_response_92H(char* output_buffer, size_t buf_size, const BatteryState* state) {
    char info_data[128];
    snprintf(info_data, sizeof(info_data), "02%04lX%04lX%04X%04X%02X",
             state->charge_vol_limit,
             state->discharge_vol_limit,
             (uint16_t)state->max_chg_current,
             (uint16_t)state->max_dis_current,
             state->status);

    int info_len = strlen(info_data);
    uint16_t length_field = calculate_length_field(info_len);
    snprintf(output_buffer, buf_size, "%c%s%s%s%s%04X%s",
             SOI, VER, ADR, CID1, RTN_NORMAL, length_field, info_data);
    calculate_and_append_chksum(output_buffer);

    int len = strlen(output_buffer);
    if (len < buf_size - 2) {
        output_buffer[len] = EOI;
        output_buffer[len+1] = '\0';
    }
}

void get_response_61H(char* output_buffer, size_t buf_size, const BatteryState* state) {
    char info_data[512];

    // --- ДИНАМИЧЕСКОЕ ВЫЧИСЛЕНИЕ ЕМКОСТИ ---
    uint16_t total_capacity;

    // Берем ток разряда по модулю
    int16_t safe_dis_current = abs(state->max_dis_current);

    if (safe_dis_current > 0) {
        total_capacity = (uint16_t)(safe_dis_current / 5);
    } else {
        total_capacity = 100;
    }

    uint16_t remain_capacity = (total_capacity * state->soc) / 100;

    snprintf(info_data, sizeof(info_data), "%04X%04X%02X%04X%04X%02X%02X%04X%04X%04X%04X%04X%04X%04X%04X%04X%04X%04X%04X%04X%04X%04X%04X%04X%04X%04X",
             state->voltage,
             (uint16_t)state->current,
             state->soc,
             remain_capacity,
             total_capacity,
             state->avg_soh,
             state->min_soh,
             state->cell_max,
             state->cell_max_module,
             state->cell_min,
             state->cell_min_module,

// --- ПЕРЕВОДИМ 0.1 °C В 0.1 KELVIN ---
             state->temp_cell_avg + 2731,
             state->temp_cell_max + 2731,
             state->temp_cell_max_module + 2731,
             state->temp_cell_min + 2731,
             state->temp_cell_min_module + 2731,
             state->temp_cell_avg + 2731,
             state->temp_cell_max + 2731,
             state->temp_cell_max_module + 2731,
             state->temp_cell_min + 2731,
             state->temp_cell_min_module + 2731,
             state->temp_bms_avg + 2731,
             state->temp_bms_max + 2731,
             state->temp_bms_max_module + 2731,
             state->temp_bms_min + 2731,
             state->temp_bms_min_module + 2731
            );

    int info_len = strlen(info_data);
    uint16_t length_field = calculate_length_field(info_len);
    snprintf(output_buffer, buf_size, "%c%s%s%s%s%04X%s",
             SOI, VER, ADR, CID1, RTN_NORMAL, length_field, info_data);
    calculate_and_append_chksum(output_buffer);

    int len = strlen(output_buffer);
    if (len < buf_size - 2) {
        output_buffer[len] = EOI;
        output_buffer[len+1] = '\0';
    }
}

void task_rs485_proxy(void *pvParameters) {
    ESP_LOGI(TAG_RS485, "RS485 Task Started");

    uart_config_t cfg = {
        .baud_rate = BAUDRATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    uart_param_config(SERIAL_PORT, &cfg);

    uart_set_pin(SERIAL_PORT, TX_PIN, RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(SERIAL_PORT, 2048, 2048, 0, NULL, 0);
    uart_set_mode(SERIAL_PORT, UART_MODE_UART);


    char rx_buf[1024];
    int rx_idx = 0;
    char tx_buf[1024];
    BatteryState local_bat;
//Подписка на Watchdog
    esp_task_wdt_add(NULL);

    while (1) {

// Сброс таймера
        esp_task_wdt_reset();
        uint8_t byte;
        int len = uart_read_bytes(SERIAL_PORT, &byte, 1, 10 / portTICK_PERIOD_MS);
        if (len > 0) {
            if (byte == SOI) rx_idx = 0;
            if (rx_idx < 1023) rx_buf[rx_idx++] = byte;

            if (byte == EOI) {
                rx_buf[rx_idx] = '\0';
//                 ESP_LOGI(TAG_RS485, "RX: %s", rx_buf); // log

                if (xSemaphoreTake(batMutex, portMAX_DELAY) == pdTRUE) {
                    memcpy(&local_bat, &bat, sizeof(BatteryState));
                    xSemaphoreGive(batMutex);
                }

                char cid2[3] = {rx_buf[7], rx_buf[8], '\0'};
                int handled = 0;

                if (strcmp(cid2, "61") == 0) {
                    get_response_61H(tx_buf, sizeof(tx_buf), &local_bat);
                    handled = 1;
                } else if (strcmp(cid2, "92") == 0) {
                    get_response_92H(tx_buf, sizeof(tx_buf), &local_bat);
                    handled = 1;
                } else if (strcmp(cid2, "63") == 0) {
                    get_response_63H(tx_buf, sizeof(tx_buf), &local_bat);
                    handled = 1;
                }

                if (handled) {
                    uart_write_bytes(SERIAL_PORT, tx_buf, strlen(tx_buf));
//                     ESP_LOGI(TAG_RS485, "TX: %s", tx_buf); // Log
                }
                rx_idx = 0;
            }
        }
    }
}

// ============================================================================
//                                   ЧАСТЬ 3: WI-FI И MQTT
// ============================================================================

static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < WIFI_MAX_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGW(TAG_WIFI, "Попытка переподключения к Wi-Fi (%d/%d)...", s_retry_num, WIFI_MAX_RETRY);
        } else {
            ESP_LOGE(TAG_WIFI, "Не удалось подключиться к Wi-Fi. Ждем...");
            vTaskDelay(pdMS_TO_TICKS(30000));
            s_retry_num = 0;
            esp_wifi_connect();
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG_WIFI, "Wi-Fi подключен! IP адрес: " IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
    }
}

void wifi_init_sta(void) {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG_WIFI, "Wi-Fi инициализирован в фоновом режиме.");
}

// Вспомогательная функция для публикации автообнаружения Home Assistant
static void publish_ha_discovery(esp_mqtt_client_handle_t client) {
    char topic[128];
    char payload[512];

    typedef struct {
        const char *id;
        const char *name;
        const char *dev_class;
        const char *unit;
        const char *json_key;
    } ha_sensor_t;

    const ha_sensor_t sensors[] = {
        {"voltage", "Battery Voltage", "voltage", "V", "voltage"},
        {"current", "Battery Current", "current", "A", "current"},
        {"soc", "Battery SOC", "battery", "%", "soc"},
        {"soh", "Battery SOH", "battery", "%", "soh"},
        {"temp_avg", "Battery Temp Avg", "temperature", "°C", "temp_avg_c"},
        {"lim_chg_v", "Charge Voltage Limit", "voltage", "V", "lim_chg_v"},
        {"max_chg_a", "Max Charge Current", "current", "A", "max_chg_a"},
        {"max_dis_a", "Max Discharge Current", "current", "A", "max_dis_a"}
    };

    int num_sensors = sizeof(sensors) / sizeof(sensors[0]);

    for (int i = 0; i < num_sensors; i++) {
        snprintf(topic, sizeof(topic), "%s/sensor/%s/%s/config",
                 HA_DISCOVERY_PREFIX, HA_NODE_ID, sensors[i].id);

        snprintf(payload, sizeof(payload),
            "{"
            "\"name\":\"%s\","
            "\"state_topic\":\"%s\","
            "\"unit_of_measurement\":\"%s\","
            "\"device_class\":\"%s\","
            "\"state_class\":\"measurement\","
            "\"value_template\":\"{{ value_json.%s }}\","
            "\"unique_id\":\"%s_%s\","
            "%s"
            "}",
            sensors[i].name,
            MQTT_TOPIC,
            sensors[i].unit,
            sensors[i].dev_class,
            sensors[i].json_key,
            HA_NODE_ID, sensors[i].id,
            HA_DEVICE_DICT
        );

        esp_mqtt_client_publish(client, topic, payload, 0, 1, 1);
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    ESP_LOGI(TAG_MQTT, "HA Discovery messages published!");
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
    esp_mqtt_event_handle_t event = event_data;
    switch ((esp_mqtt_event_id_t)event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG_MQTT, "MQTT Connected");
            publish_ha_discovery(event->client);
            break;
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGW(TAG_MQTT, "MQTT Disconnected");
            break;
        case MQTT_EVENT_ERROR:
            ESP_LOGE(TAG_MQTT, "MQTT Error");
            break;
        default:
            break;
    }
}

void task_mqtt_publisher(void *pvParameters) {
    ESP_LOGI(TAG_MQTT, "MQTT Task Started");

    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = MQTT_BROKER_URI,
        .credentials.username = MQTT_USER,
        .credentials.authentication.password = MQTT_PASS
    };

    esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(client);

    char json_payload[512];
    BatteryState local_bat;
//Подписка на Watchdog
    esp_task_wdt_add(NULL);

    while (1) {
        //Сброс таймера
        esp_task_wdt_reset();
        vTaskDelay(pdMS_TO_TICKS(5000));

        if (xSemaphoreTake(batMutex, portMAX_DELAY) == pdTRUE) {
            memcpy(&local_bat, &bat, sizeof(BatteryState));
            xSemaphoreGive(batMutex);
        }

        snprintf(json_payload, sizeof(json_payload),
            "{"
            "\"voltage\":%.2f,"
            "\"current\":%.1f,"
            "\"soc\":%d,"
            "\"soh\":%d,"
            "\"cell_max_mv\":%d,"
            "\"cell_min_mv\":%d,"
            "\"temp_avg_c\":%.1f,"
            "\"lim_chg_v\":%.2f,"
            "\"lim_dis_v\":%.2f,"
            "\"max_chg_a\":%.1f,"
            "\"max_dis_a\":%.1f"
            "}",
            local_bat.voltage / 100.0,
            -(local_bat.current / 10.0),
            local_bat.soc,
            local_bat.avg_soh,
            local_bat.cell_max,
            local_bat.cell_min,
            local_bat.temp_cell_avg / 10.0,
            local_bat.charge_vol_limit / 1000.0,
            local_bat.discharge_vol_limit / 1000.0,
            local_bat.max_chg_current / 10.0,
            abs(local_bat.max_dis_current) / 10.0
        );

        esp_mqtt_client_publish(client, MQTT_TOPIC, json_payload, 0, 0, 0);
    }
}

// ============================================================================
//                                   MAIN
// ============================================================================

void app_main() {
    batMutex = xSemaphoreCreateMutex();

//Настройка сторожевого таймера (Watchdog) ---
    esp_task_wdt_config_t twdt_config = {
        .timeout_ms = 10000,
        .idle_core_mask = (1 << portNUM_PROCESSORS) - 1,
        .trigger_panic = true,
    };

    esp_task_wdt_reconfigure(&twdt_config);
    // -----------------------------------------------------------

    bat.voltage = 4800; // 48.00V
    bat.soc = 50;
    bat.charge_vol_limit = 52000;
    bat.discharge_vol_limit = 45000;
    bat.max_chg_current = 200; // 20A
    bat.max_dis_current = 200;

    bat.temp_cell_max_module = 0;
    bat.temp_cell_min_module = 0;
    bat.temp_bms_max_module = 0;
    bat.temp_bms_min_module = 0;

    // Инициализация NVS (обязательно для драйвера Wi-Fi)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Первоочередной запуск критически важных задач (инвертор не будет ждать сеть)
    xTaskCreatePinnedToCore(task_can_bus, "CAN_TASK", 4096, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(task_rs485_proxy, "RS485_TASK", 4096, NULL, 5, NULL, 0);

    // Фоновая инициализация сетевой инфраструктуры
    wifi_init_sta();
    xTaskCreatePinnedToCore(task_mqtt_publisher, "MQTT_TASK", 4096, NULL, 4, NULL, 0);
}
