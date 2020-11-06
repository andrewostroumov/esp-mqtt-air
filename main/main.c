#include <string.h>
#include <stdbool.h>
#include <math.h>
#include "sdkconfig.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include <cJSON.h>
#include <driver/ledc.h>

#include "esp_system.h"
#include "esp_spi_flash.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_tls.h"
#include "mqtt_client.h"
#include "driver/i2c.h"
#include "driver/uart.h"

#include "esp_air.h"
#include "esp_state.h"
#include "user_i2c.h"

#include "bsec_integration.h"
#include "bsec_serialized_configurations_iaq.h"

static QueueHandle_t esp_air_queue, esp_air_co2_queue, esp_air_co_queue, esp_uart_co_queue, esp_uart_co2_queue;
static EventGroupHandle_t wifi_event_group, mqtt_event_group, esp_event_group;
static esp_mqtt_client_handle_t client;

static const int CONNECTED_BIT = BIT0;
static const char *TAG = "air";
static const char *sensor_binary = "sensor_blob";

extern const uint8_t ca_cert_pem_start[] asm("_binary_ca_crt_start");
extern const uint8_t ca_cert_pem_end[] asm("_binary_ca_crt_end");

const char *STATUS_ONLINE = "online";
const char *STATUS_OFFLINE = "offline";

const char *MQTT_DEVICE_TOPIC = "discovery/sensor/espressif";
const char *MQTT_STATE_IAQ_PATH = "state/iaq";
const char *MQTT_STATE_CO_PATH = "state/co";
const char *MQTT_STATE_CO2_PATH = "state/co2";
const char *MQTT_STATUS_PATH = "status";
const char *MQTT_ATTRS_PATH = "attributes";

char *MQTT_STATE_IAQ_TOPIC;
char *MQTT_STATE_CO_TOPIC;
char *MQTT_STATE_CO2_TOPIC;
char *MQTT_STATUS_TOPIC;
char *MQTT_ATTRS_TOPIC;

#define MQTT_PUBLISH_DELAY 5000

#define CO_GAS_TYPE_BYTE  1
#define CO_HIGH_BYTE      4
#define CO_LOW_BYTE       5
#define CO_CHECK_SUM_BYTE 8
#define CO_MESSAGE_LENGTH 9
#define CO_TYPE           0x04

#define CO2_SENSOR_BYTE    1
#define CO2_CHECK_SUM_BYTE 8
#define CO2_HIGH_BYTE      2
#define CO2_LOW_BYTE       3
#define CO2_MESSAGE_LENGTH 9
#define CO2_TYPE           0x86

#define UART_CO_BAUD_RATE              9600
#define UART_CO_BUF_SIZE               256
#define UART_CO_QUEUE_SIZE             128

#define UART_CO2_BAUD_RATE              9600
#define UART_CO2_BUF_SIZE               256
#define UART_CO2_QUEUE_SIZE             128

#define UART_CO_TXD   23
#define UART_CO_RXD   22

#define UART_CO2_TXD   16
#define UART_CO2_RXD   17

#define I2C_MASTER_SCL_IO           19
#define I2C_MASTER_SDA_IO           18
#define I2C_MASTER_FREQ_HZ          100000

#define AIR_QUEUE_SIZE              1
#define STORAGE_NAMESPACE           "storage"

#define SERIAL_KEY                  "serial"
#define SERIAL_LENGTH               8

//#define DEFINE_CO_SENSOR

static uint8_t serial[SERIAL_LENGTH];

esp_state_t esp_state = {};
esp_air_t esp_air = {};
esp_air_co2_t esp_air_co2 = {};

esp_err_t uart_co2_init() {
    esp_err_t err;
    uart_config_t uart_config = {
            .baud_rate = UART_CO2_BAUD_RATE,
            .data_bits = UART_DATA_8_BITS,
            .parity = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    err = uart_param_config(UART_NUM_2, &uart_config);
    if (err != ESP_OK) { return err; }

    err = uart_driver_install(UART_NUM_2, UART_FIFO_LEN * 2, UART_FIFO_LEN * 2, UART_CO2_QUEUE_SIZE,
                              &esp_uart_co2_queue, 0);
    if (err != ESP_OK) { return err; }

    err = uart_set_pin(UART_NUM_2, UART_CO2_TXD, UART_CO2_RXD, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (err != ESP_OK) { return err; }

    err = uart_pattern_queue_reset(UART_NUM_2, UART_CO2_QUEUE_SIZE);
    return err;
}

esp_err_t i2c_master_init() {
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    i2c_param_config(I2C_NUM_0, &conf);
    return i2c_driver_install(I2C_NUM_0, conf.mode, 0, 0, 0);
}

int64_t user_time_us() {
    return esp_timer_get_time();
}

static esp_err_t mqtt_event_handler(esp_mqtt_event_handle_t event) {
    switch (event->event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "[MQTT] Connected");
            xEventGroupSetBits(mqtt_event_group, CONNECTED_BIT);
            esp_mqtt_client_publish(event->client, MQTT_STATUS_TOPIC, STATUS_ONLINE, 0, 0, true);
            break;
        case MQTT_EVENT_DISCONNECTED:
            xEventGroupClearBits(mqtt_event_group, CONNECTED_BIT);
            ESP_LOGI(TAG, "[MQTT] Disconnected");
            break;
        default:
            ESP_LOGV(TAG, "[MQTT] Event ID %d", event->event_id);
            break;
    }
    return ESP_OK;
}

uint32_t user_bsec_config_load(uint8_t *config_buffer, uint32_t n_buffer) {
    ESP_LOGI(TAG, "[BSEC] Loading configuration: buffer-size %d config size %d", n_buffer, sizeof(bsec_config_iaq));
    assert(n_buffer >= sizeof(bsec_config_iaq));
    memcpy(config_buffer, bsec_config_iaq, sizeof(bsec_config_iaq));

    return sizeof(bsec_config_iaq);
}

uint32_t user_bsec_state_load(uint8_t *state_buffer, uint32_t n_buffer) {
    nvs_handle_t nvs;
    esp_err_t err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &nvs);
    ESP_ERROR_CHECK(err);

    err = nvs_get_blob(nvs, sensor_binary, state_buffer, &n_buffer);
    nvs_close(nvs);

    if (err != ESP_OK) {
        ESP_LOGW(TAG, "[NVS] Loading sensor binary blob error code %d", err);
        return 0;
    }

    return n_buffer;
}

void user_bsec_state_save(const uint8_t *state_buffer, uint32_t length) {
    nvs_handle_t nvs;
    esp_err_t err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &nvs);
    ESP_ERROR_CHECK(err);

    err = nvs_set_blob(nvs, sensor_binary, state_buffer, length);
    ESP_ERROR_CHECK(err);
    err = nvs_commit(nvs);
    ESP_ERROR_CHECK(err);
    nvs_close(nvs);
}

void user_output_ready(int64_t timestamp, float iaq, uint8_t iaq_accuracy, float temperature, float humidity,
                       float pressure, float raw_temperature, float raw_humidity, float gas,
                       bsec_library_return_t bsec_status,
                       float static_iaq, float co2_equivalent, float breath_voc_equivalent) {
    esp_air.temperature = temperature;
    esp_air.humidity = humidity;
    esp_air.pressure = 0.01f * pressure;
    esp_air.iaq = iaq;
    esp_air.iaq_accuracy = iaq_accuracy;
    esp_air.gas = gas;
    esp_air.co2_equivalent = co2_equivalent;
    esp_air.breath_voc_equivalent = breath_voc_equivalent;
    build_air_quality_index(&esp_air);

    xQueueSend(esp_air_queue, &esp_air, (TickType_t) 0);
}

uint8_t uart_checksum(uint8_t *data, size_t size) {
    uint8_t sum = 0;
    for (size_t i = 1; i < 8; i++) {
        sum += data[i];
    }

    return ~sum + 1;
}

esp_err_t uart_read_co2(uint8_t *data, size_t size, esp_air_co2_t *esp_air_local) {
    if (size != CO2_MESSAGE_LENGTH) { return ESP_FAIL; }
    if (data[CO2_SENSOR_BYTE] != CO2_TYPE) { return ESP_FAIL; }
    if (data[CO2_CHECK_SUM_BYTE] != uart_checksum(data, size)) { return ESP_FAIL; }

    esp_air_local->co2 = (data[CO2_HIGH_BYTE] * 256 + data[CO2_LOW_BYTE]);
    return ESP_OK;
}

void task_co2_receive(void *param) {
    uart_event_t event;
    uint8_t *data = (uint8_t *) malloc(UART_CO2_BUF_SIZE);

    while (true) {
        xQueueReceive(esp_uart_co2_queue, &event, portMAX_DELAY);
        bzero(data, UART_CO2_BUF_SIZE);

        switch (event.type) {
            case UART_DATA:
                uart_read_bytes(UART_NUM_2, data, event.size, portMAX_DELAY);
                esp_err_t err = uart_read_co2(data, event.size, &esp_air_co2);
                if (err != ESP_OK) {
                    ESP_LOGI(TAG, "[UART][CO2] Received unsupported data %d", event.size);
                    break;
                }
                build_co2_index(&esp_air_co2);
                xQueueSend(esp_air_co2_queue, &esp_air_co2, (TickType_t) 0);
                break;
            case UART_FIFO_OVF:
            case UART_BUFFER_FULL:
                uart_flush_input(UART_NUM_2);
                xQueueReset(esp_uart_co2_queue);
                break;
            default:
                ESP_LOGE(TAG, "[UART][CO2] Event ID %d", event.type);
                break;
        }
    }
}

void task_co2_poll(void *param) {
    uint8_t data[9] = {0xFF, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79};

    while (true) {
        uart_write_bytes(UART_NUM_2, (const char *) data, sizeof(data));
        vTaskDelay(10000 / portTICK_PERIOD_MS);
    }
}

void task_air_retrieve(void *param) {
    bsec_iot_loop(user_delay_ms, user_time_us, user_output_ready, user_bsec_state_save, 10000);
}

void task_air_publish(void *param) {
    TickType_t last_tick = 0;
    TickType_t tick = 0;
    uint tick_shift = 0;

    esp_air_t esp_air_local;
    char *json;

    while (true) {
        xEventGroupWaitBits(mqtt_event_group, CONNECTED_BIT, false, true, portMAX_DELAY);
        xQueueReceive(esp_air_queue, &esp_air_local, portMAX_DELAY);

        tick = xTaskGetTickCount();
        tick_shift = (tick - last_tick) * portTICK_RATE_MS;

        if (last_tick && tick_shift < MQTT_PUBLISH_DELAY) {
            continue;
        } else {
            last_tick = tick;
        }

        xEventGroupSetBits(esp_event_group, CONNECTED_BIT);
        json = serialize_esp_air(&esp_air_local);
        ESP_LOGI(TAG, "[MQTT] Publish topic %.*s data %.*s", strlen(MQTT_STATE_IAQ_TOPIC), MQTT_STATE_IAQ_TOPIC,
                 strlen(json), json);
        esp_mqtt_client_publish(client, MQTT_STATE_IAQ_TOPIC, json, 0, 0, true);

        free(json);
    }
}

void task_co2_publish(void *param) {
    TickType_t last_tick = 0;
    TickType_t tick = 0;
    uint tick_shift = 0;

    esp_air_co2_t esp_air_local;
    char *json;

    while (true) {
        xEventGroupWaitBits(mqtt_event_group, CONNECTED_BIT, false, true, portMAX_DELAY);
        xQueueReceive(esp_air_co2_queue, &esp_air_local, portMAX_DELAY);

        tick = xTaskGetTickCount();
        tick_shift = (tick - last_tick) * portTICK_RATE_MS;

        if (last_tick && tick_shift < MQTT_PUBLISH_DELAY) {
            continue;
        } else {
            last_tick = tick;
        }

        json = serialize_esp_air_co2(&esp_air_local);
        ESP_LOGI(TAG, "[MQTT] Publish topic %.*s data %.*s", strlen(MQTT_STATE_CO2_TOPIC), MQTT_STATE_CO2_TOPIC,
                 strlen(json), json);
        esp_mqtt_client_publish(client, MQTT_STATE_CO2_TOPIC, json, 0, 0, true);

        free(json);
    }
}

void task_esp_publish(void *param) {
    char *json;
    while (true) {
        xEventGroupWaitBits(mqtt_event_group, CONNECTED_BIT, false, true, portMAX_DELAY);
        xEventGroupWaitBits(esp_event_group, CONNECTED_BIT, true, true, portMAX_DELAY);
        update_esp_state(&esp_state);
        json = serialize_esp_state(&esp_state);
        ESP_LOGI(TAG, "[MQTT] Publish topic %.*s data %.*s", strlen(MQTT_ATTRS_TOPIC), MQTT_ATTRS_TOPIC, strlen(json),
                 json);
        esp_mqtt_client_publish(client, MQTT_ATTRS_TOPIC, json, 0, 0, true);
        free(json);

    }
}

static void mqtt_app_start(void) {
    const esp_mqtt_client_config_t mqtt_cfg = {
            .uri = CONFIG_MQTT_URI,
            .username = CONFIG_MQTT_USERNAME,
            .password = CONFIG_MQTT_PASSWORD,
            .event_handle = mqtt_event_handler,
            .lwt_topic = MQTT_STATUS_TOPIC,
            .lwt_msg = STATUS_OFFLINE,
            .lwt_qos = 0,
            .lwt_retain = true,
            .keepalive = 10
    };

    client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_start(client);
    ESP_LOGI(TAG, "[MQTT] Connecting to %s...", CONFIG_MQTT_URI);
}

void wifi_event_handler(void *handler_arg, esp_event_base_t base, int32_t id, void *event_data) {
    switch (id) {
        case WIFI_EVENT_STA_START:
            ESP_LOGI(TAG, "[WIFI] Connecting to %s...", CONFIG_WIFI_SSID);
            esp_wifi_connect();
            break;

        case WIFI_EVENT_STA_CONNECTED:
            ESP_LOGI(TAG, "[WIFI] Connected");
            break;

        case WIFI_EVENT_STA_DISCONNECTED:
            ESP_LOGI(TAG, "[WIFI] Reconnecting to %s...", CONFIG_WIFI_SSID);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            esp_wifi_connect();
            xEventGroupClearBits(wifi_event_group, CONNECTED_BIT);
            break;

        default:
            ESP_LOGI(TAG, "[WIFI] Event base %s with ID %d", base, id);
            break;
    }
}

void ip_event_handler(void *arg, esp_event_base_t base, int32_t id, void *event_data) {
    if (id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *) event_data;
        ESP_LOGI(TAG, "[IP] Got IP:"
                IPSTR, IP2STR(&event->ip_info.ip));
        xEventGroupSetBits(wifi_event_group, CONNECTED_BIT);
    } else {
        ESP_LOGI(TAG, "[IP] Event base %s with ID %d", base, id);
    }
}

void wifi_init_sta() {
    wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_event_loop_create_default());

    esp_netif_create_default_wifi_sta();

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, ESP_EVENT_ANY_ID, ip_event_handler, NULL));

    wifi_init_config_t wifi_init_config = WIFI_INIT_CONFIG_DEFAULT();
    wifi_config_t wifi_config = {
            .sta = {
                    .ssid = CONFIG_WIFI_SSID,
                    .password = CONFIG_WIFI_PASSWORD,
            }
    };

    ESP_ERROR_CHECK(esp_wifi_init(&wifi_init_config));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
}

esp_err_t ensure_serial(void) {
    esp_err_t err;
    nvs_handle_t nvs;

    err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &nvs);
    if (err != ESP_OK) return err;

    size_t required_size = 0;
    err = nvs_get_blob(nvs, SERIAL_KEY, NULL, &required_size);
    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) return err;

    if (required_size) {
        err = nvs_get_blob(nvs, SERIAL_KEY, serial, &required_size);
        if (err != ESP_OK) return err;
    } else {
        esp_fill_random(serial, SERIAL_LENGTH);
        err = nvs_set_blob(nvs, SERIAL_KEY, serial, SERIAL_LENGTH);
        if (err != ESP_OK) return err;
    }

    nvs_close(nvs);
    return ESP_OK;
}

void serial_hex(char **hex) {
    *hex = malloc(SERIAL_LENGTH * 2 + 1);
    char *p = *hex;

    for (size_t j = 0; j < SERIAL_LENGTH; ++j) {
        p += sprintf(p, "%02x", serial[j]);
    }
}

void ensure_mqtt_topics(void) {
    char *hex;
    serial_hex(&hex);

    MQTT_STATE_IAQ_TOPIC = malloc(strlen(MQTT_DEVICE_TOPIC) + strlen(hex) + strlen(MQTT_STATE_IAQ_PATH) + 1);
    MQTT_STATE_CO_TOPIC = malloc(strlen(MQTT_DEVICE_TOPIC) + strlen(hex) + strlen(MQTT_STATE_CO_PATH) + 1);
    MQTT_STATE_CO2_TOPIC = malloc(strlen(MQTT_DEVICE_TOPIC) + strlen(hex) + strlen(MQTT_STATE_CO2_PATH) + 1);
    MQTT_STATUS_TOPIC = malloc(strlen(MQTT_DEVICE_TOPIC) + strlen(hex) + strlen(MQTT_STATUS_PATH) + 1);
    MQTT_ATTRS_TOPIC = malloc(strlen(MQTT_DEVICE_TOPIC) + strlen(hex) + strlen(MQTT_ATTRS_PATH) + 1);

    char *ps = MQTT_STATE_IAQ_TOPIC;
    char *po = MQTT_STATE_CO_TOPIC;
    char *pc = MQTT_STATE_CO2_TOPIC;
    char *pt = MQTT_STATUS_TOPIC;
    char *pa = MQTT_ATTRS_TOPIC;

    ps += sprintf(ps, "%s", MQTT_DEVICE_TOPIC);
    po += sprintf(po, "%s", MQTT_DEVICE_TOPIC);
    pc += sprintf(pc, "%s", MQTT_DEVICE_TOPIC);
    pt += sprintf(pt, "%s", MQTT_DEVICE_TOPIC);
    pa += sprintf(pa, "%s", MQTT_DEVICE_TOPIC);

    ps += sprintf(ps, "/%s/", hex);
    po += sprintf(po, "/%s/", hex);
    pc += sprintf(pc, "/%s/", hex);
    pt += sprintf(pt, "/%s/", hex);
    pa += sprintf(pa, "/%s/", hex);

    strcpy(ps, MQTT_STATE_IAQ_PATH);
    strcpy(po, MQTT_STATE_CO_PATH);
    strcpy(pc, MQTT_STATE_CO2_PATH);
    strcpy(pt, MQTT_STATUS_PATH);
    strcpy(pa, MQTT_ATTRS_PATH);

    free(hex);
}

#ifdef DEFINE_CO_SENSOR
esp_air_co_t esp_air_co = {};

esp_err_t uart_co_init() {
    esp_err_t err;
    uart_config_t uart_config = {
            .baud_rate = UART_CO_BAUD_RATE,
            .data_bits = UART_DATA_8_BITS,
            .parity = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
            .source_clk = UART_SCLK_APB,
    };

    err = uart_param_config(UART_NUM_1, &uart_config);
    if (err != ESP_OK) { return err; }

    err = uart_driver_install(UART_NUM_1, UART_FIFO_LEN * 2, UART_FIFO_LEN * 2, UART_CO_QUEUE_SIZE, &esp_uart_co_queue,
                              0);
    if (err != ESP_OK) { return err; }

    err = uart_set_pin(UART_NUM_1, UART_CO_TXD, UART_CO_RXD, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (err != ESP_OK) { return err; }

    err = uart_pattern_queue_reset(UART_NUM_1, UART_CO_QUEUE_SIZE);
    return err;
}

esp_err_t uart_read_co(uint8_t *data, size_t size, esp_air_co_t *esp_air_local) {
    if (size != CO_MESSAGE_LENGTH) { return ESP_FAIL; }
    if (data[CO_GAS_TYPE_BYTE] != CO_TYPE) { return ESP_FAIL; }
    if (data[CO_CHECK_SUM_BYTE] != uart_checksum(data, size)) { return ESP_FAIL; }

    esp_air_local->co = (data[CO_HIGH_BYTE] * 256 + data[CO_LOW_BYTE]) * 0.1;
    return ESP_OK;
}

void task_co_receive(void *param) {
    uart_event_t event;
    uint8_t *data = (uint8_t *) malloc(UART_CO_BUF_SIZE);

    while (true) {
        xQueueReceive(esp_uart_co_queue, &event, portMAX_DELAY);
        bzero(data, UART_CO_BUF_SIZE);

        switch (event.type) {
            case UART_DATA:
                uart_read_bytes(UART_NUM_1, data, event.size, portMAX_DELAY);
                esp_err_t err = uart_read_co(data, event.size, &esp_air_co);
                if (err != ESP_OK) {
                    ESP_LOGE(TAG, "[UART][CO] Received unsupported data %d", event.size);
                    break;
                }
                xQueueSend(esp_air_co_queue, &esp_air_co, (TickType_t) 0);
                break;
            case UART_FIFO_OVF:
            case UART_BUFFER_FULL:
                uart_flush_input(UART_NUM_1);
                xQueueReset(esp_uart_co_queue);
                break;
            default:
                ESP_LOGE(TAG, "[UART][CO] Event ID %d", event.type);
                break;
        }
    }
}

void task_co_publish(void *param) {
    TickType_t last_tick = 0;
    TickType_t tick = 0;
    uint tick_shift = 0;

    esp_air_co_t esp_air_local;
    char *json;

    while (true) {
        xEventGroupWaitBits(mqtt_event_group, CONNECTED_BIT, false, true, portMAX_DELAY);
        xQueueReceive(esp_air_co_queue, &esp_air_local, portMAX_DELAY);

        tick = xTaskGetTickCount();
        tick_shift = (tick - last_tick) * portTICK_RATE_MS;

        if (last_tick && tick_shift < MQTT_PUBLISH_DELAY) {
            continue;
        } else {
            last_tick = tick;
        }

        json = serialize_esp_air_co(&esp_air_local);
        ESP_LOGI(TAG, "[MQTT] Publish topic %.*s data %.*s", strlen(MQTT_STATE_CO_TOPIC), MQTT_STATE_CO_TOPIC,
                 strlen(json), json);
        esp_mqtt_client_publish(client, MQTT_STATE_CO_TOPIC, json, 0, 0, true);

        free(json);
    }
}

#endif

void app_main() {
    esp_err_t res;
    return_values_init ret;

    ESP_LOGI(TAG, "[APP] Startup..");
    ESP_LOGI(TAG, "[APP] Free memory: %d bytes", esp_get_free_heap_size());
    ESP_LOGI(TAG, "[APP] IDF version: %s", esp_get_idf_version());

    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set("esp-tls", ESP_LOG_INFO);
    esp_log_level_set(TAG, ESP_LOG_INFO);

    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // NVS partition was truncated and needs to be erased
        // Retry nvs_flash_init
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(ensure_serial());

    ensure_mqtt_topics();

    res = i2c_master_init();

    if (res != ESP_OK) {
        ESP_LOGE(TAG, "I2C driver install error: %d", res);
        exit(1);
    }

    ret = bsec_iot_init(BSEC_SAMPLE_RATE_LP, 0.0f, user_i2c_write, user_i2c_read, user_delay_ms, user_bsec_state_load,
                        user_bsec_config_load);
    if (ret.bme680_status) {
        ESP_LOGE(TAG, "Initialize BME680 error %d", res);
        exit(1);
    } else if (ret.bsec_status) {
        ESP_LOGE(TAG, "Initialize BSEC error %d", res);
        exit(1);
    }

    res = uart_co2_init();
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "UART driver CO2 install error: %d", res);
        exit(1);
    }

    wifi_init_sta();
    xEventGroupWaitBits(wifi_event_group, CONNECTED_BIT, false, true, portMAX_DELAY);
    mqtt_app_start();

    mqtt_event_group = xEventGroupCreate();
    esp_event_group = xEventGroupCreate();
    esp_air_queue = xQueueCreate(AIR_QUEUE_SIZE, sizeof(esp_air_t));
    esp_air_co2_queue = xQueueCreate(AIR_QUEUE_SIZE, sizeof(esp_air_co2_t));

    if (esp_air_queue == NULL || esp_air_co2_queue == NULL) {
        ESP_LOGE(TAG, "Error initialize env data queue");
        exit(1);
    }

    xTaskCreate(task_air_retrieve, "task_air_retrieve", 4096, NULL, 0, NULL);
    xTaskCreate(task_air_publish, "task_air_publish", 4096, NULL, 0, NULL);
    xTaskCreate(task_esp_publish, "task_esp_publish", 4096, NULL, 0, NULL);
    xTaskCreate(task_co2_poll, "task_co2_poll", 4096, NULL, 0, NULL);
    xTaskCreate(task_co2_receive, "task_co2_receive", 4096, NULL, 0, NULL);
    xTaskCreate(task_co2_publish, "task_air_co2_publish", 4096, NULL, 0, NULL);

#ifdef DEFINE_CO_SENSOR
    res = uart_co_init();
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "UART driver CO install error: %d", res);
        exit(1);
    }

    esp_air_co_queue = xQueueCreate(AIR_QUEUE_SIZE, sizeof(esp_air_co_t));
    if (esp_air_co_queue == NULL) {
        ESP_LOGE(TAG, "Error initialize env data queue");
        exit(1);
    }

    xTaskCreate(task_co_receive, "task_co_receive", 4096, NULL, 0, NULL);
    xTaskCreate(task_co_publish, "task_co_publish", 4096, NULL, 0, NULL);
#endif
}
