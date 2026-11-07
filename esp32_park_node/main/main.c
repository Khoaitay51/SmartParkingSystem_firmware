#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdatomic.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "mqtt_client.h"
#include "esp_spiffs.h"
#include <string.h>

// Include thư viện quản lý Task cảm biến IR
#include "ir_task.h"
#include "ultrasonic_task.h"
#include "config.h"
#include "driver/uart.h"
#include "servo.h"
#define UART_PORT_NUM UART_NUM_0
#define BUF_SIZE 1024
#define SOH 0x01
#define STX 0x02
#define EOT 0x04
#define ACK 0x06
#define NAK 0x15
#define CA 0x18
#define CRC16_C 'C'

static const char *TAG = "MAIN";
static void mqtt_app_start(void);
// static esp_mqtt_client_handle_t mqtt_client = NULL;
app_config_t g_app_config = {
    .ir_gatein_gpio = GPIO_NUM_32,
    .ir_gateout_gpio = GPIO_NUM_33,
    .num_parking_lots = 1,
    .trigger_pins = {GPIO_NUM_26},
    .echo_pins = {GPIO_NUM_25},
    .buzzer_gpio = GPIO_NUM_13,
    .max_distance_cm = 400,
    .distance_too_close_threshold_cm = 5,
    .distance_detect_threshold_cm = 15,
    .gatein_servo_pin = GPIO_NUM_18,
    .gateout_servo_pin = GPIO_NUM_19,
    .servo_angle_open = 90,
    .servo_angle_close = 0,
};
// Đã định nghĩa biến toàn cục để file ir_task.c có thể dùng thông qua extern
esp_mqtt_client_handle_t mqtt_client = NULL;
atomic_bool is_mqtt_connected = false;
static bool mqtt_started;
static char command_in_topic[80];
static char command_out_topic[80];

bool is_having_car[MAX_PARKING_LOTS] = {false}; // Mảng` trạng thái xe trong từng slot (false: trống, true: có xe)
// uint8_t trigger_pins[MAX_PARKING_LOTS] = {GPIO_NUM_12, GPIO_NUM_13}; // Mảng lưu trữ chân trigger cho từng slot
// uint8_t echo_pins[MAX_PARKING_LOTS] = {GPIO_NUM_14, GPIO_NUM_15};    // Mảng lưu trữ chân echo cho từng slot
// Callback xử lý sự kiện Wi-Fi & IP
// Hàm tính CRC16 cho YMODEM
static uint16_t crc16_ccitt(const uint8_t *buf, int len)
{
    uint16_t crc = 0;
    while (len--)
    {
        crc ^= (uint16_t)*buf++ << 8;
        for (int i = 0; i < 8; i++)
        {
            if (crc & 0x8000)
                crc = (crc << 1) ^ 0x1021;
            else
                crc <<= 1;
        }
    }
    return crc;
}

// Task nhận file YMODEM từ UART0 và ghi vào SPIFFS
void ymodem_receive_task(void *pvParameters)
{
    uint8_t *rx_buf = (uint8_t *)malloc(1024 + 16);
    FILE *f = NULL;
    bool is_header_parsed = false;
    uint8_t expected_block = 0;

    // Tắt toàn bộ log hệ thống để không gây nhiễu UART0
    esp_log_level_set("*", ESP_LOG_NONE);
    uart_flush_input(UART_NUM_0);

    uint8_t req_c = CRC16_C;

    // 1. Gửi 'C' chờ Packet 0 từ PC
    while (!is_header_parsed)
    {
        uart_write_bytes(UART_NUM_0, (const char *)&req_c, 1);
        uart_wait_tx_done(UART_NUM_0, pdMS_TO_TICKS(50));
        vTaskDelay(pdMS_TO_TICKS(300));

        size_t buffered_len = 0;
        uart_get_buffered_data_len(UART_NUM_0, &buffered_len);
        if (buffered_len >= 133)
            break;
    }

    // 2. Vòng lặp nhận các gói dữ liệu YMODEM
    while (1)
    {
        int len = uart_read_bytes(UART_NUM_0, rx_buf, 1, pdMS_TO_TICKS(2000));
        if (len <= 0)
            continue;

        if (rx_buf[0] == SOH || rx_buf[0] == STX)
        {
            int packet_size = (rx_buf[0] == SOH) ? 128 : 1024;
            int payload_len = packet_size + 4; // Block + ~Block + Data + CRC(2B)

            int bytes_received = 0;
            while (bytes_received < payload_len)
            {
                int r = uart_read_bytes(UART_NUM_0, rx_buf + 1 + bytes_received,
                                        payload_len - bytes_received, pdMS_TO_TICKS(500));
                if (r <= 0)
                    break;
                bytes_received += r;
            }

            if (bytes_received < payload_len)
                continue;

            uint8_t blk_num = rx_buf[1];
            uint8_t blk_num_inv = rx_buf[2];

            if ((blk_num ^ blk_num_inv) != 0xFF)
            {
                uint8_t nak = NAK;
                uart_write_bytes(UART_NUM_0, (const char *)&nak, 1);
                uart_wait_tx_done(UART_NUM_0, pdMS_TO_TICKS(50));
                continue;
            }

            // --- PACKET 0 (HEADER FILE) ---
            if (blk_num == 0 && !is_header_parsed)
            {
                f = fopen("/spiffs/config.json", "wb");
                is_header_parsed = true;
                expected_block = 1;

                // CHỈ TRẢ VỀ 1 BYTE ACK (0x06) ĐỂ ĐỒNG BỘ CHUẨN VỚI PYTHON XMODEM
                uint8_t ack = ACK;
                uart_write_bytes(UART_NUM_0, (const char *)&ack, 1);
                uart_wait_tx_done(UART_NUM_0, pdMS_TO_TICKS(50));
            }
            // --- PACKET DATA (BLOCK 1, 2, ...) ---
            else if (blk_num == expected_block && f != NULL)
            {
                fwrite(rx_buf + 3, 1, packet_size, f);
                expected_block++;

                uint8_t ack = ACK;
                uart_write_bytes(UART_NUM_0, (const char *)&ack, 1);
                uart_wait_tx_done(UART_NUM_0, pdMS_TO_TICKS(50));
            }
        }
        // --- EOT (KẾT THÚC TRUYỀN FILE) ---
        else if (rx_buf[0] == EOT)
        {
            if (f)
            {
                fclose(f);
                f = NULL;
            }
            uint8_t ack = ACK;
            uart_write_bytes(UART_NUM_0, (const char *)&ack, 1);
            uart_wait_tx_done(UART_NUM_0, pdMS_TO_TICKS(50));

            // Bật lại log hệ thống
            esp_log_level_set("*", ESP_LOG_INFO);
            ESP_LOGI(TAG, "Đã nhận xong config.json! Đang load lại cấu hình...");
            load_config_from_json();
            break;
        }
    }

    free(rx_buf);
    vTaskDelete(NULL);
}

// 1. Prototype khai báo hàm để tránh lỗi implicit declaration
// bool check_and_receive_ymodem_blocking(uint32_t timeout_ms);
extern esp_err_t load_config_from_json(void);

// 2. Định nghĩa chi tiết hàm Blocking nhận YMODEM
bool check_and_receive_json_raw(uint32_t timeout_ms)
{
    // Tắt toàn bộ log hệ thống để giải phóng cổng UART0
    esp_log_level_set("*", ESP_LOG_NONE);
    uart_flush_input(UART_PORT_NUM);

    // Gửi tín hiệu READY
    const char *ready_msg = "READY_FOR_CONFIG\n";
    uart_write_bytes(UART_PORT_NUM, ready_msg, strlen(ready_msg));
    uart_wait_tx_done(UART_PORT_NUM, pdMS_TO_TICKS(50));

    char rx_buf[1024] = {0};
    int total_bytes = 0;
    uint32_t elapsed = 0;

    // Lắng nghe dữ liệu trong thời gian timeout_ms
    while (elapsed < timeout_ms)
    {
        int len = uart_read_bytes(UART_PORT_NUM, (uint8_t *)(rx_buf + total_bytes),
                                  sizeof(rx_buf) - 1 - total_bytes, pdMS_TO_TICKS(100));
        elapsed += 100;

        if (len > 0)
        {
            total_bytes += len;
            rx_buf[total_bytes] = '\0';

            // Tìm ký tự bắt đầu '{' và kết thúc '}' của JSON
            char *start_ptr = strchr(rx_buf, '{');
            char *line_end = strchr(rx_buf, '\n');
            char *end_ptr = line_end && line_end > rx_buf ? line_end - 1 : NULL;

            if (start_ptr != NULL && end_ptr != NULL && end_ptr > start_ptr && *end_ptr == '}')
            {
                // Mở file và ghi chính xác nội dung JSON vào SPIFFS
                FILE *f = fopen("/spiffs/config.json", "w");
                if (f != NULL)
                {
                    size_t json_len = (end_ptr - start_ptr) + 1;
                    fwrite(start_ptr, 1, json_len, f);
                    fclose(f);

                    // Gửi duy nhất phản hồi CONFIG_OK
                    const char *ack_msg = "CONFIG_OK\n";
                    uart_write_bytes(UART_PORT_NUM, ack_msg, strlen(ack_msg));
                    uart_wait_tx_done(UART_PORT_NUM, pdMS_TO_TICKS(100));

                    esp_log_level_set("*", ESP_LOG_INFO);
                    ESP_LOGI(TAG, "Đã lưu file config.json thành công (%d bytes)!", (int)json_len);
                    return true;
                }
            }
        }
    }

    esp_log_level_set("*", ESP_LOG_INFO);
    ESP_LOGI(TAG, "Không nhận được file config mới. Bỏ qua và Boot tiếp...");
    return false;
}
static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
    {
        esp_wifi_connect();
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        ESP_LOGW(TAG, "Mất kết nối Wi-Fi, đang thử lại...");
        esp_wifi_connect();
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "Đã kết nối Wi-Fi! IP nhận được: " IPSTR, IP2STR(&event->ip_info.ip));
        if (mqtt_client != NULL && !mqtt_started)
        {
            mqtt_started = esp_mqtt_client_start(mqtt_client) == ESP_OK;
        }
    }
}

// Khởi tạo Wi-Fi ở chế độ Station
static void wifi_init_sta(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL));

    wifi_config_t wifi_config = {0};
    strncpy((char *)wifi_config.sta.ssid, g_app_config.wifi_ssid, sizeof(wifi_config.sta.ssid) - 1);
    strncpy((char *)wifi_config.sta.password, g_app_config.wifi_pass, sizeof(wifi_config.sta.password) - 1);
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
}

// Callback xử lý sự kiện MQTT
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t event = event_data;
    switch ((esp_mqtt_event_id_t)event_id)
    {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "Đã kết nối thành công với MQTT Broker!");
        is_mqtt_connected = true;
        if (g_app_config.remote_gate_enabled) {
            esp_mqtt_client_subscribe(mqtt_client, command_in_topic, 0);
            esp_mqtt_client_subscribe(mqtt_client, command_out_topic, 0);
        }
        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGW(TAG, "Mất kết nối MQTT Broker!");
        is_mqtt_connected = false;
        break;
    case MQTT_EVENT_DATA:
        if (g_app_config.remote_gate_enabled && !event->retain &&
            event->current_data_offset == 0 && event->total_data_len == 4 &&
            event->data_len == 4 && memcmp(event->data, "open", 4) == 0) {
            if (event->topic_len == (int)strlen(command_in_topic) &&
                memcmp(event->topic, command_in_topic, event->topic_len) == 0) {
                request_gate_open(true);
            } else if (event->topic_len == (int)strlen(command_out_topic) &&
                       memcmp(event->topic, command_out_topic, event->topic_len) == 0) {
                request_gate_open(false);
            }
        }
        break;
    default:
        break;
    }
}

// Khởi tạo MQTT Client
static void mqtt_app_start(void)
{
    if (!g_app_config.lot_id[0] || !g_app_config.mqtt_broker_uri[0]) {
        ESP_LOGW(TAG, "Cần cấu hình lot_id và mqtt_broker_uri để kết nối MQTT");
        return;
    }
    snprintf(command_in_topic, sizeof(command_in_topic), "parking/%s/commands/in", g_app_config.lot_id);
    snprintf(command_out_topic, sizeof(command_out_topic), "parking/%s/commands/out", g_app_config.lot_id);
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = g_app_config.mqtt_broker_uri,
    };
    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    if (mqtt_client != NULL)
        ESP_ERROR_CHECK(esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL));
}

void app_main(void)
{
    // 1. Mount SPIFFS
    ESP_ERROR_CHECK(init_spiffs());

    // 2. Cấu hình UART0 nhận lệnh YMODEM ngay từ Boot
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    uart_param_config(UART_NUM_0, &uart_config);
    uart_driver_install(UART_NUM_0, BUF_SIZE * 2, 0, 0, NULL, 0);

    // 3. Chặn 3 giây tại Boot Mode chờ nạp YMODEM
    check_and_receive_json_raw(2000); // Chờ 2 giây ở Boot

    // 4. Load cấu hình mới (hoặc cũ) vào g_app_config
    load_config_from_json();

    // 5. Khởi tạo Flash NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    mqtt_app_start();

    ESP_ERROR_CHECK(servo_init(g_app_config.gatein_servo_pin));
    ESP_ERROR_CHECK(servo_init(g_app_config.gateout_servo_pin));
    // 7. Khởi tạo các Task cảm biến IR & Siêu âm
    ESP_LOGI(TAG, "Đang khởi tạo Task cảm biến IR...");
    init_ir_sensor_task(g_app_config.ir_gatein_gpio);
    init_ir_sensor_task(g_app_config.ir_gateout_gpio);

    for (uint8_t i = 0; i < g_app_config.num_parking_lots; i++)
    {
        ESP_LOGI(TAG, "Đang khởi tạo Task cảm biến siêu âm cho slot %d... (TRIG: %d, ECHO: %d)", i + 1, g_app_config.trigger_pins[i], g_app_config.echo_pins[i]);
        init_ultrasonic_task(i + 1, g_app_config.trigger_pins[i], g_app_config.echo_pins[i]);
    }
    if (g_app_config.wifi_ssid[0]) wifi_init_sta();
}
