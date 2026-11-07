#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "mqtt_client.h"

#include "ultrasonic.h"
#include "ultrasonic_task.h"
#include "config.h"

static const char *TAG = "ULTRASONIC_TASK";

#define MQTT_TOPIC_ULTRASONIC "esp32/ultrasonic"

extern esp_mqtt_client_handle_t mqtt_client;
extern bool is_mqtt_connected;

typedef struct
{
    uint8_t lot_id;
    gpio_num_t trigger_pin;
    gpio_num_t echo_pin;
} ultrasonic_task_config_t;

// Hàm còi báo không block task
static void trigger_buzzer_beep(void)
{
    gpio_set_level(g_app_config.buzzer_gpio, 1);
    vTaskDelay(pdMS_TO_TICKS(80));
    gpio_set_level(g_app_config.buzzer_gpio, 0);
    vTaskDelay(pdMS_TO_TICKS(80));
    gpio_set_level(g_app_config.buzzer_gpio, 1);
    vTaskDelay(pdMS_TO_TICKS(80));
    gpio_set_level(g_app_config.buzzer_gpio, 0);
}

void vTaskUltrasonic(void *pvParameters)
{
    ultrasonic_task_config_t *config = (ultrasonic_task_config_t *)pvParameters;
    uint8_t lot_id = config->lot_id;
    ultrasonic_sensor_t dev = {
        .trigger_pin = config->trigger_pin,
        .echo_pin = config->echo_pin};

    free(config);

    if (ultrasonic_init(&dev) != ESP_OK)
    {
        ESP_LOGE(TAG, "Khởi tạo cảm biến siêu âm thất bại tại Trig:%d, Echo:%d!", dev.trigger_pin, dev.echo_pin);
        vTaskDelete(NULL);
        return;
    }
    gpio_reset_pin(g_app_config.buzzer_gpio);
    gpio_set_direction(g_app_config.buzzer_gpio, GPIO_MODE_OUTPUT);
    gpio_set_pull_mode(g_app_config.buzzer_gpio, GPIO_PULLDOWN_ONLY);
    gpio_set_level(g_app_config.buzzer_gpio, 0); // Đảm bảo còi tắt ban đầu
    ESP_LOGI(TAG, "Task Siêu âm bắt đầu chạy (Lot ID: %d, Trig GPIO %d, Echo GPIO %d)", lot_id, dev.trigger_pin, dev.echo_pin);

    bool last_state = false;

    while (1)
    {
        uint32_t distance_cm = 0;
        esp_err_t res = ultrasonic_measure_cm(&dev, g_app_config.max_distance_cm, &distance_cm);

        if (res == ESP_OK)
        {
            // Đổi ESP_LOGD thành ESP_LOGI để log hiển thị liên tục mỗi 500ms
            ESP_LOGI(TAG, "[Lot %d] Khoảng cách đo được: %lu cm", lot_id, distance_cm);

            bool current_state = (distance_cm <= g_app_config.distance_detect_threshold_cm &&
                                  distance_cm > g_app_config.distance_too_close_threshold_cm);

            if (distance_cm <= g_app_config.distance_too_close_threshold_cm)
            {
                ESP_LOGW(TAG, "[Lot %d] Vật thể quá gần! Khoảng cách: %lu cm", lot_id, distance_cm);
                gpio_set_level(g_app_config.buzzer_gpio, 1); // Bật còi báo động
            }
            else if (distance_cm <= g_app_config.distance_detect_threshold_cm)
            {

                if (current_state && !last_state)
                {
                    ESP_LOGW(TAG, "-> [Lot %d] Phát hiện vật thể ở khoảng cách: %lu cm!", lot_id, distance_cm);
                    trigger_buzzer_beep();
                    if (is_mqtt_connected && mqtt_client != NULL)
                    {
                        char payload[64];
                        snprintf(payload, sizeof(payload), "{\"lot_id\": %u, \"having_car\": true}", lot_id);
                        int msg_id = esp_mqtt_client_publish(mqtt_client, MQTT_TOPIC_ULTRASONIC, payload, 0, 1, 0);
                        ESP_LOGI(TAG, "Đã gửi MQTT (msg_id=%d)", msg_id);
                    }
                    else
                    {
                        ESP_LOGE(TAG, "Chưa kết nối MQTT, không thể gửi dữ liệu!");
                    }
                }
                else if (distance_cm > g_app_config.distance_detect_threshold_cm && last_state)
                {
                    ESP_LOGI(TAG, "[Lot %d] Vật thể đã rời khỏi vùng phát hiện.", lot_id);
                    gpio_set_level(g_app_config.buzzer_gpio, 0);

                    if (is_mqtt_connected && mqtt_client != NULL)
                    {
                        char payload[64];
                        snprintf(payload, sizeof(payload), "{\"lot_id\": %u, \"having_car\": false}", lot_id);
                        int msg_id = esp_mqtt_client_publish(mqtt_client, MQTT_TOPIC_ULTRASONIC, payload, 0, 1, 0);
                        ESP_LOGI(TAG, "Đã gửi MQTT (msg_id=%d)", msg_id);
                    }
                    else
                    {
                        ESP_LOGE(TAG, "Chưa kết nối MQTT, không thể gửi dữ liệu!");
                    }
                }
            }
            else
            {
                gpio_set_level(g_app_config.buzzer_gpio, 0); // Tắt còi nếu không có vật thể gần
            }

            last_state = current_state;
        }
        else if (res == ESP_ERR_ULTRASONIC_PING_TIMEOUT || res == ESP_ERR_ULTRASONIC_ECHO_TIMEOUT)
        {
            // In log cảnh báo timeout định kỳ để biết cảm biến không thấy vật cản/vượt khoảng cách
            ESP_LOGW(TAG, "[Lot %d] Outside range / No Echo (Timeout)", lot_id);
            gpio_set_level(g_app_config.buzzer_gpio, 0); // Tắt còi nếu không có vật thể gần
            if (last_state)
            {
                ESP_LOGI(TAG, "[Lot %d] Xe đã rời đi (Timeout Echo/Ping).", lot_id);
                last_state = false;
            }
        }
        else
        {
            ESP_LOGE(TAG, "[Lot %d] Lỗi phần cứng đọc cảm biến: %d", lot_id, res);
        }

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void init_ultrasonic_task(uint8_t lot_id, gpio_num_t trigger_pin, gpio_num_t echo_pin)
{
    ultrasonic_task_config_t *config = malloc(sizeof(ultrasonic_task_config_t));
    if (config == NULL)
    {
        ESP_LOGE(TAG, "Không đủ bộ nhớ cấp phát cho config!");
        return;
    }
    config->trigger_pin = trigger_pin;
    config->echo_pin = echo_pin;
    config->lot_id = lot_id;

    xTaskCreatePinnedToCore(
        vTaskUltrasonic,
        "vTaskUltrasonic",
        3072,
        (void *)config,
        5,
        NULL,
        1);
}