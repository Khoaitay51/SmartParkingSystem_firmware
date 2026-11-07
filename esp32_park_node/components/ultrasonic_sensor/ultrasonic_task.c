#include <stdio.h>
#include <stdatomic.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "mqtt_client.h"

#include "ultrasonic.h"
#include "ultrasonic_task.h"
#include "config.h"

static const char *TAG = "ULTRASONIC_TASK";

#define REPORT_INTERVAL_MS 2000

extern esp_mqtt_client_handle_t mqtt_client;
extern atomic_bool is_mqtt_connected;

typedef struct
{
    uint8_t slot_id;
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
    uint8_t slot_id = config->slot_id;
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
    ESP_LOGI(TAG, "Task Siêu âm bắt đầu chạy (Lot ID: %d, Trig GPIO %d, Echo GPIO %d)", slot_id, dev.trigger_pin, dev.echo_pin);

    char topic[80];
    snprintf(topic, sizeof(topic), "parking/%s/slots/%u", g_app_config.lot_id, (unsigned)slot_id);
    bool reported = false;
    bool previous_valid = false;
    bool previous_occupied = false;
    TickType_t last_report = 0;

    while (true)
    {
        uint32_t distance_cm = 0;
        esp_err_t result = ultrasonic_measure_cm(&dev, g_app_config.max_distance_cm, &distance_cm);
        bool timeout = result == ESP_ERR_ULTRASONIC_PING_TIMEOUT || result == ESP_ERR_ULTRASONIC_ECHO_TIMEOUT;
        bool valid = result == ESP_OK || timeout;
        bool occupied = result == ESP_OK && distance_cm <= g_app_config.distance_detect_threshold_cm;
        bool too_close = occupied && distance_cm <= g_app_config.distance_too_close_threshold_cm;
        gpio_set_level(g_app_config.buzzer_gpio, too_close);
        if (occupied && !too_close && !previous_occupied) trigger_buzzer_beep();
        if (!valid) ESP_LOGW(TAG, "Lỗi đo siêu âm ô %u: %s", (unsigned)slot_id, esp_err_to_name(result));

        TickType_t now = xTaskGetTickCount();
        bool changed = previous_valid != valid || previous_occupied != occupied;
        if (!is_mqtt_connected) reported = false;
        else if (!reported || changed || now - last_report >= pdMS_TO_TICKS(REPORT_INTERVAL_MS))
        {
            char distance[12] = "null";
            if (result == ESP_OK) snprintf(distance, sizeof(distance), "%lu", (unsigned long)distance_cm);
            char payload[96];
            snprintf(payload, sizeof(payload), "{\"totalSlots\":%u,\"occupied\":%s,\"distanceCm\":%s}",
                (unsigned)g_app_config.num_parking_lots, valid ? (occupied ? "true" : "false") : "null", distance);
            reported = esp_mqtt_client_publish(mqtt_client, topic, payload, 0, 0, 0) >= 0;
            last_report = now;
        }
        previous_valid = valid;
        previous_occupied = occupied;
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void init_ultrasonic_task(uint8_t slot_id, gpio_num_t trigger_pin, gpio_num_t echo_pin)
{
    ultrasonic_task_config_t *config = malloc(sizeof(ultrasonic_task_config_t));
    if (config == NULL)
    {
        ESP_LOGE(TAG, "Không đủ bộ nhớ cấp phát cho config!");
        return;
    }
    config->trigger_pin = trigger_pin;
    config->echo_pin = echo_pin;
    config->slot_id = slot_id;

    xTaskCreatePinnedToCore(
        vTaskUltrasonic,
        "vTaskUltrasonic",
        3072,
        (void *)config,
        5,
        NULL,
        1);
}