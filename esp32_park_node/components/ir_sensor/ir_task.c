#include <stdio.h>
#include <stdatomic.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "mqtt_client.h"
#include "config.h"
#include "ir_sensor.h"
#include "ir_task.h"
#include "servo.h"

#define REMOTE_OPEN_MS 4000
#define REPORT_INTERVAL_MS 1000

static const char *TAG = "IR_TASK";
static TaskHandle_t gate_tasks[2];

extern esp_mqtt_client_handle_t mqtt_client;
extern atomic_bool is_mqtt_connected;

void request_gate_open(bool gate_in)
{
    TaskHandle_t task = gate_tasks[gate_in ? 0 : 1];
    if (task) xTaskNotifyGive(task);
}

static void vTaskIRSensor(void *parameters)
{
    gpio_num_t ir_pin = (gpio_num_t)(intptr_t)parameters;
    bool gate_in = ir_pin == g_app_config.ir_gatein_gpio;
    gpio_num_t servo_pin = gate_in ? g_app_config.gatein_servo_pin : g_app_config.gateout_servo_pin;
    ir_sensor_t sensor;
    if (ir_sensor_init(&sensor, ir_pin, true) != ESP_OK)
    {
        ESP_LOGE(TAG, "Không khởi tạo được cảm biến tại GPIO %d", ir_pin);
        gate_tasks[gate_in ? 0 : 1] = NULL;
        vTaskDelete(NULL);
        return;
    }

    char topic[80];
    snprintf(topic, sizeof(topic), "parking/%s/gates/%s", g_app_config.lot_id, gate_in ? "in" : "out");
    bool initialized = false;
    bool open = false;
    bool remote_hold = false;
    TickType_t hold_started = 0;
    TickType_t last_report = 0;
    while (true)
    {
        TickType_t now = xTaskGetTickCount();
        bool detected = ir_sensor_is_detected(&sensor);
        if (ulTaskNotifyTake(pdTRUE, 0) && g_app_config.remote_gate_enabled)
        {
            remote_hold = true;
            hold_started = now;
        }
        if (remote_hold && now - hold_started >= pdMS_TO_TICKS(REMOTE_OPEN_MS))
            remote_hold = false;

        bool should_open = detected || remote_hold;
        bool changed = !initialized || open != should_open;
        if (changed)
        {
            esp_err_t result = servo_set_angle(servo_pin,
                should_open ? g_app_config.servo_angle_open : g_app_config.servo_angle_close);
            if (result == ESP_OK)
            {
                open = should_open;
                initialized = true;
            }
            else
            {
                ESP_LOGE(TAG, "Lỗi điều khiển servo GPIO %d", servo_pin);
            }
        }

        if (initialized && is_mqtt_connected &&
            (changed || now - last_report >= pdMS_TO_TICKS(REPORT_INTERVAL_MS)))
        {
            char payload[80];
            snprintf(payload, sizeof(payload), "{\"state\":\"%s\",\"remoteEnabled\":%s}",
                open ? "open" : "closed", g_app_config.remote_gate_enabled ? "true" : "false");
            esp_mqtt_client_publish(mqtt_client, topic, payload, 0, 0, 0);
            last_report = now;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void init_ir_sensor_task(gpio_num_t ir_pin)
{
    bool gate_in = ir_pin == g_app_config.ir_gatein_gpio;
    xTaskCreatePinnedToCore(vTaskIRSensor, "vTaskIRSensor", 3072,
        (void *)(intptr_t)ir_pin, 5, &gate_tasks[gate_in ? 0 : 1], 1);
}
