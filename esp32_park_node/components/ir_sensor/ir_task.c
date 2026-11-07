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

#define CMD_OPEN_TIMEOUT_MS 15000 // Tự đóng sau 15s nếu xe đứng ngâm không đi qua

static const char *TAG = "IR_TASK";
static TaskHandle_t gate_tasks[2];

extern esp_mqtt_client_handle_t mqtt_client;
extern atomic_bool is_mqtt_connected;

void request_gate_open(bool gate_in)
{
    TaskHandle_t task = gate_tasks[gate_in ? 0 : 1];
    if (task)
        xTaskNotifyGive(task);
}

static void vTaskIRSensor(void *parameters)
{
    gpio_num_t ir_pin = (gpio_num_t)(intptr_t)parameters;
    bool gate_in = (ir_pin == g_app_config.ir_gatein_gpio);
    gpio_num_t servo_pin = gate_in ? g_app_config.gatein_servo_pin : g_app_config.gateout_servo_pin;
    const char *lane_name = gate_in ? "in" : "out";

    ir_sensor_t sensor;
    if (ir_sensor_init(&sensor, ir_pin, true) != ESP_OK)
    {
        ESP_LOGE(TAG, "Khởi tạo thất bại cảm biến IR GPIO %d", ir_pin);
        gate_tasks[gate_in ? 0 : 1] = NULL;
        vTaskDelete(NULL);
        return;
    }

    char ir_topic[128];
    snprintf(ir_topic, sizeof(ir_topic), "parking/v1/lots/%s/ir/%s/event", g_app_config.lot_id, lane_name);

    char gate_status_topic[128];
    snprintf(gate_status_topic, sizeof(gate_status_topic), "parking/v1/lots/%s/gates/%s/status", g_app_config.lot_id, lane_name);

    bool initialized = false;
    bool open = false;
    bool prev_detected = false;
    bool gate_opened_by_cmd = false; // CHỈ mở cổng khi biến này = true
    bool car_was_on_ir = false;
    TickType_t cmd_opened_time = 0;
    TickType_t last_report = 0;

    while (true)
    {
        TickType_t now = xTaskGetTickCount();
        bool detected = ir_sensor_is_detected(&sensor);

        // 1. Chỉ bắn MQTT báo tin cho Node B biết có xe tới, CHƯA MỞ CỔNG TẠI ĐÂY
        if (detected != prev_detected)
        {
            if (is_mqtt_connected)
            {
                char payload[128];
                snprintf(payload, sizeof(payload),
                         "{\"lotId\":\"%s\",\"lane\":\"%s\",\"event\":\"%s\",\"detected\":%s}",
                         g_app_config.lot_id,
                         lane_name,
                         detected ? "VEHICLE_ARRIVED" : "VEHICLE_LEFT",
                         detected ? "true" : "false");
                esp_mqtt_client_publish(mqtt_client, ir_topic, payload, 0, 1, 0);
            }
            ESP_LOGI(TAG, "[LÀN %s] Phát hiện xe: %s (Đang chờ lệnh từ Node B...)",
                     lane_name, detected ? "CÓ XE" : "TRỐNG");
            prev_detected = detected;
        }

        // 2. CHỈ MỞ CỔNG khi nhận được thông báo Notify từ MQTT (Node B gửi lệnh 'open')
        if (ulTaskNotifyTake(pdTRUE, 0))
        {
            gate_opened_by_cmd = true;
            car_was_on_ir = detected;
            cmd_opened_time = now;
            ESP_LOGI(TAG, "[LÀN %s] ===> ĐÃ NHẬN LỆNH MQTT: MỞ CỔNG!", lane_name);
        }

        // 3. Logic tự động đóng cổng sau khi xe đi qua
        if (gate_opened_by_cmd)
        {
            if (detected)
            {
                car_was_on_ir = true; // Bắt được trạng thái xe đang che mắt IR
            }

            // Khi xe đã đè IR và vừa di chuyển ra khỏi mắt IR -> ĐÓNG CỔNG
            if (car_was_on_ir && !detected)
            {
                gate_opened_by_cmd = false;
                car_was_on_ir = false;
                ESP_LOGI(TAG, "[LÀN %s] Xe đã đi qua hoàn toàn -> ĐÓNG CỔNG", lane_name);
            }
            // Trường hợp quá thời gian chờ (Timeout) -> Tự đóng an toàn
            else if (now - cmd_opened_time >= pdMS_TO_TICKS(CMD_OPEN_TIMEOUT_MS))
            {
                gate_opened_by_cmd = false;
                car_was_on_ir = false;
                ESP_LOGW(TAG, "[LÀN %s] Hết thời gian chờ -> ĐÓNG CỔNG TỰ ĐỘNG", lane_name);
            }
        }

        // Cổng CHỈ MỞ khi gate_opened_by_cmd == true
        bool should_open = gate_opened_by_cmd;
        bool state_changed = !initialized || (open != should_open);

        if (state_changed)
        {
            esp_err_t result = servo_set_angle(servo_pin,
                                               should_open ? g_app_config.servo_angle_open : g_app_config.servo_angle_close);
            if (result == ESP_OK)
            {
                open = should_open;
                initialized = true;
            }
        }

        // Báo cáo trạng thái cổng
        if (initialized && is_mqtt_connected &&
            (state_changed || now - last_report >= pdMS_TO_TICKS(1000)))
        {
            char payload[96];
            snprintf(payload, sizeof(payload), "{\"state\":\"%s\",\"remoteEnabled\":%s}",
                     open ? "open" : "closed", g_app_config.remote_gate_enabled ? "true" : "false");
            esp_mqtt_client_publish(mqtt_client, gate_status_topic, payload, 0, 0, 0);
            last_report = now;
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void init_ir_sensor_task(gpio_num_t ir_pin)
{
    bool gate_in = (ir_pin == g_app_config.ir_gatein_gpio);
    xTaskCreatePinnedToCore(vTaskIRSensor, "vTaskIRSensor", 3072,
                            (void *)(intptr_t)ir_pin, 5, &gate_tasks[gate_in ? 0 : 1], 1);
}