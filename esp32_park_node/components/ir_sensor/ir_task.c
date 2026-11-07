#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "mqtt_client.h" // Thêm thư viện MQTT client
#include "config.h"
#include "ir_sensor.h"
#include "ir_task.h"
#include "servo.h"
// Khai báo TAG cho logging
static const char *TAG = "IR_TASK";

// Khai báo extern các biến/handle MQTT được định nghĩa ở file main.c (hoặc file mqtt.c)
extern esp_mqtt_client_handle_t mqtt_client;
extern bool is_mqtt_connected;

#define MQTT_TOPIC_DETECT "esp32/ir_sensor/object_detected"

void vTaskIRSensor(void *pvParameters)
{
    // Giải ép kiểu lấy lại chân GPIO
    gpio_num_t ir_pin = (gpio_num_t)(intptr_t)pvParameters;
    gpio_num_t gate_servo_pin = (ir_pin == g_app_config.ir_gatein_gpio) ? g_app_config.gatein_servo_pin : g_app_config.gateout_servo_pin; // Lấy chân servo từ cấu hình
    ir_sensor_t ir_sensor;
    if (ir_sensor_init(&ir_sensor, ir_pin, true) != ESP_OK)
    {
        ESP_LOGE(TAG, "Khởi tạo cảm biến thất bại tại GPIO %d!", ir_pin);
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "Task cảm biến IR bắt đầu giám sát tại GPIO %d", ir_pin);

    bool last_state = false;

    while (1)
    {

        bool current_state = ir_sensor_is_detected(&ir_sensor);

        // Phát hiện sườn lên (Rising Edge): Vật thể vừa xuất hiện
        if (current_state && !last_state)
        {
            ESP_LOGW(TAG, "[GPIO %d] -> Phát hiện vật thể xuất hiện!", ir_pin);
            servo_set_angle(gate_servo_pin, g_app_config.servo_angle_open); // Mở cổng vào
            if (is_mqtt_connected && mqtt_client != NULL)
            {
                const char *payload = "{\"status\": \"object_detected\", \"value\": 1}";
                int msg_id = esp_mqtt_client_publish(mqtt_client, MQTT_TOPIC_DETECT, payload, 0, 1, 0);
                ESP_LOGI(TAG, "Đã gửi MQTT (msg_id=%d)", msg_id);
            }
            else
            {
                ESP_LOGE(TAG, "Chưa kết nối MQTT, không thể gửi dữ liệu!");
            }
        }
        else if (!current_state && last_state)
        {
            servo_set_angle(gate_servo_pin, g_app_config.servo_angle_close); // Đóng cổng vào
            ESP_LOGI(TAG, "[GPIO %d] Vật thể đã rời đi.", ir_pin);
        }

        last_state = current_state;
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void init_ir_sensor_task(gpio_num_t ir_pin)
{
    // Nếu dùng xTaskCreatePinnedToCore (Ghim Task vào Core 1 của ESP32):
    xTaskCreatePinnedToCore(
        vTaskIRSensor,            // Hàm thực thi Task
        "vTaskIRSensor",          // Tên Task
        3072,                     // Stack size (tăng lên 3072 bytes cho an toàn với MQTT/Logging)
        (void *)(intptr_t)ir_pin, // Tham số truyền vào
        5,                        // Độ ưu tiên (Priority)
        NULL,                     // Task handle
        1                         // Ghim vào Core 1
    );

    /*
    Hoặc nếu chỉ dùng xTaskCreate chuẩn của FreeRTOS (không chỉ định Core):
    xTaskCreate(vTaskIRSensor, "vTaskIRSensor", 3072, (void *)(intptr_t)ir_pin, 5, NULL);
    */
}