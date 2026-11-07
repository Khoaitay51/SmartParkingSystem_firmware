#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "esp_spiffs.h"
#include "cJSON.h"
#include "driver/gpio.h"
#include "config.h"

static const char *TAG = "PARK_NODE_CONFIG";
void clean_string(char *str)
{
    int len = strlen(str);
    while (len > 0 && (str[len - 1] == '\r' || str[len - 1] == '\n' || str[len - 1] == ' '))
    {
        str[len - 1] = '\0';
        len--;
    }
}

// Gọi hàm này ngay sau khi cJSON parse xong chuỗi mqtt_broker_uri:

// 1. Mount hệ thống file SPIFFS
esp_err_t init_spiffs(void)
{
    esp_vfs_spiffs_conf_t conf = {
        .base_path = "/spiffs",
        .partition_label = "storage", // Phải trùng cột Name trong partitions.csv
        .max_files = 5,
        .format_if_mount_failed = true};

    esp_err_t ret = esp_vfs_spiffs_register(&conf);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Lỗi mount SPIFFS (%s)", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "Mount SPIFFS thành công!");
    return ESP_OK;
}

// 2. Hàm đọc và bóc tách toàn bộ file config.json
esp_err_t load_config_from_json(void)
{

    FILE *f = fopen("/spiffs/config.json", "r");
    if (f == NULL)
    {
        ESP_LOGW(TAG, "Không mở được /spiffs/config.json, sử dụng cấu hình mặc định!");
        return ESP_ERR_NOT_FOUND;
    }

    char buffer[1024];
    size_t bytes_read = fread(buffer, 1, sizeof(buffer) - 1, f);
    fclose(f);
    buffer[bytes_read] = '\0';

    cJSON *json = cJSON_Parse(buffer);
    if (json == NULL)
    {
        ESP_LOGE(TAG, "Lỗi cú pháp file JSON!");
        return ESP_FAIL;
    }

    // Parse Chuỗi (WiFi & MQTT)
    cJSON *item = cJSON_GetObjectItemCaseSensitive(json, "wifi_ssid");
    if (cJSON_IsString(item) && item->valuestring)
    {
        strncpy(g_app_config.wifi_ssid, item->valuestring, sizeof(g_app_config.wifi_ssid) - 1);
    }

    item = cJSON_GetObjectItemCaseSensitive(json, "wifi_pass");
    if (cJSON_IsString(item) && item->valuestring)
    {
        strncpy(g_app_config.wifi_pass, item->valuestring, sizeof(g_app_config.wifi_pass) - 1);
    }

    item = cJSON_GetObjectItemCaseSensitive(json, "mqtt_broker_uri");
    if (cJSON_IsString(item) && item->valuestring)
    {
        strncpy(g_app_config.mqtt_broker_uri, item->valuestring, sizeof(g_app_config.mqtt_broker_uri) - 1);
    }
    clean_string(g_app_config.mqtt_broker_uri);
    // Parse Chân GPIO IR & Buzzer
    item = cJSON_GetObjectItemCaseSensitive(json, "ir_gatein_gpio");
    if (cJSON_IsNumber(item))
        g_app_config.ir_gatein_gpio = (gpio_num_t)item->valueint;

    item = cJSON_GetObjectItemCaseSensitive(json, "ir_gateout_gpio");
    if (cJSON_IsNumber(item))
        g_app_config.ir_gateout_gpio = (gpio_num_t)item->valueint;

    item = cJSON_GetObjectItemCaseSensitive(json, "buzzer_gpio");
    if (cJSON_IsNumber(item))
        g_app_config.buzzer_gpio = (gpio_num_t)item->valueint;

    // Parse Thông số Khoảng cách (Ultrasonic Thresholds)
    item = cJSON_GetObjectItemCaseSensitive(json, "max_distance_cm");
    if (cJSON_IsNumber(item))
        g_app_config.max_distance_cm = (uint32_t)item->valueint;

    item = cJSON_GetObjectItemCaseSensitive(json, "distance_too_close_threshold_cm");
    if (cJSON_IsNumber(item))
        g_app_config.distance_too_close_threshold_cm = (uint32_t)item->valueint;

    item = cJSON_GetObjectItemCaseSensitive(json, "distance_detect_threshold_cm");
    if (cJSON_IsNumber(item))
        g_app_config.distance_detect_threshold_cm = (uint32_t)item->valueint;

    // Parse Số lượng Slot
    item = cJSON_GetObjectItemCaseSensitive(json, "num_parking_lots");
    if (cJSON_IsNumber(item))
        g_app_config.num_parking_lots = (uint8_t)item->valueint;
    item = cJSON_GetObjectItemCaseSensitive(json, "gatein_servo_pin");
    if (cJSON_IsNumber(item))
        g_app_config.gatein_servo_pin = (gpio_num_t)item->valueint;
    item = cJSON_GetObjectItemCaseSensitive(json, "gateout_servo_pin");
    if (cJSON_IsNumber(item))
        g_app_config.gateout_servo_pin = (gpio_num_t)item->valueint;
    item = cJSON_GetObjectItemCaseSensitive(json, "servo_angle_open");
    if (cJSON_IsNumber(item))
        g_app_config.servo_angle_open = (uint8_t)item->valueint;
    item = cJSON_GetObjectItemCaseSensitive(json, "servo_angle_close");
    if (cJSON_IsNumber(item))
        g_app_config.servo_angle_close = (uint8_t)item->valueint;

    // Parse Mảng Trigger Pins
    cJSON *triggers = cJSON_GetObjectItemCaseSensitive(json, "trigger_pins");
    if (cJSON_IsArray(triggers))
    {
        int count = cJSON_GetArraySize(triggers);
        if (count > MAX_PARKING_LOTS)
        {
            count = MAX_PARKING_LOTS;
        }

        for (int i = 0; i < count; i++)
        {
            cJSON *p = cJSON_GetArrayItem(triggers, i);
            if (cJSON_IsNumber(p))
            {
                // Lấy valuedouble rồi ép kiểu uint8_t để an toàn với mọi định dạng số JSON
                g_app_config.trigger_pins[i] = (uint8_t)p->valuedouble;
            }
        }
    }

    // Parse Mảng Echo Pins
    cJSON *echos = cJSON_GetObjectItemCaseSensitive(json, "echo_pins");
    if (cJSON_IsArray(echos))
    {
        int count = cJSON_GetArraySize(echos);
        if (count > MAX_PARKING_LOTS)
        {
            count = MAX_PARKING_LOTS;
        }

        for (int i = 0; i < count; i++)
        {
            cJSON *p = cJSON_GetArrayItem(echos, i);
            if (cJSON_IsNumber(p))
            {
                g_app_config.echo_pins[i] = (uint8_t)p->valuedouble;
            }
        }
    }

    cJSON_Delete(json);

    // Log kiểm tra kết quả
    ESP_LOGI(TAG, "=== ĐÃ LẠP THÀNH CÔNG G_APP_CONFIG ===");
    ESP_LOGI(TAG, "WiFi: SSID='%s', Pass='%s'", g_app_config.wifi_ssid, g_app_config.wifi_pass);
    ESP_LOGI(TAG, "MQTT Broker: %s", g_app_config.mqtt_broker_uri);
    ESP_LOGI(TAG, "GPIOs: IR_In=%d, IR_Out=%d, Buzzer=%d",
             g_app_config.ir_gatein_gpio, g_app_config.ir_gateout_gpio, g_app_config.buzzer_gpio);
    ESP_LOGI(TAG, "Ultrasonic Thresholds: Max=%ldcm, TooClose=%ldcm, Detect=%ldcm",
             g_app_config.max_distance_cm,
             g_app_config.distance_too_close_threshold_cm,
             g_app_config.distance_detect_threshold_cm);
    ESP_LOGI(TAG, "Số Slot bãi xe: %d", g_app_config.num_parking_lots);
    for (int i = 0; i < g_app_config.num_parking_lots; i++)
    {
        ESP_LOGI(TAG, "  -> Slot [%d]: Trig=GPIO_%d, Echo=GPIO_%d",
                 i, g_app_config.trigger_pins[i], g_app_config.echo_pins[i]);
    }

    return ESP_OK;
}