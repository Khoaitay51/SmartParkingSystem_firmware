#include "ir_sensor.h"
#include "esp_log.h"

static const char *TAG = "IR_SENSOR_LIB";

esp_err_t ir_sensor_init(ir_sensor_t *sensor, gpio_num_t pin, bool active_low)
{
    if (sensor == NULL)
    {
        ESP_LOGE(TAG, "Con trỏ sensor bị NULL!");
        return ESP_ERR_INVALID_ARG;
    }

    sensor->pin = pin;
    sensor->active_low = active_low;

    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << pin),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE};

    esp_err_t err = gpio_config(&io_conf);
    if (err == ESP_OK)
    {
        ESP_LOGI(TAG, "Đã khởi tạo cảm biến IR tại GPIO %d", pin);
    }
    else
    {
        ESP_LOGE(TAG, "Lỗi cấu hình GPIO %d", pin);
    }

    return err;
}

bool ir_sensor_is_detected(const ir_sensor_t *sensor)
{
    if (sensor == NULL)
    {
        return false;
    }

    int level = gpio_get_level(sensor->pin);

    // Nếu active_low = true, mức LOW (0) tương ứng với có vật thể
    if (sensor->active_low)
    {
        return (level == 0);
    }
    else
    {
        return (level == 1);
    }
}