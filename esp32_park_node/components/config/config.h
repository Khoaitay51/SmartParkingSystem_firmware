#ifndef CONFIG_H
#define CONFIG_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <driver/gpio.h>
#define MAX_PARKING_LOTS 10
    // Hàm khởi tạo và chạy FreeRTOS Task cho cảm biến Siêu âm
    typedef struct
    {
        char wifi_ssid[32];
        char wifi_pass[64];
        char mqtt_broker_uri[128];
        gpio_num_t ir_gatein_gpio;
        gpio_num_t ir_gateout_gpio;
        uint8_t max_distance_cm;
        uint8_t distance_too_close_threshold_cm;
        uint8_t distance_detect_threshold_cm;
        uint8_t num_parking_lots;
        uint8_t trigger_pins[MAX_PARKING_LOTS];
        uint8_t echo_pins[MAX_PARKING_LOTS];
        gpio_num_t buzzer_gpio;
        gpio_num_t gatein_servo_pin;
        gpio_num_t gateout_servo_pin;
        uint8_t servo_angle_open;  // Góc mở cổng (0-180)
        uint8_t servo_angle_close; // Góc đóng cổng (0-180)
    } app_config_t;
    extern app_config_t g_app_config;
    esp_err_t init_spiffs(void);
    esp_err_t load_config_from_json(void);
#ifdef __cplusplus
}
#endif

#endif // CONFIG_H