#include <stdbool.h>
#include <stddef.h>
#include "driver/ledc.h"
#include "servo.h"

// Định nghĩa thông số cấu hình LEDC
#define LEDC_TIMER LEDC_TIMER_0
#define LEDC_MODE LEDC_LOW_SPEED_MODE
#define LEDC_DUTY_RES LEDC_TIMER_13_BIT // Độ phân giải 13-bit (0 - 8191)
#define LEDC_FREQUENCY 50               // Tần số PWM 50 Hz (20ms)

// Thông số độ rộng xung tiêu chuẩn (SG90)
#define SERVO_MIN_PULSEWIDTH_US 500
#define SERVO_MAX_PULSEWIDTH_US 2500
#define SERVO_MAX_DEGREE 180

typedef struct
{
    gpio_num_t pin;
    ledc_channel_t channel;
    bool initialized;
} servo_channel_t;

static servo_channel_t servo_channels[LEDC_CHANNEL_MAX];
static bool timer_initialized;

static servo_channel_t *find_servo_channel(gpio_num_t servo_pin)
{
    for (size_t i = 0; i < LEDC_CHANNEL_MAX; i++)
    {
        if (servo_channels[i].initialized && servo_channels[i].pin == servo_pin)
        {
            return &servo_channels[i];
        }
    }

    return NULL;
}

static uint32_t angle_to_duty(uint32_t angle)
{
    if (angle > SERVO_MAX_DEGREE)
    {
        angle = SERVO_MAX_DEGREE;
    }

    uint32_t pulse_width_us = SERVO_MIN_PULSEWIDTH_US +
                              ((SERVO_MAX_PULSEWIDTH_US - SERVO_MIN_PULSEWIDTH_US) * angle) / SERVO_MAX_DEGREE;

    return (pulse_width_us * ((1 << LEDC_DUTY_RES) - 1)) / 20000;
}

esp_err_t servo_init(gpio_num_t servo_pin)
{
    if (servo_pin < 0 || servo_pin >= GPIO_NUM_MAX)
    {
        return ESP_ERR_INVALID_ARG;
    }

    servo_channel_t *servo = find_servo_channel(servo_pin);
    if (servo == NULL)
    {
        for (size_t i = 0; i < LEDC_CHANNEL_MAX; i++)
        {
            if (!servo_channels[i].initialized)
            {
                servo = &servo_channels[i];
                servo->pin = servo_pin;
                servo->channel = (ledc_channel_t)i;
                break;
            }
        }
    }
    if (servo == NULL)
    {
        return ESP_ERR_NO_MEM;
    }

    if (!timer_initialized)
    {
        ledc_timer_config_t ledc_timer = {
            .speed_mode = LEDC_MODE,
            .timer_num = LEDC_TIMER,
            .duty_resolution = LEDC_DUTY_RES,
            .freq_hz = LEDC_FREQUENCY,
            .clk_cfg = LEDC_AUTO_CLK};
        esp_err_t ret = ledc_timer_config(&ledc_timer);
        if (ret != ESP_OK)
        {
            return ret;
        }
        timer_initialized = true;
    }

    ledc_channel_config_t ledc_channel = {
        .speed_mode = LEDC_MODE,
        .channel = servo->channel,
        .timer_sel = LEDC_TIMER,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = servo_pin,
        .duty = 0,
        .hpoint = 0};
    esp_err_t ret = ledc_channel_config(&ledc_channel);
    if (ret == ESP_OK)
    {
        servo->initialized = true;
    }
    return ret;
}

esp_err_t servo_set_angle(gpio_num_t servo_pin, uint32_t angle)
{
    servo_channel_t *servo = find_servo_channel(servo_pin);
    if (servo == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    uint32_t duty = angle_to_duty(angle);
    esp_err_t ret = ledc_set_duty(LEDC_MODE, servo->channel, duty);
    if (ret != ESP_OK)
    {
        return ret;
    }
    return ledc_update_duty(LEDC_MODE, servo->channel);
}

esp_err_t servo_deinit(void)
{
    esp_err_t first_error = ESP_OK;
    for (size_t i = 0; i < LEDC_CHANNEL_MAX; i++)
    {
        if (servo_channels[i].initialized)
        {
            esp_err_t ret = ledc_stop(LEDC_MODE, servo_channels[i].channel, 0);
            if (ret != ESP_OK && first_error == ESP_OK)
            {
                first_error = ret;
            }
            if (ret == ESP_OK)
            {
                servo_channels[i].initialized = false;
            }
        }
    }
    return first_error;
}