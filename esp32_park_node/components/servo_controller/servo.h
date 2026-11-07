#ifndef SERVO_H
#define SERVO_H

#include <stdint.h>
#include "esp_err.h"
#include "driver/gpio.h"
#include "driver/ledc.h"

#ifdef __cplusplus
extern "C"
{
#endif

    esp_err_t servo_init(gpio_num_t servo_pin);
    esp_err_t servo_set_angle(gpio_num_t servo_pin, uint32_t angle);
    esp_err_t servo_deinit(void);

#ifdef __cplusplus
}
#endif

#endif // SERVO_H