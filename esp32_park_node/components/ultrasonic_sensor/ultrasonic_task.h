#ifndef ULTRASONIC_TASK_H
#define ULTRASONIC_TASK_H

#include "driver/gpio.h"
#include "ultrasonic.h"
#include "config.h"
#ifdef __cplusplus
extern "C"
{
#endif
    // Hàm khởi tạo và chạy FreeRTOS Task cho cảm biến Siêu âm
    void init_ultrasonic_task(uint8_t lot_id, gpio_num_t trigger_pin, gpio_num_t echo_pin);
    extern bool is_having_car[MAX_PARKING_LOTS]; // Mảng trạng thái xe trong từng slot (false: trống, true: có xe)
#ifdef __cplusplus
}
#endif

#endif // ULTRASONIC_TASK_H