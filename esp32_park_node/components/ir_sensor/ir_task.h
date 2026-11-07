#ifndef IR_TASK_H
#define IR_TASK_H

#include "driver/gpio.h"

void init_ir_sensor_task(gpio_num_t ir_pin);

#endif // IR_TASK_H