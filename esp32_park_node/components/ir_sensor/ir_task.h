#ifndef IR_TASK_H
#define IR_TASK_H

#include "driver/gpio.h"
#include <stdbool.h>

void init_ir_sensor_task(gpio_num_t ir_pin);
void request_gate_open(bool gate_in);

#endif // IR_TASK_H
