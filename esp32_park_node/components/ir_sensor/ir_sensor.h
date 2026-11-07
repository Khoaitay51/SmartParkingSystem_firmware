#ifndef IR_SENSOR_H
#define IR_SENSOR_H

#include "driver/gpio.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C"
{
#endif

    // Cấu trúc quản lý đối tượng cảm biến IR
    typedef struct
    {
        gpio_num_t pin;  // Chân GPIO kết nối với OUT
        bool active_low; // true nếu cảm biến xuất mức LOW khi có vật thể
    } ir_sensor_t;

    /**
     * @brief Khởi tạo cảm biến IR
     * @param sensor Con trỏ đến cấu trúc ir_sensor_t
     * @param pin Chân GPIO nối với tín hiệu OUT
     * @param active_low Đặt true nếu module xuất mức LOW khi phát hiện vật thể (thường là true)
     * @return esp_err_t ESP_OK nếu thành công
     */
    esp_err_t ir_sensor_init(ir_sensor_t *sensor, gpio_num_t pin, bool active_low);

    /**
     * @brief Kiểm tra xem có phát hiện vật thể hay không
     * @param sensor Con trỏ đến cấu trúc ir_sensor_t
     * @return true nếu có vật thể, false nếu không có
     */
    bool ir_sensor_is_detected(const ir_sensor_t *sensor);

#ifdef __cplusplus
}
#endif

#endif // IR_SENSOR_H