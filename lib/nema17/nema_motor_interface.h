
#pragma once

#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif



/**
 * @brief BDC motor interface definition
 */
struct motor_t {
    /**
     * @brief Enable motor
     *
     * @param motor: Motor handle
     *
     * @return
     *      - ESP_OK: Enable motor successfully
     *      - ESP_ERR_INVALID_ARG: Enable motor failed because of invalid parameters
     *      - ESP_FAIL: Enable motor failed because other error occurred
     */
    esp_err_t (*enable)(motor_t *motor);

    /**
     * @brief Disable motor
     *
     * @param motor: Motor handle
     *
     * @return
     *      - ESP_OK: Disable motor successfully
     *      - ESP_ERR_INVALID_ARG: Disable motor failed because of invalid parameters
     *      - ESP_FAIL: Disable motor failed because other error occurred
     */
    esp_err_t (*disable)(motor_t *motor);

    /**
     * @brief Set speed for nema motor
     *
     * @param motor: Motor handle
     * @param speed: speed
     *
     * @return
     *      - ESP_OK: Set motor speed successfully
     *      - ESP_ERR_INVALID_ARG: Set motor speed failed because of invalid parameters
     *      - ESP_FAIL: Set motor speed failed because other error occurred
     */
    esp_err_t (*set_speed)(motor_t *motor, uint32_t speed);


#ifdef __cplusplus
}
#endif