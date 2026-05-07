#pragma once

#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C"{
#endif

typedef struct nema_motor_t *nema_motor_handle_t;



/**
 * @brief Enable Nema motor
 *
 * @param motor: Nema motor handle
 *
 * @return
 *      - ESP_OK: Enable motor successfully
 *      - ESP_ERR_INVALID_ARG: Enable motor failed because of invalid parameters
 *      - ESP_FAIL: Enable motor failed because other error occurred
 */
esp_err_t nema_motor_enable(nema_motor_handle_t motor);

/**
 * @brief Disable Nema motor
 *
 * @param motor: Nema motor
 *
 * @return
 *      - ESP_OK: Disable motor successfully
 *      - ESP_ERR_INVALID_ARG: Disable motor failed because of invalid parameters
 *      - ESP_FAIL: Disable motor failed because other error occurred
 */
esp_err_t nema_motor_disable(nema_motor_handle_t motor);


/**
 * @brief Set speed for Nema motor
 *
 * @param motor: Nema motor handle
 * @param speed: Nema speed
 *
 * @return
 *      - ESP_OK: Set motor speed successfully
 *      - ESP_ERR_INVALID_ARG: Set motor speed failed because of invalid parameters
 *      - ESP_FAIL: Set motor speed failed because other error occurred
 */
esp_err_t nema_motor_set_speed(nema_motor_handle_t motor, uint32_t speed);



/**
 * @brief BDC Motor Configuration
 */
typedef struct
{
    uint32_t step_gpio_num; /* Nema motor step(pulse) gpio number */
    uint32_t dir_gpio_num; /* Nema motor direction gpio number */
}nema_motor_config_t;


/**
 * @brief Nema Motor MCPWM specific configuration
 */
typedef struct 
{
    int group_id; /* MCPWM group number */
    uint32_t resolution_hz; /* MCPWM timer resolution */
}nema_motor_mcpwm_config_t;


/**
 * @brief Create BDC Motor based on MCPWM peripheral
 *
 * @param motor_config: BDC Motor configuration
 * @param mcpwm_config: MCPWM specific configuration
 * @param ret_motor Returned BDC Motor handle
 * @return
 *      - ESP_OK: Create BDC Motor handle successfully
 *      - ESP_ERR_INVALID_ARG: Create BDC Motor handle failed because of invalid argument
 *      - ESP_ERR_NO_MEM: Create BDC Motor handle failed because of out of memory
 *      - ESP_FAIL: Create BDC Motor handle failed because some other error
 */
esp_err_t nema_motor_new_mcpwm_device(const nema_motor_config_t *motor_config, const nema_motor_mcpwm_config_t *mcpwm_config, nema_motor_handle_t *ret_motor);

#ifdef __cplusplus
}
#endif
