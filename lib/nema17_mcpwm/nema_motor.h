#pragma once

#include <stdint.h>
#include "esp_err.h"

typedef struct motor_t motor_t; 
typedef struct motor_t *motor_handle_t;

typedef struct motor_mcpwm {
    int group_id; /* MCPWM group number */
    uint32_t resolution_hz; /* MCPWM timer resolution */
}motor_mcpwm_config_t;

typedef struct motor_config
{
    uint32_t pwma_gpio_num; /* Nema motor step(pulse) gpio number */
    uint32_t pwm_freq_hz; /* PWM frequency in Hz */
    uint32_t dir_gpio_num;
}motor_config_t;



esp_err_t motor_enable(motor_handle_t motor);

esp_err_t motor_disable(motor_handle_t motor);

esp_err_t motor_set_speed(motor_handle_t motor, uint32_t speed_hz);

esp_err_t motor_new_mcpwm_device(const motor_config_t *motor_config, const motor_mcpwm_config_t *mcpwm_config, motor_handle_t *ret_motor);

