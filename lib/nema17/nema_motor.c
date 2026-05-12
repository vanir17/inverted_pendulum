#include <stdlib.h>
#include <string.h>
#include <sys/cdefs.h>
#include "esp_log.h"
#include "esp_check.h"
#include "nema_motor.h"
#include "nema_motor_interface.h"

static const char *TAG = "motor";

esp_err_t motor_enable(motor_handle_t motor)
{
    ESP_RETURN_ON_FALSE(motor, ESP_ERR_INVALID_ARG, TAG, "invalid argument");
    return motor->enable(motor);
}

esp_err_t motor_disable(motor_handle_t motor);
{
    ESP_RETURN_ON_FALSE(motor, ESP_ERR_INVALID_ARG, TAG, "invalid argument");
    return motor->disable(motor);
}

esp_err_t motor_set_speed(motor_handle_t motor, uint32_t speed_hz)
{
    ESP_RETURN_ON_FALSE(motor, ESP_ERR_INVALID_ARG, TAG, "invalid argument");
    return motor->set_speed(motor, speed_hz);
}
