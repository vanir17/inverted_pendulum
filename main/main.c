#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/mcpwm_prelude.h"
#include "driver/gpio.h"
#include "math.h"
#include "esp_log.h"
#include <math.h> 

#include "nema_motor.h"
#include "nema_motor_defs.h"


#define PPR      3200
#define MOTOR_TIMEBASE_PERIOD 20000
#define STEP_GPIO 23
#define DIR_GPIO  22
static const char* TAG = "--MAIN";




void app_main(void) 
{

    motor_handle_t motor = NULL;

    motor_mcpwm_config_t mcpwm_config = 
    {
        .group_id = 0,
        .resolution_hz = MCPWM_RESOLUTION_HZ,
    };

    motor_config_t motor_conf = {
        .pwma_gpio_num = STEP_GPIO,
        .dir_gpio_num = DIR_GPIO,
        .pwm_freq_hz = 1000, 
    };

    ESP_LOGI(TAG, "Initializing Nema 17 motor...");
    ESP_ERROR_CHECK(motor_new_mcpwm_device(&motor_conf, &mcpwm_config, &motor));

    motor_enable(motor);

    while(1)
    {
        motor_set_speed(motor, 0);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }



}







