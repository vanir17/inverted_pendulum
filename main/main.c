#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/mcpwm_prelude.h"
#include "driver/gpio.h"
#include "math.h"
#include "esp_log.h"
#include <math.h> 

#include "nema_motor.h"
#include "nema_motor_defs.h"
#include "encoder.h"
#include "encoder_defs.h"

#define PPR      3200
#define MOTOR_TIMEBASE_PERIOD 20000
#define STEP_GPIO 23
#define DIR_GPIO  22
static const char* TAG = "--MAIN";




void app_main(void) 
{

    pcnt_unit_handle_t encoder_hn3806_unit = NULL;
    encoder_t* encoder_hn3806 = NULL;
    ESP_ERROR_CHECK(init_encoder(&encoder_hn3806_unit, &encoder_hn3806));

    float angle_radian = 0.0;
    float angle_degreee = 0.0;


    while(1)
    {

        ESP_ERROR_CHECK(encoder_get_radian(encoder_hn3806, &angle_radian));
        ESP_ERROR_CHECK(encoder_get_degrees(encoder_hn3806, &angle_degreee));
        printf("/*%.2f*/\n", angle_degreee);
        vTaskDelay(pdMS_TO_TICKS(10));
    }

}







