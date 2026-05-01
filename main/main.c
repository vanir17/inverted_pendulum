
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "driver/pulse_cnt.h"
#include "driver/gpio.h"
#include "esp_sleep.h"
#include "esp_err.h"
#include "encoder.h"
#include "encoder_defs.h"

static const char *TAG = "Example";






void app_main(void)
{

    pcnt_unit_handle_t encoder_hn3806_t;
    encoder_t* encoder_handle_t;
    ESP_ERROR_CHECK(init_encoder(&encoder_hn3806_t, &encoder_handle_t));
    ESP_LOGI(TAG, "Initiated successfully!\n\n\n");



    float angle = 0;

    
    while (1)
     {


        if((encoder_get_degrees(encoder_handle_t, &angle)) == ESP_OK)
        {
            ESP_LOGI(TAG, "Angle: %.2f", angle);
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }

}
