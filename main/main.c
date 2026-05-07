#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/mcpwm_prelude.h"
#include "driver/gpio.h"
#include "math.h"
#include "esp_log.h"
#include <math.h> 


#define STEP_GPIO_NUM            23
#define DIR_GPIO_NUM             22
#define MCPWM_RESOLUTION_HZ 1000000 // 1MHz 
#define STEPS_PER_REV       3200
#define PULLEY_CIRCUM_MM    40.0    // 20 răng * 2mm
#define STEPS_PER_MM        (STEPS_PER_REV / PULLEY_CIRCUM_MM) // 80 steps/mm
#define MOTOR_TIMEBASE_PERIOD 20000

static const char* TAG = "TEST_MCPWM";



void app_main(void) 
{

    ESP_LOGI(TAG, "Create timer and operator");
    /*Timer*/
    mcpwm_timer_handle_t timer = NULL;
    mcpwm_timer_config_t timer_config = {
        .group_id = 0,
        .clk_src = 1,
        .resolution_hz = MCPWM_RESOLUTION_HZ,
        .period_ticks = MOTOR_TIMEBASE_PERIOD,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
        .flags.update_period_on_empty = true,
    };
    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &timer));

    /*Operator*/
    mcpwm_oper_handle_t oper = NULL;
    mcpwm_operator_config_t operator_config = {
        .group_id = 0, // must be as the same as timer_config
    };
    ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &oper));

    ESP_LOGI(TAG, "Connect timer and operator");
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper, timer));


    ESP_LOGI(TAG, "Create comparator and generator from the operator");
    
    mcpwm_cmpr_handle_t comparator = NULL;
    mcpwm_comparator_config_t comparator_config = {
        .flags.update_cmp_on_tez = true,
    };
    ESP_ERROR_CHECK(mcpwm_new_comparator(oper, &comparator_config, &comparator));

    mcpwm_gen_handle_t generator = NULL;
    mcpwm_generator_config_t generator_config = {
        .gen_gpio_num = STEP_GPIO_NUM,
    };
    ESP_ERROR_CHECK(mcpwm_new_generator(oper, &generator_config, &generator));


    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(generator, 
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
    
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(generator,
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparator, MCPWM_GEN_ACTION_LOW)));


    /*Initiate Direction GPIO*/
    gpio_set_direction(DIR_GPIO_NUM, GPIO_MODE_OUTPUT);



    ESP_LOGI(TAG, "Enable and start timer");
    ESP_ERROR_CHECK(mcpwm_timer_enable(timer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP));

    const float RAY_SAFE_LIMIT_MM = 200.0; // Đi từ tâm ra biên là 200mm (tổng sải 400mm)
    const float SINE_PERIOD_SEC = 10.0;    // Chu kỳ 10 giây
    const float PULLEY_CIRCUM = 80.0;    
    const float MIN_SPEED_HZ = 20.0;
    float max_rpm = 75;
    float t = 0.0;
    const float dt = 0.02;
    while(1)
    {
        float current_rpm = max_rpm * sinf(2.0f * M_PI * (1.0f / SINE_PERIOD_SEC) * t);        // float speed = (rpm * 3200.0) / 60.0;
        // uint32_t period = MCPWM_RESOLUTION_HZ / (uint32_t)speed;
        gpio_set_level(DIR_GPIO_NUM, (current_rpm >= 0) ? 0 : 1);

        float speed_hz = (fabsf(current_rpm) * 3200.0f) / 60.0f;
        printf(">RPM:%.2f\n", current_rpm);
    if (speed_hz > MIN_SPEED_HZ) {
            uint32_t period = (uint32_t)(MCPWM_RESOLUTION_HZ / speed_hz);
            
            // Giới hạn period an toàn (thường MCPWM timer 16-bit max là 65535)
            if (period > 65000) period = 65000;

            mcpwm_timer_set_period(timer, period);
            mcpwm_comparator_set_compare_value(comparator, period / 2);
            
            // Đảm bảo Generator đang phát xung
            mcpwm_generator_set_force_level(generator, -1, true); 
        } else {
            // Tốc độ quá thấp (điểm chết của sóng sine) -> Dừng phát xung
            mcpwm_generator_set_force_level(generator, 0, true);
        }

        // 5. Cập nhật thời gian và Delay
        t += dt;
        // Reset t để tránh sai số trôi điểm động khi chạy thời gian cực dài
        if (t > SINE_PERIOD_SEC) t -= SINE_PERIOD_SEC;

        vTaskDelay(pdMS_TO_TICKS(20));

    }



}







