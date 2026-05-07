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

    const float RAY_SAFE_LIMIT_MM = 100.0; // Đi từ tâm ra biên là 200mm (tổng sải 400mm)
    const float SINE_PERIOD_SEC = 10.0;    // Chu kỳ 10 giây
    const float PULLEY_CIRCUM = 80.0;    
    const float MIN_SPEED_HZ = 20.0;
    float max_rpm = 75;
    float t = 0.0;
    const float dt = 0.02;
    const float MAX_SPEED_MM_S = 400.0f;  // Vận tốc tối đa (rất nhanh)
const float ACCEL_MM_S2 = 1200.0f;    // Gia tốc (mm/s^2)
const float SAFE_BOUNDARY = 200.0f;   // Biên âm -200mm, biên dương +200mm (tổng 400mm)
const float STEPS_PER_MM = 40.0f;
float current_pos_mm = 0.0f;
float current_vel_mm_s = 0.0f;
float target_pos_mm = -SAFE_BOUNDARY; // Bắt đầu lao sang trái
while(1)
{
    // 1. Xác định hướng đi hiện tại
    float direction = (target_pos_mm > current_pos_mm) ? 1.0f : -1.0f;
    float dist_to_target = fabsf(target_pos_mm - current_pos_mm);

    // 2. Tính toán Vận tốc Hình thang (Trapezoidal)
    if (dist_to_target > 0.5f) { // Nếu chưa chạm đích
        // Tính quãng đường cần thiết để phanh dừng lại: s = v^2 / (2a)
        float braking_dist = (current_vel_mm_s * current_vel_mm_s) / (2.0f * ACCEL_MM_S2);

        if (dist_to_target > braking_dist) {
            // Đang xa đích -> Tăng tốc
            current_vel_mm_s += ACCEL_MM_S2 * 0.02f; // dt = 20ms = 0.02s
            if (current_vel_mm_s > MAX_SPEED_MM_S) current_vel_mm_s = MAX_SPEED_MM_S;
        } else {
            // Sắp đến đích -> Giảm tốc (phanh)
            current_vel_mm_s -= ACCEL_MM_S2 * 0.02f;
            if (current_vel_mm_s < 20.0f) current_vel_mm_s = 20.0f; // Vận tốc tối thiểu để không đứng im sớm
        }
    } else {
        // Đã chạm biên -> Dừng 0.3s rồi đổi mục tiêu sang biên ngược lại
        current_vel_mm_s = 0;
        vTaskDelay(pdMS_TO_TICKS(300));
        target_pos_mm = (target_pos_mm == -SAFE_BOUNDARY) ? SAFE_BOUNDARY : -SAFE_BOUNDARY;
    }

    // 3. Cập nhật vị trí giả lập và hướng motor
    current_pos_mm += current_vel_mm_s * 0.02f * direction;
    gpio_set_level(DIR_GPIO_NUM, (direction >= 0) ? 0 : 1);

    // 4. Quy đổi sang Hz và nạp vào MCPWM
    float speed_hz = current_vel_mm_s * STEPS_PER_MM;
    
    printf(">Pos_mm:%.2f\n", current_pos_mm);
    printf(">Vel_mm_s:%.2f\n", current_vel_mm_s * direction);

    if (speed_hz > MIN_SPEED_HZ) {
        uint32_t period = (uint32_t)(MCPWM_RESOLUTION_HZ / speed_hz);
        if (period > 65000) period = 65000;

        mcpwm_timer_set_period(timer, period);
        mcpwm_comparator_set_compare_value(comparator, period / 2);
        mcpwm_generator_set_force_level(generator, -1, true); 
    } 
    else {
        mcpwm_generator_set_force_level(generator, 0, true);
    }

    vTaskDelay(pdMS_TO_TICKS(20));
}



}







