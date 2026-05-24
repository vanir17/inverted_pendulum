#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/mcpwm_prelude.h"
#include "driver/pulse_cnt.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_attr.h"

#include "nema_motor.h"
#include "nema_motor_defs.h"

// --- Cấu hình Hardware ---
#define STEP_GPIO       23
#define DIR_GPIO        22
#define PCNT_STEP_INPUT 21 // Nối dây từ 23 sang 21
#define PCNT_DIR_INPUT  19 // Nối dây từ 22 sang 19

// --- Thông số hành trình ---
#define STEPS_PER_MM    40
#define RANGE_MM        200
#define MAX_STEPS       (RANGE_MM * STEPS_PER_MM) // 9000

static const char* TAG = "PENDULUM_SAFETY";

pcnt_unit_handle_t pcnt_unit = NULL;
motor_handle_t stepper_motor = NULL;

static bool IRAM_ATTR on_reach_boundary(pcnt_unit_handle_t unit, const pcnt_watch_event_data_t *edata, void *user_ctx)
{
    motor_set_speed(stepper_motor, 0);
    return false;
}

void init_system() 
{
    motor_mcpwm_config_t mcpwm_config = { .group_id = 0, .resolution_hz = 1000000 };
    motor_config_t stepper_config = {
        .dir_gpio_num = DIR_GPIO,
        .pwma_gpio_num = STEP_GPIO,
        .pwm_freq_hz = 1000,
    };
    ESP_ERROR_CHECK(motor_new_mcpwm_device(&stepper_config, &mcpwm_config, &stepper_motor));
    motor_enable(stepper_motor);

    pcnt_unit_config_t unit_config = {
        .high_limit = 10000, 
        .low_limit = -10000,
    };
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &pcnt_unit));

    pcnt_chan_config_t chan_config = { .edge_gpio_num = PCNT_STEP_INPUT, .level_gpio_num = PCNT_DIR_INPUT };
    pcnt_channel_handle_t pcnt_chan = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_config, &pcnt_chan));

    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_HOLD));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_unit, MAX_STEPS));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_unit, -MAX_STEPS));

    pcnt_event_callbacks_t cbs = { .on_reach = on_reach_boundary };
    ESP_ERROR_CHECK(pcnt_unit_register_event_callbacks(pcnt_unit, &cbs, NULL));

    ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit)); 
    ESP_ERROR_CHECK(pcnt_unit_start(pcnt_unit));
}

void safe_move(int speed_hz, int direction) 
{
    int current_pos = 0;
    pcnt_unit_get_count(pcnt_unit, &current_pos);

    if (direction == 1 && current_pos >= MAX_STEPS) {
        motor_set_speed(stepper_motor, 0);
        return;
    }
    if (direction == 0 && current_pos <= -MAX_STEPS) {
        motor_set_speed(stepper_motor, 0);
        return;
    }

    gpio_set_level(DIR_GPIO, direction);
    motor_set_speed(stepper_motor, speed_hz);
}

void app_main(void) 
{
    init_system();
    ESP_LOGI(TAG, "He thong san sang. Vi tri hien tai la TAM (0).");
    int motor_hz = 10000;    
    int target_direction = 1; 
    while(1) 
    {
        int pos = 0;
        pcnt_unit_get_count(pcnt_unit, &pos);

        if (target_direction == 1 && pos >= MAX_STEPS) {
            target_direction = 0; // Đổi sang trái
            vTaskDelay(pdMS_TO_TICKS(500)); 
        } 
        else if (target_direction == 0 && pos <= -MAX_STEPS) {
            target_direction = 1; 
            vTaskDelay(pdMS_TO_TICKS(500)); 
        }

        safe_move(motor_hz, target_direction);

        ESP_LOGI(TAG, "Vi tri: %d | Huong: %s", pos, target_direction ? "PHAI" : "TRAI");
        
        vTaskDelay(pdMS_TO_TICKS(20)); 
    }
}