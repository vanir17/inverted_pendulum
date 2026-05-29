#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/mcpwm_prelude.h"
#include "driver/pulse_cnt.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_attr.h"
#include "stdio.h"
#include "math.h"


#include "encoder.h"
#include "encoder_defs.h"
#include "nema_motor.h"
#include "nema_motor_defs.h"

// --- Cấu hình Hardware ---
#define STEP_GPIO       23
#define DIR_GPIO_NUM        22
#define PCNT_STEP_INPUT 21 // Nối dây từ 23 sang 21
#define PCNT_DIR_INPUT  19 // Nối dây từ 22 sang 19
#define EN_GPIO_TMC2209 18

#define STEPS_PER_MM    40
#define RANGE_MM        200
#define MAX_STEPS       (RANGE_MM * STEPS_PER_MM) // 9000
// #define M_PI 3.14159f
#define DT 0.005f
#define MAX_ACCEL 10 // m/s^2
#define MAX_VELOCITY 10 // m/s
#define PULLEY_CIRCUMFERENCE 0.08f // Pulley: C = 2 * pi *  R = 0.08m
#define MICROSTEPS 3200.0f

static const char* TAG = "Inverted Pendulum";

encoder_t* encoder_hn3806 = NULL;
pcnt_unit_handle_t pcnt_unit = NULL;
motor_handle_t stepper_motor = NULL;
pcnt_unit_handle_t encoder_hn3806_unit = NULL;
motor_handle_t motor = NULL;
motor_mcpwm_config_t mcpwm_config = {
    .group_id = 0,
    .resolution_hz = MCPWM_RESOLUTION_HZ,
};
motor_config_t motor_conf = {
    .dir_gpio_num = 22,
    .pwma_gpio_num = 23,
    .pwm_freq_hz = 1000,
};
float prev_x = 0.0f;
float prev_theta = 0.0f;
float current_velocity = 0.0f;  

const float K[4] = {-35.5340f, -250.6085f, -33.7086f, -50.2635f};

static bool IRAM_ATTR on_reach_boundary(pcnt_unit_handle_t unit, const pcnt_watch_event_data_t *edata, void *user_ctx)
{
    motor_set_speed(stepper_motor, 0);
    return false;
}

float wrap_angle(float angle) 
{
    return atan2f(sinf(angle), cosf(angle));
}

void init_system() 
{
    motor_mcpwm_config_t mcpwm_config = { .group_id = 0, .resolution_hz = 1000000 };
    motor_config_t stepper_config = {
        .dir_gpio_num = DIR_GPIO_NUM,
        .pwma_gpio_num = STEP_GPIO_NUM,
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



    gpio_config_t en_conf = {
        .pin_bit_mask = (1ULL << EN_GPIO_TMC2209),
        .mode = GPIO_MODE_OUTPUT,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&en_conf);

    gpio_set_level(EN_GPIO_TMC2209, 0); // 0 = enable
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

    gpio_set_level(DIR_GPIO_NUM, direction);
    motor_set_speed(stepper_motor, speed_hz);
}

void set_stepper_velocity(float velocity_m_s)
{
    if (velocity_m_s > 0) 
    {
        gpio_set_level(DIR_GPIO_NUM, 1); 
    } 
    else if (velocity_m_s < 0) 
    {
        gpio_set_level(DIR_GPIO_NUM, 0); 
    }

    float abs_v = fabsf(velocity_m_s);
    float freq_hz = (abs_v / PULLEY_CIRCUMFERENCE) * MICROSTEPS;

    // Giới hạn tần số tối đa an toàn cho MCPWM (thử ở mức 20kHz - 25kHz)
    if (freq_hz > 40000) freq_hz = 40000; 
    
    // Tránh tần số quá thấp gây lỗi chia cho 0 hoặc period quá lớn
    if (freq_hz < 10) freq_hz = 0; 


    motor_set_speed(stepper_motor, freq_hz);

}

void lqr_control(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    uint32_t delay_ms = (uint32_t)(DT*1000.0f);
    const TickType_t xFrequency = pdMS_TO_TICKS(delay_ms) > 0 ? pdMS_TO_TICKS(delay_ms) : 1;
        

    while(1)
    {

        int x_steps = 0;
        pcnt_unit_get_count(pcnt_unit, &x_steps);
        float x = (float)x_steps * (PULLEY_CIRCUMFERENCE/ MICROSTEPS);

        float raw_theta = 0.0f;
        encoder_get_radian(encoder_hn3806, &raw_theta);
        float theta = wrap_angle(raw_theta + (float)M_PI);

        float x_dot = (x - prev_x) / DT;

        float delta_theta = wrap_angle(theta - prev_theta);
        float theta_dot = delta_theta / DT;

        prev_x = x;
        prev_theta = theta;

        //u = - K * x
        float u_accel =  (K[0] * x + K[1] * theta + K[2] * x_dot + K[3] * theta_dot);

        if (u_accel > MAX_ACCEL) u_accel = MAX_ACCEL;
        if (u_accel < -MAX_ACCEL) u_accel = -MAX_ACCEL;

        
        current_velocity += (u_accel * DT);
        // v = v0 + a*t
        // current_velocity = current_velocity + (u_accel * DT);

        if (current_velocity > MAX_VELOCITY) current_velocity = MAX_VELOCITY;
        if (current_velocity < -MAX_VELOCITY) current_velocity = -MAX_VELOCITY;
        
        
        
        if(fabs(theta) < 0.7f)
        {
            gpio_set_level(EN_GPIO_TMC2209, 0); // Đảm bảo driver được bật
            set_stepper_velocity(current_velocity);
        }
        else
        {
            current_velocity = 0.0f;
            set_stepper_velocity(0.0f);
            gpio_set_level(EN_GPIO_TMC2209, 1); // 1 = off, no more holding torque
        }
        ESP_LOGI(TAG,"Theta: %.3f rad | %.2f deg | %.2f m/s", theta, theta * 180.0f / 3.1415f, current_velocity);        
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

    }

}

void app_main(void) 
{
    
    // init_system();
    // ESP_LOGI(TAG, "He thong san sang. Vi tri hien tai la TAM (0).");
    // int motor_hz = 10000;    
    // int target_direction = 1; 
    // while(1) 
    // {
    //     int pos = 0;
    //     pcnt_unit_get_count(pcnt_unit, &pos);

    //     if (target_direction == 1 && pos >= MAX_STEPS) {
    //         target_direction = 0; // Đổi sang trái
    //         vTaskDelay(pdMS_TO_TICKS(500)); 
    //     } 
    //     else if (target_direction == 0 && pos <= -MAX_STEPS) {
    //         target_direction = 1; 
    //         vTaskDelay(pdMS_TO_TICKS(500)); 
    //     }

    //     safe_move(motor_hz, target_direction);

    //     ESP_LOGI(TAG, "Vi tri: %d | Huong: %s", pos, target_direction ? "PHAI" : "TRAI");
        
    //     vTaskDelay(pdMS_TO_TICKS(20)); 
    // }


    init_system();
    ESP_ERROR_CHECK(init_encoder(&encoder_hn3806_unit, &encoder_hn3806));

    vTaskDelay(pdMS_TO_TICKS(100));

    xTaskCreatePinnedToCore(lqr_control, "LQR_Task", 4095, NULL, 5, NULL, 1);
}