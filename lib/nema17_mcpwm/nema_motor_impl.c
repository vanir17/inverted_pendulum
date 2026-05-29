#include <stdlib.h>
#include <string.h>
#include <sys/cdefs.h>
#include "esp_log.h"
#include "esp_check.h"
#include "driver/gpio.h"
#include "driver/mcpwm_prelude.h"

#include "nema_motor.h"
#include "nema_motor_defs.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

static const char *TAG = "motor_mcpwm";


typedef struct mcpwm_obj
{
    motor_t base;
    mcpwm_timer_handle_t timer;
    mcpwm_oper_handle_t operator;
    mcpwm_cmpr_handle_t cmp;
    mcpwm_gen_handle_t gen;
    gpio_num_t dir_gpio_num;    
}motor_mcpwm_obj;
/*******************************************************************************
 * Prototypes
 ******************************************************************************/
esp_err_t motor_enable(motor_handle_t motor);
esp_err_t motor_set_speed(motor_handle_t motor, uint32_t speed_hz);
esp_err_t motor_disable(motor_handle_t motor);
static esp_err_t motor_mcpwm_del(motor_t *motor);


/*******************************************************************************
 * Code
 ******************************************************************************/


esp_err_t motor_enable(motor_handle_t motor)
 {
    motor_mcpwm_obj *mcpwm_motor = __containerof(motor, motor_mcpwm_obj, base);
    ESP_RETURN_ON_ERROR(mcpwm_timer_enable(mcpwm_motor->timer), TAG, "enable timer failed");
    ESP_RETURN_ON_ERROR(mcpwm_timer_start_stop(mcpwm_motor->timer, MCPWM_TIMER_START_NO_STOP), TAG, "start timer failed");
    return ESP_OK;
 }

esp_err_t motor_disable(motor_handle_t motor)
{
    motor_mcpwm_obj *mcpwm_motor = __containerof(motor, motor_mcpwm_obj, base);
    ESP_RETURN_ON_ERROR(mcpwm_timer_start_stop(mcpwm_motor->timer, MCPWM_TIMER_STOP_EMPTY), TAG, "stop timer failed");
    ESP_RETURN_ON_ERROR(mcpwm_timer_disable(mcpwm_motor->timer), TAG, "disable timer failed");
        return ESP_OK;

}

static esp_err_t motor_mcpwm_del(motor_t *motor)
{
    motor_mcpwm_obj *mcpwm_motor = __containerof(motor, motor_mcpwm_obj, base);
    
    if (mcpwm_motor->gen) mcpwm_del_generator(mcpwm_motor->gen);
    if (mcpwm_motor->cmp) mcpwm_del_comparator(mcpwm_motor->cmp);
    if (mcpwm_motor->operator) mcpwm_del_operator(mcpwm_motor->operator);
    if (mcpwm_motor->timer) mcpwm_del_timer(mcpwm_motor->timer);
    
    free(mcpwm_motor);
    return ESP_OK;
}

    
esp_err_t motor_set_speed(motor_handle_t motor, uint32_t speed_hz)
{
    ESP_RETURN_ON_FALSE(motor, ESP_ERR_INVALID_ARG, TAG, "motor handle is NULL");
    motor_mcpwm_obj *mcpwm_motor = __containerof(motor, motor_mcpwm_obj, base);

    if (mcpwm_motor->timer == NULL) 
    {
        return ESP_ERR_INVALID_STATE;
    }
    if(speed_hz == 0)
    {
        mcpwm_generator_set_force_level(mcpwm_motor->gen, 0, true);
        return ESP_OK;
    }

    uint32_t period = MCPWM_RESOLUTION_HZ / speed_hz;

    mcpwm_timer_set_period(mcpwm_motor->timer, period);

    mcpwm_comparator_set_compare_value(mcpwm_motor->cmp, period/2); // square pulse

    mcpwm_generator_set_force_level(mcpwm_motor->gen, -1, true);

    ESP_LOGI(TAG, "speed_hz: %d", speed_hz);
    return ESP_OK;
}

esp_err_t motor_new_mcpwm_device(const motor_config_t *motor_config, const motor_mcpwm_config_t *mcpwm_config, motor_handle_t *ret_motor)
{
    motor_mcpwm_obj *mcpwm_motor = NULL;
    esp_err_t ret = ESP_OK;

    /* Check input arguments are valid or invalid */
    ESP_GOTO_ON_FALSE(motor_config && mcpwm_config && ret_motor, ESP_ERR_INVALID_ARG, err, TAG, "invalid argument");
    mcpwm_motor = calloc(1, sizeof(motor_mcpwm_obj));
    ESP_GOTO_ON_FALSE(mcpwm_motor, ESP_ERR_NO_MEM, err, TAG, "no memory for motor object");

    /* Direction GPIO configuration */
    mcpwm_motor->dir_gpio_num = motor_config->dir_gpio_num;
    gpio_config_t dir_config = {
        .pin_bit_mask = (1ULL << motor_config->dir_gpio_num),
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&dir_config);

    /* ------------------------- MCPWM Timer ---------------------------------- */
    mcpwm_timer_config_t timer_config = {
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .group_id = mcpwm_config->group_id,
        .resolution_hz = mcpwm_config->resolution_hz, 
        .period_ticks = mcpwm_config->resolution_hz / motor_config->pwm_freq_hz,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
    };
    ESP_GOTO_ON_ERROR(mcpwm_new_timer(&timer_config, &mcpwm_motor-> timer), err, TAG, "create MCPWM Timer failed");

    /* ------------------------- MCPWM Operator ---------------------------------- */

    mcpwm_operator_config_t operator_config = {
        .group_id = mcpwm_config->group_id,
    };
    ESP_GOTO_ON_ERROR(mcpwm_new_operator(&operator_config, &mcpwm_motor->operator), err, TAG, "create MCPWM Operator failed");

    //connect operator and timer
    ESP_GOTO_ON_ERROR(mcpwm_operator_connect_timer(mcpwm_motor->operator, mcpwm_motor->timer), err, TAG, "connect timer and operator failed");


    /* --------------------- MCPWM Comparator ----------------------- */
    mcpwm_comparator_config_t comparator_config = {
        .flags.update_cmp_on_tez = true,
    };
    ESP_GOTO_ON_ERROR(mcpwm_new_comparator(mcpwm_motor->operator, &comparator_config, &mcpwm_motor->cmp), 
                                                                err, TAG, "create MCPWM Comparator failed");
    mcpwm_comparator_set_compare_value(mcpwm_motor->cmp, 0);

    /* --------------------- MCPWM Generator ----------------------- */
    mcpwm_generator_config_t generator_config = {
        .gen_gpio_num = motor_config->pwma_gpio_num,
    };
    ESP_GOTO_ON_ERROR(mcpwm_new_generator(mcpwm_motor->operator, &generator_config, &mcpwm_motor->gen), 
                                                                   err, TAG, "create generator failed");

    mcpwm_generator_set_action_on_timer_event(mcpwm_motor->gen, 
    MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH));
    
    mcpwm_generator_set_action_on_compare_event(mcpwm_motor->gen, 
    MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, mcpwm_motor->cmp, MCPWM_GEN_ACTION_LOW));


    *ret_motor = &(mcpwm_motor->base);

    ESP_LOGI(TAG, "Motor initialized successfully at %p", mcpwm_motor);
    return ESP_OK;



    err:
    if(mcpwm_motor)
    {
        motor_mcpwm_del(&mcpwm_motor->base);
    
    }
    return ret;
}