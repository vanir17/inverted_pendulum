#include "encoder.h"
#include "encoder_defs.h"
#include "driver/pulse_cnt.h"
#include "esp_log.h"
#include "stdint.h"

#define _impl(x)    ((encoder_esp_t*)(x))
#define M_PI 3.14159265358979323846f



static const char* TAG = "Rotary_Encoder_HN3806";

typedef struct
{
    pcnt_unit_handle_t unit;
}encoder_esp_t;



esp_err_t encoder_get_radian(encoder_t* _this, float* _result)
{
    encoder_esp_t* encoder_t = _impl(_this);
    int pulseCount = 0;

    ESP_ERROR_CHECK(pcnt_unit_get_count(encoder_t->unit, &pulseCount));
    int32_t count = pulseCount;

    if(count < 0) 
    {
        count = PULSES_PER_REV + (count % PULSES_PER_REV);
    }

    count = count % PULSES_PER_REV;

    if (count > PULSES_PER_REV / 2) 
    {
        count -= PULSES_PER_REV;
    }

    *_result = ((float)count * 2.0f * (float)M_PI) / PULSES_PER_REV;


    return ESP_OK;
}

esp_err_t init_encoder(pcnt_unit_handle_t* pcnt_unit, encoder_t** result)
{
    encoder_esp_t* new_encoder = (encoder_esp_t*)malloc(sizeof(encoder_esp_t));
    
    if(new_encoder == NULL)
    {
        printf("No memory!\n");
        return ESP_ERR_NO_MEM;
    }

    //----------------PCNT Unit-------------

    ESP_LOGI(TAG, "install pcnt unit");
    pcnt_unit_config_t unit_config = 
    {
        .high_limit = PCNT_HIGH_LIMIT,
        .low_limit = PCNT_LOW_LIMIT,
    };

    // pcnt_unit_handle_t pcnt_unit = NULL;
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, pcnt_unit));

    ESP_LOGI(TAG, "set glitch filter");
    pcnt_glitch_filter_config_t filter_config = 
    {
        .max_glitch_ns = 100,
    };
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(*pcnt_unit, &filter_config));


    //----------------PCNT Channel A & B-------------

    ESP_LOGI(TAG, "install pcnt channels");
    pcnt_chan_config_t chan_a_config =
    {
        .edge_gpio_num = GPIO_CHAN_A,
        .level_gpio_num = GPIO_CHAN_B,
    };
    pcnt_channel_handle_t pcnt_chan_a = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(*pcnt_unit, &chan_a_config, &pcnt_chan_a));
    new_encoder->unit = *pcnt_unit;

    pcnt_chan_config_t chan_b_config = 
    {
        .edge_gpio_num = GPIO_CHAN_B,
        .level_gpio_num = GPIO_CHAN_A,
    };
    pcnt_channel_handle_t pcnt_chan_b = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(*pcnt_unit, &chan_b_config, &pcnt_chan_b));


    //----------------PCNT Set Action-------------

    ESP_LOGI(TAG, "set edge and level actions for pcnt channels");
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE)); // rising -; falling +
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

    //----------------PCNT-------------

    ESP_LOGI(TAG, "enable pcnt unit");
    ESP_ERROR_CHECK(pcnt_unit_enable(*pcnt_unit));
    ESP_LOGI(TAG, "clear pcnt unit");
    ESP_ERROR_CHECK(pcnt_unit_clear_count(*pcnt_unit));
    ESP_LOGI(TAG, "start pcnt unit");
    ESP_ERROR_CHECK(pcnt_unit_start(*pcnt_unit));

    *result = (encoder_t*)new_encoder;
    return ESP_OK;
}

esp_err_t encoder_get_degrees(encoder_t* _this, float* _result)
{

    encoder_esp_t* encoder_t = _impl(_this);
    int pulseCount = 0;
    ESP_ERROR_CHECK(pcnt_unit_get_count(encoder_t->unit, &pulseCount));
    int32_t count = pulseCount;
    if(count < 0)
    {
        count = PULSES_PER_REV + (count % PULSES_PER_REV);
    }
    count = count % PULSES_PER_REV;

    *_result = ((float)count * 360.0f) / PULSES_PER_REV;

    return ESP_OK;
}
