#include "rmt_nema.h"

#define DIR_CW 0
#define DIR_CCW !DIR_CW


/*******************************************************************************
 * Variables
 ******************************************************************************/
static rmt_symbol_word_t *g_payload = NULL;
static rmt_encoder_handle_t g_copy_encoder = NULL;
static rmt_channel_handle_t g_tx_channel = NULL;

/*******************************************************************************
 * Functions
 ******************************************************************************/

esp_err_t rmt_nema_set_speed(rmt_nema_config_t *config, int64_t speed_hz)
{
    rmt_disable(g_tx_channel);
    if(speed_hz == 0)
    {
        // rmt_disable();
        return ESP_OK;
    }
    bool clockwise = (speed_hz > 0);
    gpio_set_level(config->dir_pin, clockwise ? DIR_CW : DIR_CCW);
    
    uint64_t speed = llabs(speed_hz);
    uint64_t half_period = (uint32_t)config->rmt_resolution_hz / (speed * 2);

    g_payload[0] = (rmt_symbol_word_t) {
        .level0 = 1, 
        .duration0 = half_period,
        .level1 = 0,
        .duration1 = half_period,
    };
    rmt_transmit_config_t tx_config = {
        .loop_count = -1, 
    };
    // ESP_ERROR_CHECK();
    // ESP_LOGD(TAG, "Hello %d", speed);
    rmt_enable(g_tx_channel);
    return rmt_transmit(g_tx_channel, g_copy_encoder, g_payload, sizeof(rmt_symbol_word_t), &tx_config);

}

esp_err_t init_rmt(const rmt_nema_config_t *config)
{
    gpio_config_t dir_conf = {
        .pin_bit_mask = (1ULL << config->dir_pin), //1: giá trị 01, ULL: unsigned long long(8byte - 64bit), <<: dịch trái
        .mode = GPIO_MODE_OUTPUT, // OUTPUT
        .pull_up_en = GPIO_PULLUP_DISABLE, 
        .pull_down_en = GPIO_PULLDOWN_DISABLE, 
        .intr_type = GPIO_INTR_DISABLE, 
    };
    gpio_config(&dir_conf);

    //RMT Channel Config
    rmt_tx_channel_config_t tx_chan_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT, 
        .gpio_num = config->step_pin,   
        .mem_block_symbols = 128, 
        .resolution_hz = config->rmt_resolution_hz, 
        .trans_queue_depth = 4, 
    };
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config, &g_tx_channel));
    ESP_ERROR_CHECK(rmt_enable(g_tx_channel));

    rmt_copy_encoder_config_t copy_encoder_config = {};
    ESP_ERROR_CHECK(rmt_new_copy_encoder(&copy_encoder_config, &g_copy_encoder));
    // size_t size = config->pulses_per_rev * 
    g_payload = (rmt_symbol_word_t *)malloc(sizeof(rmt_symbol_word_t));
    if(g_payload == NULL)
    {
        return ESP_FAIL;
    }
    return ESP_OK;
}


esp_err_t rmt_tx_stop(rmt_channel_t channel)
{
    ESP_RETURN_ON_FALSE(RMT_IS_TX_CHANNEL(channel), ESP_ERR_INVALID_ARG, TAG, RMT_CHANNEL_ERROR_STR);
    RMT_ENTER_CRITICAL();
#if SOC_RMT_SUPPORT_ASYNC_STOP
    rmt_ll_tx_stop(rmt_contex.hal.regs, channel);
#else
    // write ending marker to stop the TX channel
    RMTMEM.chan[channel].data32[0].val = 0;
#endif
    rmt_ll_tx_reset_pointer(rmt_contex.hal.regs, channel);
    RMT_EXIT_CRITICAL();
    return ESP_OK;
}