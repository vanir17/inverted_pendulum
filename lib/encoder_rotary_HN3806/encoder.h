#ifndef ENCODER_H
#define ENCODER_H

#include "esp_log.h"
#include "esp_err.h"             
#include "driver/pulse_cnt.h"

typedef void encoder_t;
esp_err_t init_encoder(pcnt_unit_handle_t* pcnt_unit, encoder_t** result);
esp_err_t encoder_get_degrees(encoder_t* _this, float* _result);
esp_err_t deinit_encoder();

#endif  