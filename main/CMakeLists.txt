idf_component_register(SRCS "main.c"
                       PRIV_REQUIRES 
                       esp_driver_pcnt 
                       esp_driver_gpio 
                       encoder_rotary_HN3806 
                       nema17_mcpwm
                       esp_timer
                       driver
                       esp_driver_mcpwm
                       INCLUDE_DIRS ".")