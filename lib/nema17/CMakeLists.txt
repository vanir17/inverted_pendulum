idf_component_register(SRCS "rmt_nema.c"
                    INCLUDE_DIRS "." 
                    REQUIRES esp_driver_gpio esp_driver_rmt)