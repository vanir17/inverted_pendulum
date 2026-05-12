idf_component_register(
    SRCS "nema_motor_impl.c"  
    INCLUDE_DIRS "." 
    REQUIRES 
        driver                
        esp_driver_mcpwm      
        esp_driver_gpio
)