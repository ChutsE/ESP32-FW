#include "drivers/gpio.h"
#include "delay.h"

int app_main() {

    setup_pin(2);
    
    fall_pin(2);
    delay_us(10000000);
    rise_pin(2);
    delay_us(10000000);
    fall_pin(2);
    delay_us(10000000);
    rise_pin(2);

    return 0;
}