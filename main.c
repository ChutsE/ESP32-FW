#include "drivers/include/gpio.h"
#include "drivers/include/rtc.h"

#define DELAY 100

int app_main() {

    setup_pin(4);

    while (1)
    {
        rise_pin(4);
        delay_ms(DELAY);
        fall_pin(4);
        delay_ms(DELAY);
    }
}