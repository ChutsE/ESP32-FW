#include "include/gpio_regs.h"
#include "include/gpio.h"
#include "include/utils.h"

void setup_pin(unsigned int pin) {
    uint32_t gpio_en_reg = REG_READ(GPIO_ENABLE_REG);
    uint32_t setted_value = SET_BIT(gpio_en_reg, pin);
    REG_WRITE(GPIO_ENABLE_REG, setted_value);
}

void rise_pin(unsigned int pin) {
    uint32_t gpio_out_reg = REG_READ(GPIO_OUT_REG);
    uint32_t setted_value = SET_BIT(gpio_out_reg, pin);
    REG_WRITE(GPIO_OUT_REG, setted_value);
}

void fall_pin(unsigned int pin) {
    uint32_t gpio_out_reg = REG_READ(GPIO_OUT_REG);
    uint32_t cleared_value = CLEAR_BIT(gpio_out_reg, pin);
    REG_WRITE(GPIO_OUT_REG, cleared_value);
}

int read_pin(unsigned int pin) {
    uint32_t gpio_in_reg = REG_READ(GPIO_IN_REG);
    return READ_BIT(gpio_in_reg, pin);
}