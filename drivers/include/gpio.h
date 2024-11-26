#ifndef GPIO_H
#define GPIO_H

void setup_pin(unsigned int pin);
void rise_pin(unsigned int pin);
void fall_pin(unsigned int pin);
int read_pin(unsigned int pin);

#endif