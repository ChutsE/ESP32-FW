#ifndef GPIO_HIF_H
#define GPIO_HIF_H

#define GPIO_BASE 0x3FF44000
#define GPIO_ENABLE_REG (GPIO_BASE + 0x0020)
#define GPIO_OUT_REG (GPIO_BASE + 0x0004)
#define GPIO_IN_REG (GPIO_BASE + 0x003C)

#endif // GPIO_HIF_H