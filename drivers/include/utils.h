#ifndef UTILS_H
#define UTILS_H

#include <stdint.h>

#define REG_WRITE(addr, val) (*(volatile uint32_t *)(addr) = (val))
#define REG_READ(addr) (*(volatile uint32_t *)(addr))

#define SET_BIT(reg, bit) ((reg) | (1 << (bit)))
#define CLEAR_BIT(reg, bit) ((reg) & ~(1 << (bit)))
#define READ_BIT(reg, bit) (((reg) >> (bit)) & 1)

#endif // UTILS_H