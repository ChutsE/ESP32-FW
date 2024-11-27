#ifndef ESP32_UART_HALL_H
#define ESP32_UART_HAL_H

#include <stdint.h>

// Base addresses for UART peripherals
#define UART0_BASE_ADDR     0x3FF40000
#define UART1_BASE_ADDR     0x3FF50000
#define UART2_BASE_ADDR     0x3FF6E000

// DPORT Registers
#define DPORT_PERIP_CLK_EN_REG  0x3FF000C0 // Clock Gating
#define DPORT_PERIP_RST_EN_REG  0x3FF000C4 // Reset

// UART config register offsets
#define UART_CONF0_OFFSET       0x20
#define UART_CONF1_OFFSET       0x24
#define UART_CLKDIV_OFFSET      0x14
#define UART_FLOW_CONF_OFFSET   0x34
#define UART_SWFC_CONF_OFFSET   0x3C
#define UART_SLEEP_CONF_OFFSET  0x38
#define UART_IDLE_CONF_OFFSET   0x40
#define UART_RS485_CONF_OFFSET  0x44

// UART status offsets
#define UART_STATUS_OFFSET  0x1C
#define UART_FIFO_OFFSET    0x00

// GPIO matrix base addresses
#define GPIO_PIN_BASE_ADDR              0x3FF44088
#define GPIO_FUNC_OUT_SEL_CFG_BASE_ADDR 0x3FF44530
#define GPIO_FUNC_IN_SEL_CFG_BASE_ADDR  0x3FF44130
#define GPIO_ENABLE_REG                 0x3FF44020
#define GPIO_ENABLE1_REG                0x3FF4402C

// I/O MUX base address
#define IO_MUX_BASE_ADDR    0x3FF49000

// GPIO addresses mapping
extern const uint32_t io_mux_gpio_reg_map[];

// Functions to interface with hardware
void hal_uart_configure(uint32_t base_addr, uint32_t baudrate, uint8_t data_bits, uint8_t stop_bits, uint8_t parity);
void hal_uart_set_pins(uint32_t base_addr, uint8_t tx_pin, uint8_t rx_pin);
void hal_uart_write_byte(uint32_t base_addr, uint8_t byte);
uint8_t hal_uart_read_byte(uint32_t base_addr);

#endif 