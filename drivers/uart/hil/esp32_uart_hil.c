#include "esp32_uart_hal.h"
#include "esp32_uart_hil.h"

/**********************
 ** UART 0 Functions **
 **********************/ 

void uart0_init(uint32_t baudrate, uint8_t data_bits, uint8_t stop_bits, uint8_t parity, uint8_t tx_pin, uint8_t rx_pin)
{
    hal_uart_set_pins(UART0_BASE_ADDR, tx_pin, rx_pin);
    hal_uart_configure(UART0_BASE_ADDR, baudrate, data_bits, stop_bits, parity);
}

void uart0_write_byte(uint8_t byte)
{
    hal_uart_write_byte(UART0_BASE_ADDR, byte);
}

uint8_t uart0_read_byte(void)
{
    return hal_uart_read_byte(UART0_BASE_ADDR);
}

/**********************
 ** UART 1 Functions **
 **********************/

void uart1_init(uint32_t baudrate, uint8_t data_bits, uint8_t stop_bits, uint8_t parity, uint8_t tx_pin, uint8_t rx_pin)
{
    hal_uart_set_pins(UART1_BASE_ADDR, tx_pin, rx_pin);
    hal_uart_configure(UART1_BASE_ADDR, baudrate, data_bits, stop_bits, parity);
}

void uart1_write_byte(uint8_t byte)
{
    hal_uart_write_byte(UART1_BASE_ADDR, byte);
}

uint8_t uart1_read_byte(void)
{
    return hal_uart_read_byte(UART1_BASE_ADDR);
}

/**********************
 ** UART 2 Functions **
 **********************/

void uart2_init(uint32_t baudrate, uint8_t data_bits, uint8_t stop_bits, uint8_t parity, uint8_t tx_pin, uint8_t rx_pin)
{
    hal_uart_set_pins(UART2_BASE_ADDR, tx_pin, rx_pin);
    hal_uart_configure(UART2_BASE_ADDR, baudrate, data_bits, stop_bits, parity);
}

void uart2_write_byte(uint8_t byte)
{
    hal_uart_write_byte(UART2_BASE_ADDR, byte);
}

uint8_t uart2_read_byte(void)
{
    return hal_uart_read_byte(UART2_BASE_ADDR);
}