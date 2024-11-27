#ifndef ESP32_UART_HIL_H
#define ESP32_UART_HIL_H

#include <stdint.h>

// Public API for UART configuration
void hil_uart_init(uint8_t uart_number, uint32_t baudrate, uint8_t data_bits, uint8_t stop_bits, uint8_t parity, uint8_t tx_pin, uint8_t rx_pin);
void hil_uart_send_char(uint8_t uart_number, char character);
uint8_t hil_uart_receive_byte(uint8_t uart_number);

#endif 