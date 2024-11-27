#include "esp32_uart_hal.h"
#include "esp32_uart_hil.h"

// UART base addresses
static const uint32_t uart_base_addresses[] = {UART0_BASE_ADDR, UART1_BASE_ADDR, UART2_BASE_ADDR};

// FUNCTIONAL
void hil_uart_init(uint8_t uart_number, uint32_t baudrate, uint8_t data_bits, uint8_t stop_bits, uint8_t parity, uint8_t tx_pin, uint8_t rx_pin) {
    uint32_t base_addr = uart_base_addresses[uart_number];
    hal_uart_set_pins(base_addr, tx_pin, rx_pin);
    hal_uart_configure(base_addr, baudrate, data_bits, stop_bits, parity);
}

// SEMI-FUNCTIONAL. See hal .c
void hil_uart_send_char(uint8_t uart_number, char character) {
    uint32_t base_addr = uart_base_addresses[uart_number];
    hal_uart_write_byte(base_addr, (uint8_t)character);
}

// UNDER DEVELOPMENT. DO NOT USE. -Carlos
uint8_t hil_uart_receive_byte(uint8_t uart_number) {
    uint32_t base_addr = uart_base_addresses[uart_number];
    return hal_uart_read_byte(base_addr);
}
