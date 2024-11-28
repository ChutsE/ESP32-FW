#ifndef ESP32_UART_HIL_H
#define ESP32_UART_HIL_H

#include <stdint.h>

/**********************
 ** UART 0 Functions **
 **********************/ 

/// @brief Initializes UART0 with the params provided.
/// @param baudrate Speed of data transfered by UART0.
/// @param data_bits Data length in bits. 
/// @param stop_bits Amount of stop bits for the data frame.
/// @param parity Enable(1)/Disable(0) parity.
/// @param tx_pin Set GPIOn as Tx pin.
/// @param rx_pin Set GPIOn as Rx pin.
void uart0_init(uint32_t baudrate, uint8_t data_bits, uint8_t stop_bits, uint8_t parity, uint8_t tx_pin, uint8_t rx_pin);

/// @brief Writes byte to UART0 Tx.
/// @param byte Byte of data to be sent (5-8 depending on the config).
void uart0_write_byte(uint8_t byte);

/// @brief Reads byte from UART0 Rx.
/// @param None
/// @return uint8_t byte of data read.
uint8_t uart0_read_byte(void);

/**********************
 ** UART 1 Functions **
 **********************/

/// @brief Initializes UART1 with the params provided.
/// @param baudrate Speed of data transfered by UART1.
/// @param data_bits Data length in bits. 
/// @param stop_bits Amount of stop bits for the data frame.
/// @param parity Enable(1)/Disable(0) parity.
/// @param tx_pin Set GPIOn as Tx pin.
/// @param rx_pin Set GPIOn as Rx pin.
void uart1_init(uint32_t baudrate, uint8_t data_bits, uint8_t stop_bits, uint8_t parity, uint8_t tx_pin, uint8_t rx_pin);

/// @brief Writes byte to UART1 Tx.
/// @param byte Byte of data to be sent (5-8 depending on the config).
void uart1_write_byte(uint8_t byte);

/// @brief Reads byte from UART1 Rx.
/// @param None
/// @return uint8_t byte of data read.
uint8_t uart1_read_byte(void);

/**********************
 ** UART 2 Functions **
 **********************/

/// @brief Initializes UART2 with the params provided.
/// @param baudrate Speed of data transfered by UART2.
/// @param data_bits Data length in bits. 
/// @param stop_bits Amount of stop bits for the data frame.
/// @param parity Enable(1)/Disable(0) parity.
/// @param tx_pin Set GPIOn as Tx pin.
/// @param rx_pin Set GPIOn as Rx pin.
void uart2_init(uint32_t baudrate, uint8_t data_bits, uint8_t stop_bits, uint8_t parity, uint8_t tx_pin, uint8_t rx_pin);

/// @brief Writes byte to UART2 Tx.
/// @param byte Byte of data to be sent (5-8 depending on the config).
void uart2_write_byte(uint8_t byte);

/// @brief Reads byte from UART2 Rx.
/// @param None
/// @return uint8_t byte of data read.
uint8_t uart2_read_byte(void);

#endif 