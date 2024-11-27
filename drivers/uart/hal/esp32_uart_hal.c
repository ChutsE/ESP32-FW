#include "esp32_uart_hal.h"
#include <stdio.h>

#define APB_CLK 80000000U

// GPIO addresses mapping
const uint32_t io_mux_gpio_reg_map[40] = {
    IO_MUX_BASE_ADDR+0x44, // GPIO0
    IO_MUX_BASE_ADDR+0x88, // GPIO1
    IO_MUX_BASE_ADDR+0x40, // GPIO2
    IO_MUX_BASE_ADDR+0x84, // ...
    IO_MUX_BASE_ADDR+0x48,
    IO_MUX_BASE_ADDR+0x6C,
    IO_MUX_BASE_ADDR+0x60,
    IO_MUX_BASE_ADDR+0x64,
    IO_MUX_BASE_ADDR+0x68,
    IO_MUX_BASE_ADDR+0x54,
    IO_MUX_BASE_ADDR+0x58,
    IO_MUX_BASE_ADDR+0x5C,
    IO_MUX_BASE_ADDR+0x34,
    IO_MUX_BASE_ADDR+0x38,
    IO_MUX_BASE_ADDR+0x30,
    IO_MUX_BASE_ADDR+0x3C,
    IO_MUX_BASE_ADDR+0x4C,
    IO_MUX_BASE_ADDR+0x50,
    IO_MUX_BASE_ADDR+0x70,
    IO_MUX_BASE_ADDR+0x74,
    IO_MUX_BASE_ADDR+0xFF, // FF unsupported GPIOs
    IO_MUX_BASE_ADDR+0x7C,
    IO_MUX_BASE_ADDR+0x80,
    IO_MUX_BASE_ADDR+0x8C,
    IO_MUX_BASE_ADDR+0x90,
    IO_MUX_BASE_ADDR+0x24,
    IO_MUX_BASE_ADDR+0x28,
    IO_MUX_BASE_ADDR+0x2C,
    IO_MUX_BASE_ADDR+0xFF,
    IO_MUX_BASE_ADDR+0xFF,
    IO_MUX_BASE_ADDR+0xFF,
    IO_MUX_BASE_ADDR+0xFF,
    IO_MUX_BASE_ADDR+0x1C,
    IO_MUX_BASE_ADDR+0x20,
    IO_MUX_BASE_ADDR+0x14,
    IO_MUX_BASE_ADDR+0x18,
    IO_MUX_BASE_ADDR+0x04,
    IO_MUX_BASE_ADDR+0x08,
    IO_MUX_BASE_ADDR+0x10,
};

// FUNCTIONAL
void hal_uart_configure(uint32_t base_addr, uint32_t baudrate, uint8_t data_bits, uint8_t stop_bits, uint8_t parity) {
    printf("\nBEGINNING UART config\n");
    volatile uint32_t* uart_clkdiv_reg = (volatile uint32_t*)(base_addr + UART_CLKDIV_OFFSET);
    volatile uint32_t* uart_conf0_reg = (volatile uint32_t*)(base_addr + UART_CONF0_OFFSET);
    // volatile uint32_t* uart_conf1_reg = (volatile uint32_t*)(base_addr + UART_CONF1_OFFSET);
    volatile uint32_t* uart_idle_conf_reg = (volatile uint32_t*)(base_addr + UART_IDLE_CONF_OFFSET);
    volatile uint32_t* dport_perip_clk_en = (volatile uint32_t*)DPORT_PERIP_CLK_EN_REG;
    volatile uint32_t* dport_perip_rst_en = (volatile uint32_t*)DPORT_PERIP_RST_EN_REG;

    // Clear reset and set clk enable
    // For shared UARTs memory
    *dport_perip_rst_en &= ~(1 << 24);
    *dport_perip_clk_en |= (1 << 24);

    if(base_addr == UART0_BASE_ADDR){
        // For UART2
        *dport_perip_rst_en &= ~(1 << 2);
        *dport_perip_clk_en |=  (1 << 2);

    }else if (base_addr == UART1_BASE_ADDR)
    {
        *dport_perip_rst_en &= ~(1 << 5);
        *dport_perip_clk_en |=  (1 << 5);
    }else if (base_addr == UART2_BASE_ADDR)
    {
        *dport_perip_rst_en &= ~(1 << 23);
        *dport_perip_clk_en |=  (1 << 23);
    }

    // Select APB clock as Ref Clock
    *uart_conf0_reg |= (1 << 27);

    // Set baud rate
    uint32_t clk_div, clk_div_int, clk_div_frag;
    clk_div = ((APB_CLK) << 4) / baudrate;
    clk_div_int = clk_div >> 4;
    clk_div_frag = clk_div & 0xF;
    *uart_clkdiv_reg &= ~((0xF << 20) | (0xFFFFF << 0)); 
    *uart_clkdiv_reg |= (clk_div_frag << 20) | (clk_div_int << 0); 

    // Configure data lenght. (bits 3 and 2) from p. 369
    if(data_bits == 5){
        // 5 bits lenght: 00
        *uart_conf0_reg &= ~((1 << 3) | (1 << 2));  
    }else if(data_bits == 6){
        // 6 bits lenght: 01
        *uart_conf0_reg &= ~(1 << 3);
        *uart_conf0_reg |= (1 << 2);               
    }else if(data_bits == 7){
        // 7 bits lenght: 10
        *uart_conf0_reg |= (1 << 3);               
        *uart_conf0_reg &= ~(1 << 2);
    }else if(data_bits == 8){
        // 8 bits lenght: 11
        *uart_conf0_reg |= ((1 << 3) | (1 << 2));
    }

    // Configure stop bits (bits 5 and 4) from p. 369
    if(stop_bits == 1){
        // 1 stop bits: 01        
        *uart_conf0_reg &= ~(1 << 5);
        *uart_conf0_reg |= (1 << 4);               
    } else if(stop_bits == 1.5){
        // 1.5 stop bits: 10
        *uart_conf0_reg |= (1 << 5);
        *uart_conf0_reg &= ~(1 << 4);              
    } else if(stop_bits == 2){
        // 2 stop bits: 11
        *uart_conf0_reg |= (1 << 5) | (1 << 4);
    } 

    // Configure parity (bits 1 and 0) from p. 369
    if(parity == 0){
        // No parity (clean bits 1 and 0)
        *uart_conf0_reg &= ~((1 << 1) | (1 << 0)); 
    } else if(parity == 1){
        // Enable parity (bit 1) and set even parity (bit 0)
        *uart_conf0_reg |= (1 << 1);              
        *uart_conf0_reg &= ~(1 << 0);
    } else if(parity == 2){
        // Enable parity (bit 1) and set even parity (bit 0)
        *uart_conf0_reg |= (1 << 1) | (1 << 0);    // Odd parity
    }

    // Remove time spaces between transmits
    *uart_idle_conf_reg &= ~(0x3F << 10); 
    printf("\nFINISHED UART config\n");
}

// FUNCTIONAL
void hal_uart_set_pins(uint32_t base_addr, uint8_t tx_pin, uint8_t rx_pin) {  
    printf("\nBEGINNING GPIO CONFIG\n");
    // Determine UART signal indices (same for input and output) see p. 56 table 4-2
    uint32_t uart_signal = (base_addr == UART0_BASE_ADDR) ? 14 : 
                           (base_addr == UART1_BASE_ADDR) ? 17 : 198;

    // GPIO Matrix and IO MUX registers
    volatile uint32_t* gpio_func_out_sel_cfg_reg = (volatile uint32_t*)(GPIO_FUNC_OUT_SEL_CFG_BASE_ADDR + tx_pin * 4);
    volatile uint32_t* gpio_func_in_sel_cfg_reg = (volatile uint32_t*)(GPIO_FUNC_IN_SEL_CFG_BASE_ADDR + uart_signal * 4);
    volatile uint32_t* gpio_enable_reg = (volatile uint32_t*) GPIO_ENABLE_REG;
    volatile uint32_t* gpio_enable1_reg = (volatile uint32_t*) GPIO_ENABLE1_REG;
    // volatile uint32_t* gpio_pin_tx_reg = (volatile uint32_t*)(GPIO_PIN_BASE_ADDR + tx_pin * 4);
    // volatile uint32_t* gpio_pin_rx_reg = (volatile uint32_t*)(GPIO_PIN_BASE_ADDR + rx_pin * 4);

    volatile uint32_t* io_mux_tx_reg = (volatile uint32_t*)io_mux_gpio_reg_map[tx_pin];
    volatile uint32_t* io_mux_rx_reg = (volatile uint32_t*)io_mux_gpio_reg_map[rx_pin];

    /*##########################
      ## UART OUTPUT VIA GPIO ##
      ##########################
      p. 51, 4.3.2*/

    /*1. Configure the GPIO_FUNCx_OUT_SEL_CFG register and GPIO_ENABLE_DATA[x] field corresponding 
    to GPIO X in the GPIO Matrix:*/
    *gpio_func_out_sel_cfg_reg = (uart_signal << 0);  // OUT_SEL: Sets the peripheral signal to UART TX
                                //  (1 << 10);            // OEN_SEL: Enable as always output

    if (tx_pin <= 31)
    {
        *gpio_enable_reg |= (1 << tx_pin);     // Output enable for tx_pin GPIO (0-31)
    }else if(tx_pin == 32){
        *gpio_enable1_reg |= (1 << 0);         // (32)
    }else{
        printf("GPIO %d is input only.", tx_pin);
        // Implement error
    }
    
    /*2. For an open drain output, set the GPIO_PINx_PAD_DRIVER bit in the GPIO_PINx register corresponding 
    to GPIO pad X. For push/pull mode (default), clear this bit.*/
    // *gpio_pin_tx_reg = (1 << 2); // Open drain for further pull-up resistor enable  
     
    /*3. Configure the IO_MUX to select the GPIO Matrix. Set the IO_MUX_x_REG register corresponding 
    to GPIO pad X */
    *io_mux_tx_reg |= (2 << 12);   // Set IOMUX function to GPIO function (always func 2) 
    *io_mux_tx_reg |= (2 << 10);   // Set FUN_DRV to 2 (default 20mA)
    *io_mux_tx_reg |= (1 << 9);   // FUN_IE input enable
    *io_mux_tx_reg |= (1 << 8);    // Set FUN_WPU to 1 (enable pull-up resistor)  

    /*###########################
      ### UART INPUT VIA GPIO ###
      ###########################
      p. 50, 4.2.2*/
    
    /*1. Configure the GPIO_FUNCy_IN_SEL_CFG register corresponding to peripheral signal Y 
    in the GPIO Matrix*/
    *gpio_func_in_sel_cfg_reg &= ~(0xFF << 0);  // Clean IN_SEL fields
    printf("-Setting IN_SEL gpio to %x.\n", rx_pin);
    *gpio_func_in_sel_cfg_reg |= (rx_pin << 0); // Set GPIO rx_pin number as input for the uart rx signal
    *gpio_func_in_sel_cfg_reg |= (1 << 7);      // Enable peripheral signal input via GPIO matrix

    /*2. Configure the GPIO_FUNCx_OUT_SEL_CFG register and clear the GPIO_ENABLE_DATA[x] field corresponding to 
    GPIO pad X in the GPIO Matrix:*/
    // volatile uint32_t* rx_func_out_sel_cfg_reg = (volatile uint32_t*)(GPIO_FUNC_OUT_SEL_CFG_BASE_ADDR + rx_pin * 4);
    // *rx_func_out_sel_cfg_reg |= (1 << 10);

    if (tx_pin <= 31){
        *gpio_enable_reg &= ~(1 << rx_pin);         // Output disable for rx_pin GPIO (0-31)
    }else if(tx_pin >= 32){
        *gpio_enable1_reg &= ~(1 << (rx_pin - 32)); // (32-39)
    }

    /*3. Configure the IO_MUX to select the GPIO Matrix. Set the IO_MUX_x_REG register corresponding to GPIO
    pad X as follows:*/
    *io_mux_rx_reg |= (2 << 12);   // Set IOMUX function to GPIO function (always func 2) 
    *io_mux_rx_reg |= (1 << 9);   // Set FUN_IE for input enable
    // *io_mux_rx_reg |= (1 << 8);    // Set FUN_WPU to 1 (enable pull-up resistor)  

    printf("\nFINISHED GPIO CONFIG\n");
}

// SEMI-FUNCTIONAL. Issue: sends the char 2 times (lol)
void hal_uart_write_byte(uint32_t base_addr, uint8_t byte) {
    volatile uint32_t* uart_status_reg = (volatile uint32_t*)(base_addr + UART_STATUS_OFFSET);
    volatile uint32_t* uart_fifo_reg = (volatile uint32_t*)(base_addr + UART_FIFO_OFFSET);
    
    // *uart_fifo_reg &= ~(0xFF << 0);
    printf("\nFIFO 1st. value: %lx\n", (*uart_fifo_reg & 0xFF));

    while ((*uart_status_reg & (1 << 16)));
    *uart_fifo_reg = byte;

    printf("\nFIFO last value: %lx\n", (*uart_fifo_reg & 0xFF));

    // printf("Sent: %c\n", (char)byte);
}

// UNDER DEVELOPMENT, DO NOT USE. -Carlos
uint8_t hal_uart_read_byte(uint32_t base_addr) {
    volatile uint32_t* uart_status_reg = (volatile uint32_t*)(base_addr + UART_STATUS_OFFSET);
    volatile uint32_t* uart_fifo_reg = (volatile uint32_t*)(base_addr + UART_FIFO_OFFSET);

    while (!(*uart_status_reg & (1 << 0))); // Wait for RX FIFO data
    return (uint8_t)(*uart_fifo_reg & 0xFF);
}
