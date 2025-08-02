#pragma once

#include "pico/stdlib.h"

#include "pico/stdio/driver.h"

#define UART_PORT           (uart0)        // UART interface for the serial console
#define UART_IRQ            (UART0_IRQ)    // UART interrupt number

// UART defines
#define UART_BAUDRATE       115200
#define UART_DATABITS       8
#define UART_STOPBITS       1
#define UART_PARITY         UART_PARITY_NONE

#define UART_TX             0
#define UART_RX             1

#define UART_BUFFER_SIZE    256


extern stdio_driver_t serial_stdio_driver;

// Function prototypes
void serial_init(uint baudrate, uint databits, uint stopbits, uart_parity_t parity);
bool serial_input_available(void);
char serial_get_char(void);
bool serial_output_available(void);
void serial_put_char(char ch);