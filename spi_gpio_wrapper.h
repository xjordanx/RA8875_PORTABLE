/*
** Wrapper for some dumb arduino functions:
** 
** digitalWrite(pin, value)
** digitalRead(pin, value)
** 
** SPI_CS(value)
** SPI_begin()
** etc.
**
**
*/
#ifndef SPI_GPIO_WRAPPER_H
#define SPI_GPIO_WRAPPER_H

#ifndef __GPIO__
#include "gpio.h" // may be auto-included by platform.
#endif

#include "efb_spi.h"

#ifndef HIGH
#define HIGH (0xFF)
#endif

#ifndef LOW
#define LOW  (0x00)
#endif

#ifndef INPUT
#define INPUT 0
#endif

#ifndef OUTPUT
#define OUTPUT 1
#endif

extern struct gpio_instance gpio_inst;
extern struct efb_spi_instance spi_inst;

// Arduino digital read and write wrappers
uint8_t digitalWrite(uint32_t pin_num, uint32_t value);
uint8_t pinMode(uint32_t pin_num, enum gpio_direction io_dir);
uint8_t digitalRead(uint32_t pin_num);

void SPI_CS(uint32_t cs_pin, uint8_t value);
//SPI_Begin();
//SPI_End();
uint8_t SPI_transfer(uint8_t byte_to_send);
uint8_t SPI_transfer16(uint16_t word_to_send);

#endif /* SPI_GPIO_WRAPPER_H */
