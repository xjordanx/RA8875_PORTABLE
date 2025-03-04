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
#include "spi_gpio_wrapper.h"
// Arduino digital read and write wrappers
uint8_t digitalWrite(uint32_t pin_num, uint32_t value){
	return gpio_output_write(&gpio_inst, pin_num, value);
}
uint8_t pinMode(uint32_t pin_num, enum gpio_direction io_dir){
	return gpio_set_direction(&gpio_inst, pin_num, io_dir);
}
uint8_t digitalRead(uint32_t pin_num){
	uint32_t data;
	if (!gpio_input_get(&gpio_inst, pin_num, &data)) {
		return (unsigned char)data;
	}
	return 0; // error condition.
}

void SPI_CS(uint32_t cs_pin, uint8_t value){
	efb_spi_cs(&spi_inst,cs_pin, value);
}
//void SPI_Begin(void){}
//void SPI_End(void){}
uint8_t SPI_transfer(uint8_t byte_to_send){
	return efb_spi_txrx(&spi_inst, byte_to_send);
}

uint8_t SPI_transfer16(uint16_t word_to_send){
	efb_spi_txrx(&spi_inst, (uint8_t)(word_to_send >> 8));
	return SPI_transfer((uint8_t)(word_to_send & 0xFF));
}
