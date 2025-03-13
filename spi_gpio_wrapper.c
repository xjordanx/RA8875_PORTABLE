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
unsigned char digitalWrite(unsigned int pin_num, unsigned int value){
	if (cb_gpio_write == NULL) return 0;
	return cb_gpio_write(gpio_i, pin_num, value);
}
unsigned char pinMode(unsigned int pin_num, enum gpio_direction io_dir){
	return cb_gpio_dir(gpio_i, pin_num, io_dir);
}
unsigned char digitalRead(unsigned int pin_num){
	unsigned int data;
	if (!cb_gpio_read(gpio_i, pin_num, &data)) {
		return (unsigned char)data;
	}
	return 0; // error condition.
}

void SPI_CS(unsigned int cs_pin, unsigned char value){
	cb_spi_cs(spi_i, cs_pin, value);
}

unsigned char SPI_transfer(unsigned char byte_to_send){
	return cb_spi_txrx(spi_i, byte_to_send);
}

unsigned char SPI_transfer16(uint16_t word_to_send){
	cb_spi_txrx(spi_i, (unsigned char)(word_to_send >> 8));
	return SPI_transfer((unsigned char)(word_to_send & 0xFF));
}

unsigned char SPI_send(unsigned char *bytes, int num){
	cb_spi_send(spi_i, bytes, num);
}

void register_spi_gpio_callbacks( void *cs,
								  void *send,
								  void *txrx,
								  void *write,
								  void *read,
								  void *dir,
								  void *spi_inst,
								  void *gpio_inst ) {
	cb_spi_cs 	  = (spi_cs_callback_t     *)cs ;
	cb_spi_send   = (spi_send_callback_t   *)send ;
	cb_spi_txrx   = (spi_txrx_callback_t   *)txrx ;
	cb_gpio_write = (gpio_write_callback_t *)write;
	cb_gpio_read  = (gpio_read_callback_t  *)read ;
	cb_gpio_dir   = (gpio_dir_callback_t   *)dir ;
	spi_i		  = (spi_instance_t  *)spi_inst;
	gpio_i		  = (gpio_instance_t *)gpio_inst;
}
