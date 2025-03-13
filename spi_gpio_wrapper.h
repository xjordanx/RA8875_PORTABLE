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
extern enum gpio_direction;
typedef unsigned char (*spi_cs_callback_t)( void *context,
											unsigned int cs_pin_mask,
											unsigned char value );
typedef unsigned char (*spi_send_callback_t)( 	void *context,
												unsigned char *bytes,
												unsigned char length );
typedef unsigned char (*spi_txrx_callback_t)( 	void *context,
												unsigned char txbyte );
typedef unsigned char (*gpio_write_callback_t)( void *context,
												unsigned int pin_num,
												unsigned int value );
typedef unsigned char (*gpio_read_callback_t)( 	void *context,
												unsigned int pin_num,
												unsigned int value );
typedef unsigned char (*gpio_dir_callback_t)( 	void *context,
												unsigned int pin_num,
												enum gpio_direction gpio_dir );
typedef void *gpio_instance_t;
typedef void *spi_instance_t;
/*! Instantiate the callback functions. These must be assigned valid
 *  function handles BEFORE the user makes any attempt to call a
 *  driver or library function, as they will initially be NULL.
 */
spi_cs_callback_t     cb_spi_cs;
spi_send_callback_t   cb_spi_send;
spi_txrx_callback_t   cb_spi_txrx;
spi_instance_t		  spi_i;
gpio_write_callback_t cb_gpio_write;
gpio_read_callback_t  cb_gpio_read;
gpio_dir_callback_t   cb_gpio_dir;
gpio_instance_t	      gpio_i;

void register_spi_gpio_callbacks( void *cs,
								  void *send,
								  void *txrx,
								  void *write,
								  void *read,
								  void *dir,
								  void *spi_inst,
								  void *gpio_inst );

// Arduino digital read and write wrappers
unsigned char digitalWrite( uint32_t pin_num, uint32_t value );
unsigned char pinMode( uint32_t pin_num, enum gpio_direction io_dir );
unsigned char digitalRead( uint32_t pin_num );

void SPI_CS( uint32_t cs_pin, unsigned char value );
unsigned char SPI_transfer( unsigned char byte_to_send );
unsigned char SPI_transfer16( uint16_t word_to_send );
unsigned char SPI_send(unsigned char *bytes, int num)

#endif /* SPI_GPIO_WRAPPER_H */
