/*!
 * @file     RA8875.c
 *
 * @mainpage RA8875 TFT C Driver (adapted from Adafruit)
 *
 * @author   Limor Friend/Ladyada, K.Townsend/KTOWN for Adafruit Industries, B. Jordan (modifications)
 *
 * @section intro_sec Introduction
 *
 * This is the library for the Adafruit RA8875 Driver board for TFT displays
 * ---------------> http://www.adafruit.com/products/1590
 * The RA8875 is a TFT driver for up to 800x480 dotclock'd displays
 * It is tested to work with displays in the Adafruit shop. Other displays
 * may need timing adjustments and are not guanteed to work.
 *
 * Adafruit invests time and resources providing this open
 * source code, please support Adafruit and open-source hardware
 * by purchasing products from Adafruit!
 *
 * @section author Author
 *
 * Written by Limor Fried/Ladyada for Adafruit Industries. Adapted to C for bare-metal by B. Jordan for Quilter.ai.
 *
 * @section license License
 *
 * BSD license, check license.txt for more information.
 * All text above must be included in any redistribution.
 *
 * @section  HISTORY
 *
 * v1.1 - Adaptation to C by B. Jordan.
 * v1.0 - First release
 *
 */
#include <stdio.h>
#include <stdbool.h>
#include "RA8875.h"

#ifndef HIGH
#define HIGH 1
#endif

#ifndef LOW
#define LOW 0
#endif

#if defined(EEPROM_SUPPORTED)
#include <EEPROM.h>
#endif

#include "spi_gpio_wrapper.h" // Wrapper for low-level system HAL and SPI driver.

#define spi_speed 3800000 // 3.8MHz

// If the SPI library has transaction support, these functions
// establish settings and protect from interference from other
// libraries.  Otherwise, they simply do nothing.
#ifdef SPI_HAS_TRANSACTION
static inline void spi_begin(void) __attribute__((always_inline));
static inline void spi_begin(void) {
  // max speed!
  SPI_beginTransaction(SPISettings(spi_speed, MSBFIRST, SPI_MODE0));
}
static inline void spi_end(void) __attribute__((always_inline));
static inline void spi_end(void) { SPI_endTransaction(); }
#else
#define spi_begin() ///< Create dummy Macro Function
#define spi_end()   ///< Create dummy Macro Function
#endif

/**************************************************************************/
/*!
      Initialises the LCD driver and any HW required by the display

      @param s The display size, which can be either:
                  'RA8875_480x80'  (3.8" displays) or
                  'RA8875_480x128' (3.9" displays) or
                  'RA8875_480x272' (4.3" displays) or
                  'RA8875_800x480' (5" and 7" displays)

      @return True if we reached the end
*/
/**************************************************************************/
unsigned char RA8875_init(struct RA8875_instance *thistft,
			   tft_size_t size, uint8_t cs_pin, 
			   uint8_t rst_pin, uint8_t textScale,
			   uint8_t voffset, uint8_t rotation )
{
  if (size == RA8875_480x80) {
    thistft->width = 480;
    thistft->height = 80;
  } else if (size == RA8875_480x128) {
    thistft->width = 480;
    thistft->height = 128;
  } else if (size == RA8875_480x272) {
    thistft->width = 480;
    thistft->height = 272;
  } else if (size == RA8875_800x480) {
    thistft->width = 800;
    thistft->height = 480;
  } else {
    return false;
  }
  thistft->size = size;
  thistft->rotation = rotation;
  thistft->cs_pin = cs_pin;
  thistft->rst_pin = rst_pin;
  thistft->textScale = textScale;
  thistft->voffset = voffset;
  thistft->rotation = rotation;
  
  SPI_CS(thistft->cs_pin, HIGH);
  digitalWrite(thistft->rst_pin, LOW);
  delay(100);
  digitalWrite(thistft->rst_pin, HIGH);
  delay(100);
  //SPI_begin();

  uint8_t x = RA8875_readReg(thistft, 0);
  if (x != 0x75) {
#ifdef ANSI_CODES_H
    printf("\n%s calling RA8875_readReg(0) returned value %0x\n", ANSI_BOLD ANSI_FG_BRIGHT_RED "ERROR:", x);
#else
	printf("ERROR: calling RA8875_readReg(0) returned value %0x\n", x);
#endif
    return false;
  }
  RA8875_initialize(thistft);
  return true;
}

/************************* Initialization *********************************/

/**************************************************************************/
/*!
      Performs a SW-based reset of the RA8875
*/
/**************************************************************************/
void RA8875_softReset(struct RA8875_instance *thistft) {
  RA8875_writeCommand(thistft, RA8875_PWRR);
  RA8875_writeData(thistft, RA8875_PWRR_SOFTRESET);
  RA8875_writeData(thistft, RA8875_PWRR_NORMAL);
  delay(1);
}

/**************************************************************************/
/*!
      Initialise the PLL
*/
/**************************************************************************/
void RA8875_PLLinit(struct RA8875_instance *thistft) {
  if (	thistft->size == RA8875_480x80 ||
		thistft->size == RA8875_480x128 ||
		thistft->size == RA8875_480x272) {
    RA8875_writeReg(thistft, RA8875_PLLC1, RA8875_PLLC1_PLLDIV1 + 10);
    delay(1);
    RA8875_writeReg(thistft, RA8875_PLLC2, RA8875_PLLC2_DIV4);
    delay(1);
  } else /* (_size == RA8875_800x480) */ {
    RA8875_writeReg(thistft, RA8875_PLLC1, RA8875_PLLC1_PLLDIV1 + 11);
    delay(1);
    RA8875_writeReg(thistft, RA8875_PLLC2, RA8875_PLLC2_DIV4);
    delay(1);
  }
}

/**************************************************************************/
/*!
      Initialises the driver IC (clock setup, etc.)
*/
/**************************************************************************/
void RA8875_initialize(struct RA8875_instance *thistft) {
	// RA8875_init must call this function to set size first.
  RA8875_PLLinit(thistft);
  RA8875_writeReg(thistft, RA8875_SYSR, RA8875_SYSR_16BPP | RA8875_SYSR_MCU8);

  /* Timing values */
  uint8_t pixclk;
  uint8_t hsync_start;
  uint8_t hsync_pw;
  uint8_t hsync_finetune;
  uint8_t hsync_nondisp;
  uint8_t vsync_pw;
  uint16_t vsync_nondisp;
  uint16_t vsync_start;

  /* Set the correct values for the display being used */
  if (thistft->size == RA8875_480x80) {
    pixclk = RA8875_PCSR_PDATL | RA8875_PCSR_4CLK;
    hsync_nondisp = 10;
    hsync_start = 8;
    hsync_pw = 48;
    hsync_finetune = 0;
    vsync_nondisp = 3;
    vsync_start = 8;
    vsync_pw = 10;
    thistft->voffset = 192; // This uses the bottom 80 pixels of a 272 pixel controller
  } else if (thistft->size == RA8875_480x128 || thistft->size == RA8875_480x272) {
    pixclk = RA8875_PCSR_PDATL | RA8875_PCSR_4CLK;
    hsync_nondisp = 10;
    hsync_start = 8;
    hsync_pw = 48;
    hsync_finetune = 0;
    vsync_nondisp = 3;
    vsync_start = 8;
    vsync_pw = 10;
    thistft->voffset = 0;
  } else // (thistft->size == RA8875_800x480)
  {
    pixclk = RA8875_PCSR_PDATL | RA8875_PCSR_2CLK;
    hsync_nondisp = 26;
    hsync_start = 32;
    hsync_pw = 96;
    hsync_finetune = 0;
    vsync_nondisp = 32;
    vsync_start = 23;
    vsync_pw = 2;
    thistft->voffset = 0;
  }

  RA8875_writeReg(thistft, RA8875_PCSR, pixclk);
  delay(1);

  /* Horizontal settings registers */
  RA8875_writeReg(thistft, RA8875_HDWR, (thistft->width / 8) - 1); // H width: (HDWR + 1) * 8 = 480
  RA8875_writeReg(thistft, RA8875_HNDFTR, RA8875_HNDFTR_DE_HIGH + hsync_finetune);
  RA8875_writeReg(thistft, RA8875_HNDR, (hsync_nondisp - hsync_finetune - 2) /
                            8); // H non-display: HNDR * 8 + HNDFTR + 2 = 10
  RA8875_writeReg(thistft, RA8875_HSTR, hsync_start / 8 - 1); // Hsync start: (HSTR + 1)*8
  RA8875_writeReg(thistft, RA8875_HPWR,
           RA8875_HPWR_LOW +
               (hsync_pw / 8 - 1)); // HSync pulse width = (HPWR+1) * 8

  /* Vertical settings registers */
  RA8875_writeReg(thistft, RA8875_VDHR0,
		(uint16_t)(thistft->height - 1 + thistft->voffset) & 0xFF);
  RA8875_writeReg(thistft, RA8875_VDHR1,
		(uint16_t)(thistft->height - 1 + thistft->voffset) >> 8);
  RA8875_writeReg(thistft, RA8875_VNDR0, vsync_nondisp - 1); // V non-display period = VNDR + 1
  RA8875_writeReg(thistft, RA8875_VNDR1, vsync_nondisp >> 8);
  RA8875_writeReg(thistft, RA8875_VSTR0, vsync_start - 1); // Vsync start position = VSTR + 1
  RA8875_writeReg(thistft, RA8875_VSTR1, vsync_start >> 8);
  RA8875_writeReg(thistft, RA8875_VPWR,
           RA8875_VPWR_LOW + vsync_pw - 1); // Vsync pulse width = VPWR + 1

  /* Set active window X */
  RA8875_writeReg(thistft, RA8875_HSAW0, 0); // horizontal start point
  RA8875_writeReg(thistft, RA8875_HSAW1, 0);
  RA8875_writeReg(thistft, RA8875_HEAW0, (uint16_t)(thistft->width - 1) & 0xFF); // horizontal end point
  RA8875_writeReg(thistft, RA8875_HEAW1, (uint16_t)(thistft->width - 1) >> 8);

  /* Set active window Y */
  RA8875_writeReg(thistft, RA8875_VSAW0, 0 + thistft->voffset); // vertical start point
  RA8875_writeReg(thistft, RA8875_VSAW1, 0 + thistft->voffset);
  RA8875_writeReg(thistft, RA8875_VEAW0,
        (uint16_t)(thistft->height - 1 + thistft->voffset) & 0xFF); // vertical end point
  RA8875_writeReg(thistft, RA8875_VEAW1,
		(uint16_t)(thistft->height - 1 + thistft->voffset) >> 8);

  /* ToDo: Setup touch panel? */

  /* Clear the entire window */
  RA8875_writeReg(thistft, RA8875_MCLR, RA8875_MCLR_START | RA8875_MCLR_FULL);
  delay(500);
}

/**************************************************************************/
/*!
      Returns the display width in pixels

      @return  The 1-based display width in pixels
*/
/**************************************************************************/
uint16_t RA8875_width(struct RA8875_instance *thistft)
	{ return thistft->width; }

/**************************************************************************/
/*!
      Returns the display height in pixels

      @return  The 1-based display height in pixels
*/
/**************************************************************************/
uint16_t RA8875_height(struct RA8875_instance *thistft)
	{ return thistft->height; }

/**************************************************************************/
/*!
 Returns the current rotation (0-3)

 @return  The Rotation Setting
 */
/**************************************************************************/
int8_t RA8875_getRotation(struct RA8875_instance *thistft)
	{ return thistft->rotation; }

/**************************************************************************/
/*!
 Sets the current rotation (0-3)

 @param rotation The Rotation Setting
 */
/**************************************************************************/
void RA8875_setRotation(struct RA8875_instance *thistft, int8_t rotation) {
  switch (rotation) {
  case 2:
    thistft->rotation = rotation;
    break;
  default:
    thistft->rotation = 0;
    break;
  }
}

/************************* Text Mode ***********************************/

/**************************************************************************/
/*!
      Sets the display in text mode (as opposed to graphics mode)
*/
/**************************************************************************/
void RA8875_textMode(struct RA8875_instance *thistft) {
  /* Set text mode */
  RA8875_writeCommand(thistft, RA8875_MWCR0);
  uint8_t temp = RA8875_readData(thistft);
  temp |= RA8875_MWCR0_TXTMODE; // Set bit 7
  RA8875_writeData(thistft, temp);

  /* Select the internal (ROM) font */
  RA8875_writeCommand(thistft, 0x21);
  temp = RA8875_readData(thistft);
  temp &= ~((1 << 7) | (1 << 5)); // Clear bits 7 and 5
  RA8875_writeData(thistft, temp);
}

/**************************************************************************/
/*!
      Sets the display in text mode (as opposed to graphics mode)

      @param x The x position of the cursor (in pixels, 0..1023)
      @param y The y position of the cursor (in pixels, 0..511)
*/
/**************************************************************************/
void RA8875_textSetCursor(struct RA8875_instance *thistft,
		uint16_t x, uint16_t y) {
  x = RA8875_applyRotationX(thistft, x);
  y = RA8875_applyRotationY(thistft, y);

  /* Set cursor location */
  RA8875_writeCommand(thistft, 0x2A);
  RA8875_writeData(thistft, x & 0xFF);
  RA8875_writeCommand(thistft, 0x2B);
  RA8875_writeData(thistft, x >> 8);
  RA8875_writeCommand(thistft, 0x2C);
  RA8875_writeData(thistft, y & 0xFF);
  RA8875_writeCommand(thistft, 0x2D);
  RA8875_writeData(thistft, y >> 8);
}

/**************************************************************************/
/*!
      Sets the fore and background color when rendering text

      @param foreColor The RGB565 color to use when rendering the text
      @param bgColor   The RGB565 colot to use for the background
*/
/**************************************************************************/
void RA8875_textColor(struct RA8875_instance *thistft,
		uint16_t foreColor, uint16_t bgColor) {
  /* Set Fore Color */
  RA8875_writeCommand(thistft, 0x63);
  RA8875_writeData(thistft, (foreColor & 0xf800) >> 11);
  RA8875_writeCommand(thistft, 0x64);
  RA8875_writeData(thistft, (foreColor & 0x07e0) >> 5);
  RA8875_writeCommand(thistft, 0x65);
  RA8875_writeData(thistft, (foreColor & 0x001f));

  /* Set Background Color */
  RA8875_writeCommand(thistft, 0x60);
  RA8875_writeData(thistft, (bgColor & 0xf800) >> 11);
  RA8875_writeCommand(thistft, 0x61);
  RA8875_writeData(thistft, (bgColor & 0x07e0) >> 5);
  RA8875_writeCommand(thistft, 0x62);
  RA8875_writeData(thistft, (bgColor & 0x001f));

  /* Clear transparency flag */
  RA8875_writeCommand(thistft, 0x22);
  uint8_t temp = RA8875_readData(thistft);
  temp &= ~(1 << 6); // Clear bit 6
  RA8875_writeData(thistft, temp);
}

/**************************************************************************/
/*!
      Sets the fore color when rendering text with a transparent bg

      @param foreColor The RGB565 color to use when rendering the text
*/
/**************************************************************************/
void RA8875_textTransparent(struct RA8875_instance *thistft,
		uint16_t foreColor) {
  /* Set Fore Color */
  RA8875_writeCommand(thistft, 0x63);
  RA8875_writeData(thistft, (foreColor & 0xf800) >> 11);
  RA8875_writeCommand(thistft, 0x64);
  RA8875_writeData(thistft, (foreColor & 0x07e0) >> 5);
  RA8875_writeCommand(thistft, 0x65);
  RA8875_writeData(thistft, (foreColor & 0x001f));

  /* Set transparency flag */
  RA8875_writeCommand(thistft, 0x22);
  uint8_t temp = RA8875_readData(thistft);
  temp |= (1 << 6); // Set bit 6
  RA8875_writeData(thistft, temp);
}

/**************************************************************************/
/*!
      Sets the text enlarge settings, using one of the following values:

      0 = 1x zoom
      1 = 2x zoom
      2 = 3x zoom
      3 = 4x zoom

      @param scale   The zoom factor (0..3 for 1-4x zoom)
*/
/**************************************************************************/
void RA8875_textEnlarge(struct RA8875_instance *thistft, uint8_t scale) {
  if (scale > 3)
    scale = 3; // highest setting is 3

  /* Set font size flags */
  RA8875_writeCommand(thistft, 0x22);
  uint8_t temp = RA8875_readData(thistft);
  temp &= ~(0xF); // Clears bits 0..3
  temp |= scale << 2;
  temp |= scale;

  RA8875_writeData(thistft, temp);

  thistft->textScale = scale;
}

/**************************************************************************/
/*!
     Enable Cursor Visibility and Blink
     Here we set bits 6 and 5 in 40h
     As well as the set the blink rate in 44h
     The rate is 0 through max 255
     the lower the number the faster it blinks (00h is 1 frame time,
     FFh is 256 Frames time.
     Blink Time (sec) = BTCR[44h]x(1/Frame_rate)

     @param rate The frame rate to blink
 */
/**************************************************************************/

void RA8875_cursorBlink(struct RA8875_instance *thistft,
		uint8_t rate) {

  RA8875_writeCommand(thistft, RA8875_MWCR0);
  uint8_t temp = RA8875_readData(thistft);
  temp |= RA8875_MWCR0_CURSOR;
  RA8875_writeData(thistft, temp);

  RA8875_writeCommand(thistft, RA8875_MWCR0);
  temp = RA8875_readData(thistft);
  temp |= RA8875_MWCR0_BLINK;
  RA8875_writeData(thistft, temp);

  if (rate > 255)
    rate = 255;
  RA8875_writeCommand(thistft, RA8875_BTCR);
  RA8875_writeData(thistft, rate);
}

/**************************************************************************/
/*!
      Renders some text on the screen when in text mode

      @param buffer    The buffer containing the characters to render
      @param len       The size of the buffer in bytes
*/
/**************************************************************************/
void RA8875_textWrite(struct RA8875_instance *thistft, const char *buffer, uint16_t len) {
  RA8875_writeCommand(thistft, RA8875_MRWC);
  for (uint16_t i = 0; i < len; i++) {
    RA8875_writeData(thistft, buffer[i]);
/// @cond DISABLE
#if defined(__arm__)
    /// @endcond
    // This delay is needed with textEnlarge(1) because
    // Teensy 3.X is much faster than Arduino Uno
    if (thistft->textScale > 0)
      delay(1);
/// @cond DISABLE
#else
    /// @endcond
    // For others, delay starting with textEnlarge(2)
    if (thistft->textScale > 1)
      delay(1);
/// @cond DISABLE
#endif
    /// @endcond
  }
}

/************************* Graphics ***********************************/

/**************************************************************************/
/*!
      Sets the display in graphics mode (as opposed to text mode)
*/
/**************************************************************************/
void RA8875_graphicsMode(struct RA8875_instance *thistft) {
  RA8875_writeCommand(thistft, RA8875_MWCR0);
  uint8_t temp = RA8875_readData(thistft);
  temp &= ~RA8875_MWCR0_TXTMODE; // bit #7
  RA8875_writeData(thistft, temp);
}

/**************************************************************************/
/*!
      Waits for screen to finish by polling the status!

      @param regname The register name to check
      @param waitflag The value to wait for the status register to match

      @return True if the expected status has been reached
*/
/**************************************************************************/
bool RA8875_waitPoll(struct RA8875_instance *thistft,
		uint8_t regname, uint8_t waitflag) {
  /* Wait for the command to finish */
  while (1) {
    uint8_t temp = RA8875_readReg(thistft, regname);
    if (!(temp & waitflag))
      return true;
  }
  return false; // MEMEFIX: yeah i know, unreached! - add timeout?
}

/**************************************************************************/
/*!
      Sets the current X/Y position on the display before RA8875_drawing

      @param x The 0-based x location
      @param y The 0-base y location
*/
/**************************************************************************/
void RA8875_setXY(struct RA8875_instance *thistft,
		uint16_t x, uint16_t y) {
  RA8875_writeReg(thistft, RA8875_CURH0, x);
  RA8875_writeReg(thistft, RA8875_CURH1, x >> 8);
  RA8875_writeReg(thistft, RA8875_CURV0, y);
  RA8875_writeReg(thistft, RA8875_CURV1, y >> 8);
}

/**************************************************************************/
/*!
      HW accelerated function to push a chunk of raw pixel data

      @param num The number of pixels to push
      @param p   The pixel color to use
*/
/**************************************************************************/
void RA8875_pushPixels(struct RA8875_instance *thistft,
		uint32_t num, uint16_t p) {
  SPI_CS(thistft->cs_pin, LOW);
  SPI_transfer(RA8875_DATAWRITE);
  while (num--) {
    SPI_transfer(p >> 8);
    SPI_transfer(p);
  }
  SPI_CS(thistft->cs_pin, HIGH);
}

/**************************************************************************/
/*!
    Fill the screen with the current color
*/
/**************************************************************************/
void RA8875_fillAll(struct RA8875_instance *thistft) {
  RA8875_writeCommand(thistft, RA8875_DCR);
  RA8875_writeData(thistft, RA8875_DCR_LINESQUTRI_STOP | RA8875_DCR_DRAWSQUARE);
  RA8875_writeData(thistft, RA8875_DCR_LINESQUTRI_START | RA8875_DCR_FILL |
            RA8875_DCR_DRAWSQUARE);
}

/**************************************************************************/
/*!
    Apply current rotation in the X direction

    @return the X value with current rotation applied
 */
/**************************************************************************/
int16_t RA8875_applyRotationX(struct RA8875_instance *thistft, int16_t x) {
  switch (thistft->rotation) {
  case 2:
    x = thistft->width - 1 - x;
    break;
  }

  return x;
}

/**************************************************************************/
/*!
    Apply current rotation in the Y direction

    @return the Y value with current rotation applied
 */
/**************************************************************************/
int16_t RA8875_applyRotationY(struct RA8875_instance *thistft, int16_t y) {
  switch (thistft->rotation) {
  case 2:
    y = thistft->height - 1 - y;
    break;
  }

  return y + thistft->voffset;
}

/**************************************************************************/
/*!
      Draws a single pixel at the specified location

      @param x     The 0-based x location
      @param y     The 0-base y location
      @param color The RGB565 color to use when RA8875_drawing the pixel
*/
/**************************************************************************/
void RA8875_drawPixel( struct RA8875_instance *thistft,
					int16_t x, int16_t y, uint16_t color) {
  x = RA8875_applyRotationX(thistft, x);
  y = RA8875_applyRotationY(thistft, y);

  RA8875_writeReg(thistft, RA8875_CURH0, x);
  RA8875_writeReg(thistft, RA8875_CURH1, x >> 8);
  RA8875_writeReg(thistft, RA8875_CURV0, y);
  RA8875_writeReg(thistft, RA8875_CURV1, y >> 8);
  RA8875_writeCommand(thistft, RA8875_MRWC);
  SPI_CS(thistft->cs_pin, LOW);
  SPI_transfer(RA8875_DATAWRITE);
  SPI_transfer(color >> 8);
  SPI_transfer(color);
  SPI_CS(thistft->cs_pin, HIGH);
}

/**************************************************************************/
/*!
 Draws a series of pixels at the specified location without the overhead

 @param p     An array of RGB565 color pixels
 @param num   The number of the pixels to RA8875_draw
 @param x     The 0-based x location
 @param y     The 0-base y location
 */
/**************************************************************************/
void RA8875_drawPixels(struct RA8875_instance *thistft,
		uint16_t *p, uint32_t num, int16_t x,
                                 int16_t y) {
  x = RA8875_applyRotationX(thistft, x);
  y = RA8875_applyRotationY(thistft, y);

  RA8875_writeReg(thistft, RA8875_CURH0, x);
  RA8875_writeReg(thistft, RA8875_CURH1, x >> 8);
  RA8875_writeReg(thistft, RA8875_CURV0, y);
  RA8875_writeReg(thistft, RA8875_CURV1, y >> 8);

  uint8_t dir = RA8875_MWCR0_LRTD;
  if (thistft->rotation == 2) {
    dir = RA8875_MWCR0_RLTD;
  }
  RA8875_writeReg(thistft, RA8875_MWCR0,
		  	  	  (RA8875_readReg(thistft, RA8875_MWCR0)
		  	  			  & ~RA8875_MWCR0_DIRMASK) | dir);

  RA8875_writeCommand(thistft, RA8875_MRWC);
  SPI_CS(thistft->cs_pin, LOW);
  SPI_transfer(RA8875_DATAWRITE);
  while (num--) {
    SPI_transfer16(*p++);
  }
  SPI_CS(thistft->cs_pin, HIGH);
}

/**************************************************************************/
/*!
      Draws a HW accelerated line on the display

      @param x0    The 0-based starting x location
      @param y0    The 0-base starting y location
      @param x1    The 0-based ending x location
      @param y1    The 0-base ending y location
      @param color The RGB565 color to use when RA8875_drawing the pixel
*/
/**************************************************************************/
void RA8875_drawLine(struct RA8875_instance *thistft,
		int16_t x0, int16_t y0, int16_t x1, int16_t y1,
                               uint16_t color) {
  x0 = RA8875_applyRotationX(thistft, x0);
  y0 = RA8875_applyRotationY(thistft, y0);
  x1 = RA8875_applyRotationX(thistft, x1);
  y1 = RA8875_applyRotationY(thistft, y1);

  /* Set X */
  RA8875_writeCommand(thistft, 0x91);
  RA8875_writeData(thistft, x0);
  RA8875_writeCommand(thistft, 0x92);
  RA8875_writeData(thistft, x0 >> 8);

  /* Set Y */
  RA8875_writeCommand(thistft, 0x93);
  RA8875_writeData(thistft, y0);
  RA8875_writeCommand(thistft, 0x94);
  RA8875_writeData(thistft, y0 >> 8);

  /* Set X1 */
  RA8875_writeCommand(thistft, 0x95);
  RA8875_writeData(thistft, x1);
  RA8875_writeCommand(thistft, 0x96);
  RA8875_writeData(thistft, (x1) >> 8);

  /* Set Y1 */
  RA8875_writeCommand(thistft, 0x97);
  RA8875_writeData(thistft, y1);
  RA8875_writeCommand(thistft, 0x98);
  RA8875_writeData(thistft, (y1) >> 8);

  /* Set Color */
  RA8875_writeCommand(thistft, 0x63);
  RA8875_writeData(thistft, (color & 0xf800) >> 11);
  RA8875_writeCommand(thistft, 0x64);
  RA8875_writeData(thistft, (color & 0x07e0) >> 5);
  RA8875_writeCommand(thistft, 0x65);
  RA8875_writeData(thistft, (color & 0x001f));

  /* Draw! */
  RA8875_writeCommand(thistft, RA8875_DCR);
  RA8875_writeData(thistft, 0x80);

  /* Wait for the command to finish */
  RA8875_waitPoll(thistft, RA8875_DCR, RA8875_DCR_LINESQUTRI_STATUS);
}

/**************************************************************************/
/*!
    Draw a vertical line

    @param x The X position
    @param y The Y position
    @param h Height
    @param color The color
*/
/**************************************************************************/
void RA8875_drawFastVLine(struct RA8875_instance *thistft,
		int16_t x, int16_t y, int16_t h,
                                    uint16_t color) {
  RA8875_drawLine(thistft, x, y, x, y + h, color);
}

/**************************************************************************/
/*!
     Draw a horizontal line

     @param x The X position
     @param y The Y position
     @param w Width
     @param color The color
*/
/**************************************************************************/
void RA8875_drawFastHLine(struct RA8875_instance *thistft,
		int16_t x, int16_t y, int16_t w,
                                    uint16_t color) {
  RA8875_drawLine(thistft, x, y, x + w, y, color);
}

/**************************************************************************/
/*!
      Draws a HW accelerated rectangle on the display

      @param x     The 0-based x location of the top-right corner
      @param y     The 0-based y location of the top-right corner
      @param w     The rectangle width
      @param h     The rectangle height
      @param color The RGB565 color to use when RA8875_drawing the pixel
*/
/**************************************************************************/
void RA8875_drawRect(struct RA8875_instance *thistft,
		int16_t x, int16_t y, int16_t w, int16_t h,
                               uint16_t color) {
  RA8875_rectHelper(thistft, x, y, x + w - 1, y + h - 1, color, false);
}

/**************************************************************************/
/*!
      Draws a HW accelerated filled rectangle on the display

      @param x     The 0-based x location of the top-right corner
      @param y     The 0-based y location of the top-right corner
      @param w     The rectangle width
      @param h     The rectangle height
      @param color The RGB565 color to use when RA8875_drawing the pixel
*/
/**************************************************************************/
void RA8875_fillRect(struct RA8875_instance *thistft,
		int16_t x, int16_t y, int16_t w, int16_t h,
                               uint16_t color) {
  RA8875_rectHelper(thistft, x, y, x + w - 1, y + h - 1, color, true);
}

/**************************************************************************/
/*!
      Fills the screen with the spefied RGB565 color

      @param color The RGB565 color to use when RA8875_drawing the pixel
*/
/**************************************************************************/
void RA8875_fillScreen(struct RA8875_instance *thistft,
		uint16_t color) {
  RA8875_rectHelper(thistft, 0, 0,
		  thistft->width - 1,
		  thistft->height - 1, color, true);
}

/**************************************************************************/
/*!
      Draws a HW accelerated circle on the display

      @param x     The 0-based x location of the center of the circle
      @param y     The 0-based y location of the center of the circle
      @param r     The circle's radius
      @param color The RGB565 color to use when RA8875_drawing the pixel
*/
/**************************************************************************/
void RA8875_drawCircle(struct RA8875_instance *thistft,
		int16_t x, int16_t y, int16_t r,
                                 uint16_t color) {
  RA8875_circleHelper(thistft, x, y, r, color, false);
}

/**************************************************************************/
/*!
      Draws a HW accelerated filled circle on the display

      @param x     The 0-based x location of the center of the circle
      @param y     The 0-based y location of the center of the circle
      @param r     The circle's radius
      @param color The RGB565 color to use when RA8875_drawing the pixel
*/
/**************************************************************************/
void RA8875_fillCircle(struct RA8875_instance *thistft,
		int16_t x, int16_t y, int16_t r,
        uint16_t color) {
  RA8875_circleHelper(thistft, x, y, r, color, true);
}

/**************************************************************************/
/*!
      Draws a HW accelerated triangle on the display

      @param x0    The 0-based x location of point 0 on the triangle
      @param y0    The 0-based y location of point 0 on the triangle
      @param x1    The 0-based x location of point 1 on the triangle
      @param y1    The 0-based y location of point 1 on the triangle
      @param x2    The 0-based x location of point 2 on the triangle
      @param y2    The 0-based y location of point 2 on the triangle
      @param color The RGB565 color to use when RA8875_drawing the pixel
*/
/**************************************************************************/
void RA8875_drawTriangle(struct RA8875_instance *thistft,
		int16_t x0, int16_t y0, int16_t x1,
        int16_t y1, int16_t x2, int16_t y2,
        uint16_t color) {
  RA8875_triangleHelper(thistft, x0, y0, x1, y1, x2, y2, color, false);
}

/**************************************************************************/
/*!
      Draws a HW accelerated filled triangle on the display

      @param x0    The 0-based x location of point 0 on the triangle
      @param y0    The 0-based y location of point 0 on the triangle
      @param x1    The 0-based x location of point 1 on the triangle
      @param y1    The 0-based y location of point 1 on the triangle
      @param x2    The 0-based x location of point 2 on the triangle
      @param y2    The 0-based y location of point 2 on the triangle
      @param color The RGB565 color to use when RA8875_drawing the pixel
*/
/**************************************************************************/
void RA8875_fillTriangle(struct RA8875_instance *thistft,
		int16_t x0, int16_t y0, int16_t x1,
        int16_t y1, int16_t x2, int16_t y2,
        uint16_t color) {
  RA8875_triangleHelper(thistft, x0, y0, x1, y1, x2, y2, color, true);
}

/**************************************************************************/
/*!
      Draws a HW accelerated ellipse on the display

      @param xCenter   The 0-based x location of the ellipse's center
      @param yCenter   The 0-based y location of the ellipse's center
      @param longAxis  The size in pixels of the ellipse's long axis
      @param shortAxis The size in pixels of the ellipse's short axis
      @param color     The RGB565 color to use when RA8875_drawing the pixel
*/
/**************************************************************************/
void RA8875_drawEllipse(struct RA8875_instance *thistft,
		int16_t xCenter, int16_t yCenter,
        int16_t longAxis, int16_t shortAxis,
        uint16_t color) {
  RA8875_ellipseHelper(thistft, xCenter, yCenter, longAxis, shortAxis, color, false);
}

/**************************************************************************/
/*!
      Draws a HW accelerated filled ellipse on the display

      @param xCenter   The 0-based x location of the ellipse's center
      @param yCenter   The 0-based y location of the ellipse's center
      @param longAxis  The size in pixels of the ellipse's long axis
      @param shortAxis The size in pixels of the ellipse's short axis
      @param color     The RGB565 color to use when RA8875_drawing the pixel
*/
/**************************************************************************/
void RA8875_fillEllipse(struct RA8875_instance *thistft,
		int16_t xCenter, int16_t yCenter,
        int16_t longAxis, int16_t shortAxis,
        uint16_t color) {
  RA8875_ellipseHelper(thistft, xCenter, yCenter, longAxis, shortAxis, color, true);
}

/**************************************************************************/
/*!
      Draws a HW accelerated curve on the display

      @param xCenter   The 0-based x location of the ellipse's center
      @param yCenter   The 0-based y location of the ellipse's center
      @param longAxis  The size in pixels of the ellipse's long axis
      @param shortAxis The size in pixels of the ellipse's short axis
      @param curvePart The corner to RA8875_draw, where in clock-wise motion:
                            0 = 180-270°
                            1 = 270-0°
                            2 = 0-90°
                            3 = 90-180°
      @param color     The RGB565 color to use when RA8875_drawing the pixel
*/
/**************************************************************************/
void RA8875_drawCurve(struct RA8875_instance *thistft,
		int16_t xCenter, int16_t yCenter,
        int16_t longAxis, int16_t shortAxis,
         uint8_t curvePart, uint16_t color) {
  RA8875_curveHelper(thistft, xCenter, yCenter, longAxis, shortAxis, curvePart, color, false);
}

/**************************************************************************/
/*!
      Draws a HW accelerated filled curve on the display

      @param xCenter   The 0-based x location of the ellipse's center
      @param yCenter   The 0-based y location of the ellipse's center
      @param longAxis  The size in pixels of the ellipse's long axis
      @param shortAxis The size in pixels of the ellipse's short axis
      @param curvePart The corner to RA8875_draw, where in clock-wise motion:
                            0 = 180-270°
                            1 = 270-0°
                            2 = 0-90°
                            3 = 90-180°
      @param color     The RGB565 color to use when RA8875_drawing the pixel
*/
/**************************************************************************/
void RA8875_fillCurve(struct RA8875_instance *thistft,
		int16_t xCenter, int16_t yCenter,
        int16_t longAxis, int16_t shortAxis,
        uint8_t curvePart, uint16_t color) {
  RA8875_curveHelper(thistft, xCenter, yCenter, longAxis, shortAxis, curvePart, color, true);
}

/**************************************************************************/
/*!
      Draws a HW accelerated rounded rectangle on the display

      @param x   The 0-based x location of the rectangle's upper left corner
      @param y   The 0-based y location of the rectangle's upper left corner
      @param w   The size in pixels of the rectangle's width
      @param h   The size in pixels of the rectangle's height
      @param r   The radius of the curves in the corners of the rectangle
      @param color  The RGB565 color to use when RA8875_drawing the pixel
 */
/**************************************************************************/
void RA8875_drawRoundRect(struct RA8875_instance *thistft,
		int16_t x, int16_t y, int16_t w, int16_t h,
        int16_t r, uint16_t color) {
  RA8875_roundRectHelper(thistft, x, y, x + w, y + h, r, color, false);
}

/**************************************************************************/
/*!
      Draws a HW accelerated filled rounded rectangle on the display

      @param x   The 0-based x location of the rectangle's upper left corner
      @param y   The 0-based y location of the rectangle's upper left corner
      @param w   The size in pixels of the rectangle's width
      @param h   The size in pixels of the rectangle's height
      @param r   The radius of the curves in the corners of the rectangle
      @param color  The RGB565 color to use when RA8875_drawing the pixel
 */
/**************************************************************************/
void RA8875_fillRoundRect(struct RA8875_instance *thistft,
		int16_t x, int16_t y, int16_t w, int16_t h,
        int16_t r, uint16_t color) {
  RA8875_roundRectHelper(thistft, x, y, x + w, y + h, r, color, true);
}

/**************************************************************************/
/*!
      Helper function for higher level circle RA8875_drawing code
*/
/**************************************************************************/
void RA8875_circleHelper(struct RA8875_instance *thistft,
		int16_t x, int16_t y, int16_t r,
        uint16_t color, bool filled) {
  x = RA8875_applyRotationX(thistft, x);
  y = RA8875_applyRotationY(thistft, y);

  /* Set X */
  RA8875_writeCommand(thistft, 0x99);
  RA8875_writeData(thistft, x);
  RA8875_writeCommand(thistft, 0x9a);
  RA8875_writeData(thistft, x >> 8);

  /* Set Y */
  RA8875_writeCommand(thistft, 0x9b);
  RA8875_writeData(thistft, y);
  RA8875_writeCommand(thistft, 0x9c);
  RA8875_writeData(thistft, y >> 8);

  /* Set Radius */
  RA8875_writeCommand(thistft, 0x9d);
  RA8875_writeData(thistft, r);

  /* Set Color */
  RA8875_writeCommand(thistft, 0x63);
  RA8875_writeData(thistft, (color & 0xf800) >> 11);
  RA8875_writeCommand(thistft, 0x64);
  RA8875_writeData(thistft, (color & 0x07e0) >> 5);
  RA8875_writeCommand(thistft, 0x65);
  RA8875_writeData(thistft, (color & 0x001f));

  /* Draw! */
  RA8875_writeCommand(thistft, RA8875_DCR);
  if (filled) {
    RA8875_writeData(thistft, RA8875_DCR_CIRCLE_START | RA8875_DCR_FILL);
  } else {
    RA8875_writeData(thistft, RA8875_DCR_CIRCLE_START | RA8875_DCR_NOFILL);
  }

  /* Wait for the command to finish */
  RA8875_waitPoll(thistft, RA8875_DCR, RA8875_DCR_CIRCLE_STATUS);
}

/**************************************************************************/
/*!
      Helper function for higher level rectangle RA8875_drawing code
*/
/**************************************************************************/
void RA8875_rectHelper(struct RA8875_instance *thistft,
		int16_t x, int16_t y, int16_t w, int16_t h,
        uint16_t color, bool filled) {
  x = RA8875_applyRotationX(thistft, x);
  y = RA8875_applyRotationY(thistft, y);
  w = RA8875_applyRotationX(thistft, w);
  h = RA8875_applyRotationY(thistft, h);

  /* Set X */
  RA8875_writeCommand(thistft, 0x91);
  RA8875_writeData(thistft, x);
  RA8875_writeCommand(thistft, 0x92);
  RA8875_writeData(thistft, x >> 8);

  /* Set Y */
  RA8875_writeCommand(thistft, 0x93);
  RA8875_writeData(thistft, y);
  RA8875_writeCommand(thistft, 0x94);
  RA8875_writeData(thistft, y >> 8);

  /* Set X1 */
  RA8875_writeCommand(thistft, 0x95);
  RA8875_writeData(thistft, w);
  RA8875_writeCommand(thistft, 0x96);
  RA8875_writeData(thistft, (w) >> 8);

  /* Set Y1 */
  RA8875_writeCommand(thistft, 0x97);
  RA8875_writeData(thistft, h);
  RA8875_writeCommand(thistft, 0x98);
  RA8875_writeData(thistft, (h) >> 8);

  /* Set Color */
  RA8875_writeCommand(thistft, 0x63);
  RA8875_writeData(thistft, (color & 0xf800) >> 11);
  RA8875_writeCommand(thistft, 0x64);
  RA8875_writeData(thistft, (color & 0x07e0) >> 5);
  RA8875_writeCommand(thistft, 0x65);
  RA8875_writeData(thistft, (color & 0x001f));

  /* Draw! */
  RA8875_writeCommand(thistft, RA8875_DCR);
  if (filled) {
    RA8875_writeData(thistft, 0xB0);
  } else {
    RA8875_writeData(thistft, 0x90);
  }

  /* Wait for the command to finish */
  RA8875_waitPoll(thistft, RA8875_DCR, RA8875_DCR_LINESQUTRI_STATUS);
}

/**************************************************************************/
/*!
      Helper function for higher level triangle RA8875_drawing code
*/
/**************************************************************************/
void RA8875_triangleHelper(struct RA8875_instance *thistft,
		int16_t x0, int16_t y0, int16_t x1,
        int16_t y1, int16_t x2, int16_t y2,
        uint16_t color, bool filled) {
  x0 = RA8875_applyRotationX(thistft, x0);
  y0 = RA8875_applyRotationY(thistft, y0);
  x1 = RA8875_applyRotationX(thistft, x1);
  y1 = RA8875_applyRotationY(thistft, y1);
  x2 = RA8875_applyRotationX(thistft, x2);
  y2 = RA8875_applyRotationY(thistft, y2);

  /* Set Point 0 */
  RA8875_writeCommand(thistft, 0x91);
  RA8875_writeData(thistft, x0);
  RA8875_writeCommand(thistft, 0x92);
  RA8875_writeData(thistft, x0 >> 8);
  RA8875_writeCommand(thistft, 0x93);
  RA8875_writeData(thistft, y0);
  RA8875_writeCommand(thistft, 0x94);
  RA8875_writeData(thistft, y0 >> 8);

  /* Set Point 1 */
  RA8875_writeCommand(thistft, 0x95);
  RA8875_writeData(thistft, x1);
  RA8875_writeCommand(thistft, 0x96);
  RA8875_writeData(thistft, x1 >> 8);
  RA8875_writeCommand(thistft, 0x97);
  RA8875_writeData(thistft, y1);
  RA8875_writeCommand(thistft, 0x98);
  RA8875_writeData(thistft, y1 >> 8);

  /* Set Point 2 */
  RA8875_writeCommand(thistft, 0xA9);
  RA8875_writeData(thistft, x2);
  RA8875_writeCommand(thistft, 0xAA);
  RA8875_writeData(thistft, x2 >> 8);
  RA8875_writeCommand(thistft, 0xAB);
  RA8875_writeData(thistft, y2);
  RA8875_writeCommand(thistft, 0xAC);
  RA8875_writeData(thistft, y2 >> 8);

  /* Set Color */
  RA8875_writeCommand(thistft, 0x63);
  RA8875_writeData(thistft, (color & 0xf800) >> 11);
  RA8875_writeCommand(thistft, 0x64);
  RA8875_writeData(thistft, (color & 0x07e0) >> 5);
  RA8875_writeCommand(thistft, 0x65);
  RA8875_writeData(thistft, (color & 0x001f));

  /* Draw! */
  RA8875_writeCommand(thistft, RA8875_DCR);
  if (filled) {
    RA8875_writeData(thistft, 0xA1);
  } else {
    RA8875_writeData(thistft, 0x81);
  }

  /* Wait for the command to finish */
  RA8875_waitPoll(thistft, RA8875_DCR, RA8875_DCR_LINESQUTRI_STATUS);
}

/**************************************************************************/
/*!
      Helper function for higher level ellipse RA8875_drawing code
*/
/**************************************************************************/
void RA8875_ellipseHelper(struct RA8875_instance *thistft,
		int16_t xCenter, int16_t yCenter,
        int16_t longAxis, int16_t shortAxis,
        uint16_t color, bool filled) {
  xCenter = RA8875_applyRotationX(thistft, xCenter);
  yCenter = RA8875_applyRotationY(thistft, yCenter);

  /* Set Center Point */
  RA8875_writeCommand(thistft, 0xA5);
  RA8875_writeData(thistft, xCenter);
  RA8875_writeCommand(thistft, 0xA6);
  RA8875_writeData(thistft, xCenter >> 8);
  RA8875_writeCommand(thistft, 0xA7);
  RA8875_writeData(thistft, yCenter);
  RA8875_writeCommand(thistft, 0xA8);
  RA8875_writeData(thistft, yCenter >> 8);

  /* Set Long and Short Axis */
  RA8875_writeCommand(thistft, 0xA1);
  RA8875_writeData(thistft, longAxis);
  RA8875_writeCommand(thistft, 0xA2);
  RA8875_writeData(thistft, longAxis >> 8);
  RA8875_writeCommand(thistft, 0xA3);
  RA8875_writeData(thistft, shortAxis);
  RA8875_writeCommand(thistft, 0xA4);
  RA8875_writeData(thistft, shortAxis >> 8);

  /* Set Color */
  RA8875_writeCommand(thistft, 0x63);
  RA8875_writeData(thistft, (color & 0xf800) >> 11);
  RA8875_writeCommand(thistft, 0x64);
  RA8875_writeData(thistft, (color & 0x07e0) >> 5);
  RA8875_writeCommand(thistft, 0x65);
  RA8875_writeData(thistft, (color & 0x001f));

  /* Draw! */
  RA8875_writeCommand(thistft, 0xA0);
  if (filled) {
    RA8875_writeData(thistft, 0xC0);
  } else {
    RA8875_writeData(thistft, 0x80);
  }

  /* Wait for the command to finish */
  RA8875_waitPoll(thistft, RA8875_ELLIPSE, RA8875_ELLIPSE_STATUS);
}

/**************************************************************************/
/*!
      Helper function for higher level curve RA8875_drawing code
*/
/**************************************************************************/
void RA8875_curveHelper(struct RA8875_instance *thistft,
		int16_t xCenter, int16_t yCenter,
        int16_t longAxis, int16_t shortAxis,
        uint8_t curvePart, uint16_t color,
        bool filled) {
  xCenter = RA8875_applyRotationX(thistft, xCenter);
  yCenter = RA8875_applyRotationY(thistft, yCenter);
  curvePart = (curvePart + thistft->rotation) % 4;

  /* Set Center Point */
  RA8875_writeCommand(thistft, 0xA5);
  RA8875_writeData(thistft, xCenter);
  RA8875_writeCommand(thistft, 0xA6);
  RA8875_writeData(thistft, xCenter >> 8);
  RA8875_writeCommand(thistft, 0xA7);
  RA8875_writeData(thistft, yCenter);
  RA8875_writeCommand(thistft, 0xA8);
  RA8875_writeData(thistft, yCenter >> 8);

  /* Set Long and Short Axis */
  RA8875_writeCommand(thistft, 0xA1);
  RA8875_writeData(thistft, longAxis);
  RA8875_writeCommand(thistft, 0xA2);
  RA8875_writeData(thistft, longAxis >> 8);
  RA8875_writeCommand(thistft, 0xA3);
  RA8875_writeData(thistft, shortAxis);
  RA8875_writeCommand(thistft, 0xA4);
  RA8875_writeData(thistft, shortAxis >> 8);

  /* Set Color */
  RA8875_writeCommand(thistft, 0x63);
  RA8875_writeData(thistft, (color & 0xf800) >> 11);
  RA8875_writeCommand(thistft, 0x64);
  RA8875_writeData(thistft, (color & 0x07e0) >> 5);
  RA8875_writeCommand(thistft, 0x65);
  RA8875_writeData(thistft, (color & 0x001f));

  /* Draw! */
  RA8875_writeCommand(thistft, 0xA0);
  if (filled) {
    RA8875_writeData(thistft, 0xD0 | (curvePart & 0x03));
  } else {
    RA8875_writeData(thistft, 0x90 | (curvePart & 0x03));
  }

  /* Wait for the command to finish */
  RA8875_waitPoll(thistft, RA8875_ELLIPSE, RA8875_ELLIPSE_STATUS);
}

/**************************************************************************/
/*!
      Helper function for higher level rounded rectangle RA8875_drawing code
 */
/**************************************************************************/
void RA8875_roundRectHelper(struct RA8875_instance *thistft,
		int16_t x, int16_t y, int16_t w,
        int16_t h, int16_t r, uint16_t color,
        bool filled) {
  x = RA8875_applyRotationX(thistft, x);
  y = RA8875_applyRotationY(thistft, y);
  w = RA8875_applyRotationX(thistft, w);
  h = RA8875_applyRotationY(thistft, h);
  if (x > w)
    RA8875_swap(&x, &w);
  if (y > h)
    RA8875_swap(&y, &h);

  /* Set X */
  RA8875_writeCommand(thistft, 0x91);
  RA8875_writeData(thistft, x);
  RA8875_writeCommand(thistft, 0x92);
  RA8875_writeData(thistft, x >> 8);

  /* Set Y */
  RA8875_writeCommand(thistft, 0x93);
  RA8875_writeData(thistft, y);
  RA8875_writeCommand(thistft, 0x94);
  RA8875_writeData(thistft, y >> 8);
  
  /* Set X1 */
  RA8875_writeCommand(thistft, 0x95);
  RA8875_writeData(thistft, w);
  RA8875_writeCommand(thistft, 0x96);
  RA8875_writeData(thistft, (w) >> 8);
  
  /* Set Y1 */
  RA8875_writeCommand(thistft, 0x97);
  RA8875_writeData(thistft, h);
  RA8875_writeCommand(thistft, 0x98);
  RA8875_writeData(thistft, (h) >> 8);
  
  RA8875_writeCommand(thistft, 0xA1);
  RA8875_writeData(thistft, r);
  RA8875_writeCommand(thistft, 0xA2);
  RA8875_writeData(thistft, (r) >> 8);
  
  RA8875_writeCommand(thistft, 0xA3);
  RA8875_writeData(thistft, r);
  RA8875_writeCommand(thistft, 0xA4);
  RA8875_writeData(thistft, (r) >> 8);

  /* Set Color */
  RA8875_writeCommand(thistft, 0x63);
  RA8875_writeData(thistft, (color & 0xf800) >> 11);
  RA8875_writeCommand(thistft, 0x64);
  RA8875_writeData(thistft, (color & 0x07e0) >> 5);
  RA8875_writeCommand(thistft, 0x65);
  RA8875_writeData(thistft, (color & 0x001f));

  /* Draw! */
  RA8875_writeCommand(thistft, RA8875_ELLIPSE);
  if (filled) {
    RA8875_writeData(thistft, 0xE0);
  } else {
    RA8875_writeData(thistft, 0xA0);
  }

  /* Wait for the command to finish */
  RA8875_waitPoll(thistft, RA8875_ELLIPSE, RA8875_DCR_LINESQUTRI_STATUS);
}
/**************************************************************************/
/*!
      Set the scroll window

      @param x  X position of the scroll window
      @param y  Y position of the scroll window
      @param w  Width of the Scroll Window
      @param h  Height of the Scroll window
      @param mode Layer to Scroll

 */
/**************************************************************************/
void RA8875_setScrollWindow(struct RA8875_instance *thistft,
		int16_t x, int16_t y, int16_t w,
        int16_t h, uint8_t mode) {
  // Horizontal Start point of Scroll Window
  RA8875_writeCommand(thistft, 0x38);
  RA8875_writeData(thistft, x);
  RA8875_writeCommand(thistft, 0x39);
  RA8875_writeData(thistft, x >> 8);

  // Vertical Start Point of Scroll Window
  RA8875_writeCommand(thistft, 0x3a);
  RA8875_writeData(thistft, y);
  RA8875_writeCommand(thistft, 0x3b);
  RA8875_writeData(thistft, y >> 8);

  // Horizontal End Point of Scroll Window
  RA8875_writeCommand(thistft, 0x3c);
  RA8875_writeData(thistft, x + w);
  RA8875_writeCommand(thistft, 0x3d);
  RA8875_writeData(thistft, (x + w) >> 8);

  // Vertical End Point of Scroll Window
  RA8875_writeCommand(thistft, 0x3e);
  RA8875_writeData(thistft, y + h);
  RA8875_writeCommand(thistft, 0x3f);
  RA8875_writeData(thistft, (y + h) >> 8);

  // Scroll function setting
  RA8875_writeCommand(thistft, 0x52);
  RA8875_writeData(thistft, mode);
}

/**************************************************************************/
/*!
    Scroll in the X direction

    @param dist The distance to scroll

 */
/**************************************************************************/
void RA8875_scrollX(struct RA8875_instance *thistft,
		int16_t dist) {
  RA8875_writeCommand(thistft, 0x24);
  RA8875_writeData(thistft, dist);
  RA8875_writeCommand(thistft, 0x25);
  RA8875_writeData(thistft, dist >> 8);
}

/**************************************************************************/
/*!
     Scroll in the Y direction

     @param dist The distance to scroll

 */
/**************************************************************************/
void RA8875_scrollY(struct RA8875_instance *thistft,
		int16_t dist) {
  RA8875_writeCommand(thistft, 0x26);
  RA8875_writeData(thistft, dist);
  RA8875_writeCommand(thistft, 0x27);
  RA8875_writeData(thistft, dist >> 8);
}

/************************* Mid Level ***********************************/

/**************************************************************************/
/*!
    Set the Extra General Purpose IO Register

    @param on Whether to turn Extra General Purpose IO on or not

 */
/**************************************************************************/
void RA8875_GPIOX_onoff(struct RA8875_instance *thistft, bool on) {
  if (on) {
	  RA8875_writeReg(thistft, RA8875_GPIOX, 1);
  } else {
	  RA8875_writeReg(thistft, RA8875_GPIOX, 0);
  }
}

/**************************************************************************/
/*!
    Set the duty cycle of the PWM 1 Clock

    @param p The duty Cycle (0-255)
*/
/**************************************************************************/
void RA8875_PWM1out(struct RA8875_instance *thistft, uint8_t p)
{ RA8875_writeReg(thistft, RA8875_P1DCR, p); }

/**************************************************************************/
/*!
     Set the duty cycle of the PWM 2 Clock

     @param p The duty Cycle (0-255)
*/
/**************************************************************************/
void RA8875_PWM2out(struct RA8875_instance *thistft, uint8_t p)
{ RA8875_writeReg(thistft, RA8875_P2DCR, p); }

/**************************************************************************/
/*!
    Configure the PWM 1 Clock

    @param on Whether to enable the clock
    @param clock The Clock Divider
*/
/**************************************************************************/
void RA8875_PWM1config( struct RA8875_instance *thistft,
						bool on, uint8_t clock) {
  if (on) {
    RA8875_writeReg(thistft, RA8875_P1CR, RA8875_P1CR_ENABLE | (clock & 0xF));
  } else {
    RA8875_writeReg(thistft, RA8875_P1CR, RA8875_P1CR_DISABLE | (clock & 0xF));
  }
}

/**************************************************************************/
/*!
     Configure the PWM 2 Clock

     @param on Whether to enable the clock
     @param clock The Clock Divider
*/
/**************************************************************************/
void RA8875_PWM2config(	struct RA8875_instance *thistft,
						bool on, uint8_t clock) {
  if (on) {
    RA8875_writeReg(thistft, RA8875_P2CR, RA8875_P2CR_ENABLE | (clock & 0xF));
  } else {
    RA8875_writeReg(thistft, RA8875_P2CR, RA8875_P2CR_DISABLE | (clock & 0xF));
  }
}

/**************************************************************************/
/*!
      Enables or disables the on-chip touch screen controller

      @param on Whether to turn touch sensing on or not
*/
/**************************************************************************/
void RA8875_touchEnable( struct RA8875_instance *thistft, bool on) {
  uint8_t adcClk = (uint8_t)RA8875_TPCR0_ADCCLK_DIV4;

  if (thistft->size == RA8875_800x480) // match up touch size with LCD size
    adcClk = (uint8_t)RA8875_TPCR0_ADCCLK_DIV16;

  if (on) {
    /* Enable Touch Panel (Reg 0x70) */
    RA8875_writeReg(thistft, RA8875_TPCR0, RA8875_TPCR0_ENABLE | RA8875_TPCR0_WAIT_4096CLK |
                               RA8875_TPCR0_WAKEENABLE | adcClk); // 10mhz max!
    /* Set Auto Mode      (Reg 0x71) */
    RA8875_writeReg(thistft, RA8875_TPCR1, RA8875_TPCR1_AUTO |
                               // RA8875_TPCR1_VREFEXT |
                               RA8875_TPCR1_DEBOUNCE);
    /* Enable TP INT */
    RA8875_writeReg(thistft, RA8875_INTC1, RA8875_readReg(thistft, RA8875_INTC1) | RA8875_INTC1_TP);
  } else {
    /* Disable TP INT */
    RA8875_writeReg(thistft, RA8875_INTC1, RA8875_readReg(thistft, RA8875_INTC1) & ~RA8875_INTC1_TP);
    /* Disable Touch Panel (Reg 0x70) */
    RA8875_writeReg(thistft, RA8875_TPCR0, RA8875_TPCR0_DISABLE);
  }
}

/**************************************************************************/
/*!
      Checks if a touch event has occured

      @return  True is a touch event has occured (reading it via
               touchRead() will clear the interrupt in memory)
*/
/**************************************************************************/
bool RA8875_touched(struct RA8875_instance *thistft) {
  if (RA8875_readReg(thistft, RA8875_INTC2) & RA8875_INTC2_TP)
    return true;
  return false;
}

/**************************************************************************/
/*!
      Reads the last touch event

      @param x  Pointer to the uint16_t field to assign the raw X value
      @param y  Pointer to the uint16_t field to assign the raw Y value

      @return True if successful

      @note Calling this function will clear the touch panel interrupt on
            the RA8875, resetting the flag used by the 'touched' function
*/
/**************************************************************************/
bool RA8875_touchRead(struct RA8875_instance *thistft,
						 uint16_t *x, uint16_t *y) {
  uint16_t tx, ty;
  uint8_t temp;

  tx = RA8875_readReg(thistft, RA8875_TPXH);
  ty = RA8875_readReg(thistft, RA8875_TPYH);
  temp = RA8875_readReg(thistft, RA8875_TPXYL);
  tx <<= 2;
  ty <<= 2;
  tx |= temp & 0x03;        // get the bottom x bits
  ty |= (temp >> 2) & 0x03; // get the bottom y bits

  *x = tx;
  *y = ty;

  /* Clear TP INT Status */
  RA8875_writeReg(thistft, RA8875_INTC2, RA8875_INTC2_TP);

  return true;
}

/**************************************************************************/
/*!
      Turns the display on or off

      @param on Whether to turn the display on or not
*/
/**************************************************************************/
void RA8875_displayOn(struct RA8875_instance *thistft, bool on) {
  if (on)
    RA8875_writeReg(thistft, RA8875_PWRR, RA8875_PWRR_NORMAL | RA8875_PWRR_DISPON);
  else
    RA8875_writeReg(thistft, RA8875_PWRR, RA8875_PWRR_NORMAL | RA8875_PWRR_DISPOFF);
}

/**************************************************************************/
/*!
    Puts the display in sleep mode, or disables sleep mode if enabled

    @param sleep Whether to sleep or not
*/
/**************************************************************************/
void RA8875_sleep(struct RA8875_instance *thistft, bool sleep) {
  if (sleep)
    RA8875_writeReg(thistft, RA8875_PWRR, RA8875_PWRR_DISPOFF | RA8875_PWRR_SLEEP);
  else
    RA8875_writeReg(thistft, RA8875_PWRR, RA8875_PWRR_DISPOFF);
}

/************************* Low Level ***********************************/

/**************************************************************************/
/*!
    Write data to the specified register

    @param reg Register to write to
    @param val Value to write
*/
/**************************************************************************/
void RA8875_writeReg(struct RA8875_instance *thistft, uint8_t reg, uint8_t val) {
  RA8875_writeCommand(thistft, reg);
  RA8875_writeData(thistft, val);
}

/**************************************************************************/
/*!
    Set the register to read from

    @param reg Register to read

    @return The value
*/
/**************************************************************************/
uint8_t RA8875_readReg(struct RA8875_instance *thistft, uint8_t reg) {
  RA8875_writeCommand(thistft, reg);
  return RA8875_readData(thistft);
}

/**************************************************************************/
/*!
    Write data to the current register

    @param d Data to write
*/
/**************************************************************************/
void RA8875_writeData(struct RA8875_instance *thistft, uint8_t d) {
	SPI_CS(thistft->cs_pin, LOW);

	SPI_transfer(RA8875_DATAWRITE);
	SPI_transfer(d);
	SPI_CS(thistft->cs_pin, HIGH);
}

/**************************************************************************/
/*!
    Read the data from the current register

    @return The Value
*/
/**************************************************************************/
uint8_t RA8875_readData(struct RA8875_instance *thistft) {
	SPI_CS(thistft->cs_pin, LOW);
	SPI_transfer(RA8875_DATAREAD);
	uint8_t x = SPI_transfer(0x0);
	SPI_CS(thistft->cs_pin, HIGH);
	return x;
}

/**************************************************************************/
/*!
    Write a command to the current register

    @param d The data to write as a command
 */
/**************************************************************************/
void RA8875_writeCommand(struct RA8875_instance *thistft, uint8_t d) {
	SPI_CS(thistft->cs_pin, LOW);
	SPI_transfer(RA8875_CMDWRITE);
	SPI_transfer(d);
	SPI_CS(thistft->cs_pin, HIGH);
}

/**************************************************************************/
/*!
    Read the status from the current register

    @return The value
 */
/**************************************************************************/
uint8_t RA8875_readStatus(struct RA8875_instance *thistft) {
	SPI_CS(thistft->cs_pin, LOW);
	SPI_transfer(RA8875_CMDREAD);
	uint8_t x = SPI_transfer(0x0);
	SPI_CS(thistft->cs_pin, HIGH);
	return x;
}

void RA8875_swap(int16_t *x, int16_t *y) {
    int16_t temp = *x;
    *x = *y;
    *y = temp;
}


/// @cond DISABLE
#if defined(EEPROM_SUPPORTED)
/// @endcond
/**************************************************************************/
/*!
    Read from the EEPROM location

    @param location The location of the EEPROM to read

    @return The value
*/
/**************************************************************************/

uint32_t RA8875_eepromReadS32(struct RA8875_instance *thistft, int location) {
  uint32_t value = ((uint32_t)EEPROM_read(location)) << 24;
  value = value | ((uint32_t)EEPROM_read(location + 1)) << 16;
  value = value | ((uint32_t)EEPROM_read(location + 2)) << 8;
  value = value | ((uint32_t)EEPROM_read(location + 3));
  return value;
}

/**************************************************************************/
/*!
    Write to the EEPROM location

    @param location The location of the EEPROM to write to
    @param value The value to write
 */
/**************************************************************************/
void RA8875_eepromWriteS32(struct RA8875_instance *thistft, int location, int32_t value) {
  EEPROM_write(location, (value >> 24) & 0xff);
  EEPROM_write(location + 1, (value >> 16) & 0xff);
  EEPROM_write(location + 2, (value >> 8) & 0xff);
  EEPROM_write(location + 3, (value)&0xff);
}

/**************************************************************************/
/*!
     Read Calibration Data from the EEPROM location

     @param location The location of the EEPROM to read from
     @param matrixPtr The pointer to the Matrix Variable

     @return success
 */
/**************************************************************************/
bool RA8875_readCalibration(struct RA8875_instance *thistft, int location, tsMatrix_t *matrixPtr) {
  if (location + sizeof(tsMatrix_t) > EEPROMSIZE) {
    return false; // readCalibration::Calibration location outside of EEPROM
                  // memory bound
  }
  if (EEPROM_read(location + CFG_EEPROM_TOUCHSCREEN_CALIBRATED) == 1) {
    matrixPtr->An = eepromReadS32(location + CFG_EEPROM_TOUCHSCREEN_CAL_AN);
    matrixPtr->Bn = eepromReadS32(location + CFG_EEPROM_TOUCHSCREEN_CAL_BN);
    matrixPtr->Cn = eepromReadS32(location + CFG_EEPROM_TOUCHSCREEN_CAL_CN);
    matrixPtr->Dn = eepromReadS32(location + CFG_EEPROM_TOUCHSCREEN_CAL_DN);
    matrixPtr->En = eepromReadS32(location + CFG_EEPROM_TOUCHSCREEN_CAL_EN);
    matrixPtr->Fn = eepromReadS32(location + CFG_EEPROM_TOUCHSCREEN_CAL_FN);
    matrixPtr->Divider =
        eepromReadS32(location + CFG_EEPROM_TOUCHSCREEN_CAL_DIVIDER);
    return true;
  }
  return false;
}

/**************************************************************************/
/*!
     Write Calibration Data to the EEPROM location

     @param location The location of the EEPROM to write to
     @param matrixPtr The pointer to the Matrix Variable
 */
/**************************************************************************/
void RA8875_writeCalibration(struct RA8875_instance *thistft, int location, tsMatrix_t *matrixPtr) {
  if (location + sizeof(tsMatrix_t) <
      EEPROMSIZE) { // Check to see it calibration location outside of EEPROM
                    // memory bound
    eepromWriteS32(location + CFG_EEPROM_TOUCHSCREEN_CAL_AN, matrixPtr->An);
    eepromWriteS32(location + CFG_EEPROM_TOUCHSCREEN_CAL_BN, matrixPtr->Bn);
    eepromWriteS32(location + CFG_EEPROM_TOUCHSCREEN_CAL_CN, matrixPtr->Cn);
    eepromWriteS32(location + CFG_EEPROM_TOUCHSCREEN_CAL_DN, matrixPtr->Dn);
    eepromWriteS32(location + CFG_EEPROM_TOUCHSCREEN_CAL_EN, matrixPtr->En);
    eepromWriteS32(location + CFG_EEPROM_TOUCHSCREEN_CAL_FN, matrixPtr->Fn);
    eepromWriteS32(location + CFG_EEPROM_TOUCHSCREEN_CAL_DIVIDER,
                   matrixPtr->Divider);
    EEPROM_write(location + CFG_EEPROM_TOUCHSCREEN_CALIBRATED, 1);
  }
}
/// @cond DISABLE
#endif
/// @endcond
