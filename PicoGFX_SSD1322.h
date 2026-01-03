/*********************************************************************
PicoGFX_SSD1322

This is a library for the Grayscale SSD1322 Controller based OLED
License: BSD

Originally forked in 09/2021 by venice1200 from the Adafruit SSD1327 Library 
and modified for an SSD1322 OLED with 256x64 pixels.
Other OLED resolutions and interfaces (I2C, 3SPI, 6800, 80xx) are currently (2021-09-14) not tested.

Modified in 2025 by The Happy Grey to support non-blocking DMA SPI transfers on Raspberry Pi Pico
using the Arduino-Pico core by Earl Philhower. Renamed to PicoGFX_SSD1322 to reflect
optimizations for the Raspberry Pi Pico.

-----------------------------------------------------------
Original Adafruit Header for the SSD1327 OLED (BSD License)
-----------------------------------------------------------

This is a library for our Grayscale OLEDs based on SSD1327 drivers
  Pick one up today in the Adafruit shop!
  ------> https://www.adafruit.com/products/4741

These displays use I2C or SPI to communicate

Adafruit invests time and resources providing this open source code,
please support Adafruit and open-source hardware by purchasing
products from Adafruit!

Original SSD1327 Library written by Limor Fried/Ladyada for Adafruit Industries.
BSD license, check license.txt for more information
All text above, and the splash screen must be included in any redistribution

You will find the original Library here https://github.com/adafruit/Adafruit_SSD1327
*********************************************************************/

#ifndef PICOGFX_SSD1322_H
#define PICOGFX_SSD1322_H

#include <Adafruit_GrayOLED.h>

#define SSD1322_BLACK 0x0
#define SSD1322_WHITE 0xF
#define SSD1322_I2C_ADDRESS 0x3D

#define SSD1322_SETCOLUMN 0x15
#define SSD1322_ENWRITEDATA 0x5C               // Enable Write Data
#define SSD1322_SETROW 0x75
#define SSD1322_SEGREMAP 0xA0
#define SSD1322_SETSTARTLINE 0xA1
#define SSD1322_SETDISPLAYOFFSET 0xA2
#define SSD1322_DISPLAYALLOFF 0xA4              // All Pixel OFF in GS level 0
#define SSD1322_DISPLAYALLON 0xA5               // All Pixel ON in GS level 15
#define SSD1322_NORMALDISPLAY 0xA6
#define SSD1322_INVERTDISPLAY 0xA7
#define SSD1322_ENPARTDISPLAY 0xA8
#define SSD1322_EXITPARTDISPLAY 0xA9
#define SSD1322_SETMULTIPLEX 0xCA
#define SSD1322_FUNCSEL 0xAB
#define SSD1322_DISPLAYOFF 0xAE
#define SSD1322_DISPLAYON 0xAF
#define SSD1322_PHASELEN 0xB1
#define SSD1322_DISPLAYCLK 0xB3
#define SSD1322_DISPLAYENHA 0xB4
#define SSD1322_SETGPIO 0xB5
#define SSD1322_PRECHARGE2 0xB6
#define SSD1322_GRAYTABLE 0xB8
#define SSD1322_PRECHARGE 0xBB
#define SSD1322_SETVCOM 0xBE
#define SSD1322_SETCONTRAST 0xC1
#define SSD1322_MASTERCONTRAST 0xC7
#define SSD1322_DISPLAYENHB 0xD1
#define SSD1322_FUNCSELB 0xD5
#define SSD1322_CMDLOCK 0xFD
//#define SSD1322_SETBRIGHTNESS 0x82
//#define SSD1322_SETLUT 0x91

/*! The controller object for SSD1322 OLED displays */
class PicoGFX_SSD1322 : public Adafruit_GrayOLED {
public:
  PicoGFX_SSD1322(uint16_t w, uint16_t h, TwoWire *twi = &Wire,
                  int8_t rst_pin = -1, uint32_t preclk = 400000,
                  uint32_t postclk = 100000);
  PicoGFX_SSD1322(uint16_t w, uint16_t h, int8_t mosi_pin, int8_t sclk_pin,
                  int8_t dc_pin, int8_t rst_pin, int8_t cs_pin);
  PicoGFX_SSD1322(uint16_t w, uint16_t h, SPIClass *spi, int8_t dc_pin,
                  int8_t rst_pin, int8_t cs_pin, uint32_t bitrate = 8000000UL);
  ~PicoGFX_SSD1322(void);

  bool begin(uint8_t i2caddr = SSD1322_I2C_ADDRESS, bool reset = true);
  void oled_data(uint8_t c);
  void display();                   // Non-blocking (async for SPI)
  void displayBlocking();           // Blocking version (waits until complete)
  void invertDisplay(bool i);
  void displayOff();
  void displayOn();
  void allPixelOff();
  void allPixelOn();
  void setContrast(uint8_t level);
  void draw4bppBitmap(int16_t x, int16_t y, const uint8_t *bitmap, uint16_t w, uint16_t h);
  void draw4bppBitmap(int16_t x, int16_t y, uint8_t *bitmap, uint16_t w, uint16_t h);
  GFXfont * getFont();
  bool checkTransferComplete();     // Check if async transfer is done
  bool isTransferComplete();        // Public method to check transfer completion

  using DisplayCompleteCallback = void (*)(void);
  void setDisplayCompleteCallback(DisplayCompleteCallback cb) { completeCallback = cb; }

protected:
  SPIClass *spi = nullptr; // Pointer to SPI interface
  int8_t page_offset = 0;
  int8_t column_offset = 0;
  int16_t current_row;
  int16_t last_row;
  int16_t row_start;
  int16_t row_end;
  uint8_t bytes_per_row;
  bool transferInProgress;
  static PicoGFX_SSD1322 *currentInstance;
  void handleTransferComplete();
  uint8_t* bufferA = nullptr;
  uint8_t* bufferB = nullptr;
  uint8_t* drawBuffer = nullptr;    // Points to the buffer we're drawing into (replaces base 'buffer')
  uint8_t* displayBuffer = nullptr; // Points to the buffer currently being sent/displayed
  int16_t transfer_byte_start;      // Starting byte offset for dirty region transfers
  uint8_t transfer_bytes_per_row;   // Bytes to transfer per row in dirty region

  DisplayCompleteCallback completeCallback = nullptr;
};

#endif