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

#include "PicoGFX_SSD1322.h"
#include "splash.h"
#include <SPI.h>


// CONSTRUCTORS, DESTRUCTOR ------------------------------------------------

/*!
    @brief  Constructor for I2C-interfaced SSD1322 displays.
    @param  w
            Display width in pixels
    @param  h
            Display height in pixels
    @param  twi
            Pointer to an existing TwoWire instance (e.g. &Wire, the
            microcontroller's primary I2C bus).
    @param  rst_pin
            Reset pin (using Arduino pin numbering), or -1 if not used
            (some displays might be wired to share the microcontroller's
            reset pin).
    @param  clkDuring
            Speed (in Hz) for Wire transmissions in PicoGFX_SSD1322 library calls.
            Defaults to 400000 (400 KHz), a known 'safe' value for most
            microcontrollers, and meets the SSD1322 datasheet spec.
            Some systems can operate I2C faster (800 KHz for ESP32, 1 MHz
            for many other 32-bit MCUs), and some (perhaps not all)
            SSD1322's can work with this -- so it's optionally be specified
            here and is not a default behavior. (Ignored if using pre-1.5.7
            Arduino software, which operates I2C at a fixed 100 KHz.)
    @param  clkAfter
            Speed (in Hz) for Wire transmissions following PicoGFX_SSD1322 library
            calls. Defaults to 100000 (100 KHz), the default Arduino Wire
            speed. This is done rather than leaving it at the 'during' speed
            because other devices on the I2C bus might not be compatible
            with the faster rate. (Ignored if using pre-1.5.7 Arduino
            software, which operates I2C at a fixed 100 KHz.)
    @note   Call the object's begin() function before use -- buffer
            allocation is performed there!
*/
PicoGFX_SSD1322 *PicoGFX_SSD1322::currentInstance = nullptr;

PicoGFX_SSD1322::PicoGFX_SSD1322(uint16_t w, uint16_t h, TwoWire *twi,
                                 int8_t rst_pin, uint32_t clkDuring,
                                 uint32_t clkAfter)
    : Adafruit_GrayOLED(4, w, h, twi, rst_pin, clkDuring, clkAfter), spi(nullptr) {
    transferInProgress = false;
    bufferA = nullptr;
    bufferB = nullptr;
    drawBuffer = nullptr;
    displayBuffer = nullptr;
}

/*!
    @brief  Constructor for SPI SSD1322 displays, using software (bitbang)
            SPI.
    @param  w
            Display width in pixels
    @param  h
            Display height in pixels
    @param  mosi_pin
            MOSI (master out, slave in) pin (using Arduino pin numbering).
            This transfers serial data from microcontroller to display.
    @param  sclk_pin
            SCLK (serial clock) pin (using Arduino pin numbering).
            This clocks each bit from MOSI.
    @param  dc_pin
            Data/command pin (using Arduino pin numbering), selects whether
            display is receiving commands (low) or data (high).
    @param  rst_pin
            Reset pin (using Arduino pin numbering), or -1 if not used
            (some displays might be wired to share the microcontroller's
            reset pin).
    @param  cs_pin
            Chip-select pin (using Arduino pin numbering) for sharing the
            bus with other devices. Active low.
    @note   Call the object's begin() function before use -- buffer
            allocation is performed there!
*/
PicoGFX_SSD1322::PicoGFX_SSD1322(uint16_t w, uint16_t h, int8_t mosi_pin,
                                 int8_t sclk_pin, int8_t dc_pin,
                                 int8_t rst_pin, int8_t cs_pin)
    : Adafruit_GrayOLED(4, w, h, mosi_pin, sclk_pin, dc_pin, rst_pin, cs_pin), spi(nullptr) {
    transferInProgress = false;
    bufferA = nullptr;
    bufferB = nullptr;
    drawBuffer = nullptr;
    displayBuffer = nullptr;
}

/*!
    @brief  Constructor for SPI SSD1322 displays, using native hardware SPI.
    @param  w
            Display width in pixels
    @param  h
            Display height in pixels
    @param  spi
            Pointer to an existing SPIClass instance (e.g. &SPI, the
            microcontroller's primary SPI bus).
    @param  dc_pin
            Data/command pin (using Arduino pin numbering), selects whether
            display is receiving commands (low) or data (high).
    @param  rst_pin
            Reset pin (using Arduino pin numbering), or -1 if not used
            (some displays might be wired to share the microcontroller's
            reset pin).
    @param  cs_pin
            Chip-select pin (using Arduino pin numbering) for sharing the
            bus with other devices. Active low.
    @param  bitrate
            SPI clock rate for transfers to this display. Default if
            unspecified is 8000000UL (8 MHz).
    @note   Call the object's begin() function before use -- buffer
            allocation is performed there!
*/
PicoGFX_SSD1322::PicoGFX_SSD1322(uint16_t w, uint16_t h, SPIClass *spi,
                                 int8_t dc_pin, int8_t rst_pin, int8_t cs_pin,
                                 uint32_t bitrate)
    : Adafruit_GrayOLED(4, w, h, spi, dc_pin, rst_pin, cs_pin, bitrate), spi(spi) {
    transferInProgress = false;
    bufferA = nullptr;
    bufferB = nullptr;
    drawBuffer = nullptr;
    displayBuffer = nullptr;
}

/*!
    @brief  Destructor for PicoGFX_SSD1322 object.
*/
PicoGFX_SSD1322::~PicoGFX_SSD1322(void) {
    delete[] bufferA;
    delete[] bufferB;
    bufferA = bufferB = nullptr;
}

// ALLOCATE & INIT DISPLAY -------------------------------------------------

/*!
    @brief  Allocate RAM for image buffer, initialize peripherals and pins.
    @param  addr
            I2C address of corresponding SSD1322 display.
            SPI displays (hardware or software) do not use addresses, but
            this argument is still required (pass 0 or any value really,
            it will simply be ignored). Default if unspecified is 0.
    @param  reset
            If true, and if the reset pin passed to the constructor is
            valid, a hard reset will be performed before initializing the
            display. If using multiple SSD1322 displays on the same bus, and
            if they all share the same reset pin, you should only pass true
            on the first display being initialized, false on all others,
            else the already-initialized displays would be reset. Default if
            unspecified is true.
    @return true on successful allocation/init, false otherwise.
            Well-behaved code should check the return value before
            proceeding.
    @note   MUST call this function before any drawing or updates!
*/
bool PicoGFX_SSD1322::begin(uint8_t addr, bool reset) {
  if (!Adafruit_GrayOLED::_init(addr, reset)) {
    return false;
  }

  // FREE the buffer that the base class allocated – we don't want it
  delete[] buffer;
  buffer = nullptr;  // Prevent base class from deleting it later

  uint16_t buffer_size = WIDTH * HEIGHT / 2;  // 4 bits per pixel

  bufferA = new uint8_t[buffer_size];
  bufferB = new uint8_t[buffer_size];

  if (!bufferA || !bufferB) {
    delete[] bufferA;
    delete[] bufferB;
    bufferA = bufferB = nullptr;
    return false;
  }

  memset(bufferA, SSD1322_BLACK, buffer_size);
  memset(bufferB, SSD1322_BLACK, buffer_size);

  drawBuffer = bufferA;     // Start drawing into bufferA
  displayBuffer = bufferB;  // First display() will send bufferB (initially black – safe)

  // IMPORTANT: Make base class use our draw buffer
  this->buffer = drawBuffer;

  // Init Sequence
  oled_command(SSD1322_CMDLOCK);            // 0xFD
  oled_data(0x12);                          // 0x12 (Unlock OLED driver IC MCU interface)
  oled_command(SSD1322_DISPLAYOFF);         // 0xAE
  oled_command(SSD1322_DISPLAYCLK);         // 0xB3 (Set Oscillator Freq. & Display Clock Divider)
  oled_data(0x91);                          // 0x91 (Divide by 2, ~80 Frames/sec)
  oled_command(SSD1322_SETMULTIPLEX);       // 0xCA
  oled_data(0x3F);                          // 0x3F (1/64)
  oled_command(SSD1322_SETDISPLAYOFFSET);   // 0xA2
  oled_data(0x00);                          // 0x00 (No Offset)
  oled_command(SSD1322_SETSTARTLINE);       // 0xA1
  oled_data(0x00);                          // 0x00 (Start Line 0)
  oled_command(SSD1322_SEGREMAP);           // 0xA0
  oled_data(0x14);                          // 0x14 (Horizontal address increment, Disable Column Address Re-map, Enable Nibble Re-map)
  oled_data(0x11);                          // 0x11 (Enable Dual COM mode)
  oled_command(SSD1322_SETGPIO);            // 0xB5
  oled_data(0x00);                          // 0x00 (Input disabled)
  oled_command(SSD1322_FUNCSEL);            // 0xAB
  oled_data(0x01);                          // 0x01 (Enable internal VDD regulator)
  oled_command(SSD1322_DISPLAYENHA);        // 0xB4
  oled_data(0xA0);                          // 0xA0 (Enable External VSL)
  oled_data(0xFD);                          // 0xFD (Enhanced low GS display quality)
  oled_command(SSD1322_SETCONTRAST);        // 0xC1
  oled_data(0x05);                          // 0x05 (5)
  oled_command(SSD1322_MASTERCONTRAST);     // 0xC7
  oled_data(0x0F);                          // 0x0F (No change)
  oled_command(SSD1322_PHASELEN);           // 0xB1
  oled_data(0xE2);                          // 0xE2 (P1:5DCLKs,P2:14DCLKs)
  oled_command(SSD1322_DISPLAYENHB);        // 0xD1
  oled_data(0xA2);                          // 0xA2 (Normal)
  oled_data(0x20);                          // 0x20
  oled_command(SSD1322_PRECHARGE);          // 0xBB
  oled_data(0x1F);                          // 0x1F (0.6xVCC)
  oled_command(SSD1322_PRECHARGE2);         // 0xB6
  oled_data(0x08);                          // 0x08 (8 dclks)
  oled_command(SSD1322_SETVCOM);            // 0xBE
  oled_data(0x07);                          // 0x07 (0.86xVCC)
  oled_command(SSD1322_NORMALDISPLAY);      // 0xA6
  oled_command(SSD1322_EXITPARTDISPLAY);    // 0xA9
  oled_command(SSD1322_DISPLAYON);          // 0xAF

  delay(100);                               // 100ms delay recommended
  return true; // Success
}

/*!
    @brief  Do the actual writing of the internal frame buffer to display RAM using DMA for SPI
*/
void PicoGFX_SSD1322::display(void) {
  yield();

  // Calculate dirty region
  int16_t first_row = max(0, window_y1);
  int16_t last_row = min(HEIGHT - 1, window_y2);
  int16_t byte_start = window_x1 / 2;  // Integer division for byte index
  int16_t byte_end = window_x2 / 2;    // Integer division for byte index
  uint8_t column_start = 28 + (byte_start / 2);  // Map byte index to column
  uint8_t column_end = 28 + (byte_end / 2);      // Map byte index to column
  transfer_byte_start = (column_start - 28) * 2; // Starting byte offset
  transfer_bytes_per_row = (column_end - column_start + 1) * 2; // Bytes per row

  // Swap: what we were drawing into becomes the one to display
  uint8_t* temp = drawBuffer;
  drawBuffer = displayBuffer;
  displayBuffer = temp;

  // Update base class pointer so drawing calls go to the new draw buffer
  this->buffer = drawBuffer;

  if (i2c_dev) { // I2C case
    i2c_dev->setSpeed(i2c_preclk);
    uint8_t maxbuff = i2c_dev->maxBufferSize() - 1;

    oled_command(SSD1322_SETROW);       // 0x75
    oled_data(first_row);               // Row start
    oled_data(last_row);                // Row end
    oled_command(SSD1322_SETCOLUMN);    // 0x15
    oled_data(column_start);            // Column start
    oled_data(column_end);              // Column end
    oled_command(SSD1322_ENWRITEDATA);  // 0x5C

    uint8_t *ptr = displayBuffer + first_row * (WIDTH / 2) + transfer_byte_start;
    uint8_t dc_byte = 0x40;

    for (uint8_t row = first_row; row <= last_row; row++) {
      uint8_t bytes_remaining = transfer_bytes_per_row;
      while (bytes_remaining) {
        uint8_t to_write = min(bytes_remaining, maxbuff);
        i2c_dev->write(ptr, to_write, true, &dc_byte, 1);
        ptr += to_write;
        bytes_remaining -= to_write;
        yield();
      }
      ptr += (WIDTH / 2) - transfer_bytes_per_row; // Move to next row
    }
    i2c_dev->setSpeed(i2c_postclk);

    // Reset dirty region for I2C (blocking)
    window_x1 = WIDTH;
    window_y1 = HEIGHT;
    window_x2 = -1;
    window_y2 = -1;

    // Call callback if set
    if (completeCallback) {
      completeCallback();
    }
  } else { // SPI with DMA
    // Prevent starting a new transfer if one is already in progress
    if (transferInProgress) {
      return; // Ignore new transfer request
    }

    // Send commands (blocking) using existing methods
    oled_command(SSD1322_SETROW);       // 0x75
    oled_data(first_row);               // Row start
    oled_data(last_row);                // Row end
    oled_command(SSD1322_SETCOLUMN);    // 0x15
    oled_data(column_start);            // Column start
    oled_data(column_end);              // Column end
    oled_command(SSD1322_ENWRITEDATA);  // 0x5C

    // Set state variables for DMA transfers
    current_row = first_row;
    this->last_row = last_row;
    this->bytes_per_row = WIDTH / 2;

    // Calculate pointer for the first row of the display buffer
    uint8_t *ptr = displayBuffer + first_row * bytes_per_row + transfer_byte_start;

    // Manually assert CS# low and set DC# high for data mode
    digitalWrite(csPin, LOW);
    digitalWrite(dcPin, HIGH);

    // Start the first async DMA transfer
    currentInstance = this;
    static_cast<SPIClassRP2040*>(this->spi)->transferAsync(ptr, nullptr, transfer_bytes_per_row);
    transferInProgress = true;
  }
}

/*!
    @brief  Check if the current async transfer is complete and handle it
    @return true if a transfer was completed, false if still in progress
*/
bool PicoGFX_SSD1322::checkTransferComplete() {
  if (!transferInProgress || !static_cast<SPIClassRP2040*>(this->spi)->finishedAsync()) {
    return false;
  }

  current_row++;
  if (current_row <= last_row) {
    // Calculate buffer pointer for the next row using displayBuffer
    uint8_t *ptr = displayBuffer + current_row * bytes_per_row + transfer_byte_start;
    // Initiate the next async DMA transfer
    static_cast<SPIClassRP2040*>(this->spi)->transferAsync(ptr, nullptr, transfer_bytes_per_row);
    return false;
  } else {
    // All rows have been sent; deassert CS# high
    digitalWrite(csPin, HIGH);

    // Reset dirty rectangle ONLY when transfer fully complete
    window_x1 = WIDTH;
    window_y1 = HEIGHT;
    window_x2 = -1;
    window_y2 = -1;

    transferInProgress = false;
    currentInstance = nullptr;

    if (completeCallback) {
      completeCallback();
    }
    return true;
  }
}

/*!
    @brief  Public method to check if the async transfer is complete
    @return true if no transfer is in progress or all rows are sent, false otherwise
*/
bool PicoGFX_SSD1322::isTransferComplete() {
  if (!transferInProgress) {
    return true;
  }
  return checkTransferComplete();
}

/*!
    @brief  Handle completion of a DMA transfer, start next transfer or finish
*/
void PicoGFX_SSD1322::handleTransferComplete() {
  checkTransferComplete();
}

/*!
    @brief  Blocking version of display() – waits until transfer is fully complete
*/
void PicoGFX_SSD1322::displayBlocking() {
  display();  // Start non-blocking transfer
  while (!isTransferComplete()) {
    yield();  // Allow other tasks to run
  }
}

/*!
    @brief Issue single data byte to OLED, using I2C or hard/soft SPI as needed.
    @param c The single byte data
*/
void PicoGFX_SSD1322::oled_data(uint8_t c) {
  if (i2c_dev) {                // I2C
    uint8_t buf[2] = {0x00, c}; // Co = 0, D/C = 0
    i2c_dev->write(buf, 2);
  } else { // SPI (hw or soft) -- transaction started in calling function
    digitalWrite(dcPin, HIGH);
    spi_dev->write(&c, 1);
  }
}

/*!
    @brief  Enable or disable display invert mode (white-on-black vs
            black-on-white). Handy for testing!
    @param  i
            If true, switch to invert mode (black-on-white), else normal
            mode (white-on-black).
*/
void PicoGFX_SSD1322::invertDisplay(bool i) {
  oled_command(i ? SSD1322_INVERTDISPLAY : SSD1322_NORMALDISPLAY);
}

/*!
    @brief  Power Display off
*/
void PicoGFX_SSD1322::displayOff(void) {
  oled_command(SSD1322_DISPLAYOFF);
}

/*!
    @brief  Power Display on
*/
void PicoGFX_SSD1322::displayOn(void) {
  oled_command(SSD1322_DISPLAYON);
}

/*!
    @brief  Set all Pixel full off (GS=0)
*/
void PicoGFX_SSD1322::allPixelOff(void) {
  oled_command(SSD1322_DISPLAYALLOFF);
}

/*!
    @brief  Set all Pixel full on (GS=15)
*/
void PicoGFX_SSD1322::allPixelOn(void) {
  oled_command(SSD1322_DISPLAYALLON);
}

/*!
    @brief  Adjust the display contrast.
    @param  level The contrast level from 0 to 0xFF
    @note   This has an immediate effect on the display, no need to call the
            display() function -- buffer contents are not changed.
*/
void PicoGFX_SSD1322::setContrast(uint8_t level) {
  oled_command(SSD1322_SETCONTRAST);
  oled_data(level);
}

/*!
    @brief  Returns the (Pointer to the) actual Font
    @param  None
*/
GFXfont * PicoGFX_SSD1322::getFont(void) {
  return gfxFont;
}

/*!
    @brief  Draw a PROGMEM-resident 4bpp bitmap to the internal buffer
    @param  x       X-coordinate of top-left corner on display
    @param  y       Y-coordinate of top-left corner on display
    @param  bitmap  Pointer to PROGMEM bitmap array (4 bits per pixel, two pixels per byte)
    @param  w       Bitmap width in pixels
    @param  h       Bitmap height in pixels
    @note   Updates internal buffer; call display() to show on OLED
*/
void PicoGFX_SSD1322::draw4bppBitmap(int16_t x, int16_t y, const uint8_t *bitmap, uint16_t w, uint16_t h) {
    if (w <= 0 || h <= 0) return;

    // Update dirty rect
    window_x1 = min(window_x1, x);
    window_y1 = min(window_y1, y);
    window_x2 = max(window_x2, x + w - 1);
    window_y2 = max(window_y2, y + h - 1);

    int16_t x2 = x + w - 1;
    int16_t y2 = y + h - 1;
    if (x2 < 0 || x >= WIDTH || y2 < 0 || y >= HEIGHT) return;

    for (int16_t jy = 0; jy < h; jy++) {
        int16_t py = y + jy;
        if (py < 0 || py >= HEIGHT) continue;

        for (int16_t ix = 0; ix < w; ix += 2) {  // Process 2 pixels per byte
            uint16_t byteIndex = (jy * w + ix) / 2;
            uint8_t data = pgm_read_byte(&bitmap[byteIndex]);

            uint8_t pixel1 = (data >> 4) & 0x0F;
            uint8_t pixel2 = data & 0x0F;

            int16_t px1 = x + ix;
            int16_t px2 = x + ix + 1;

            if (px1 >= 0 && px1 < WIDTH) {
                drawPixel(px1, py, pixel1);
            }
            if (ix + 1 < w && px2 >= 0 && px2 < WIDTH) {
                drawPixel(px2, py, pixel2);
            }
        }
    }
}

/*!
    @brief  Draw a RAM-resident 4bpp bitmap to the internal buffer
    @param  x       X-coordinate of top-left corner on display
    @param  y       Y-coordinate of top-left corner on display
    @param  bitmap  Pointer to RAM bitmap array (4 bits per pixel, two pixels per byte)
    @param  w       Bitmap width in pixels
    @param  h       Bitmap height in pixels
    @note   Updates internal buffer; call display() to show on OLED
*/
void PicoGFX_SSD1322::draw4bppBitmap(int16_t x, int16_t y, uint8_t *bitmap, uint16_t w, uint16_t h) {
    if (w <= 0 || h <= 0) return;

    // Update dirty region
    window_x1 = min(window_x1, x);
    window_y1 = min(window_y1, y);
    window_x2 = max(window_x2, x + w - 1);
    window_y2 = max(window_y2, y + h - 1);

    int16_t x2 = x + w - 1;
    int16_t y2 = y + h - 1;
    if (x2 < 0 || x >= WIDTH || y2 < 0 || y >= HEIGHT) return;

    for (int16_t jy = 0; jy < h; jy++) {
        int16_t py = y + jy;
        if (py < 0 || py >= HEIGHT) continue;

        for (int16_t ix = 0; ix < w; ix += 2) {  // Process 2 pixels per byte
            uint16_t byteIndex = (jy * w + ix) / 2;
            uint8_t data = bitmap[byteIndex];

            uint8_t pixel1 = (data >> 4) & 0x0F;
            uint8_t pixel2 = data & 0x0F;

            int16_t px1 = x + ix;
            int16_t px2 = x + ix + 1;

            if (px1 >= 0 && px1 < WIDTH) {
                drawPixel(px1, py, pixel1);
            }
            if (ix + 1 < w && px2 >= 0 && px2 < WIDTH) {
                drawPixel(px2, py, pixel2);
            }
        }
    }
}