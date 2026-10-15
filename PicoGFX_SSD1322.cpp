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
#include <new>
#include <stdlib.h>
#include <string.h>


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
    bufferA = bufferB = drawBuffer = displayBuffer = nullptr;
    // The base class frees `buffer` with free(). It points at one of the two above, which have
    // just gone and were never malloc'd in the first place, so leave nothing for it to find.
    this->buffer = nullptr;
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

  // Release the buffer the base class allocated - we keep our own pair instead. It came from
  // malloc() in Adafruit_GrayOLED::_init(), so it has to go back with free(); delete[] on it is
  // undefined behaviour even where it happens to work.
  free(buffer);
  buffer = nullptr;

  const size_t buffer_size = (size_t)WIDTH * HEIGHT / 2;  // 4 bits per pixel

  bufferA = new (std::nothrow) uint8_t[buffer_size];
  bufferB = new (std::nothrow) uint8_t[buffer_size];

  if (!bufferA || !bufferB) {
    delete[] bufferA;
    delete[] bufferB;
    bufferA = bufferB = nullptr;
    return false;
  }

  memset(bufferA, SSD1322_BLACK, buffer_size);
  memset(bufferB, SSD1322_BLACK, buffer_size);

  drawBuffer = bufferA;     // Start drawing into bufferA
  displayBuffer = bufferB;  // ... and bufferB is what the swap in display() will send first

  transferInProgress = false;
  transfer_rows_per_chunk = 1;
  bytes_per_row = WIDTH / 2;

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
bool PicoGFX_SSD1322::displayAsync(void) {
  yield();

  // A transfer already running owns displayBuffer. Swapping the buffers under it would hand the
  // caller the one being sent to draw into, and send the one they were drawing - so this test
  // has to come before anything is touched, not after the swap.
  if (transferInProgress) {
    checkTransferComplete();
    if (transferInProgress) return false;
  }

  // Clamp the dirty region into the panel, and take an empty one for what it is: nothing was
  // drawn since the last update, so there is nothing to send. Clamping the other way - letting
  // window_y1 of HEIGHT through as a first row of HEIGHT - produced an inverted row range, a
  // byte count that underflowed to 130, and a transfer that read past the end of the buffer.
  if (window_x2 < window_x1 || window_y2 < window_y1) return false;
  int16_t first_row = window_y1 < 0 ? 0 : window_y1;
  int16_t last_row = window_y2 > (int16_t)(HEIGHT - 1) ? (int16_t)(HEIGHT - 1) : window_y2;
  int16_t first_px = window_x1 < 0 ? 0 : window_x1;
  int16_t last_px = window_x2 > (int16_t)(WIDTH - 1) ? (int16_t)(WIDTH - 1) : window_x2;
  if (last_row < first_row || last_px < first_px) return false;

  // The controller addresses four pixels (two bytes) at a time, so widen to those boundaries.
  const uint8_t column_start = (uint8_t)(28 + (first_px / 4));
  const uint8_t column_end = (uint8_t)(28 + (last_px / 4));
  transfer_byte_start = (int16_t)((column_start - 28) * 2);
  transfer_bytes_per_row = (uint8_t)((column_end - column_start + 1) * 2);
  this->bytes_per_row = WIDTH / 2;
  current_row = first_row;
  this->last_row = last_row;

  // Swap: what we were drawing into becomes the one to display.
  uint8_t* temp = drawBuffer;
  drawBuffer = displayBuffer;
  displayBuffer = temp;
  this->buffer = drawBuffer;

  // The region is captured now, so the next frame starts with a clean window. Clearing it when
  // the transfer finishes instead would throw away whatever the caller drew in the meantime -
  // which is precisely what double buffering is for.
  window_x1 = WIDTH;
  window_y1 = HEIGHT;
  window_x2 = -1;
  window_y2 = -1;

  openWindow(column_start, column_end, first_row, last_row);

  // I2C and software SPI have no DMA path; both send and return done.
  if (i2c_dev || !spi) {
    sendBlocking(first_row, last_row);
    syncDrawBuffer();
    if (completeCallback) completeCallback();
    return true;
  }

  // Where the region spans the full width its rows are one unbroken run of bytes, so the whole
  // update goes in a single DMA instead of one per row.
  transfer_rows_per_chunk =
      (transfer_bytes_per_row == bytes_per_row) ? (uint8_t)(last_row - first_row + 1) : 1;

  uint8_t *ptr = displayBuffer + (size_t)first_row * bytes_per_row + transfer_byte_start;

  // CS stays low for the whole multi-row transfer; DC high selects data.
  digitalWrite(csPin, LOW);
  digitalWrite(dcPin, HIGH);

  currentInstance = this;
  const size_t bytes = (size_t)transfer_bytes_per_row * transfer_rows_per_chunk;
  if (!static_cast<SPIClassRP2040*>(this->spi)->transferAsync(ptr, nullptr, bytes)) {
    // No DMA channel was free. Say so rather than leaving CS low and a transfer that never ran
    // looking, to finishedAsync(), exactly like one that completed.
    digitalWrite(csPin, HIGH);
    currentInstance = nullptr;
    return false;
  }
  transferInProgress = true;

  // Bring the buffer we are about to draw into up to date with the one now on its way to the
  // panel. Without this it still holds the frame before that, so anything drawn next that does
  // not cover the whole dirty rectangle lets two-frames-old pixels through. Both buffers are
  // only read here, so it is safe to do while the DMA streams.
  syncDrawBuffer();
  return true;
}

/*!
    @brief  Send everything drawn since the last update. Adafruit_GrayOLED requires this to return
            nothing; displayAsync() is the same call with its result.
*/
void PicoGFX_SSD1322::display(void) { (void)displayAsync(); }

/*!
    @brief  Point the controller's write window at the dirty region and open it for data.
*/
void PicoGFX_SSD1322::openWindow(uint8_t column_start, uint8_t column_end, int16_t first_row,
                                 int16_t last_row) {
  if (i2c_dev) i2c_dev->setSpeed(i2c_preclk);
  oled_command(SSD1322_SETROW);       // 0x75
  oled_data((uint8_t)first_row);
  oled_data((uint8_t)last_row);
  oled_command(SSD1322_SETCOLUMN);    // 0x15
  oled_data(column_start);
  oled_data(column_end);
  oled_command(SSD1322_ENWRITEDATA);  // 0x5C
}

/*!
    @brief  Send the dirty region and wait for it. Used by I2C and by software SPI.
*/
void PicoGFX_SSD1322::sendBlocking(int16_t first_row, int16_t last_row) {
  const uint8_t maxbuff = i2c_dev ? (uint8_t)(i2c_dev->maxBufferSize() - 1) : transfer_bytes_per_row;
  uint8_t dc_byte = 0x40;

  for (int16_t row = first_row; row <= last_row; row++) {
    uint8_t *ptr = displayBuffer + (size_t)row * bytes_per_row + transfer_byte_start;
    uint8_t bytes_remaining = transfer_bytes_per_row;
    while (bytes_remaining) {
      const uint8_t to_write = min(bytes_remaining, maxbuff);
      if (i2c_dev) {
        i2c_dev->write(ptr, to_write, true, &dc_byte, 1);
      } else {
        digitalWrite(dcPin, HIGH);
        spi_dev->write(ptr, to_write);
      }
      ptr += to_write;
      bytes_remaining -= to_write;
      yield();
    }
  }
  if (i2c_dev) i2c_dev->setSpeed(i2c_postclk);
}

/*!
    @brief  Copy the frame just sent into the buffer the caller will draw next.
*/
void PicoGFX_SSD1322::syncDrawBuffer(void) {
  if (drawBuffer && displayBuffer) memcpy(drawBuffer, displayBuffer, (size_t)WIDTH * HEIGHT / 2);
}

/*!
    @brief  Check if the current async transfer is complete and handle it
    @return true if a transfer was completed, false if still in progress
*/
bool PicoGFX_SSD1322::checkTransferComplete() {
  if (!transferInProgress || !spi) return false;
  if (!static_cast<SPIClassRP2040*>(this->spi)->finishedAsync()) return false;

  current_row += transfer_rows_per_chunk;
  if (current_row <= last_row) {
    uint8_t *ptr = displayBuffer + (size_t)current_row * bytes_per_row + transfer_byte_start;
    const size_t bytes = (size_t)transfer_bytes_per_row * transfer_rows_per_chunk;
    if (static_cast<SPIClassRP2040*>(this->spi)->transferAsync(ptr, nullptr, bytes)) return false;
    // Out of DMA channels part way through. Finish here rather than spin: the rest of the frame
    // is missing, but the bus is released and the next display() can draw it again.
  }

  // Done (or given up): release the bus.
  digitalWrite(csPin, HIGH);
  transferInProgress = false;
  currentInstance = nullptr;

  // The dirty window is NOT cleared here. display() cleared it when it captured this region,
  // so anything drawn while the transfer was in flight is still recorded and will be sent next.
  if (completeCallback) completeCallback();
  return true;
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
  displayAsync();  // Start non-blocking transfer
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
    @brief  Draw a 4bpp bitmap into the frame buffer.
    @param  x        X-coordinate of top-left corner on display
    @param  y        Y-coordinate of top-left corner on display
    @param  bitmap   4 bits per pixel, two pixels per byte, high nibble is the left pixel
    @param  w        Bitmap width in pixels
    @param  h        Bitmap height in pixels
    @param  progmem  True to read the bitmap through pgm_read_byte
    @note   Updates the frame buffer; call display() to show it.

    Where the bitmap lands on a byte boundary and the panel is unrotated, its bytes already match
    the buffer's layout two pixels at a time, so whole rows are copied straight in. That is the
    common case for art produced by a 4bpp converter, and it replaces two bounds-checked virtual
    drawPixel() calls per byte with one memcpy per row. Anything else - an odd x, an odd width, a
    rotated panel, a bitmap hanging off the edge - takes the per-pixel path, which is unchanged.
*/
void PicoGFX_SSD1322::drawBitmap4bpp(int16_t x, int16_t y, const uint8_t *bitmap, uint16_t w,
                                     uint16_t h, bool progmem) {
    if (!bitmap || w == 0 || h == 0 || !buffer) return;

    const int16_t x2 = (int16_t)(x + w - 1);
    const int16_t y2 = (int16_t)(y + h - 1);
    if (x2 < 0 || x >= (int16_t)WIDTH || y2 < 0 || y >= (int16_t)HEIGHT) return;  // fully off-panel

    // Mark only the part that actually lands on the panel. Marking the whole bitmap, as this used
    // to, lets a sprite that is mostly off-screen drag the dirty rectangle out past the edge and
    // cost a far larger transfer than it needs to.
    const int16_t cx1 = x < 0 ? 0 : x;
    const int16_t cy1 = y < 0 ? 0 : y;
    const int16_t cx2 = x2 > (int16_t)(WIDTH - 1) ? (int16_t)(WIDTH - 1) : x2;
    const int16_t cy2 = y2 > (int16_t)(HEIGHT - 1) ? (int16_t)(HEIGHT - 1) : y2;
    window_x1 = min(window_x1, cx1);
    window_y1 = min(window_y1, cy1);
    window_x2 = max(window_x2, cx2);
    window_y2 = max(window_y2, cy2);

    const bool aligned = (getRotation() == 0) && ((x & 1) == 0) && ((w & 1) == 0) &&
                         (x >= 0) && (x2 < (int16_t)WIDTH);
    const uint16_t srcStride = (uint16_t)(w / 2);

    if (aligned) {
        for (uint16_t jy = 0; jy < h; jy++) {
            const int16_t py = (int16_t)(y + jy);
            if (py < 0 || py >= (int16_t)HEIGHT) continue;
            uint8_t *dst = buffer + (size_t)py * (WIDTH / 2) + (x / 2);
            const uint8_t *src = bitmap + (size_t)jy * srcStride;
            if (progmem) {
                memcpy_P(dst, src, srcStride);
            } else {
                memcpy(dst, src, srcStride);
            }
        }
        return;
    }

    for (uint16_t jy = 0; jy < h; jy++) {
        const int16_t py = (int16_t)(y + jy);
        if (py < 0 || py >= (int16_t)HEIGHT) continue;
        for (uint16_t ix = 0; ix < w; ix += 2) {
            const size_t byteIndex = (size_t)jy * srcStride + (ix / 2);
            const uint8_t data = progmem ? pgm_read_byte(&bitmap[byteIndex]) : bitmap[byteIndex];
            const int16_t px1 = (int16_t)(x + ix);
            const int16_t px2 = (int16_t)(x + ix + 1);
            if (px1 >= 0 && px1 < (int16_t)WIDTH) drawPixel(px1, py, (data >> 4) & 0x0F);
            if (ix + 1 < w && px2 >= 0 && px2 < (int16_t)WIDTH) drawPixel(px2, py, data & 0x0F);
        }
    }
}

/*!
    @brief  Draw a PROGMEM-resident 4bpp bitmap. See drawBitmap4bpp().
*/
void PicoGFX_SSD1322::draw4bppBitmap(int16_t x, int16_t y, const uint8_t *bitmap, uint16_t w, uint16_t h) {
    drawBitmap4bpp(x, y, bitmap, w, h, true);
}

/*!
    @brief  Draw a RAM-resident 4bpp bitmap. See drawBitmap4bpp().
*/
void PicoGFX_SSD1322::draw4bppBitmap(int16_t x, int16_t y, uint8_t *bitmap, uint16_t w, uint16_t h) {
    drawBitmap4bpp(x, y, bitmap, w, h, false);
}
