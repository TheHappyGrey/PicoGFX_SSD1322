# PicoGFX_SSD1322

## Overview

PicoGFX_SSD1322 is an Arduino library specifically designed for driving 256×64 grayscale OLED displays based on the SSD1322 controller **on Raspberry Pi Pico (RP2040) and Pico 2 (RP2350)**.

It features high-performance **non-blocking DMA SPI transfers** using Earle F. Philhower III's Arduino-Pico core, enabling smooth, high-frame-rate updates while freeing the CPU for other tasks.

While the library technically supports both SPI and I2C interfaces, **the advanced DMA optimizations and full functionality are available only on RP2040/RP2350 boards using the official Arduino-Pico core**. On other microcontrollers, the library will not compile due to Pico-specific SPI extensions.

The library requires at least 16 KB of SRAM for double-buffering (8 KB × 2) and is not suitable for low-memory boards like Arduino UNO.

This library is a fork of the Adafruit SSD1327 library, modified by venice1200 in September 2021 for SSD1322, and further enhanced in 2026 by The Happy Grey for Pico-specific optimizations.

Please support Adafruit and open-source hardware by purchasing products from [Adafruit](https://www.adafruit.com/).

## Features

- 256×64 pixel SSD1322-based grayscale OLED support with 16 levels of grayscale
- Optimized for Raspberry Pi Pico / Pico 2 with non-blocking DMA SPI transfers
- Double buffering for tear-free updates
- Dirty rectangle optimization to reduce data transfer
- Completion callback for asynchronous SPI updates
- Full graphics support via Adafruit_GFX (text, shapes, rotation)
- Custom 4-bit grayscale bitmap drawing (PROGMEM and RAM versions)
- I2C and SPI interface support (SPI strongly recommended)

## Why DMA?

Non-blocking DMA transfers allow the display to update in the background while your sketch continues running. This is ideal for real-time applications (e.g., games, UI animations, sensor dashboards) where smooth updates and responsive code are essential.

## Compatibility

| Platform  | DMA SPI (Non-blocking) | Basic SPI | I2C | Notes                              |
|-----------|------------------------|-----------|-----|------------------------------------|
| RP2040    | Full support           | Yes       | Yes | Optimal performance                |
| RP2350    | Full support           | Yes       | Yes | Optimal performance                |
| Other     | Not supported          | No        | No  | Pico-specific async SPI code       |

**Note**: Future versions may add a compile-time fallback to blocking SPI for other boards, but the primary focus remains high-performance use on Pico/Pico 2.

## License

BSD 3-Clause License. See `LICENSE.md` for details. The original Adafruit SSD1327 header and splash screen must be included in any redistribution.

## History

- **Original**: Adafruit SSD1327 Library by Limor Fried/Ladyada for Adafruit Industries.
- **Forked**: September 2021 by venice1200, adapted for SSD1322 with 256×64 resolution.
- **Modified**: 2026 by The Happy Grey, renamed to PicoGFX_SSD1322, optimized for Raspberry Pi Pico/Pico 2 with non-blocking DMA SPI support using the Arduino-Pico core.
- Original Adafruit SSD1327: https://github.com/adafruit/Adafruit_SSD1327

## Installation

### Prerequisites

- Arduino IDE 1.8.0 or later
- **Arduino-Pico Core** by Earle F. Philhower III (required for DMA features)
- Target board: Raspberry Pi Pico or Pico 2
- Dependencies (install via Library Manager):
  - Adafruit GFX Library
  - Adafruit Gray OLED Library

### Install the Library

1. Download or clone this repository: https://github.com/TheHappyGrey/PicoGFX_SSD1322
2. In Arduino IDE: **Sketch → Include Library → Add .ZIP Library…** and select the downloaded ZIP  
   OR place the folder in your Arduino libraries directory
3. Restart the IDE

Examples will appear under **File → Examples → PicoGFX_SSD1322**

## Hardware Connections

### SPI (Recommended)

| Display Pin | Pico Pin | Notes                     |
|-------------|----------|---------------------------|
| MOSI        | GP7      |                           |
| SCLK        | GP6      |                           |
| DC          | GP8      | Data/Command              |
| CS          | GP9      | Chip Select               |
| RST         | GP10     | Reset (or -1 if not used) |
| VCC         | 3.3V     |                           |
| GND         | GND      |                           |

### I2C

| Display Pin | Pico Pin | Notes                     |
|-------------|----------|---------------------------|
| SDA         | GP4      |                           |
| SCL         | GP5      |                           |
| RST         | GP10     | Reset (or -1 if not used) |
| VCC         | 3.3V     |                           |
| GND         | GND      |                           |

**Note**: Add 4.7 kΩ pull-up resistors on SDA/SCL if not using Qwiic/STEMMA QT cables.

## Usage

#include <PicoGFX_SSD1322.h>
#include <Adafruit_GFX.h>

Constructors
Hardware SPI (Recommended)

PicoGFX_SSD1322 display(256, 64, &SPI, DC_PIN, RST_PIN, CS_PIN, bitrate);

Example:

PicoGFX_SSD1322 display(256, 64, &SPI, 8, 10, 9);  // Default 8 MHz; up to 16 MHz possible

Software SPI

PicoGFX_SSD1322 display(256, 64, MOSI_PIN, SCLK_PIN, DC_PIN, RST_PIN, CS_PIN);

I2C

PicoGFX_SSD1322 display(256, 64, &Wire, RST_PIN);

Initialization

void setup() {
  if (!display.begin()) {
    Serial.println("Display init failed!");
    while (1);
  }
}

Key Methods

display() – Starts non-blocking DMA transfer (SPI) or blocking update (I2C)
displayBlocking() – Waits until update completes
isTransferComplete() – Returns true when DMA transfer is finished
setDisplayCompleteCallback(callback) – Execute function when update completes
invertDisplay(bool i) – Invert colors
displayOn() / displayOff() – Power control
allPixelOn() / allPixelOff() – All pixels full bright / off
setContrast(uint8_t level) – 0–255
draw4bppBitmap(x, y, bitmap, w, h) – Draw grayscale bitmap (PROGMEM or RAM)
getFont() – Returns current GFX font

Full Example Sketch

#include <PicoGFX_SSD1322.h>
#include <Adafruit_GFX.h>
#include "splash.h"  // Optional: included sample bitmap

// Adjust pins to your wiring
#define DC_PIN   8
#define RST_PIN  10
#define CS_PIN   9

PicoGFX_SSD1322 display(256, 64, &SPI, DC_PIN, RST_PIN, CS_PIN);

void setup() {
  Serial.begin(115200);

  if (!display.begin()) {
    Serial.println("Display initialization failed!");
    while (1);
  }

  // Clear screen
  display.fillRect(0, 0, 256, 64, 0);

  // Text demo
  display.setTextColor(15);           // Brightest grayscale level
  display.setCursor(10, 10);
  display.print("PicoGFX_SSD1322");
  display.setCursor(10, 30);
  display.print("256x64 Grayscale OLED");

  // Shape demo
  display.drawRect(80, 20, 100, 30, 10);

  // Bitmap demo (using included splash)
  display.draw4bppBitmap(80, 0, splash1_data, splash1_width, splash1_height);

  display.displayBlocking();  // Wait until fully updated
}

void loop() {
  // Rotation demonstration
  static uint8_t rot = 0;
  display.setRotation(rot);
  display.fillRect(0, 0, 256, 64, 0);
  display.setCursor(20, 20);
  display.print("Rotation: ");
  display.print(rot * 90);
  display.print(" deg");
  display.displayBlocking();

  rot = (rot + 1) % 4;
  delay(2000);
}

Asynchronous Example with Callback

void onDisplayDone() {
  Serial.println("Display update complete!");
  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));  // Blink LED
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  // ... initialize display ...
  display.setDisplayCompleteCallback(onDisplayDone);
}

void loop() {
  static int frame = 0;
  // Update display non-blocking
  display.fillCircle(128, 32, 20 + (frame % 20), frame % 16);
  display.display();  // Returns immediately

  // Do other work while display updates
  // e.g., read sensors, handle inputs, etc.

  frame++;
}

Drawing a PROGMEM Bitmap

#include "mybitmap.h"  // Your converted 4bpp bitmap

display.draw4bppBitmap(0, 0, myBitmap, 256, 64);
display.displayBlocking();

Troubleshooting

Init fails → Check wiring, 3.3V power, pin definitions, and that the Arduino-Pico core is installed
DMA not working → Confirm you're using hardware SPI and the official Philhower core
Screen tearing/flickering → Always wait for isTransferComplete() or use displayBlocking()
Compilation errors on non-Pico boards → This library is intentionally Pico-specific

Limitations

Requires Arduino-Pico core for full functionality
~16 KB SRAM needed for double buffer
Only 4-wire SPI and I2C tested
SPI commands (not data) and all I2C operations are blocking
Long-term OLED use may cause pixel dimming — call displayOff() when idle to extend life

Contributing
Contributions are welcome!

Format code with clang-format
Add Doxygen-style comments
Follow the Code of Conduct
See Adafruit Doxygen guides: https://learn.adafruit.com/the-well-automated-arduino-library/doxygen

Changelog
See Changelog.md
Support
Open an issue: https://github.com/TheHappyGrey/PicoGFX_SSD1322/issues
Arduino License: BSD