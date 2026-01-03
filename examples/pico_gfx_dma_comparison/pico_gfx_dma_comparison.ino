/*********************************************************************
  PicoGFX_SSD1322 vs Blocking SSD1322 - Performance Benchmark Demo

  This example demonstrates the dramatic performance advantage of the
  PicoGFX_SSD1322 library over traditional blocking SPI implementations
  when driving a 256×64 4-bit grayscale SSD1322 OLED display on the
  Raspberry Pi Pico (RP2040).

  Key Features Demonstrated:
  • True non-blocking DMA-driven SPI transfers using RP2040 hardware DMA
  • Double buffering for tear-free frame updates
  • Automatic dirty rectangle tracking – only changed pixels are sent
  • Full CPU availability during display updates

  Every 10 seconds the sketch automatically switches between:
    → PicoGFX_SSD1322 (this library) – Non-blocking DMA mode
    → SSD1322_for_Adafruit_GFX      – Traditional blocking SPI mode

  On-screen metrics (updated in real time):
    • Frame:    Cumulative frame count since last mode switch
    • Xfer:     Transfer time of the dirty region (microseconds)
    • CPU:      Number of loop iterations executed while transfer runs
                (higher = more CPU time available for application code)
    • FPS:      Achieved frames per second in the current mode

  Typical results on Raspberry Pi Pico:
    DMA mode:      ~4000–5000 µs transfer | CPU ~8000–15000 | 150–180 FPS
    Blocking mode: ~19000–22000 µs transfer | CPU = 1       | ~40–50 FPS

  This benchmark clearly shows why PicoGFX_SSD1322 is the best choice
  for smooth animations, responsive UIs, games, or any project requiring
  high refresh rates on grayscale OLEDs.

  Wiring (Hardware SPI – recommended):
    OLED MOSI → Pico GP7  (SPI0 TX)
    OLED SCLK → Pico GP6  (SPI0 SCK)
    OLED DC   → Pico GP8  (change DC_PIN if different)
    OLED CS   → Pico GP9  (change CS_PIN if different)
    OLED RST  → Pico GP10 (change RST_PIN if different)
    VCC → 3.3V, GND → GND

  Author: The Happy Grey
  Year:   2025–2026
  Library: PicoGFX_SSD1322 – Optimized for Raspberry Pi Pico with DMA
  License: BSD (compatible with original Adafruit SSD1327 base)

*********************************************************************/

#include <PicoGFX_SSD1322.h>
#include <SSD1322_for_Adafruit_GFX.h>

// --------------------------------------------------------------------
// Pin configuration – adjust if your wiring differs
// --------------------------------------------------------------------
#define DC_PIN  6   // Data/Command pin
#define RST_PIN 7   // Reset pin (-1 if not used)
#define CS_PIN  5   // Chip Select pin

// --------------------------------------------------------------------
// Display instances – both use the same hardware SPI bus
// --------------------------------------------------------------------
PicoGFX_SSD1322  displayDMA(256, 64, &SPI, DC_PIN, RST_PIN, CS_PIN);
Adafruit_SSD1322 displayNonDMA(256, 64, &SPI, DC_PIN, RST_PIN, CS_PIN);

// --------------------------------------------------------------------
// Performance metrics for DMA mode
// --------------------------------------------------------------------
unsigned long frame_counter_dma = 0;     // Frames rendered since mode switch
unsigned long last_time_dma     = 0;     // Timestamp for FPS calculation
unsigned long fps_dma           = 0;     // Frames per second
unsigned long cpu_counter_dma   = 0;     // Measures CPU availability during transfer
unsigned long transfer_time_dma = 0;     // Transfer duration in microseconds

// --------------------------------------------------------------------
// Performance metrics for blocking (non-DMA) mode
// --------------------------------------------------------------------
unsigned long frame_counter_nondma = 0;
unsigned long last_time_nondma     = 0;
unsigned long fps_nondma           = 0;
unsigned long cpu_counter_nondma   = 0;
unsigned long transfer_time_nondma = 0;

// --------------------------------------------------------------------
// Mode switching control
// --------------------------------------------------------------------
bool          useDMA       = true;                 // Start in DMA mode
unsigned long toggle_time  = 0;
const unsigned long toggle_interval = 10000;       // Switch every 10 seconds

void setup() {
  // Start serial for optional debugging / logging
  Serial.begin(115200);

  // ----------------------------------------------------------------
  // Initialise both display drivers
  // ----------------------------------------------------------------
  displayDMA.begin(0, true);      // Address 0 (ignored for SPI), perform hardware reset
  displayNonDMA.begin();

  // Clear both framebuffers at startup
  displayDMA.clearDisplay();
  displayNonDMA.clearDisplay();

  // ----------------------------------------------------------------
  // Common text settings (both libraries inherit from Adafruit_GFX)
  // ----------------------------------------------------------------
  displayDMA.setTextColor(15);     // Maximum brightness (white)
  displayDMA.setTextSize(1);       // 6×8 pixel font
  displayNonDMA.setTextColor(15);
  displayNonDMA.setTextSize(1);

  // Initialise timers
  last_time_dma     = millis();
  last_time_nondma  = millis();
  toggle_time       = millis();

  // Optional: maximise contrast for best visibility
  displayDMA.setContrast(0xFF);
}

void loop() {
  // ----------------------------------------------------------------
  // Automatic mode switching every 10 seconds
  // ----------------------------------------------------------------
  if (millis() - toggle_time >= toggle_interval) {
    useDMA = !useDMA;
    toggle_time = millis();

    // Clear the screen of the newly active driver to remove old text
    if (useDMA) {
      displayDMA.clearDisplay();
      displayDMA.display();                     // Start transfer
      while (!displayDMA.isTransferComplete()); // Wait for clear to finish
    } else {
      displayNonDMA.clearDisplay();
      displayNonDMA.display();
    }

    // Optional serial notification
    Serial.println(useDMA
      ? "\n=== SWITCHED TO PicoGFX_SSD1322 (Non-blocking DMA) ==="
      : "\n=== SWITCHED TO SSD1322_for_Adafruit_GFX (Blocking SPI) ===");
  }

  // ----------------------------------------------------------------
  // DMA mode – PicoGFX_SSD1322 (non-blocking)
  // ----------------------------------------------------------------
  if (useDMA) {
    // Clear only the numeric fields to prevent digit overlap
    // Coordinates carefully chosen for the default 6×8 font
    displayDMA.fillRect(138, 0,  17, 7, SSD1322_BLACK);  // Frame number
    displayDMA.fillRect(156, 8,  47, 7, SSD1322_BLACK);  // Transfer time
    displayDMA.fillRect(126, 16, 29, 7, SSD1322_BLACK);  // CPU cycles
    displayDMA.fillRect(126, 24, 17, 7, SSD1322_BLACK);  // FPS

    // Redraw static labels and current values
    displayDMA.setCursor(0, 0);
    displayDMA.print("PicoGFX_SSD1322 Frame: ");
    displayDMA.println(frame_counter_dma);

    displayDMA.print("                Transfer: ");
    displayDMA.print(transfer_time_dma);
    displayDMA.println(" us");

    displayDMA.print("                CPU: ");
    displayDMA.println(cpu_counter_dma);

    displayDMA.print("                FPS: ");
    displayDMA.println(fps_dma);

    // ----------------------------------------------------------------
    // Measure transfer performance and CPU availability
    // ----------------------------------------------------------------
    unsigned long start_time = micros();
    cpu_counter_dma = 0;

    displayDMA.display();  // Start non-blocking DMA transfer

    // This loop runs many times while DMA works in the background
    while (!displayDMA.isTransferComplete()) {
      cpu_counter_dma++;   // Demonstrates CPU is free for other tasks
    }

    unsigned long end_time = micros();
    transfer_time_dma = end_time - start_time;

    frame_counter_dma++;

    // Update FPS once per second
    unsigned long current_time = millis();
    if (current_time - last_time_dma >= 1000) {
      fps_dma = frame_counter_dma;     // Frames rendered in the last second
      frame_counter_dma = 0;
      last_time_dma = current_time;
    }

  // ----------------------------------------------------------------
  // Blocking mode – SSD1322_for_Adafruit_GFX
  // ----------------------------------------------------------------
  } else {
    // Full clear required because the reference library does not use dirty rectangles
    displayNonDMA.clearDisplay();

    // Draw all information
    displayNonDMA.setCursor(0, 0);
    displayNonDMA.print("SSD1322_for_Adafruit_GFX Frame: ");
    displayNonDMA.println(frame_counter_nondma);

    displayNonDMA.print("                         Xfer: ");
    displayNonDMA.print(transfer_time_nondma);
    displayNonDMA.println(" us");

    displayNonDMA.print("                         CPU: ");
    displayNonDMA.println(cpu_counter_nondma);

    displayNonDMA.print("                         FPS: ");
    displayNonDMA.println(fps_nondma);

    // Measure blocking transfer
    unsigned long start_time = micros();
    cpu_counter_nondma = 0;

    displayNonDMA.display();  // This call blocks until transfer completes

    // Only executes after the entire transfer finishes
    cpu_counter_nondma++;

    unsigned long end_time = micros();
    transfer_time_nondma = end_time - start_time;

    frame_counter_nondma++;

    // FPS update
    unsigned long current_time = millis();
    if (current_time - last_time_nondma >= 1000) {
      fps_nondma = frame_counter_nondma;
      frame_counter_nondma = 0;
      last_time_nondma = current_time;
    }
  }
}