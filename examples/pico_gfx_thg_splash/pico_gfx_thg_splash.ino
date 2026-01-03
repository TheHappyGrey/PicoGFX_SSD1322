#include <PicoGFX_SSD1322.h>
#include "SPLASH.h"

// Pin definitions (adjust if your wiring is different)
#define DC_PIN  6
#define RST_PIN 7
#define CS_PIN  5

// Display size
#define SSD1322_WIDTH  256
#define SSD1322_HEIGHT 64

// Initialize display using hardware SPI (recommended for Pico)
PicoGFX_SSD1322 display(SSD1322_WIDTH, SSD1322_HEIGHT, &SPI, DC_PIN, RST_PIN, CS_PIN);

void setup() {
  Serial.begin(115200);

  // Initialize the display
  if (!display.begin(0, true)) {
    Serial.println("SSD1322 initialization failed!");
    while (1); // Halt
  }

  // Optional: Set a nice contrast
  display.setContrast(0xFF); // Full contrast (adjust 0x00–0xFF as needed)

  // Clear both buffers to black
  display.clearDisplay();

  // Optional: Set a callback so we know when a frame is fully sent
  display.setDisplayCompleteCallback([] {
    Serial.println("Splash image fully transferred to display!");
  });

  // Draw the splash screen once
  display.draw4bppBitmap(SPLASH_x_offset, SPLASH_y_offset,
                         SPLASH, SPLASH_width, SPLASH_height);

  // Start non-blocking transfer
  display.display();  // This returns immediately – DMA works in background
}

void loop() {
  // Check if the current transfer has finished
  if (display.isTransferComplete()) {
    // Nothing to do here for a static splash, but this is where you'd:
    // - Start drawing the next frame (e.g., animation)
    // - Update sensors, UI, etc.
    // - Call display.display() again when ready

    // For a simple static splash, we can just wait a bit and keep it on screen
    delay(10); // Small delay to avoid busy-looping at 100%
  }
  // If transfer is still in progress, loop() continues immediately
  // → Your code can do other useful work here (true non-blocking!)
}