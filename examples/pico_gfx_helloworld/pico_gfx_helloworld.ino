#include <PicoGFX_SSD1322.h>

// Display dimensions
#define SCREEN_WIDTH 256
#define SCREEN_HEIGHT 64

// Pin definitions
#define CS_PIN 5
#define DC_PIN 6
#define RST_PIN 7

// Display object with SPI and specified pins
PicoGFX_SSD1322 display(SCREEN_WIDTH, SCREEN_HEIGHT, &SPI, DC_PIN, RST_PIN, CS_PIN);

// Animation variables
int x = 0;                     // X position of text
int y = 0;                     // Y position of text
int dx = 1;                    // X velocity
int dy = 1;                    // Y velocity
int prevX = -100;              // Previous X position (invalid initially)
int prevY = -100;              // Previous Y position (invalid initially)
unsigned long lastUpdate = 0;  // Last time position was updated
const int UPDATE_INTERVAL = 50;// Update position every 50ms
const int TEXT_PADDING = 1;    // Extra pixels to clear around text 

// Flag to track if display update is needed
bool needsDisplayUpdate = true;

void setup() {
  // Initialize display
  display.begin(0, true);
}

void loop() {
  unsigned long currentMillis = millis();

  // Update position only if UPDATE_INTERVAL has passed and previous transfer is complete
  if (currentMillis - lastUpdate >= UPDATE_INTERVAL && display.isTransferComplete()) {
    x += dx;
    y += dy;

    // Get text bounds for accurate dimensions
    int16_t bx, by, textWidth, textHeight;
    uint16_t bw, bh;
    display.getTextBounds("Hello World!", x, y, &bx, &by, &bw, &bh);
    textWidth = bw + TEXT_PADDING * 2; // Add padding to width
    textHeight = bh + TEXT_PADDING * 2; // Add padding to height
    bx -= TEXT_PADDING; // Adjust x to include left padding
    by -= TEXT_PADDING; // Adjust y to include top padding

    // Handle X boundaries (bounce off left and right)
    if (x <= 0) {
      dx = 1;  // Move right
      x = 0;   // Clamp to left edge
    } else if (x >= SCREEN_WIDTH - textWidth) {
      dx = -1;                        // Move left
      x = SCREEN_WIDTH - textWidth;  // Clamp to right edge
    }

    // Handle Y boundaries (bounce off top and bottom)
    if (y <= 0) {
      dy = 1;  // Move down
      y = 0;   // Clamp to top edge
    } else if (y >= SCREEN_HEIGHT - textHeight) {
      dy = -1;                          // Move up
      y = SCREEN_HEIGHT - textHeight;  // Clamp to bottom edge
    }

    // Clear previous text region if it exists
    if (prevX >= 0 && prevY >= 0) {
      // Use previous text bounds with padding
      int16_t prevBx, prevBy;
      uint16_t prevBw, prevBh;
      display.getTextBounds("Hello World!", prevX, prevY, &prevBx, &prevBy, &prevBw, &prevBh);
      display.fillRect(prevBx - TEXT_PADDING, prevBy - TEXT_PADDING, prevBw + TEXT_PADDING, prevBh + TEXT_PADDING, SSD1322_BLACK);
    }
    // Set cursor to current position
    display.setCursor(x, y);
    // Draw text
    display.println("Hello World!");
    // Start non-blocking display update
    display.display();
    needsDisplayUpdate = false; // Mark update as started
    // Update previous position
    prevX = x;
    prevY = y;
    lastUpdate = currentMillis; // Record update time
  }

  // Check if the transfer is complete
  if (!display.isTransferComplete()) {
    // Optional: Perform other tasks while waiting for transfer
    // For this example, we let the loop continue
  } else if (needsDisplayUpdate == false) {
    needsDisplayUpdate = true; // Ready for the next update
  }
}