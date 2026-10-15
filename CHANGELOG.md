## [1.1.0] - 2026-10-15

Correctness and efficiency pass. No API was removed; `display()` keeps its signature.

### Fixed
- **The two buffers are now kept in step.** After the swap, the frame just sent is copied into the
  buffer you draw next. Without this, drawing that did not cover the whole dirty rectangle let
  pixels from two frames ago through — an outline drawn over existing artwork erased it.
- **An empty dirty window no longer reads past the end of the buffer.** Calling `display()` with
  nothing drawn since the last one produced an inverted row range, a byte count that underflowed
  to 130, and a DMA that read 128 bytes beyond the frame buffer. It now returns early.
- **The busy check happens before the buffer swap**, not after. Calling `display()` while a
  transfer was running swapped the buffers anyway, handing the caller the buffer being
  transmitted to draw into.
- **The dirty window is cleared when the region is captured**, not when the transfer completes —
  so anything drawn while a transfer is in flight is still recorded and sent next time.
- **Software SPI works.** That constructor leaves the hardware SPI pointer null, and `display()`
  dereferenced it anyway. It now falls back to a blocking send, as I2C does.
- **Allocator mismatch and double free.** The base class's buffer comes from `malloc()` and was
  released with `delete[]`; the destructor left `buffer` pointing at memory it had already freed,
  for the base destructor to free again.
- **`transferAsync()`'s return value is checked.** When no DMA channel is free the transfer never
  starts, and `finishedAsync()` reports exactly that as "finished" — so a failed frame used to
  look like a completed one while CS stayed low.
- `draw4bppBitmap()` marked the dirty rectangle from the bitmap's full extent before clipping, so
  a sprite mostly off-screen dragged the region out past the panel edge.

### Faster
- **A full-width update goes as a single DMA transfer** instead of one per row. The rows are one
  unbroken run of bytes in the buffer, so a whole-screen frame is 1 transfer rather than 64 — and
  nothing has to poll in between to advance them.
- **Byte-aligned 4bpp bitmaps are copied, not plotted.** Where the bitmap sits on a byte boundary
  on an unrotated panel, rows go in with `memcpy` instead of two virtual `drawPixel()` calls per
  byte. A full-panel bitmap: **9.80 ms → 0.12 ms** on a Pico at 133 MHz. Odd offsets, odd widths,
  rotated panels and off-edge bitmaps take the per-pixel path as before.

### Added
- `displayAsync()` — `display()` with a return value saying whether a transfer started.
- `PICOGFX_SSD1322_VERSION`, as `major * 10000 + minor * 100 + patch` (1.1.0 is `10100`). Worth
  checking in a sketch that redraws only part of the frame: the buffer-sync fix above changes
  behaviour rather than the API, so against an older copy such a sketch is quietly wrong instead
  of failing to build.

  ```cpp
  #if !defined(PICOGFX_SSD1322_VERSION) || PICOGFX_SSD1322_VERSION < 10100
  #error "Needs PicoGFX_SSD1322 1.1.0 or newer"
  #endif
  ```

## [1.0.0] - 2026-01-03
- Initial release with DMA support for Pico.
- Forked from venice1200's SSD1322 adaptation.
