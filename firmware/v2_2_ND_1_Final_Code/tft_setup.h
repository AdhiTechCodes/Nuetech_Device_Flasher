#define USER_SETUP_LOADED 1

// ─── Driver ───────────────────────────────────────────────────────────────────
// ILI9341_2_DRIVER fixes the "75% screen filled" issue on clone panels
#define ILI9341_2_DRIVER

// ─── Resolution ───────────────────────────────────────────────────────────────
#define TFT_WIDTH  240
#define TFT_HEIGHT 320

// ─── ESP32 WROOM-DA SPI pins ──────────────────────────────────────────────────
#define TFT_MOSI 23
#define TFT_SCLK 18
#define TFT_CS    5
#define TFT_DC   17
#define TFT_RST  16

// ─── SPI speed ────────────────────────────────────────────────────────────────
// 40 MHz is reliable on most ILI9341 panels with short wires.
// If you see glitches/corruption lower to 27000000.
// 2.7 MHz was far too slow — caused the sluggish/partial refresh feel.
#define SPI_FREQUENCY       40000000
#define SPI_READ_FREQUENCY   6000000   // read-back (RDID etc.)

// ─── Fonts ────────────────────────────────────────────────────────────────────
#define LOAD_GLCD    // built-in 5×7 (used for small labels)
#define LOAD_FONT2   // 16px — used for mid-size text
#define LOAD_FONT4   // 26px — used for larger numerics (optional but useful)

// ─── Sprite support ───────────────────────────────────────────────────────────
// Sprites live in PSRAM when available. If your module has PSRAM (most WROVER
// boards) define this and the sprite allocator will prefer PSRAM automatically.
// On WROOM (no PSRAM) leave it commented — sprites will use heap.
// #define CONFIG_SPIRAM_SUPPORT

// ─── DMA transfer (speeds up sprite push considerably) ────────────────────────
// Uncomment if you want DMA-assisted sprite->screen blits.
// #define SPI_HAS_TRANSACTION  // already defined by Arduino SPI lib
