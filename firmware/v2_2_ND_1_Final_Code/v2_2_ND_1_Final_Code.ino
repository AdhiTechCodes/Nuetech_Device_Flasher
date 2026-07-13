// ═══════════════════════════════════════════════════════════════════════════════
//  NUETECH CONTROLLER  —  FIRMWARE (WiFi + BLE Combined)
//  Platform : ESP32 + ILI9341 TFT 320×240 (landscape, rotation 3)
//  Sensors  : DS3231 RTC · AHT20 ambient · NTC tank
//  Actuator : Relay (ACTIVE HIGH) · Passive buzzer
//
//  NETWORK  : WiFi  OR  BLE  — user-selectable from Manual Mode
//  COLOR DEPTH: 1-bit sprite (MONOCHROME — 2 colors only: 0=BLACK, 1=WHITE)
//
//  Author  : M Adarsha (Embedded System Engineering in NUETECH)
//  Version : v2.2
// ═══════════════════════════════════════════════════════════════════════════════

// ─────────────────────────────────────────────────────────────────────────────
// INCLUDES
// ─────────────────────────────────────────────────────────────────────────────
#include <Wire.h>
#include <FS.h>
using namespace fs;
#include <TFT_eSPI.h>
#include <BluetoothSerial.h>
#include <Preferences.h>
#include <math.h>
#include <RTClib.h>
#include <WiFi.h>
#include <WebServer.h>

#include <ArduinoJson.h>
#include <Adafruit_AHTX0.h>
#include <esp_log.h>
#include <time.h>

// ─────────────────────────────────────────────────────────────────────────────
// DISPLAY  —  ILI9341  320 × 240  (landscape, rotation 3)
// ─────────────────────────────────────────────────────────────────────────────
TFT_eSPI    tft    = TFT_eSPI();   // main display object
TFT_eSprite spr   = TFT_eSprite(&tft); // 1-bit monochrome off-screen sprite
bool        sprReady = false;      // true once sprite RAM is allocated

#define SCREEN_W 320
#define SCREEN_H 240

// ─────────────────────────────────────────────────────────────────────────────
// PERIPHERAL OBJECTS
// ─────────────────────────────────────────────────────────────────────────────
BluetoothSerial SerialBT;   // Classic BT serial (ESP32)
Preferences     prefs;      // NVS key-value storage
RTC_DS3231      rtc;        // Real-time clock
Adafruit_AHTX0  aht;        // AHT20 temperature / humidity sensor
WebServer       server(80); // HTTP server (WiFi mode)

// ─────────────────────────────────────────────────────────────────────────────
// EEPROM  —  AT24C32 on DS3231 module (I²C address 0x50)
// ─────────────────────────────────────────────────────────────────────────────
#define EEPROM_I2C_ADDR   0x50   // AT24C32 default I²C address
#define EEPROM_SIZE       4096   // 32 Kbit = 4096 bytes
#define EEPROM_PAGE_SIZE  32     // AT24C32 page size for writes

#define LOG_HEADER_SIZE   8      // header bytes at start of EEPROM
#define LOG_RECORD_SIZE   12     // bytes per daily log record
#define LOG_MAX_RECORDS   31     // 1 month of daily data
#define LOG_DATA_START    LOG_HEADER_SIZE  // first record address
#define LOG_MAGIC         0x4E55 // "NU" — validates EEPROM is initialized
#define LOG_VERSION       0x01

// Packed daily log record (12 bytes)
struct DailyLogRecord {
  uint8_t  day;            // 1–31
  uint8_t  month;          // 1–12
  uint8_t  yearOff;        // year - 2020
  uint16_t mornOnTimeSec;  // morning relay on-time (seconds, max 65535)
  uint8_t  mornPeakTemp;   // morning peak tank temp (°C)
  uint16_t eveOnTimeSec;   // evening relay on-time (seconds, max 65535)
  uint8_t  evePeakTemp;    // evening peak tank temp (°C)
  uint8_t  ambientTemp;    // ambient temp × 2 (0.5°C resolution, uint8)
  uint8_t  errorFlags;     // E1/E2/E3 bitmask accumulated during the day
};

bool eepromFound = false;  // true if AT24C32 responded at boot


// ─────────────────────────────────────────────────────────────────────────────
// PIN MAP
// ─────────────────────────────────────────────────────────────────────────────
#define BTN_SET    25   // SET button  (INPUT_PULLUP → active LOW)
#define BTN_UP     33   // UP  button
#define BTN_DOWN   32   // DOWN button
#define RELAY_TANK 26   // Tank heating relay output
#define NTC_TANK   35   // NTC thermistor ADC input (tank water temperature)
#define BUZZER_PIN 27   // Passive buzzer (LEDC PWM)

// ─────────────────────────────────────────────────────────────────────────────
// RELAY  —  ACTIVE HIGH
// ─────────────────────────────────────────────────────────────────────────────
#define RELAY_ON  HIGH
#define RELAY_OFF LOW

// ─────────────────────────────────────────────────────────────────────────────
// BUZZER  —  LEDC  2700 Hz / 8-bit
// ─────────────────────────────────────────────────────────────────────────────
#define BUZZER_FREQ 2700
#define BUZZER_RES   8

void buzzerOn()  { ledcWrite(BUZZER_PIN, 128); }
void buzzerOff() { ledcWrite(BUZZER_PIN, 0);   }
void buzzerBeep(int ms) { buzzerOn(); delay(ms); buzzerOff(); }

// ─────────────────────────────────────────────────────────────────────────────
// WELCOME BEEP  —  3-tone rising melody played once at boot
// ─────────────────────────────────────────────────────────────────────────────
void playWelcomeBeep() {
  ledcChangeFrequency(BUZZER_PIN, 897,  BUZZER_RES);
  ledcWrite(BUZZER_PIN, 128); delay(135); ledcWrite(BUZZER_PIN, 0); delay(30);
  ledcChangeFrequency(BUZZER_PIN, 1206, BUZZER_RES);
  ledcWrite(BUZZER_PIN, 128); delay(200); ledcWrite(BUZZER_PIN, 0); delay(30);
  ledcChangeFrequency(BUZZER_PIN, 1603, BUZZER_RES);
  ledcWrite(BUZZER_PIN, 128); delay(340); ledcWrite(BUZZER_PIN, 0);
  ledcChangeFrequency(BUZZER_PIN, BUZZER_FREQ, BUZZER_RES); // restore default freq
}

// ─────────────────────────────────────────────────────────────────────────────
// COMMUNICATIONS MODE  —  persisted in NVS ("cfg" namespace)
//   MODE_BLE  : Classic Bluetooth serial
//   MODE_WIFI : WiFi + HTTP REST API
// ─────────────────────────────────────────────────────────────────────────────
enum CommsMode { MODE_BLE = 0, MODE_WIFI = 1 };
CommsMode commsMode = MODE_BLE;   // default; overridden by loadCommsMode()

void saveCommsMode() {
  prefs.begin("cfg", false);
  prefs.putInt("commsMode", (int)commsMode);
  prefs.end();
}

void loadCommsMode() {
  prefs.begin("cfg", true);
  commsMode = (CommsMode)prefs.getInt("commsMode", (int)MODE_BLE);
  prefs.end();
}

// ─────────────────────────────────────────────────────────────────────────────
// BLUETOOTH DEVICE NAME  —  changeable via Serial Monitor (AT+NAME=xxx)
//   Persisted in NVS ("cfg" namespace, key "btName")
//   Default: "Nuetech"
// ─────────────────────────────────────────────────────────────────────────────
#define BT_NAME_MAX_LEN 32
String btDeviceName = "Nuetech";   // runtime copy; overridden by loadBtName()

void saveBtName() {
  prefs.begin("cfg", false);
  prefs.putString("btName", btDeviceName);
  prefs.end();
}

void loadBtName() {
  prefs.begin("cfg", true);
  btDeviceName = prefs.getString("btName", "Nuetech");
  prefs.end();
}

// ─────────────────────────────────────────────────────────────────────────────
// TEMPERATURE SETTINGS
// ─────────────────────────────────────────────────────────────────────────────
#define TEMP_MIN        20   // minimum settable temperature (°C)
#define TEMP_MAX        60   // maximum settable temperature (°C)
#define TEMP_HYSTERESIS  5   // hysteresis band (°C) to prevent relay chatter

bool tempHysteresisReady = true; // true = allowed to heat again
int  ST1 = 50;                   // set-point temperature (user adjustable)
int  RT1 = 0;                    // real-time tank temperature (NTC reading)
bool wasFaultLastTime = false;   // used by buzzer fault-beep logic

// ─────────────────────────────────────────────────────────────────────────────
// PIN PROTECTION  (items 7 & 10 in Manual Mode are PIN-locked)
// ─────────────────────────────────────────────────────────────────────────────
#define PIN_SECRET 11   // correct PIN value

enum PinState { PIN_IDLE, PIN_ENTERING, PIN_GRANTED };
PinState  pinState      = PIN_IDLE;
int       pinEnteredVal = 0;   // running value the user is dialling in
int       pinTarget     = 0;   // which manual item triggered the PIN entry

// ─────────────────────────────────────────────────────────────────────────────
// ERROR CODE MAP
//   E1 = Sensor Fault      — NTC disconnected / out of ADC range
//   E2 = Max Coil On Time  — coil ran for > 3 h total without reset
//   E3 = Coil Not Working  — no temperature rise detected in check window
// ─────────────────────────────────────────────────────────────────────────────
#define ERR_NONE          0x00
#define ERR_SENSOR_FAULT  0x01   // E1
#define ERR_MAX_COIL      0x02   // E2
#define ERR_COIL_NO_WORK  0x04   // E3

// Interval between error broadcasts (BLE push / Serial log)
#define ERROR_BROADCAST_INTERVAL_MS  2000UL

static unsigned long lastErrorBroadcast = 0;  // timestamp of last broadcast

// ─────────────────────────────────────────────────────────────────────────────
// COIL TRACKING  —  accumulated on-time, effectiveness check, etc.
// ─────────────────────────────────────────────────────────────────────────────
const unsigned long maxCoilOnTime     = 10800000UL; // 3 h in ms
unsigned long       coilOnTime        = 0;           // cumulative coil on-time (ms)
unsigned long       coilOnStart       = 0;           // millis() when coil last turned ON
bool                coilWasOn         = false;       // previous relay state
unsigned long       relayOffStart     = 0;           // when relay last turned OFF
bool                relayOffTimerActive = false;     // 2-min cooldown timer active?
unsigned long       lastCoilSaveTime  = 0;           // last NVS save timestamp

bool maxCoilError          = false;  // E2 active flag
bool coilErrorAcknowledged = false;  // user held SET to acknowledge

// Coil effectiveness check parameters
unsigned long coilEffCheckDurationMs = 1200000UL; // default 20 min in ms
int           coilEffCheckMinutes    = 20;         // user-adjustable (5–120 min)
int           coilEffMinRiseDeg      = 1;          // min temp rise for coil check (1–10 °C)
bool          coilEffectivenessActive = false;     // check in progress?
unsigned long coilEffCheckStart      = 0;          // when the check started
int           tempAtCoilStart        = 0;          // tank temp when check began
bool          coilNotWorkingError    = false;      // E3 active flag

// "Hold SET to clear" UI state for coil-not-working error
unsigned long coilErrClearHoldStart  = 0;
bool          coilErrClearHolding    = false;

// ─────────────────────────────────────────────────────────────────────────────
// SLOT SCHEDULE
//   Slot 0 = Morning slot
//   Slot 1 = Evening slot
// ─────────────────────────────────────────────────────────────────────────────
bool slotEnabled[2] = {true, true};  // ON/OFF toggle for each slot
int  S_H[2] = {1,  13};             // slot start hour
int  S_M[2] = {0,   0};             // slot start minute
int  E_H[2] = {12, 23};             // slot end hour
int  E_M[2] = {0,  59};             // slot end minute
int preHeatSteps = 0;               // 0 = off; 1 = 30 min; 2 = 1 h; … 4 = 2 h

inline int preHeatMinutes() { return preHeatSteps * 30; }

// ─────────────────────────────────────────────────────────────────────────────
// SLOT HOUR RANGE RESTRICTIONS
//   Morning slot   (index 0) : 01:00 – 12:59
//   Evening slot   (index 1) : 13:00 – 23:59
// ─────────────────────────────────────────────────────────────────────────────
#define SLOT0_MIN_HOUR  1    // Morning slot earliest hour
#define SLOT0_MAX_HOUR 12    // Morning slot latest hour  (12:59 max)
#define SLOT1_MIN_HOUR 13    // Evening slot earliest hour
#define SLOT1_MAX_HOUR 23    // Evening slot latest hour (23:59 max)

inline int slotMinHour(int slot) { return (slot == 0) ? SLOT0_MIN_HOUR : SLOT1_MIN_HOUR; }
inline int slotMaxHour(int slot) { return (slot == 0) ? SLOT0_MAX_HOUR : SLOT1_MAX_HOUR; }

#define MIN_SLOT_GAP_MINS 30   // minimum gap between start and end time (minutes)

bool          slotRelayActive[2]  = {false, false}; // current relay state per slot
unsigned long slotDailyOnTime[2]  = {0, 0};         // seconds relay was ON today
static unsigned long lastRelayUpdate = 0;

// ─────────────────────────────────────────────────────────────────────────────
// MANUAL MODE  (12 menu items)
// ─────────────────────────────────────────────────────────────────────────────
#define MANUAL_ITEMS 13

bool          manualMode     = false;
bool          timeIsSet      = false;  // false when RTC has lost power
int           editItem       = 0;      // currently selected menu item (0–11)
int           editField      = 0;      // sub-field index (e.g. HH vs MM)
bool          editActive     = false;  // true = user is actively editing this item
unsigned long lastActionTime = 0;      // last button press timestamp

unsigned long setHoldStart    = 0;     // when SET button hold started
unsigned long manualEntryTime = 0;     // last activity in manual mode
bool          postInitDone    = false; // one-shot pin re-init after boot

static unsigned long targetNextTick = 0; // next 1 Hz accumulation tick
static unsigned long lastTick       = 0;
static unsigned long lastRelayCheck = 0;

// ─────────────────────────────────────────────────────────────────────────────
// SCREEN CYCLING
//   Index 0–4 : normal rotating screens
// ─────────────────────────────────────────────────────────────────────────────
unsigned long screenCycleTime    = 0;
int           currentScreenIndex = 0;
const int     TOTAL_SCREENS      = 6;

// ─────────────────────────────────────────────────────────────────────────────
// [C6] SLOT-RUNNING SCREEN LOCK STATE
//   When a slot is active, auto-cycling is suspended and the display locks
//   to the running slot screen.  UP/DOWN let the user browse freely; after
//   SLOT_BROWSE_TIMEOUT_MS of no UP/DOWN activity the display snaps back.
// ─────────────────────────────────────────────────────────────────────────────
#define SLOT_BROWSE_TIMEOUT_MS  10000UL   // 10 s idle → snap back to slot screen

bool          slotScreenLocked      = false;  // true while any slot is running
bool          slotBrowsing          = false;  // true while user is browsing away
unsigned long slotBrowseLastPress   = 0;      // millis() of last UP/DOWN press
int           lockedSlotScreenIndex = 2;      // screen 2 = morning, 3 = evening

// ─────────────────────────────────────────────────────────────────────────────
// AHT20 AMBIENT SENSOR
// ─────────────────────────────────────────────────────────────────────────────
float ahtTemp     = 0.0f;
float ahtHumidity = 0.0f;
bool  ahtOk       = false;      // last reading was valid
bool  ahtPresent  = false;      // sensor found on I²C bus at boot

// ─────────────────────────────────────────────────────────────────────────────
// DAILY LOG PEAK TRACKERS  (reset at daily reset, saved to EEPROM)
// ─────────────────────────────────────────────────────────────────────────────
int     mornPeakTemp   = 0;    // highest RT1 seen during morning slot
int     evePeakTemp    = 0;    // highest RT1 seen during evening slot
uint8_t dayErrorAccum  = 0;    // accumulated error flags during the day

// ─────────────────────────────────────────────────────────────────────────────
// NOTIFICATION POPUP  (WiFi / NTP / BLE updates shown on-screen briefly)
// ─────────────────────────────────────────────────────────────────────────────
bool          notifPopupActive = false;
unsigned long notifPopupTime   = 0;
String        notifPopupMsg    = "";
String        notifPopupMsg2   = "";

// ─────────────────────────────────────────────────────────────────────────────
// WIFI STATE MACHINE
// ─────────────────────────────────────────────────────────────────────────────
String  wifiSSID     = "";
String  wifiPassword = "";
bool    wifiConnected   = false;
String  deviceIP        = "";

enum WifiState { WIFI_OK, WIFI_RETRYING, WIFI_BT_PROV_MODE };
WifiState     wifiState              = WIFI_RETRYING;
bool          btProvisioningActive   = false;
CommsMode     btProvOldCommsMode     = MODE_WIFI;
unsigned long wifiRetryStart         = 0;
int           wifiRetryCount         = 0;
const int     WIFI_MAX_RETRY         = 3;
const unsigned long WIFI_RETRY_INTERVAL_MS = 30000UL; // 30 s between retries
unsigned long lastWifiRetryAttempt   = 0;

// ─────────────────────────────────────────────────────────────────────────────
// BT WIFI PROVISIONING (boot-time + watchdog fallback)
// ─────────────────────────────────────────────────────────────────────────────
#define BT_PROV_TIMEOUT_MS   5000UL   // 5 s to press SET at boot
#define BT_PROV_WAIT_MS     90000UL   // 90 s to receive WIFI= command via BT

// ─────────────────────────────────────────────────────────────────────────────
// NTP / RTC SYNC  (WiFi mode only)
// ─────────────────────────────────────────────────────────────────────────────
#define NTP_SERVER1  "pool.ntp.org"
#define NTP_SERVER2  "time.google.com"
#define NTP_GMT_OFFSET_SEC   19800   // UTC+5:30 (India Standard Time)
#define NTP_DAYLIGHT_OFFSET  0

bool          ntpSyncedToday = false;
unsigned long lastNtpSyncMs  = 0;
const unsigned long NTP_DAILY_INTERVAL_MS = 86400000UL; // 24 h

// ─────────────────────────────────────────────────────────────────────────────
// LAYOUT CONSTANTS  (used by sprite-based drawing functions)
// ─────────────────────────────────────────────────────────────────────────────
#define MARGIN     4
#define HDR_H     30
#define HDR_Y      2
#define CONTENT_Y 38
#define FOOTER_Y  218
#define CORNER     6

// ─────────────────────────────────────────────────────────────────────────────
// RAW 16-BIT COLORS  —  used ONLY by direct tft.* calls (boot splash)
// ─────────────────────────────────────────────────────────────────────────────
#define C_BG      TFT_BLACK
#define C_WHITE   TFT_WHITE
#define C_GRAY    0x7BEF
#define C_DKGRAY  0x4208
#define C_BORDER  0xFFFF

// ─────────────────────────────────────────────────────────────────────────────
// 1-BIT MONOCHROME PALETTE  (sprite drawing)
//   P_BG = 0 (black),  P_BORDER / P_ACCENT / … = 1 (white)
// ─────────────────────────────────────────────────────────────────────────────
#define P_BG      0
#define P_PANEL   0
#define P_BORDER  1
#define P_ACCENT  1
#define P_GOLD    1
#define P_GREEN   1
#define P_RED     1
#define P_MAGENTA 1
#define P_ORANGE  1
#define P_DIM     1
#define P_WHITE   1
#define P_PURPLE  1
#define P_DKGRN   0
#define P_DKRED   0
#define P_YELLOW  1
#define P_LTGRAY  1

// ─────────────────────────────────────────────────────────────────────────────
// DEG_TO_RAD constant (used by the logo drawing routines)
// ─────────────────────────────────────────────────────────────────────────────
#ifndef DEG_TO_RAD
#define DEG_TO_RAD 0.017453292519943295769f
#endif

// ═══════════════════════════════════════════════════════════════════════════
//  BASIC HELPERS
// ═══════════════════════════════════════════════════════════════════════════

// Yield to the ESP32 watchdog / scheduler
inline void feedWDT()     { yield(); }

// Returns true during the first half of an 800 ms blink cycle
bool blinkOn()            { return (millis() % 800) < 400; }

// Raw pin state (active-LOW buttons)
bool pressed(int pin)     {
  if (pin == BTN_SET) pinMode(BTN_SET, INPUT_PULLUP);
  return digitalRead(pin) == LOW;
}

// Debounced button press with 150 ms lockout; updates lastActionTime
bool buttonPressed(int pin) {
  static uint32_t lastTime[40];
  uint32_t now = millis();
  if (pressed(pin) && (now - lastTime[pin] > 150)) {
    lastTime[pin] = now;
    lastActionTime = now;
    return true;
  }
  return false;
}

// ─────────────────────────────────────────────────────────────────────────────
// NTC THERMISTOR READER
// ─────────────────────────────────────────────────────────────────────────────
int readNTC(int p)
{
    int t = 0;

    for (int i = 0; i < 16; i++)
    {
        t += analogRead(p);
        delay(10);
    }

    float a = t / 16.0f;

    if (a < 10 || a > 4080)
        return -999;

    float K = 1.0f / (1.0f / 298.15f +(1.0f / 3380.0f) *log(4095.0f / (a * 1.08f) - 1.0f));

    float temp = K - 273.15f;

    // Calibration
    temp = (0.84 * temp) + 5.5;
    
    // if(temp>40)
    // return (int)(temp + 0.5)+1;

    return (int)(temp + 0.5);
}

// ─────────────────────────────────────────────────────────────────────────────
// AHT20 AMBIENT SENSOR
// ─────────────────────────────────────────────────────────────────────────────
void updateAHT20() {
  if (!ahtPresent) { ahtOk = false; return; }
  sensors_event_t h, t;
  if (aht.getEvent(&h, &t)) {
    ahtTemp = t.temperature; ahtHumidity = h.relative_humidity; ahtOk = true;
  } else { ahtOk = false; }
}

// ─────────────────────────────────────────────────────────────────────────────
// BUZZER FAULT HANDLER
// ─────────────────────────────────────────────────────────────────────────────
void handleBuzzer() {
  bool currentFault = (RT1 == -999);
  if (currentFault && !wasFaultLastTime) {
    buzzerBeep(500); delay(500);
  } else if (currentFault && wasFaultLastTime) {
    static unsigned long lastBeep = 0;
    if (millis()-lastBeep > 3000) { buzzerBeep(300); delay(200); lastBeep=millis(); }
  }
  wasFaultLastTime = currentFault;

  if (maxCoilError && !coilErrorAcknowledged) {
    static unsigned long lastMaxBeep = 0;
    if (millis()-lastMaxBeep > 5000) {
      for (int b=0;b<3;b++){buzzerBeep(200);delay(150);}
      lastMaxBeep=millis();
    }
  }
  if (coilNotWorkingError) {
    static unsigned long lastNWBeep = 0;
    if (millis()-lastNWBeep > 4000) {
      for (int b=0;b<2;b++){buzzerBeep(400);delay(200);}
      lastNWBeep=millis();
    }
  }
}

// ═══════════════════════════════════════════════════════════════════════════
//  SPRITE DRAW PRIMITIVES  (all colors are P_BG=0 or P_BORDER=1)
// ═══════════════════════════════════════════════════════════════════════════

inline void sprPush() { if (sprReady) spr.pushSprite(0,0); }

static void hLine(int y, uint8_t col=P_BORDER) {
  spr.drawFastHLine(MARGIN, y, SCREEN_W-2*MARGIN, col);
}

static void drawFrame(uint8_t col=P_BORDER) {
  spr.drawRect(0,0,SCREEN_W,SCREEN_H,col);
  spr.drawRect(2,2,SCREEN_W-4,SCREEN_H-4,col);
}

// ─────────────────────────────────────────────────────────────────────────────
// COMMUNICATIONS STATUS SYMBOL
// ─────────────────────────────────────────────────────────────────────────────
static void drawCommsSymbol(int cx, int cy) {
  // ── BT provisioning active: show both WiFi X + BT rune side by side ──
  if (btProvisioningActive) {
    int wCx = cx - 14;   // WiFi X on the left
    int bCx = cx  ;    // BT rune on the right
    // WiFi disconnected X
    spr.drawLine(wCx-5, cy-6, wCx+5, cy+6, P_BORDER);
    spr.drawLine(wCx+5, cy-6, wCx-5, cy+6, P_BORDER);
    spr.drawLine(wCx-4, cy-6, wCx+6, cy+6, P_BORDER);
    spr.drawLine(wCx+6, cy-6, wCx-4, cy+6, P_BORDER);
    // Bluetooth rune
    spr.drawFastVLine(bCx, cy-8, 16, P_BORDER);
    spr.drawFastVLine(bCx+1, cy-8, 16, P_BORDER);
    spr.drawLine(bCx, cy-8, bCx+5, cy-3, P_BORDER);
    spr.drawLine(bCx+1, cy-8, bCx+6, cy-3, P_BORDER);
    spr.drawLine(bCx, cy+1, bCx+5, cy-4, P_BORDER);
    spr.drawLine(bCx+1, cy+1, bCx+6, cy-4, P_BORDER);
    spr.drawLine(bCx, cy,   bCx+5, cy+5, P_BORDER);
    spr.drawLine(bCx+1, cy,   bCx+6, cy+5, P_BORDER);
    spr.drawLine(bCx, cy+8, bCx+5, cy+3, P_BORDER);
    spr.drawLine(bCx+1, cy+8, bCx+6, cy+3, P_BORDER);
    return;
  }
  if (commsMode == MODE_BLE) {
    spr.drawFastVLine(cx, cy-8, 16, P_BORDER);
    spr.drawFastVLine(cx+1, cy-8, 16, P_BORDER);
    spr.drawLine(cx, cy-8, cx+5, cy-3, P_BORDER);
    spr.drawLine(cx+1, cy-8, cx+6, cy-3, P_BORDER);
    spr.drawLine(cx, cy+1, cx+5, cy-4, P_BORDER);
    spr.drawLine(cx+1, cy+1, cx+6, cy-4, P_BORDER);
    spr.drawLine(cx, cy,   cx+5, cy+5, P_BORDER);
    spr.drawLine(cx+1, cy,   cx+6, cy+5, P_BORDER);
    spr.drawLine(cx, cy+8, cx+5, cy+3, P_BORDER);
    spr.drawLine(cx+1, cy+8, cx+6, cy+3, P_BORDER);
    return;
  }
  if (commsMode == MODE_WIFI && wifiConnected) {
    spr.fillCircle(cx, cy+3, 2, P_BORDER);
    spr.drawArc(cx, cy+3, 5,  3,  120, 240, P_BORDER, P_BG);
    spr.drawArc(cx, cy+3, 9,  7,  120, 240, P_BORDER, P_BG);
    spr.drawArc(cx, cy+3, 13, 11, 120, 240, P_BORDER, P_BG);
    return;
  }
  if (commsMode == MODE_WIFI && !wifiConnected) {
    spr.drawLine(cx-5, cy-6, cx+5, cy+6, P_BORDER);
    spr.drawLine(cx+5, cy-6, cx-5, cy+6, P_BORDER);
    spr.drawLine(cx-4, cy-6, cx+6, cy+6, P_BORDER);
    spr.drawLine(cx+6, cy-6, cx-4, cy+6, P_BORDER);
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// DRAW HEADER BAR
// ─────────────────────────────────────────────────────────────────────────────
static void drawHeader(const char* title, uint8_t barCol=P_ACCENT, uint8_t txtCol=P_BG) {
  spr.fillRect(MARGIN+1, HDR_Y, SCREEN_W-2*(MARGIN+1), HDR_H, barCol);
  int symCX = SCREEN_W - MARGIN - 12;
  int symCY = HDR_Y + HDR_H/2;
  if (btProvisioningActive) {
    spr.fillRect(symCX-29, HDR_Y+2, 40, HDR_H-4, P_BG);  // wider box for dual symbols
  } else {
    spr.fillRect(symCX-14, HDR_Y+2, 26, HDR_H-4, P_BG);
  }
  drawCommsSymbol(symCX, symCY);
  spr.setTextSize(2);
  spr.setTextColor(txtCol, barCol);
  int16_t tw = strlen(title) * 12;
  int tx = (SCREEN_W - tw) / 2;
  if (tx + tw > SCREEN_W - MARGIN - 32) tx = MARGIN + 6;
  spr.setCursor(tx, HDR_Y + 7);
  spr.print(title);
}

static void sprRight(int rx, int y, int sz, uint8_t col, const char* s) {
  int16_t tw = strlen(s)*6*sz;
  spr.setTextSize(sz); spr.setTextColor(col, P_BG);
  spr.setCursor(rx-tw, y); spr.print(s);
}

static void dataRow(int y, const char* label, const char* value,
                    uint8_t valCol=P_ACCENT, int valSz=2) {
  spr.setTextSize(1); spr.setTextColor(P_BORDER, P_BG);
  spr.setCursor(MARGIN+8, y+2); spr.print(label);
  sprRight(SCREEN_W-MARGIN-8, y, valSz, valCol, value);
}

static void statusBadge(int x, int y, int w, int h,
                         const char* label, uint8_t bg, uint8_t fg) {
  spr.fillRect(x, y, w, h, bg);
  spr.drawRect(x, y, w, h, fg);
  if (bg == P_BORDER) {
    spr.drawRect(x+1, y+1, w-2, h-2, P_BG);
    int16_t tw = strlen(label)*12;
    spr.setTextSize(2); spr.setTextColor(P_BG, P_BORDER);
    spr.setCursor(x+(w-tw)/2, y+(h-16)/2); spr.print(label);
  } else {
    spr.drawRect(x+1, y+1, w-2, h-2, P_BORDER);
    int16_t tw = strlen(label)*12;
    spr.setTextSize(2); spr.setTextColor(P_BORDER, P_BG);
    spr.setCursor(x+(w-tw)/2, y+(h-16)/2); spr.print(label);
  }
}

static void progressBar(int x, int y, int w, int h, int pct, uint8_t col) {
  spr.drawRect(x, y, w, h, P_BORDER);
  int fill = (pct*(w-4))/100;
  if (fill > 0) spr.fillRect(x+2, y+2, fill, h-4, P_BORDER);
}

static void bigCentred(int y, const char* s, int sz, uint8_t col) {
  int16_t tw = strlen(s)*6*sz;
  spr.setTextSize(sz); spr.setTextColor(col, P_BG);
  spr.setCursor((SCREEN_W-tw)/2, y); spr.print(s);
}

static void beginScreen(const char* title, uint8_t accent=P_ACCENT) {
  spr.fillSprite(P_BG);
  drawFrame(P_BORDER);
  drawHeader(title, P_BORDER, P_BG);
  hLine(HDR_Y+HDR_H+2, P_BORDER);
}

// ═══════════════════════════════════════════════════════════════════════════
//  WARNING TRIANGLE  (sprite)
// ═══════════════════════════════════════════════════════════════════════════
static void sprWarningTriangle(int cx, int cy, int size) {
  int h    = (size * 17) / 10;
  int x0   = cx, y0 = cy - (h * 2) / 3;
  int x1   = cx - size, y1 = cy + h / 3;
  int x2   = cx + size, y2 = y1;
  spr.fillTriangle(x0, y0, x1, y1, x2, y2, P_BORDER);
  int inset = 5;
  spr.fillTriangle(cx, y0+inset+3,
                   cx-(size-inset), y1-inset,
                   cx+(size-inset), y1-inset, P_BG);
  int stemX = cx-2, stemY = y0+inset+7;
  int stemH = (y1-inset) - (y0+inset+7) - 8;
  if (stemH > 0) spr.fillRect(stemX, stemY, 4, stemH, P_BORDER);
  spr.fillRect(stemX, y1-inset-7, 4, 4, P_BORDER);
}

// ═══════════════════════════════════════════════════════════════════════════
//  ★ NUETECH LOGO DRAWING ROUTINES  (boot splash — tft.* direct draw)
// ═══════════════════════════════════════════════════════════════════════════

static void thickArc(int cx, int cy, int r,
                     float startDeg, float endDeg,
                     int thickness, uint16_t col)
{
  float step = 1.0f;
  for (float a = startDeg; a <= endDeg; a += step) {
    float x = cx + r * sin(a * DEG_TO_RAD);
    float y = cy - r * cos(a * DEG_TO_RAD);
    tft.fillCircle((int)x, (int)y, thickness, col);
  }
}

static void drawSunSymbol(int cx, int cy, int r, uint16_t col)
{
  thickArc(cx, cy, r, -20, 110, 3, col);

  auto outwardArrow = [&](float deg, int tip, int hw) {
    float rad = deg * DEG_TO_RAD;
    float ux =  sinf(rad);
    float uy = -cosf(rad);
    float px =  uy;
    float py = -ux;
    int ax = cx + (int)((r + tip) * ux);
    int ay = cy + (int)((r + tip) * uy);
    int bx = cx + (int)((r + 10) * ux);
    int by = cy + (int)((r + 10) * uy);
    int x1 = bx + (int)(hw * px);  int y1 = by + (int)(hw * py);
    int x2 = bx - (int)(hw * px);  int y2 = by - (int)(hw * py);
    tft.fillTriangle(ax, ay, x1, y1, x2, y2, col);
  };

  outwardArrow(10.0f, 18, 10);
  outwardArrow(45.0f, 18, 10);
  outwardArrow(75.0f, 18, 10);
}

static void drawNuetechText(int x, int y, uint16_t col)
{
  tft.setTextSize(4);
  tft.setTextColor(col, TFT_BLACK);
  tft.setCursor(x + 55, y + 20);
  tft.print("NUETECH");
}

// ═══════════════════════════════════════════════════════════════════════════
//  BOOT SPLASH SCREEN
// ═══════════════════════════════════════════════════════════════════════════
void showBootSplash()
{
  tft.fillScreen(TFT_BLACK);
  tft.drawRect(0, 0, SCREEN_W, SCREEN_H, TFT_WHITE);
  tft.drawRect(2, 2, SCREEN_W-4, SCREEN_H-4, TFT_WHITE);
  tft.fillRect(0, 0, CORNER, CORNER, TFT_WHITE);
  tft.fillRect(SCREEN_W-CORNER, 0, CORNER, CORNER, TFT_WHITE);
  tft.fillRect(0, SCREEN_H-CORNER, CORNER, CORNER, TFT_WHITE);
  tft.fillRect(SCREEN_W-CORNER, SCREEN_H-CORNER, CORNER, CORNER, TFT_WHITE);

  tft.setTextSize(2);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setCursor((SCREEN_W - 120) / 2, 18);
  tft.print("WELCOME TO");

  int textX = 28;
  int textY = 50;
  drawNuetechText(textX, textY, TFT_WHITE);
  drawSunSymbol(240, 80, 34, TFT_WHITE);

  int sepY = textY + 38;
  tft.drawFastHLine(6, sepY + 20, SCREEN_W - 12, TFT_WHITE);

  tft.setTextSize(2);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setCursor((SCREEN_W - 300) / 2, sepY + 40);
  tft.print("Energy for Changing World");

  int sep2Y = sepY + 34;
  tft.drawFastHLine(6, sep2Y + 40, SCREEN_W - 12, C_DKGRAY);

  tft.setTextSize(2);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setCursor(10, sep2Y + 56);   tft.print("DEVICE: ND_1");
  tft.setCursor(220, sep2Y + 56);  tft.print("FW V2.2");

  tft.drawFastHLine(6, 200, SCREEN_W - 12, TFT_WHITE);
  tft.setTextSize(2);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setCursor(20, 210);
  tft.print("INITIALISING SYSTEMS...");

  playWelcomeBeep();
  unsigned long t0 = millis();
  while (millis() - t0 < 1200) { feedWDT(); }
}

// ═══════════════════════════════════════════════════════════════════════════
//  POPUP  (sprite — 1-bit)
// ═══════════════════════════════════════════════════════════════════════════
void showPopup(const char* msg1, const char* msg2="") {
  if (!sprReady) return;
  spr.fillSprite(P_BG);
  spr.drawRect(0, 0, SCREEN_W, SCREEN_H, P_BORDER);
  spr.drawRect(6, 6, SCREEN_W-4, SCREEN_H-4, P_BORDER);
  spr.drawRect(10, 10, SCREEN_W-12, SCREEN_H-12, P_BORDER);
  spr.fillRect(13, 13, SCREEN_W-18, 24, P_BORDER);
  spr.setTextSize(2); spr.setTextColor(P_BG, P_BORDER);
  spr.setCursor(18, 21); spr.print(">>");
  spr.setCursor((SCREEN_W-14*7)/2, 21); spr.print("NOTIFICATION");
  spr.setCursor(SCREEN_W-30, 21); spr.print("<<");
  spr.setTextSize(2); spr.setTextColor(P_BORDER, P_BG);
  int16_t mw = strlen(msg1)*12;
  spr.setCursor(max(12,(SCREEN_W-mw)/2), 58); spr.print(msg1);
  if (strlen(msg2) > 0) {
    spr.drawFastHLine(MARGIN, 94, SCREEN_W-2*MARGIN, P_BORDER);
    spr.setTextSize(2); spr.setTextColor(P_BORDER, P_BG);
    int16_t m2w = strlen(msg2)*12;
    spr.setCursor(max(12,(SCREEN_W-m2w)/2), 106); spr.print(msg2);
  }
  sprPush();
}

// ═══════════════════════════════════════════════════════════════════════════
//  WIFI STATUS SCREENS  (sprite — 1-bit)
// ═══════════════════════════════════════════════════════════════════════════
void drawWifiConnecting(const String& ssid) {
  beginScreen("  WI-FI  ", P_BORDER);
  bigCentred(CONTENT_Y+8, "CONNECTING...", 2, P_BORDER);
  hLine(78, P_BORDER);
  spr.drawRect(MARGIN+4, 86, SCREEN_W-2*(MARGIN+4), 30, P_BORDER);
  spr.setTextSize(2); spr.setTextColor(P_BORDER, P_BG);
  int16_t sw = ssid.length()*12;
  spr.setCursor(max(MARGIN+8,(SCREEN_W-sw)/2), 93); spr.print(ssid.c_str());
  const char* frames[]={"|","/","-","\\"};
  bigCentred(128, frames[(millis()/200)%4], 4, P_BORDER);
  hLine(185, P_BORDER);
  bigCentred(200, "Please wait...", 2, P_BORDER);
  sprPush();
}

void drawWifiConnected(const String& ip) {
  beginScreen(" WI-FI OK ", P_BORDER);
  spr.fillRect(MARGIN+4, 44, SCREEN_W-2*(MARGIN+4), 54, P_BORDER);
  spr.setTextSize(4); spr.setTextColor(P_BG, P_BORDER);
  spr.setCursor((SCREEN_W-4*24)/2, 54); spr.print("OK");
  hLine(106, P_BORDER);
  bigCentred(114, "DEVICE IP ADDRESS", 2, P_BORDER);
  spr.drawRect(MARGIN+4, 135, SCREEN_W-2*(MARGIN+4), 34, P_BORDER);
  spr.setTextSize(2); spr.setTextColor(P_BORDER, P_BG);
  int16_t iw = ip.length()*12;
  spr.setCursor((SCREEN_W-iw)/2, 143); spr.print(ip.c_str());
  hLine(177, P_BORDER);
  sprPush();
}

void drawWifiFailed() {
  beginScreen(" WI-FI ERROR ", P_BORDER);
  bigCentred(46, "FAILED", 4, P_BORDER);
  hLine(104, P_BORDER);
  bigCentred(125, "Auth / Timeout", 2, P_BORDER);
  bigCentred(154, "Will retry...", 2, P_BORDER);
  sprPush();
}

// ═══════════════════════════════════════════════════════════════════════════
//  ERROR SCREENS  (sprite — 1-bit)
// ═══════════════════════════════════════════════════════════════════════════
static void buildErrorScreen(const char* title1, const char* title2,
                              bool showHint, bool flashBorder) {
  spr.fillSprite(P_BG);
  uint8_t borderCol = P_BORDER;
  if (flashBorder) {
    borderCol = ((millis() / 400) % 2 == 0) ? P_BORDER : P_BG;
  }
  spr.drawRect(0, 0, SCREEN_W, SCREEN_H, P_BORDER);
  spr.drawRect(2, 2, SCREEN_W-4, SCREEN_H-4, borderCol);
  spr.fillRect(3, 3, SCREEN_W-6, 32, P_BORDER);
  uint8_t hdrTxt = P_BG;
  if (flashBorder && ((millis() / 400) % 2 != 0)) hdrTxt = P_BORDER;
  if (hdrTxt == P_BORDER) {
    spr.fillRect(3, 3, SCREEN_W-6, 32, P_BG);
    spr.drawRect(3, 3, SCREEN_W-6, 32, P_BORDER);
  }
  spr.setTextSize(2);
  spr.setTextColor(hdrTxt, (hdrTxt == P_BG) ? P_BORDER : P_BG);
  const char* hdr = "!! FAULT !!";
  int16_t hw = strlen(hdr)*12;
  spr.setCursor((SCREEN_W - hw)/2, 10); spr.print(hdr);
  spr.drawFastHLine(3, 37, SCREEN_W-6, P_BORDER);
  spr.setTextSize(3); spr.setTextColor(P_BORDER, P_BG);
  if (title2 == nullptr || strlen(title2) == 0) {
    int16_t tw = strlen(title1)*18;
    spr.setCursor((SCREEN_W-tw)/2, 50); spr.print(title1);
  } else {
    spr.setCursor((SCREEN_W-strlen(title1)*18)/2, 44); spr.print(title1);
    spr.setCursor((SCREEN_W-strlen(title2)*18)/2, 72); spr.print(title2);
  }
  int triCY = (title2 == nullptr || strlen(title2) == 0) ? 118 : 124;
  sprWarningTriangle(SCREEN_W / 2, triCY, 30);
  int svcY = triCY + 42;
  spr.setTextSize(2); spr.setTextColor(P_BORDER, P_BG);
  const char* svc = "Contact Service Centre";
  int16_t svw = strlen(svc)*12;
  spr.setCursor((SCREEN_W-svw)/2, svcY); spr.print(svc);
  if (showHint) {
    spr.setTextSize(1); spr.setTextColor(P_BORDER, P_BG);
    const char* hint = "HOLD SET FOR 5s TO RESET";
    int16_t htw = strlen(hint)*6;
    spr.setCursor((SCREEN_W-htw)/2, svcY + 20); spr.print(hint);
  }
  spr.drawFastHLine(3, FOOTER_Y-2, SCREEN_W-6, P_BORDER);
  spr.setTextSize(2); spr.setTextColor(P_BORDER, P_BG);
  const char* ftr = "RELAY OFF  |  SYSTEM SAFE";
  int16_t ftw = strlen(ftr)*12;
  spr.setCursor((SCREEN_W-ftw)/2, FOOTER_Y+3); spr.print(ftr);
}

void drawSensorError() {
  if (!sprReady) return;
  buildErrorScreen("ERROR E1", "", false, false);
  sprPush();
}

void drawMaxCoilError() {
  if (!sprReady) return;
  buildErrorScreen("ERROR E2", "", true, true);
  sprPush();
}

void drawCoilNotWorkingError() {
  if (!sprReady) return;
  buildErrorScreen("ERROR E3", "", false, true);
  if (coilErrClearHolding) {
    int svcY = 124 + 42;
    spr.fillRect(3, svcY+18, SCREEN_W-6, 26, P_BG);
    unsigned long held = millis() - coilErrClearHoldStart;
    int pct = (int)((held * 100UL) / 5000UL);
    if (pct > 100) pct = 100;
    char buf[24]; sprintf(buf, "Clearing: %3d%%", pct);
    spr.setTextSize(1); spr.setTextColor(P_BORDER, P_BG);
    int16_t bw = strlen(buf)*6;
    spr.setCursor((SCREEN_W-bw)/2, svcY+20); spr.print(buf);
    int barX = MARGIN+8, barY = svcY+32;
    int barW = SCREEN_W-2*(MARGIN+8);
    spr.drawRect(barX, barY, barW, 10, P_BORDER);
    int fill = (pct*(barW-4))/100;
    if (fill > 0) spr.fillRect(barX+2, barY+2, fill, 6, P_BORDER);
  }
  sprPush();
}

// ═══════════════════════════════════════════════════════════════════════════
//  FORMAT HELPERS
// ═══════════════════════════════════════════════════════════════════════════
static void fmtHHMM(char* buf, unsigned long secs) {
  unsigned long mins = secs / 60;
  int hh = (int)(mins / 60);
  int mm = (int)(mins % 60);
  sprintf(buf, "%dh %02dm", hh, mm);
}

// ═══════════════════════════════════════════════════════════════════════════
//  SCREEN 0 : DATE & TIME
// ═══════════════════════════════════════════════════════════════════════════
void drawDateTimeScreen(int h, int m) {
  beginScreen("  NUETECH  ", P_BORDER);
  DateTime now = rtc.now();
  char timeStr[10]; sprintf(timeStr, "%02d:%02d", h, m);
  bigCentred(50, timeStr, 6, P_BORDER);
  if (!blinkOn()) spr.fillRect(148, 50, 24, 48, P_BG);
  int sec = now.second();
  int dpx=295, dpy=62;
  spr.fillCircle(dpx, dpy,    5, (sec%2==0)?P_BORDER:P_BG);
  spr.fillCircle(dpx, dpy+22, 5, (sec%2==0)?P_BG:P_BORDER);
  spr.drawCircle(dpx, dpy,    5, P_BORDER);
  spr.drawCircle(dpx, dpy+22, 5, P_BORDER);
  hLine(112, P_BORDER);
  char dateStr[22]; sprintf(dateStr, "%02d / %02d / %04d", now.day(), now.month(), now.year());
  bigCentred(120, dateStr, 2, P_BORDER);
  hLine(150, P_BORDER);
  const char* dow[]={"SUNDAY","MONDAY","TUESDAY","WEDNESDAY","THURSDAY","FRIDAY","SATURDAY"};
  bigCentred(168, dow[now.dayOfTheWeek()], 3, P_BORDER);
  if (!timeIsSet) {
    if (blinkOn()) {
      spr.fillRect(MARGIN+3, FOOTER_Y-2, SCREEN_W-2*(MARGIN+3), 20, P_BORDER);
      spr.setTextSize(1); spr.setTextColor(P_BG, P_BORDER);
      spr.setCursor((SCREEN_W-15*6)/2, FOOTER_Y+2); spr.print("! RTC NOT SET !");
    }
  }
  sprPush();
}

// ═══════════════════════════════════════════════════════════════════════════
//  SCREEN 1 : TANK & SET-POINT TEMPERATURES
// ═══════════════════════════════════════════════════════════════════════════
void drawTempScreen() {
  beginScreen("TEMPERATURE", P_BORDER);
  hLine(CONTENT_Y+14, P_BORDER);
  spr.setTextSize(4); spr.setTextColor(P_BORDER, P_BG);
  spr.setCursor(MARGIN+14, CONTENT_Y+26);
  spr.print("Tank :");
  if (RT1 == -999) {
    spr.setCursor(MARGIN+14+13*12+6, CONTENT_Y+26); spr.print("---");
  } else {
    char buf[10]; sprintf(buf, "%d\xF7""C", RT1);
    spr.setCursor(MARGIN+14+13*12+6, CONTENT_Y+26); spr.print(buf);
  }
  hLine(CONTENT_Y+75, P_BORDER);
  spr.setTextSize(4); spr.setTextColor(P_BORDER, P_BG);
  spr.setCursor(MARGIN+14, CONTENT_Y+90);
  spr.print("Set  :");
  char stBuf[10]; sprintf(stBuf, "%d\xF7""C", ST1);
  spr.setCursor(MARGIN+14+13*12+6, CONTENT_Y+90); spr.print(stBuf);
  hLine(CONTENT_Y+134, P_BORDER);
  sprPush();
}

// ═══════════════════════════════════════════════════════════════════════════
//  SCREENS 2 & 3 : MORNING SLOT / EVENING SLOT
// ═══════════════════════════════════════════════════════════════════════════
static void drawSlotScreen(int idx, bool isActive) {
  const char* titles[] = {"MORNING SLOT", "EVENING SLOT"};
  beginScreen(titles[idx], P_BORDER);

  // ── [C6] "RUNNING" indicator footer when slot-lock is active ──────────────
  //    A small blinking banner at the very bottom hints that UP/DOWN browses.
  if (slotScreenLocked && isActive && !slotBrowsing) {
    // Draw the browse-hint banner over the footer area
    spr.fillRect(MARGIN+3, FOOTER_Y-4, SCREEN_W-2*(MARGIN+3), 22, P_BG);
    if (blinkOn()) {
      spr.fillRect(MARGIN+3, FOOTER_Y-4, SCREEN_W-2*(MARGIN+3), 22, P_BORDER);
      spr.setTextSize(2); spr.setTextColor(P_BG, P_BORDER);
      const char* hint = "Use < / > to Browse";
      //int16_t hw2 = strlen(hint)*6;
      spr.setCursor(50, FOOTER_Y+2); spr.print(hint);
    } else {
      spr.drawRect(MARGIN+3, FOOTER_Y-4, SCREEN_W-2*(MARGIN+3), 22, P_BORDER);
      spr.setTextSize(2); spr.setTextColor(P_BORDER, P_BG);
      const char* hint = "Use < / > to Browse";
      //int16_t hw2 = strlen(hint)*6;
      spr.setCursor(50, FOOTER_Y+2); spr.print(hint);
    }
  }

  const char* sLabel = !slotEnabled[idx] ? "     OFF     "
                     : isActive           ? "   RUNNING   "
                                          : "    ON       ";
  if (!slotEnabled[idx]) {
    spr.drawRect(MARGIN+3, CONTENT_Y+2, SCREEN_W-2*(MARGIN+3), 22, P_BORDER);
    spr.setTextSize(2); spr.setTextColor(P_BORDER, P_BG);
  } else if (isActive) {
    spr.fillRect(MARGIN+3, CONTENT_Y+2, SCREEN_W-2*(MARGIN+3), 22, P_BORDER);
    spr.setTextSize(2); spr.setTextColor(P_BG, P_BORDER);
  } else {
    spr.drawRect(MARGIN+3, CONTENT_Y+2, SCREEN_W-2*(MARGIN+3), 22, P_BORDER);
    spr.setTextSize(2); spr.setTextColor(P_BORDER, P_BG);
  }
  spr.setCursor((SCREEN_W-strlen(sLabel)*12)/2, CONTENT_Y+6); spr.print(sLabel);
  hLine(CONTENT_Y+30, P_BORDER);
  spr.setTextSize(3); spr.setTextColor(P_BORDER, P_BG);
  spr.setCursor(MARGIN+14, CONTENT_Y+60); spr.print("Start Time:");
  char sBuf[8]; sprintf(sBuf, "%02d:%02d", S_H[idx], S_M[idx]);
  spr.setCursor(SCREEN_W-MARGIN-40-strlen(sBuf)*12, CONTENT_Y+60); spr.print(sBuf);
  hLine(CONTENT_Y+100, P_BORDER);
  spr.setTextSize(3); spr.setTextColor(P_BORDER, P_BG);
  spr.setCursor(MARGIN+14, CONTENT_Y+130); spr.print("End   Time:");
  char eBuf[8]; sprintf(eBuf, "%02d:%02d", E_H[idx], E_M[idx]);
  spr.setCursor(SCREEN_W-MARGIN-40-strlen(eBuf)*12, CONTENT_Y+130); spr.print(eBuf);
  hLine(CONTENT_Y+170, P_BORDER);
  sprPush();
}

void drawSlot1Screen(bool isActive) { drawSlotScreen(0, isActive); }
void drawSlot2Screen(bool isActive) { drawSlotScreen(1, isActive); }

// ═══════════════════════════════════════════════════════════════════════════
//  SCREEN 4 : AMBIENT ENVIRONMENT (AHT20)
// ═══════════════════════════════════════════════════════════════════════════
void drawAmbientScreen() {
  beginScreen("Environment Temp", P_BORDER);
  if (!ahtOk) {
    spr.fillRect(MARGIN+4, 50, SCREEN_W-2*(MARGIN+4), 80, P_BORDER);
    spr.setTextSize(4); spr.setTextColor(P_BG, P_BORDER);
    spr.setCursor((SCREEN_W-24)/2, 62); spr.print("!");
    bigCentred(140, "AHT20  ERROR", 2, P_BORDER);
    sprPush(); return;
  }
  hLine(CONTENT_Y+14, P_BORDER);
  spr.setTextSize(3); spr.setTextColor(P_BORDER, P_BG);
  spr.setCursor(MARGIN+14, CONTENT_Y+34); spr.print("Temp:");
  char tbuf[18]; dtostrf(ahtTemp, 5, 1, tbuf); strcat(tbuf, " \xF7""C");
  spr.setCursor(MARGIN+14+7*12+6, CONTENT_Y+34); spr.print(tbuf);
  hLine(CONTENT_Y+70, P_BORDER);
  spr.setTextSize(3); spr.setTextColor(P_BORDER, P_BG);
  spr.setCursor(MARGIN+14, CONTENT_Y+94); spr.print("Humd:");
  char hbuf[18]; dtostrf(ahtHumidity, 5, 1, hbuf); strcat(hbuf, " %");
  spr.setCursor(MARGIN+14+7*12+6, CONTENT_Y+94); spr.print(hbuf);
  hLine(CONTENT_Y+130, P_BORDER);
  sprPush();
}

// ═══════════════════════════════════════════════════════════════════════════
//  PIN ENTRY SCREEN
// ═══════════════════════════════════════════════════════════════════════════
void drawPinScreen() {
  beginScreen("< ENTER PIN >", P_BORDER);
  char buf[10]; sprintf(buf, "%d", pinEnteredVal);
  if (blinkOn()) {
    spr.fillRect(MARGIN+4, 78, SCREEN_W-2*(MARGIN+4), 74, P_BORDER);
    spr.setTextSize(6); spr.setTextColor(P_BG, P_BORDER);
    int16_t nw = strlen(buf)*36;
    spr.setCursor((SCREEN_W-nw)/2, 90); spr.print(buf);
  } else {
    spr.drawRect(MARGIN+4, 78, SCREEN_W-2*(MARGIN+4), 74, P_BORDER);
    bigCentred(90, buf, 6, P_BORDER);
  }
  hLine(170, P_BORDER);
  spr.setTextSize(2); spr.setTextColor(P_BORDER, P_BG);
  spr.setCursor(MARGIN+8, 178); spr.print("Authorized Person Only.!");
  sprPush();
}

// ═══════════════════════════════════════════════════════════════════════════
//  MANUAL MODE FOOTER  (dot indicators + hint text)
// ═══════════════════════════════════════════════════════════════════════════
static void drawManualFooter(int currentItem) {
  int sp = (SCREEN_W-2*(MARGIN+8))/MANUAL_ITEMS;
  for (int d = 0; d < MANUAL_ITEMS; d++) {
    int dx = MARGIN+8+d*sp+sp/2;
    if (d==currentItem) spr.fillRect(dx-3, 211, 6, 6, P_BORDER);
    else                spr.drawRect(dx-3, 211, 6, 6, P_BORDER);
  }
  spr.setTextSize(2); spr.setTextColor(P_BORDER, P_BG);
  const char* hint = "< SET/SEL >";
  spr.setCursor((SCREEN_W-strlen(hint)*12)/2, 222); spr.print(hint);
}

// ═══════════════════════════════════════════════════════════════════════════
//  MANUAL MODE SCREEN  (12 items)
// ═══════════════════════════════════════════════════════════════════════════
void drawManual() {
  static const char* labels[] = {
    "Set Temp",
    "Morning Slot Start",
    "Morning Slot End",
    "Evening Slot Start",
    "Evening Slot End",
    "Daily Usage",
    "Consumption",
    "Coil Check Temp",
    "Coil Min Rise",
    "PRE-HEAT",
    "Morning Slot En",
    "Evening Slot En",
    "Network Mode"
  };

  if (pinState == PIN_ENTERING) { drawPinScreen(); return; }

  if (editItem == 5) {
    beginScreen("DAILY USAGE", P_BORDER);
    char buf0[16], buf1[16], bufT[16];
    fmtHHMM(buf0, slotDailyOnTime[0]);
    fmtHHMM(buf1, slotDailyOnTime[1]);
    fmtHHMM(bufT, slotDailyOnTime[0] + slotDailyOnTime[1]);
    hLine(CONTENT_Y+2, P_BORDER);
    spr.setTextSize(2); spr.setTextColor(P_BORDER, P_BG);
    spr.setCursor(MARGIN+10, CONTENT_Y+12); spr.print("MORNING  :");
    sprRight(SCREEN_W-MARGIN-10, CONTENT_Y+12, 2, P_BORDER, buf0);
    hLine(CONTENT_Y+36, P_BORDER);
    spr.setCursor(MARGIN+10, CONTENT_Y+46); spr.print("EVENING  :");
    sprRight(SCREEN_W-MARGIN-10, CONTENT_Y+46, 2, P_BORDER, buf1);
    hLine(CONTENT_Y+70, P_BORDER);
    spr.setCursor(MARGIN+10, CONTENT_Y+80); spr.print("TOTAL    :");
    sprRight(SCREEN_W-MARGIN-10, CONTENT_Y+80, 2, P_BORDER, bufT);
    hLine(CONTENT_Y+104, P_BORDER);
    drawManualFooter(editItem);
    sprPush(); return;
  }

  if (editItem == 6) {
    beginScreen("CONSUMPTION", P_BORDER);
    unsigned long totalSecs = slotDailyOnTime[0] + slotDailyOnTime[1];
    char bufOT[16]; fmtHHMM(bufOT, totalSecs);
    float totalH = totalSecs / 3600.0f;
    float kWh = totalH * 2.0f;
    float co2 = kWh * 0.8f;
    char bufKWh[16], bufCO2[16];
    dtostrf(kWh, 5, 1, bufKWh); strcat(bufKWh, " kWh");
    dtostrf(co2, 5, 1, bufCO2); strcat(bufCO2, " kg");
    hLine(CONTENT_Y+2, P_BORDER);
    spr.setTextSize(2); spr.setTextColor(P_BORDER, P_BG);
    spr.setCursor(MARGIN+10, CONTENT_Y+12); spr.print("ON-TIME :");
    sprRight(SCREEN_W-MARGIN-10, CONTENT_Y+12, 2, P_BORDER, bufOT);
    hLine(CONTENT_Y+36, P_BORDER);
    spr.setCursor(MARGIN+10, CONTENT_Y+46); spr.print("ENERGY  :");
    sprRight(SCREEN_W-MARGIN-10, CONTENT_Y+46, 2, P_BORDER, bufKWh);
    hLine(CONTENT_Y+70, P_BORDER);
    spr.setCursor(MARGIN+10, CONTENT_Y+80); spr.print("CO2     :");
    sprRight(SCREEN_W-MARGIN-10, CONTENT_Y+80, 2, P_BORDER, bufCO2);
    hLine(CONTENT_Y+104, P_BORDER);
    drawManualFooter(editItem);
    sprPush(); return;
  }

  if (editItem == 12) {
    beginScreen("NETWORK MODE", P_BORDER);
    hLine(CONTENT_Y+2, P_BORDER);
    bigCentred(CONTENT_Y+8, "CURRENT MODE", 2, P_BORDER);
    spr.fillRect(50, 68, 220, 38, P_BORDER);
    spr.drawRect(51, 69, 218, 36, P_BG);
    spr.setTextSize(2); spr.setTextColor(P_BG, P_BORDER);
    const char* curLabel = (commsMode==MODE_BLE) ? "  BLUETOOTH  " : "    Wi-Fi    ";
    spr.setCursor(50+(220-strlen(curLabel)*12)/2, 68+(38-16)/2); spr.print(curLabel);
    hLine(116, P_BORDER);
    bigCentred(122, "SET TO SWITCH:", 2, P_BORDER);
    const char* tgtLabel = (commsMode==MODE_BLE) ? "    Wi-Fi    " : "  BLUETOOTH  ";
    if (blinkOn()) {
      spr.fillRect(50, 148, 220, 36, P_BORDER);
      spr.drawRect(51, 149, 218, 34, P_BG);
      spr.setTextSize(2); spr.setTextColor(P_BG, P_BORDER);
      spr.setCursor(50+(220-strlen(tgtLabel)*12)/2, 148+(36-16)/2); spr.print(tgtLabel);
    } else {
      spr.drawRect(50, 148, 220, 36, P_BORDER);
      spr.setTextSize(2); spr.setTextColor(P_BORDER, P_BG);
      spr.setCursor(50+(220-strlen(tgtLabel)*12)/2, 148+(36-16)/2); spr.print(tgtLabel);
    }
    hLine(194, P_BORDER);
    drawManualFooter(editItem);
    sprPush(); return;
  }

  beginScreen("< MANUAL >", P_BORDER);
  bigCentred(CONTENT_Y+4, labels[editItem], 2, P_BORDER);
  hLine(CONTENT_Y+26, P_BORDER);

  char buf[32];

  if (editItem == 0) {
    if (!editActive || blinkOn()) {
      sprintf(buf, "%d \xF7""C", ST1);
      bigCentred(CONTENT_Y+55, buf, 5, P_BORDER);
    }
    hLine(162, P_BORDER);
    progressBar(MARGIN+8, 170, SCREEN_W-2*(MARGIN+8), 16,
                ((ST1-TEMP_MIN)*100)/(TEMP_MAX-TEMP_MIN), P_BORDER);
    spr.setTextSize(2); spr.setTextColor(P_BORDER, P_BG);
    spr.setCursor(MARGIN+10, 192); spr.print("20");
    sprRight(SCREEN_W-MARGIN-10, 192, 2, P_BORDER, "60");

  } else if (editItem >= 1 && editItem <= 4) {
    int i = (editItem-1)/2; bool start = (editItem%2) == 1;
    int H = start ? S_H[i] : E_H[i];
    int M = start ? S_M[i] : E_M[i];
    bool hBlink = editActive && editField==0;
    bool mBlink = editActive && editField==1;
    char hBuf[4], mBuf[4]; sprintf(hBuf, "%02d", H); sprintf(mBuf, "%02d", M);
    spr.setTextSize(6);
    if (!hBlink || blinkOn()) {
      if (hBlink) {
        spr.fillRect(22, 86, 108, 56, P_BORDER);
        spr.setTextColor(P_BG, P_BORDER); spr.setCursor(30, 92); spr.print(hBuf);
      } else {
        spr.setTextColor(P_BORDER, P_BG); spr.setCursor(30, 92); spr.print(hBuf);
      }
    }
    spr.setTextSize(5); spr.setTextColor(P_BORDER, P_BG);
    spr.setCursor(148, 100); spr.print(":");
    spr.setTextSize(6);
    if (!mBlink || blinkOn()) {
      if (mBlink) {
        spr.fillRect(172, 86, 120, 56, P_BORDER);
        spr.setTextColor(P_BG, P_BORDER); spr.setCursor(178, 92); spr.print(mBuf);
      } else {
        spr.setTextColor(P_BORDER, P_BG); spr.setCursor(178, 92); spr.print(mBuf);
      }
    }
    hLine(156, P_BORDER);
    if (editField==0) spr.fillRect(78,  164, 10, 10, P_BORDER);
    else              spr.drawRect(78,  164, 10, 10, P_BORDER);
    if (editField==1) spr.fillRect(230, 164, 10, 10, P_BORDER);
    else              spr.drawRect(230, 164, 10, 10, P_BORDER);

  } else if (editItem == 7) {
    if (!editActive || blinkOn()) {
      sprintf(buf, "%d min", coilEffCheckMinutes);
      bigCentred(CONTENT_Y+55, buf, 4, P_BORDER);
    }
    hLine(162, P_BORDER);
    progressBar(MARGIN+8, 170, SCREEN_W-2*(MARGIN+8), 14,
                ((coilEffCheckMinutes-5)*100)/(120-5), P_BORDER);
    spr.setTextSize(2);
    spr.setCursor(MARGIN+10, 192); spr.print("5 min");
    sprRight(SCREEN_W-MARGIN-10, 192, 2, P_BORDER, "120 min");

  } else if (editItem == 8) {
    if (!editActive || blinkOn()) {
      sprintf(buf, "%d \xF7""C", coilEffMinRiseDeg);
      bigCentred(CONTENT_Y+55, buf, 4, P_BORDER);
    }
    hLine(162, P_BORDER);
    progressBar(MARGIN+8, 170, SCREEN_W-2*(MARGIN+8), 14,
                ((coilEffMinRiseDeg-1)*100)/9, P_BORDER);
    spr.setTextSize(2);
    spr.setCursor(MARGIN+10, 192); spr.print("1 \xF7""C");
    sprRight(SCREEN_W-MARGIN-10, 192, 2, P_BORDER, "10 \xF7""C");

  } else if (editItem == 9) {
    hLine(CONTENT_Y+28, P_BORDER);
    if (!editActive || blinkOn()) {
      if (preHeatSteps == 0) {
        bigCentred(CONTENT_Y+60, "OFF", 4, P_BORDER);
      } else {
        int hours = preHeatSteps / 2;
        int mins = (preHeatSteps % 2) * 30;
        if (hours > 0 && mins > 0) {
          sprintf(buf, "%d hr %d min", hours, mins);
        } else if (hours > 0) {
          sprintf(buf, "%d hr", hours);
        } else {
          sprintf(buf, "%d min", mins);
        }
        bigCentred(CONTENT_Y+60, buf, 4, P_BORDER);
      }
    }
    hLine(162, P_BORDER);
    progressBar(MARGIN+8, 170, SCREEN_W-2*(MARGIN+8), 14,
                (preHeatSteps*100)/4, P_BORDER);
    spr.setTextSize(2); spr.setTextColor(P_BORDER, P_BG);
    spr.setCursor(MARGIN+10, 190); spr.print("OFF");
    sprRight(SCREEN_W-MARGIN-10, 190, 2, P_BORDER, "2 hr");

  } else if (editItem == 10) {
    bigCentred(CONTENT_Y+40, "ENABLE / DISABLE", 2, P_BORDER);
    hLine(CONTENT_Y+76, P_BORDER);
    if (slotEnabled[0]) {
      spr.fillRect(50, CONTENT_Y+86, 220, 48, P_BORDER);
      spr.setTextSize(3); spr.setTextColor(P_BG, P_BORDER);
      spr.setCursor(50+(220-3*18)/2, CONTENT_Y+86+15); spr.print("ON");
    } else {
      spr.drawRect(50, CONTENT_Y+86, 220, 48, P_BORDER);
      spr.setTextSize(3); spr.setTextColor(P_BORDER, P_BG);
      spr.setCursor(50+(220-3*18)/2, CONTENT_Y+86+15); spr.print("OFF");
    }

  } else if (editItem == 11) {
    bigCentred(CONTENT_Y+40, "ENABLE / DISABLE", 2, P_BORDER);
    hLine(CONTENT_Y+76, P_BORDER);
    if (slotEnabled[1]) {
      spr.fillRect(50, CONTENT_Y+86, 220, 48, P_BORDER);
      spr.setTextSize(3); spr.setTextColor(P_BG, P_BORDER);
      spr.setCursor(50+(220-3*18)/2, CONTENT_Y+86+15); spr.print("ON");
    } else {
      spr.drawRect(50, CONTENT_Y+86, 220, 48, P_BORDER);
      spr.setTextSize(3); spr.setTextColor(P_BORDER, P_BG);
      spr.setCursor(50+(220-3*18)/2, CONTENT_Y+86+15); spr.print("OFF");
    }

  } else {
    if (buttonPressed(BTN_SET)) { editActive = false; }
  }

  if (!editActive) {
    drawManualFooter(editItem);
  }
  sprPush();
}

// ═══════════════════════════════════════════════════════════════════════════
//  EEPROM  — AT24C32 I²C helpers (address 0x50)
// ═══════════════════════════════════════════════════════════════════════════

// ── Probe: returns true if AT24C32 responds on the I²C bus ───────────────
bool eepromPresent() {
  Wire.beginTransmission(EEPROM_I2C_ADDR);
  return (Wire.endTransmission() == 0);
}

// ── Write a single byte to EEPROM ────────────────────────────────────────
void eepromWriteByte(uint16_t addr, uint8_t data) {
  Wire.beginTransmission(EEPROM_I2C_ADDR);
  Wire.write((uint8_t)(addr >> 8));   // address high byte
  Wire.write((uint8_t)(addr & 0xFF)); // address low byte
  Wire.write(data);
  Wire.endTransmission();
  delay(5); // AT24C32 write cycle time
}

// ── Read a single byte from EEPROM ───────────────────────────────────────
uint8_t eepromReadByte(uint16_t addr) {
  Wire.beginTransmission(EEPROM_I2C_ADDR);
  Wire.write((uint8_t)(addr >> 8));
  Wire.write((uint8_t)(addr & 0xFF));
  Wire.endTransmission();
  Wire.requestFrom((uint8_t)EEPROM_I2C_ADDR, (uint8_t)1);
  return Wire.available() ? Wire.read() : 0xFF;
}

// ── Write a block of bytes (page-aware, handles page boundary crossing) ──
void eepromWriteBlock(uint16_t addr, const uint8_t* data, uint8_t len) {
  uint8_t pos = 0;
  while (pos < len) {
    // Bytes remaining in the current 32-byte page
    uint8_t pageRemain = EEPROM_PAGE_SIZE - ((addr + pos) % EEPROM_PAGE_SIZE);
    uint8_t chunk = min((uint8_t)(len - pos), pageRemain);

    Wire.beginTransmission(EEPROM_I2C_ADDR);
    Wire.write((uint8_t)((addr + pos) >> 8));
    Wire.write((uint8_t)((addr + pos) & 0xFF));
    for (uint8_t i = 0; i < chunk; i++) Wire.write(data[pos + i]);
    Wire.endTransmission();
    delay(5); // wait for write cycle

    pos += chunk;
  }
}

// ── Read a block of bytes from EEPROM ────────────────────────────────────
void eepromReadBlock(uint16_t addr, uint8_t* data, uint8_t len) {
  uint8_t pos = 0;
  while (pos < len) {
    uint8_t chunk = min((uint8_t)(len - pos), (uint8_t)30); // Wire buffer limit
    Wire.beginTransmission(EEPROM_I2C_ADDR);
    Wire.write((uint8_t)((addr + pos) >> 8));
    Wire.write((uint8_t)((addr + pos) & 0xFF));
    Wire.endTransmission();
    Wire.requestFrom((uint8_t)EEPROM_I2C_ADDR, chunk);
    for (uint8_t i = 0; i < chunk && Wire.available(); i++) {
      data[pos + i] = Wire.read();
    }
    pos += chunk;
  }
}

// ═══════════════════════════════════════════════════════════════════════════
//  EEPROM LOG MANAGEMENT  — 31-day circular buffer
// ═══════════════════════════════════════════════════════════════════════════

// ── Initialize EEPROM header if blank or corrupted ───────────────────────
void initLogHeader() {
  uint8_t hdr[LOG_HEADER_SIZE];
  eepromReadBlock(0, hdr, LOG_HEADER_SIZE);
  uint16_t magic = ((uint16_t)hdr[0] << 8) | hdr[1];
  if (magic != LOG_MAGIC || hdr[2] != LOG_VERSION) {
    // EEPROM is blank or corrupted — initialize header
    memset(hdr, 0, LOG_HEADER_SIZE);
    hdr[0] = (LOG_MAGIC >> 8) & 0xFF;  // 'N'
    hdr[1] = LOG_MAGIC & 0xFF;         // 'U'
    hdr[2] = LOG_VERSION;              // version
    hdr[3] = 0;                        // write index (next slot)
    hdr[4] = 0;                        // record count
    eepromWriteBlock(0, hdr, LOG_HEADER_SIZE);
    Serial.println("[EEPROM] Header initialized.");
  } else {
    Serial.printf("[EEPROM] Header OK. Records: %d, Next slot: %d\n", hdr[4], hdr[3]);
  }
}

// ── Save today's daily log record to EEPROM ──────────────────────────────
void saveDailyLog() {
  if (!eepromFound) return;

  // Read current header
  uint8_t hdr[LOG_HEADER_SIZE];
  eepromReadBlock(0, hdr, LOG_HEADER_SIZE);
  uint8_t writeIdx    = hdr[3];
  uint8_t recordCount = hdr[4];

  // Get current date from RTC
  DateTime now = rtc.now();

  // Build the record
  DailyLogRecord rec;
  rec.day           = now.day();
  rec.month         = now.month();
  rec.yearOff       = (uint8_t)(now.year() - 2020);
  rec.mornOnTimeSec = (uint16_t)min(slotDailyOnTime[0], 65535UL);
  rec.mornPeakTemp  = (uint8_t)constrain(mornPeakTemp, 0, 255);
  rec.eveOnTimeSec  = (uint16_t)min(slotDailyOnTime[1], 65535UL);
  rec.evePeakTemp   = (uint8_t)constrain(evePeakTemp, 0, 255);
  rec.ambientTemp   = (uint8_t)constrain((int)(ahtTemp * 2.0f), 0, 255);
  rec.errorFlags    = dayErrorAccum;

  // Serialize to byte array (manual packing for portability)
  uint8_t buf[LOG_RECORD_SIZE];
  buf[0]  = rec.day;
  buf[1]  = rec.month;
  buf[2]  = rec.yearOff;
  buf[3]  = (uint8_t)(rec.mornOnTimeSec & 0xFF);
  buf[4]  = (uint8_t)(rec.mornOnTimeSec >> 8);
  buf[5]  = rec.mornPeakTemp;
  buf[6]  = (uint8_t)(rec.eveOnTimeSec & 0xFF);
  buf[7]  = (uint8_t)(rec.eveOnTimeSec >> 8);
  buf[8]  = rec.evePeakTemp;
  buf[9]  = rec.ambientTemp;
  buf[10] = rec.errorFlags;
  buf[11] = 0; // reserved / padding

  // Write record at circular index
  uint16_t addr = LOG_DATA_START + (uint16_t)writeIdx * LOG_RECORD_SIZE;
  eepromWriteBlock(addr, buf, LOG_RECORD_SIZE);

  // Update header: advance write index, cap record count
  writeIdx = (writeIdx + 1) % LOG_MAX_RECORDS;
  if (recordCount < LOG_MAX_RECORDS) recordCount++;
  hdr[3] = writeIdx;
  hdr[4] = recordCount;
  eepromWriteBlock(0, hdr, LOG_HEADER_SIZE);

  Serial.printf("[LOG] Saved daily record (slot %d). Records stored: %d\n",
                (writeIdx == 0) ? LOG_MAX_RECORDS - 1 : writeIdx - 1, recordCount);
}

// ── Build JSON string with the last N daily log records ──────────────────
String buildLogJson(int maxDays) {
  if (!eepromFound) return "{\"error\":\"EEPROM not found\"}";

  uint8_t hdr[LOG_HEADER_SIZE];
  eepromReadBlock(0, hdr, LOG_HEADER_SIZE);
  uint8_t writeIdx    = hdr[3];
  uint8_t recordCount = hdr[4];

  int count = min((int)recordCount, maxDays);
  if (count == 0) return "{\"type\":\"logs\",\"count\":0,\"records\":[]}";

  String json = "{\"type\":\"logs\",\"count\":" + String(count) + ",\"records\":[";

  for (int i = 0; i < count; i++) {
    // Read records in reverse chronological order (newest first)
    int idx = ((int)writeIdx - 1 - i + LOG_MAX_RECORDS) % LOG_MAX_RECORDS;
    uint16_t addr = LOG_DATA_START + (uint16_t)idx * LOG_RECORD_SIZE;
    uint8_t buf[LOG_RECORD_SIZE];
    eepromReadBlock(addr, buf, LOG_RECORD_SIZE);

    // Deserialize
    uint8_t  day   = buf[0];
    uint8_t  month = buf[1];
    uint16_t year  = 2020 + buf[2];
    uint16_t mornSec  = (uint16_t)buf[3] | ((uint16_t)buf[4] << 8);
    uint8_t  mornPeak = buf[5];
    uint16_t eveSec   = (uint16_t)buf[6] | ((uint16_t)buf[7] << 8);
    uint8_t  evePeak  = buf[8];
    float    ambT     = buf[9] / 2.0f;
    uint8_t  errF     = buf[10];

    if (i > 0) json += ",";
    char dateBuf[12];
    snprintf(dateBuf, sizeof(dateBuf), "%04d-%02d-%02d", year, month, day);

    // Format on-times as HH:MM
    char mornFmt[8], eveFmt[8];
    snprintf(mornFmt, sizeof(mornFmt), "%02d:%02d", mornSec / 3600, (mornSec % 3600) / 60);
    snprintf(eveFmt, sizeof(eveFmt), "%02d:%02d", eveSec / 3600, (eveSec % 3600) / 60);

    json += "{\"date\":\"" + String(dateBuf) + "\"";
    json += ",\"mornSec\":" + String(mornSec);
    json += ",\"mornFmt\":\"" + String(mornFmt) + "\"";
    json += ",\"mornPeakC\":" + String(mornPeak);
    json += ",\"eveSec\":" + String(eveSec);
    json += ",\"eveFmt\":\"" + String(eveFmt) + "\"";
    json += ",\"evePeakC\":" + String(evePeak);
    json += ",\"ambientC\":" + String(ambT, 1);
    json += ",\"errors\":" + String(errF) + "}";
  }

  json += "]}";
  return json;
}

// ── Clear all log data (reset header) ────────────────────────────────────
void clearAllLogs() {
  if (!eepromFound) return;
  uint8_t hdr[LOG_HEADER_SIZE];
  memset(hdr, 0, LOG_HEADER_SIZE);
  hdr[0] = (LOG_MAGIC >> 8) & 0xFF;
  hdr[1] = LOG_MAGIC & 0xFF;
  hdr[2] = LOG_VERSION;
  hdr[3] = 0;  // write index
  hdr[4] = 0;  // record count
  eepromWriteBlock(0, hdr, LOG_HEADER_SIZE);
  Serial.println("[EEPROM] All logs cleared.");
}

// ═══════════════════════════════════════════════════════════════════════════
//  NVS  — save / load all settings
// ═══════════════════════════════════════════════════════════════════════════
void saveAll() {
  prefs.begin("slots", false);
  prefs.putInt("ST1", ST1);
  prefs.putULong("COILTIME", coilOnTime);
  prefs.putInt("COILCHKMIN", coilEffCheckMinutes);
  prefs.putInt("COILRISE", coilEffMinRiseDeg);
  prefs.putInt("PREHEATSTEP", preHeatSteps);
  char k[10];
  for (int i = 0; i < 2; i++) {
    snprintf(k,10,"SH%d",i);   prefs.putInt(k, S_H[i]);
    snprintf(k,10,"SM%d",i);   prefs.putInt(k, S_M[i]);
    snprintf(k,10,"EH%d",i);   prefs.putInt(k, E_H[i]);
    snprintf(k,10,"EM%d",i);   prefs.putInt(k, E_M[i]);
    snprintf(k,10,"ONT%d",i);  prefs.putULong(k, slotDailyOnTime[i]);
    snprintf(k,10,"SLEN%d",i); prefs.putBool(k, slotEnabled[i]);
  }
  prefs.end();
}

void loadAll() {
  prefs.begin("slots", true);
  ST1                = prefs.getInt("ST1", 50);
  coilOnTime         = prefs.getULong("COILTIME", 0);
  coilEffCheckMinutes = constrain(prefs.getInt("COILCHKMIN", 20), 5, 120);
  coilEffCheckDurationMs = (unsigned long)coilEffCheckMinutes * 60000UL;
  coilEffMinRiseDeg = constrain(prefs.getInt("COILRISE", 1), 1, 10);
  if (prefs.isKey("PREHEATSTEP")) {
    preHeatSteps = constrain(prefs.getInt("PREHEATSTEP", 0), 0, 4);
  } else {
    int oldMin = constrain(prefs.getInt("PREHEAT", 0), 0, 60);
    preHeatSteps = constrain(oldMin / 30, 0, 4);
  }
  char k[10];
  for (int i = 0; i < 2; i++) {
    snprintf(k,10,"SH%d",i);   S_H[i]          = constrain(prefs.getInt(k, i==0?1:13),  slotMinHour(i), slotMaxHour(i));
    snprintf(k,10,"SM%d",i);   S_M[i]          = constrain(prefs.getInt(k, 0),            0, 59);
    snprintf(k,10,"EH%d",i);   E_H[i]          = constrain(prefs.getInt(k, i==0?12:23),  slotMinHour(i), slotMaxHour(i));
    snprintf(k,10,"EM%d",i);   E_M[i]          = constrain(prefs.getInt(k, 0),            0, 59);
    snprintf(k,10,"ONT%d",i);  slotDailyOnTime[i] = prefs.getULong(k, 0);
    snprintf(k,10,"SLEN%d",i); slotEnabled[i]  = prefs.getBool(k, true);
  }
  prefs.end();
  if (coilOnTime >= maxCoilOnTime) { maxCoilError = true; coilErrorAcknowledged = false; }
}

void saveWifiCreds(const String& ssid, const String& pass) {
  prefs.begin("wifi", false);
  prefs.putString("ssid", ssid);
  prefs.putString("pass", pass);
  prefs.end();
}
void loadWifiCreds() {
  prefs.begin("wifi", true);
  wifiSSID     = prefs.getString("ssid", "LAB");
  wifiPassword = prefs.getString("pass", "12345678");
  prefs.end();
}

// ═══════════════════════════════════════════════════════════════════════════
//  RTC HELPERS
// ═══════════════════════════════════════════════════════════════════════════
void getTime(int &h, int &m) {
  DateTime now = rtc.now();
  if (!rtc.lostPower()) { timeIsSet = true;  h = now.hour(); m = now.minute(); }
  else                  { timeIsSet = false; h = 0; m = 0; }
}

void setRTCTime(int h, int m) {
  DateTime now = rtc.now();
  rtc.adjust(DateTime(now.year(), now.month(), now.day(), h, m, now.second()));
  timeIsSet = true;
}

// ═══════════════════════════════════════════════════════════════════════════
//  NTP SYNC
// ═══════════════════════════════════════════════════════════════════════════
void syncRtcFromNtp() {
  if (commsMode != MODE_WIFI) return;
  if (WiFi.status() != WL_CONNECTED) return;
  Serial.println("[NTP] Syncing RTC from NTP...");
  configTime(NTP_GMT_OFFSET_SEC, NTP_DAYLIGHT_OFFSET, NTP_SERVER1, NTP_SERVER2);
  struct tm timeinfo;
  unsigned long t0 = millis();
  while (!getLocalTime(&timeinfo, 1000) && millis()-t0 < 8000) { feedWDT(); delay(200); }
  if (getLocalTime(&timeinfo, 500)) {
    DateTime dt(timeinfo.tm_year+1900, timeinfo.tm_mon+1, timeinfo.tm_mday,
                timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
    rtc.adjust(dt);
    timeIsSet = true; lastNtpSyncMs = millis();
    Serial.printf("[NTP] RTC set to %04d-%02d-%02d %02d:%02d:%02d\n",
                  dt.year(), dt.month(), dt.day(), dt.hour(), dt.minute(), dt.second());
    showPopup("NTP Synced", deviceIP.c_str());
    unsigned long s = millis(); while (millis()-s < 1200) { feedWDT(); }
  } else {
    Serial.println("[NTP] Sync failed.");
  }
}

// ═══════════════════════════════════════════════════════════════════════════
//  SCHEDULE HELPERS
// ═══════════════════════════════════════════════════════════════════════════
bool isValidSchedule(int slot) {
  if (!slotEnabled[slot]) return false;
  int startMin = S_H[slot]*60 + S_M[slot];
  int endMin   = E_H[slot]*60 + E_M[slot];
  // Enforce hour range: morning 01–12, evening 13–23
  int minH = slotMinHour(slot), maxH = slotMaxHour(slot);
  if (S_H[slot] < minH || S_H[slot] > maxH) return false;
  if (E_H[slot] < minH || E_H[slot] > maxH) return false;
  // End must be at least 30 min after start
  return (endMin - startMin) >= MIN_SLOT_GAP_MINS;
}

int activeSlot(int h, int m) {
  int now = h*60 + m;
  for (int i = 0; i < 2; i++) {
    if (!isValidSchedule(i)) continue;
    if (now >= S_H[i]*60+S_M[i] && now <= E_H[i]*60+E_M[i]) return i;
  }
  return -1;
}

int preheatSlot(int h, int m) {
  int now = h*60 + m;
  int offset = (preHeatMinutes() > 0) ? preHeatMinutes() : 1;
  for (int i = 0; i < 2; i++) {
    if (!isValidSchedule(i)) continue;
    int slotStart = S_H[i]*60 + S_M[i];
    int preStart  = slotStart - offset;
    if (preStart < 0) preStart += 24*60;
    if (preStart <= now && now < slotStart) return i;
  }
  return -1;
}

void updateHysteresis() {
  if (RT1 == -999) return;
  if (tempHysteresisReady) { if (RT1 >= ST1) tempHysteresisReady = false; }
  else                     { if (RT1 <= ST1 - TEMP_HYSTERESIS) tempHysteresisReady = true; }
}

// ═══════════════════════════════════════════════════════════════════════════
//  COIL TRACKING
// ═══════════════════════════════════════════════════════════════════════════
void updateCoilOnTime(bool relayIsOn) {
  unsigned long now = millis();
  if (relayIsOn) {
    relayOffTimerActive = false;
    if (!coilWasOn) { coilOnStart = now; coilWasOn = true; }
    if (now - lastCoilSaveTime > 5000) {
      coilOnTime += (now - coilOnStart); coilOnStart = now;
      lastCoilSaveTime = now; saveAll();
      if (coilOnTime >= maxCoilOnTime && !maxCoilError) {
        maxCoilError = true; coilErrorAcknowledged = false;
      }
    }
  } else {
    if (coilWasOn) {
      coilOnTime += (now - coilOnStart); coilWasOn = false;
      lastCoilSaveTime = now; saveAll();
      relayOffStart = now; relayOffTimerActive = true;
    }
    if (relayOffTimerActive && (now - relayOffStart >= 120000UL)) {
      relayOffTimerActive = false;
      if (maxCoilError) {
        coilOnTime = 0; maxCoilError = false; coilErrorAcknowledged = false;
        coilEffectivenessActive = false; tempAtCoilStart = 0; saveAll();
      }
    }
  }
}

void updateCoilEffectivenessCheck(bool relayIsOn) {
  if (coilNotWorkingError) return;
  if (relayIsOn && !coilEffectivenessActive) {
    coilEffectivenessActive = true; coilEffCheckStart = millis();
    if (RT1 != -999) tempAtCoilStart = RT1;
    else             coilEffectivenessActive = false;
  }
  if (!relayIsOn && coilEffectivenessActive) coilEffectivenessActive = false;
  if (coilEffectivenessActive && millis()-coilEffCheckStart >= coilEffCheckDurationMs) {
    coilEffectivenessActive = false;
    if (RT1 != -999 && (RT1 - tempAtCoilStart) < coilEffMinRiseDeg) coilNotWorkingError = true;
  }
}

void handleCoilErrorClearHold() {
  if (pressed(BTN_SET)) {
    if (!coilErrClearHolding) { coilErrClearHolding = true; coilErrClearHoldStart = millis(); }
    else if (millis()-coilErrClearHoldStart >= 5000UL) {
      coilNotWorkingError = false; coilEffectivenessActive = false;
      tempAtCoilStart = 0; coilErrClearHolding = false; saveAll();
      for (int b = 0; b < 2; b++) { buzzerBeep(100); delay(100); }
      showPopup("Coil Error Cleared", "System Resuming...");
      unsigned long s = millis(); while (millis()-s < 1500) { feedWDT(); }
    }
  } else { coilErrClearHolding = false; }
}

void checkDailyReset() {
  int h, m; getTime(h, m);
  static bool dailyResetDone = false;
  if (!timeIsSet) return;
  if (h==23 && m>=55 && m<=57 && !dailyResetDone) {
    saveDailyLog();  // save today's log to EEPROM before resetting
    slotDailyOnTime[0] = 0; slotDailyOnTime[1] = 0;
    mornPeakTemp = 0; evePeakTemp = 0; dayErrorAccum = 0;  // reset trackers
    dailyResetDone = true; saveAll();
  }
  if (h == 0) dailyResetDone = false;
}

// ═══════════════════════════════════════════════════════════════════════════
//  [C2] DATA RESPONSE
// ═══════════════════════════════════════════════════════════════════════════
String buildDataResponse() {
  DateTime now = rtc.now();
  char tsStr[24];
  snprintf(tsStr, sizeof(tsStr), "%02d:%02d %02d/%02d/%04d",
           now.hour(), now.minute(), now.day(), now.month(), now.year());

  unsigned long s0 = slotDailyOnTime[0];
  unsigned long s1 = slotDailyOnTime[1];
  unsigned long sT = s0 + s1;
  char buf0[16], buf1[16], bufT[16];
  fmtHHMM(buf0, s0); fmtHHMM(buf1, s1); fmtHHMM(bufT, sT);

  float totalH = sT / 3600.0f;
  float kWh    = totalH * 2.0f;
  float co2    = kWh * 0.8f;

  StaticJsonDocument<384> doc;
  doc["type"]          = "data";
  doc["timestamp"]     = tsStr;
  doc["mornUsageSec"]  = s0;
  doc["eveUsageSec"]   = s1;
  doc["totalUsageSec"] = sT;
  doc["mornUsageFmt"]  = buf0;
  doc["eveUsageFmt"]   = buf1;
  doc["totalUsageFmt"] = bufT;
  doc["energyKWh"]     = String(kWh, 2);
  doc["co2Kg"]         = String(co2, 2);

  String out; serializeJson(doc, out);
  return out;
}

// ═══════════════════════════════════════════════════════════════════════════
//  BLUETOOTH
// ═══════════════════════════════════════════════════════════════════════════
void readBluetooth() {
  if (commsMode != MODE_BLE) return;
  if (!SerialBT.available()) return;
  String msg = SerialBT.readStringUntil('\n');
  msg.trim();

  if (msg.startsWith("ST=")) {
    ST1 = constrain(msg.substring(3).toInt(), TEMP_MIN, TEMP_MAX);
    tempHysteresisReady = (RT1 == -999 || RT1 < ST1);
    notifPopupMsg = "Temp Updated";
    notifPopupMsg2 = "Set -> " + String(ST1) + " C";

  } else if (msg.startsWith("S1S=")) {
    int p = msg.indexOf(':');
    int newH = msg.substring(4,p).toInt(), newM = constrain(msg.substring(p+1).toInt(),0,59);
    if (newH < SLOT0_MIN_HOUR || newH > SLOT0_MAX_HOUR) {
      notifPopupMsg = "Morning Slot Error";
      notifPopupMsg2 = "Hour must be 1-12";
    }
    else {
      S_H[0] = newH; S_M[0] = newM;
      // Auto-set end time = start + 30 min
      int newEndMin = newH*60 + newM + MIN_SLOT_GAP_MINS;
      int newEH = newEndMin / 60;
      int newEM = newEndMin % 60;
      if (newEH > SLOT0_MAX_HOUR) { newEH = SLOT0_MAX_HOUR; newEM = 59; }
      E_H[0] = newEH; E_M[0] = newEM;
      char tBuf[30];
      snprintf(tBuf, sizeof(tBuf), "%02d:%02d - %02d:%02d", S_H[0], S_M[0], E_H[0], E_M[0]);
      notifPopupMsg = "Morning Slot Updated";
      notifPopupMsg2 = String(tBuf);
    }

  } else if (msg.startsWith("S1E=")) {
    int p = msg.indexOf(':');
    int newH = msg.substring(4,p).toInt(), newM = constrain(msg.substring(p+1).toInt(),0,59);
    if (newH < SLOT0_MIN_HOUR || newH > SLOT0_MAX_HOUR) {
      notifPopupMsg = "Morning Slot Error";
      notifPopupMsg2 = "Hour must be 1-12";
    }
    else {
      int oldH = E_H[0], oldM = E_M[0];
      E_H[0] = newH; E_M[0] = newM;
      int gap = (E_H[0]*60+E_M[0]) - (S_H[0]*60+S_M[0]);
      if (gap < MIN_SLOT_GAP_MINS) {
        E_H[0] = oldH; E_M[0] = oldM;
        notifPopupMsg = "Morning Slot Error";
        notifPopupMsg2 = "Need 30min gap!";
      } else {
        char tBuf[30];
        snprintf(tBuf, sizeof(tBuf), "%02d:%02d - %02d:%02d", S_H[0], S_M[0], E_H[0], E_M[0]);
        notifPopupMsg = "Morning Slot Updated";
        notifPopupMsg2 = String(tBuf);
      }
    }

  } else if (msg.startsWith("S2S=")) {
    int p = msg.indexOf(':');
    int newH = msg.substring(4,p).toInt(), newM = constrain(msg.substring(p+1).toInt(),0,59);
    if (newH < SLOT1_MIN_HOUR || newH > SLOT1_MAX_HOUR) {
      notifPopupMsg = "Evening Slot Error";
      notifPopupMsg2 = "Hour must be 13-23";
    }
    else {
      S_H[1] = newH; S_M[1] = newM;
      // Auto-set end time = start + 30 min
      int newEndMin = newH*60 + newM + MIN_SLOT_GAP_MINS;
      int newEH = newEndMin / 60;
      int newEM = newEndMin % 60;
      if (newEH > SLOT1_MAX_HOUR) { newEH = SLOT1_MAX_HOUR; newEM = 59; }
      E_H[1] = newEH; E_M[1] = newEM;
      char tBuf[30];
      snprintf(tBuf, sizeof(tBuf), "%02d:%02d - %02d:%02d", S_H[1], S_M[1], E_H[1], E_M[1]);
      notifPopupMsg = "Evening Slot Updated";
      notifPopupMsg2 = String(tBuf);
    }

  } else if (msg.startsWith("S2E=")) {
    int p = msg.indexOf(':');
    int newH = msg.substring(4,p).toInt(), newM = constrain(msg.substring(p+1).toInt(),0,59);
    if (newH < SLOT1_MIN_HOUR || newH > SLOT1_MAX_HOUR) {
      notifPopupMsg = "Evening Slot Error";
      notifPopupMsg2 = "Hour must be 13-23";
    }
    else {
      int oldH = E_H[1], oldM = E_M[1];
      E_H[1] = newH; E_M[1] = newM;
      int gap = (E_H[1]*60+E_M[1]) - (S_H[1]*60+S_M[1]);
      if (gap < MIN_SLOT_GAP_MINS) {
        E_H[1] = oldH; E_M[1] = oldM;
        notifPopupMsg = "Evening Slot Error";
        notifPopupMsg2 = "Need 30min Gap!";
      } else {
        char tBuf[30];
        snprintf(tBuf, sizeof(tBuf), "%02d:%02d - %02d:%02d", S_H[1], S_M[1], E_H[1], E_M[1]);
        notifPopupMsg = "Evening Slot Updated";
        notifPopupMsg2 = String(tBuf);
      }
    }

  } else if (msg.startsWith("S1EN=")) {
    slotEnabled[0] = (msg.substring(5).toInt() != 0);
    notifPopupMsg = "Morning Slot";
    notifPopupMsg2 = slotEnabled[0] ? "Turned ON" : "Turned OFF";

  } else if (msg.startsWith("S2EN=")) {
    slotEnabled[1] = (msg.substring(5).toInt() != 0);
    notifPopupMsg = "Evening Slot";
    notifPopupMsg2 = slotEnabled[1] ? "Turned ON" : "Turned OFF";

  } else if (msg.startsWith("TIME=")) {
    int p = msg.indexOf(':', 5);
    if (p > 0) {
      int hh = msg.substring(5,p).toInt(), mm = msg.substring(p+1).toInt();
      if (hh>=0 && hh<=23 && mm>=0 && mm<=59) {
        setRTCTime(hh, mm);
        char tBuf[10];
        snprintf(tBuf, sizeof(tBuf), "%02d:%02d", hh, mm);
        notifPopupMsg = "RTC Time Set";
        notifPopupMsg2 = "Time -> " + String(tBuf);
      } else {
        notifPopupMsg = "RTC Time Error";
        notifPopupMsg2 = "Invalid format";
      }
    } else {
      notifPopupMsg = "RTC Time Error";
      notifPopupMsg2 = "Use TIME=HH:MM";
    }

  } else if (msg.startsWith("DATE=")) {
    int c1 = msg.indexOf(',',5), c2 = msg.indexOf(',',c1+1);
    if (c1>0 && c2>0) {
      int yy = msg.substring(5,c1).toInt(), mm = msg.substring(c1+1,c2).toInt(), dd = msg.substring(c2+1).toInt();
      rtc.adjust(DateTime(yy, mm, dd, 0, 0, 0));
      char dBuf[20];
      snprintf(dBuf, sizeof(dBuf), "%04d-%02d-%02d", yy, mm, dd);
      notifPopupMsg = "RTC Date Set";
      notifPopupMsg2 = String(dBuf);
    } else {
      notifPopupMsg = "RTC Date Error";
      notifPopupMsg2 = "Use YYYY,MM,DD";
    }

  } else if (msg == "RESETCOIL") {
    coilOnTime = 0; maxCoilError = false; coilErrorAcknowledged = false;
    coilNotWorkingError = false; relayOffTimerActive = false;
    coilWasOn = false; coilEffectivenessActive = false;
    saveAll();
    notifPopupMsg = "Coil Status";
    notifPopupMsg2 = "Timer Reset OK";

  } else if (msg == "Data" || msg == "DATA") {
    SerialBT.println(buildDataResponse());
    saveAll();
    return;

  } else if (msg == "ERRSTAT") {
    SerialBT.println(buildErrorJson());
    return;

  } else if (msg == "SYNC") {
    int coilStatus = maxCoilError ? 2 : ((digitalRead(RELAY_TANK)==RELAY_ON) ? 1 : 0);
    String tt = (RT1 == -999) ? "ERR" : String(RT1);
    String reply = String(ST1) + "\"" + tt + "\"" + String(coilStatus) + "\"";
    reply += String(S_H[0]) + "\"" + String(S_H[1]) + "\"";
    reply += String(S_M[0]) + "\"" + String(S_M[1]) + "\"";
    reply += String(E_H[0]) + "\"" + String(E_H[1]) + "\"";
    reply += String(E_M[0]) + "\"" + String(E_M[1]);
    reply += "\"" + String(slotEnabled[0] ? 1 : 0);
    reply += "\"" + String(slotEnabled[1] ? 1 : 0);
    SerialBT.println(reply);
    saveAll();
    return;

  } else if (msg == "LOGS") {
    SerialBT.println(buildLogJson(LOG_MAX_RECORDS));
    return;

  } else if (msg == "CLEARLOGS") {
    clearAllLogs();
    notifPopupMsg = "EEPROM Storage";
    notifPopupMsg2 = "Logs Cleared";

  } else {
    notifPopupMsg = "BLE Command";
    notifPopupMsg2 = "Invalid Syntax";
  }

  notifPopupActive = true; notifPopupTime = millis(); saveAll();
}

// ═══════════════════════════════════════════════════════════════════════════
//  HTTP API  (WiFi mode only)
// ═══════════════════════════════════════════════════════════════════════════
void addCORS() {
  server.sendHeader("Access-Control-Allow-Origin",  "*");
  server.sendHeader("Access-Control-Allow-Methods", "GET,POST,OPTIONS");
  server.sendHeader("Access-Control-Allow-Headers", "Content-Type");
}

void handleStatus() {
  addCORS();
  int coilStatus = maxCoilError ? 2 : ((digitalRead(RELAY_TANK)==RELAY_ON) ? 1 : 0);
  String tankTempStr = (RT1 == -999) ? "ERR" : String(RT1);
  char s1s[6], s1e[6], s2s[6], s2e[6];
  snprintf(s1s,6,"%02d:%02d",S_H[0],S_M[0]); snprintf(s1e,6,"%02d:%02d",E_H[0],E_M[0]);
  snprintf(s2s,6,"%02d:%02d",S_H[1],S_M[1]); snprintf(s2e,6,"%02d:%02d",E_H[1],E_M[1]);
  StaticJsonDocument<512> doc;
  doc["temperature"]    = ST1;
  doc["tankTemp"]       = tankTempStr;
  doc["coilStatus"]     = coilStatus;
  doc["mornSlotStart"]  = s1s; doc["mornSlotEnd"]  = s1e;
  doc["eveSlotStart"]   = s2s; doc["eveSlotEnd"]   = s2e;
  doc["mornSlotEnabled"]= slotEnabled[0];
  doc["eveSlotEnabled"] = slotEnabled[1];
  doc["preHeatSteps"]   = preHeatSteps;
  doc["preHeatMinutes"] = preHeatMinutes();
  doc["ambientTemp"]    = ahtOk ? String(ahtTemp,   1) : String("ERR");
  doc["ambientHumid"]   = ahtOk ? String(ahtHumidity,1) : String("ERR");
  doc["ip"]             = deviceIP;
  String body; serializeJson(doc, body);
  server.send(200, "application/json", body);
}

void handleSetTemperature() {
  addCORS();
  if (!server.hasArg("plain")) { server.send(400, "application/json", "{\"error\":\"no body\"}"); return; }
  StaticJsonDocument<64> doc;
  if (deserializeJson(doc, server.arg("plain"))) { server.send(400, "application/json", "{\"error\":\"bad json\"}"); return; }
  int newTemp = doc["temperature"] | -1;
  if (newTemp < TEMP_MIN || newTemp > TEMP_MAX) { server.send(400, "application/json", "{\"error\":\"out of range\"}"); return; }
  ST1 = newTemp; tempHysteresisReady = (RT1==-999 || RT1<ST1); saveAll();
  server.send(200, "application/json", "{\"ok\":true}");
  char popMsg[20]; sprintf(popMsg, "Set -> %d C", ST1);
  notifPopupMsg = "Temp Updated"; notifPopupMsg2 = String(popMsg);
  notifPopupActive = true; notifPopupTime = millis();
}

void handleSetSlot() {
  addCORS();
  if (!server.hasArg("plain")) { server.send(400, "application/json", "{\"error\":\"no body\"}"); return; }
  StaticJsonDocument<128> doc;
  if (deserializeJson(doc, server.arg("plain"))) { server.send(400, "application/json", "{\"error\":\"bad json\"}"); return; }
  int slot = doc["slot"] | 0;
  if (slot!=1 && slot!=2) { server.send(400, "application/json", "{\"error\":\"slot must be 1 or 2\"}"); return; }
  const char* startStr = doc["start"] | "";
  const char* endStr   = doc["end"]   | "";
  int sh, sm, eh, em;
  if (sscanf(startStr,"%d:%d",&sh,&sm)!=2 || sscanf(endStr,"%d:%d",&eh,&em)!=2) {
    server.send(400, "application/json", "{\"error\":\"bad time format\"}"); return;
  }
  int i = slot-1;
  int minH = slotMinHour(i), maxH = slotMaxHour(i);
  if (sh < minH || sh > maxH || eh < minH || eh > maxH) {
    char errBuf[80]; snprintf(errBuf, sizeof(errBuf),
      "{\"error\":\"hours must be %d-%d for slot %d\"}", minH, maxH, slot);
    server.send(400, "application/json", errBuf); return;
  }
  sm = constrain(sm,0,59); em = constrain(em,0,59);
  if ((eh*60+em) - (sh*60+sm) < MIN_SLOT_GAP_MINS) {
    server.send(400, "application/json", "{\"error\":\"end must be >= 30 min after start\"}"); return;
  }
  S_H[i] = sh; S_M[i] = sm;
  E_H[i] = eh; E_M[i] = em;
  slotEnabled[i] = doc.containsKey("enabled") ? (bool)doc["enabled"] : true;
  saveAll();
  server.send(200, "application/json", "{\"ok\":true}");
  const char* slotName = (slot==1) ? "Morning Slot" : "Evening Slot";
  notifPopupMsg  = String(slotName) + " Updated";
  notifPopupMsg2 = String(startStr) + " - " + String(endStr);
  notifPopupActive = true; notifPopupTime = millis();
}

void handleSetSlotEnable() {
  addCORS();
  if (!server.hasArg("plain")) { server.send(400, "application/json", "{\"error\":\"no body\"}"); return; }
  StaticJsonDocument<64> doc;
  if (deserializeJson(doc, server.arg("plain"))) { server.send(400, "application/json", "{\"error\":\"bad json\"}"); return; }
  int  slot = doc["slot"]    | 0;
  bool en   = doc["enabled"] | false;
  if (slot!=1 && slot!=2) { server.send(400, "application/json", "{\"error\":\"slot must be 1 or 2\"}"); return; }
  slotEnabled[slot-1] = en; saveAll();
  server.send(200, "application/json", "{\"ok\":true}");
  const char* slotName = (slot==1) ? "Morning Slot" : "Evening Slot";
  notifPopupMsg  = String(slotName);
  notifPopupMsg2 = en ? "Turned ON" : "Turned OFF";
  notifPopupActive = true; notifPopupTime = millis();
  Serial.printf("[SLOT] Slot %d %s via WiFi API\n", slot, en?"ENABLED":"DISABLED");
}

void handleGetData() {
  addCORS();
  server.send(200, "application/json", buildDataResponse());
}

void handleGetErrors() {
  addCORS();
  server.send(200, "application/json", buildErrorJson());
}

void handleOptions() { addCORS(); server.send(204); }

// ── HTTP handler: GET /logs  (daily log records from EEPROM) ─────────────
void handleGetLogs() {
  addCORS();
  int days = LOG_MAX_RECORDS;
  if (server.hasArg("days")) {
    days = constrain(server.arg("days").toInt(), 1, LOG_MAX_RECORDS);
  }
  server.send(200, "application/json", buildLogJson(days));
}

// ── HTTP handler: DELETE /logs  (clear all log data) ─────────────────────
void handleDeleteLogs() {
  addCORS();
  clearAllLogs();
  server.send(200, "application/json", "{\"status\":\"ok\",\"message\":\"Logs cleared\"}");
}

void startHttpServer() {
  server.on("/status",      HTTP_GET,     handleStatus);
  server.on("/temperature", HTTP_POST,    handleSetTemperature);
  server.on("/slot",        HTTP_POST,    handleSetSlot);
  server.on("/slot/enable", HTTP_POST,    handleSetSlotEnable);
  server.on("/data",        HTTP_GET,     handleGetData);
  server.on("/errors",      HTTP_GET,     handleGetErrors);
  server.on("/status",      HTTP_OPTIONS, handleOptions);
  server.on("/temperature", HTTP_OPTIONS, handleOptions);
  server.on("/slot",        HTTP_OPTIONS, handleOptions);
  server.on("/slot/enable", HTTP_OPTIONS, handleOptions);
  server.on("/data",        HTTP_OPTIONS, handleOptions);
  server.on("/errors",      HTTP_OPTIONS, handleOptions);
  server.on("/logs",         HTTP_GET,     handleGetLogs);
  server.on("/logs",         HTTP_DELETE,  handleDeleteLogs);
  server.on("/logs",         HTTP_OPTIONS, handleOptions);
  server.begin();
  Serial.println("[HTTP] Server started.");
}

// ═══════════════════════════════════════════════════════════════════════════
//  WIFI CONNECT HELPER
// ═══════════════════════════════════════════════════════════════════════════
bool connectWifi(const String& ssid, const String& pass, unsigned long timeoutMs=15000) {
  if (ssid.isEmpty()) return false;
  Serial.printf("[WiFi] Connecting to '%s'...\n", ssid.c_str());
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid.c_str(), pass.c_str());
  unsigned long t0 = millis();
  while (WiFi.status()!=WL_CONNECTED && millis()-t0<timeoutMs) {
    delay(300); feedWDT(); drawWifiConnecting(ssid);
  }
  if (WiFi.status() == WL_CONNECTED) {
    deviceIP = WiFi.localIP().toString(); wifiConnected = true;
    Serial.printf("[WiFi] Connected! IP=%s\n", deviceIP.c_str());
    drawWifiConnected(deviceIP); delay(1500);
    return true;
  }
  wifiConnected = false; deviceIP = "";
  Serial.println("[WiFi] FAILED.");
  drawWifiFailed(); delay(1500);
  return false;
}

// ═══════════════════════════════════════════════════════════════════════════
//  BT WIFI PROVISIONING  —  sprite screens
// ═══════════════════════════════════════════════════════════════════════════

// ── Boot-time prompt: "PRESS SET TO CHANGE WIFI" with countdown ─────────
void drawBtProvisionPrompt(int secsLeft) {
  if (!sprReady) return;
  beginScreen("  WI-FI  ", P_BORDER);
  bigCentred(CONTENT_Y+4, "WIFI SETUP", 2, P_BORDER);
  hLine(CONTENT_Y+26, P_BORDER);
  bigCentred(CONTENT_Y+38, "PRESS SET TO CHANGE", 2, P_BORDER);
  //bigCentred(CONTENT_Y+58, "CHANGE WIFI", 2, P_BORDER);
  bigCentred(CONTENT_Y+68, "WIFI CREDENTIALS", 2, P_BORDER);
  hLine(CONTENT_Y+100, P_BORDER);
  char buf[16]; sprintf(buf, "Timeout: %ds", secsLeft);
  bigCentred(CONTENT_Y+110, buf, 2, P_BORDER);
  // Blinking countdown bar
  int barW = SCREEN_W - 2*(MARGIN+8);
  int pct = (secsLeft * 100) / (BT_PROV_TIMEOUT_MS / 1000);
  spr.drawRect(MARGIN+8, CONTENT_Y+140, barW, 12, P_BORDER);
  int fill = (pct * (barW-4)) / 100;
  if (fill > 0) spr.fillRect(MARGIN+10, CONTENT_Y+142, fill, 8, P_BORDER);
  hLine(CONTENT_Y+160, P_BORDER);
  if (blinkOn()) {
    spr.fillRect(MARGIN+3, FOOTER_Y-4, SCREEN_W-2*(MARGIN+3), 22, P_BORDER);
    spr.setTextSize(2); spr.setTextColor(P_BG, P_BORDER);
    const char* hint = ">> PRESS SET <<";
    spr.setCursor((SCREEN_W-strlen(hint)*12)/2, FOOTER_Y+2); spr.print(hint);
  } else {
    spr.drawRect(MARGIN+3, FOOTER_Y-4, SCREEN_W-2*(MARGIN+3), 22, P_BORDER);
    spr.setTextSize(2); spr.setTextColor(P_BORDER, P_BG);
    const char* hint = ">> PRESS SET <<";
    spr.setCursor((SCREEN_W-strlen(hint)*12)/2, FOOTER_Y+2); spr.print(hint);
  }
  sprPush();
}

// ── BT waiting screen: shows BT name + spinner ─────────────────────────
void drawBtProvisionWaiting() {
  if (!sprReady) return;
  beginScreen(" WI-FI SETUP ", P_BORDER);
  bigCentred(CONTENT_Y+4, "WIFI CONFIG MODE", 2, P_BORDER);
  hLine(CONTENT_Y+26, P_BORDER);
  // Show the BT device name on screen
  spr.setTextSize(2); spr.setTextColor(P_BORDER, P_BG);
  spr.setCursor(MARGIN+10, CONTENT_Y+34); spr.print("DEVICE NAME:");
  spr.fillRect(MARGIN+4, CONTENT_Y+56, SCREEN_W-2*(MARGIN+4), 26, P_BORDER);
  spr.setTextSize(2); spr.setTextColor(P_BG, P_BORDER);
  String provName = btDeviceName + "-WiFi-Setup";
  int16_t nw = provName.length()*12;
  spr.setCursor((SCREEN_W-nw)/2, CONTENT_Y+61); spr.print(provName.c_str());
  hLine(CONTENT_Y+90, P_BORDER);
  spr.setTextSize(2); spr.setTextColor(P_BORDER, P_BG);
  spr.setCursor(MARGIN+10, CONTENT_Y+98); spr.print("In APP Click:");
  spr.drawRect(MARGIN+4, CONTENT_Y+118, SCREEN_W-2*(MARGIN+4), 26, P_BORDER);
  spr.setTextSize(2); spr.setTextColor(P_BORDER, P_BG);
  const char* cmd = "Upadte WIFI";
  int16_t cw = strlen(cmd)*12;
  spr.setCursor((SCREEN_W-cw)/2, CONTENT_Y+122); spr.print(cmd);
  hLine(CONTENT_Y+155, P_BORDER);

  sprPush();
}

// ── BT provisioning success screen ──────────────────────────────────────
void drawBtProvisionSuccess(const String& ip) {
  if (!sprReady) return;
  beginScreen(" WI-FI OK ", P_BORDER);
  spr.fillRect(MARGIN+4, 44, SCREEN_W-2*(MARGIN+4), 54, P_BORDER);
  spr.setTextSize(3); spr.setTextColor(P_BG, P_BORDER);
  const char* ok = "CONNECTED";
  spr.setCursor((SCREEN_W-strlen(ok)*18)/2, 58); spr.print(ok);
  hLine(106, P_BORDER);
  bigCentred(114, "DEVICE IP ADDRESS", 1, P_BORDER);
  spr.drawRect(MARGIN+4, 128, SCREEN_W-2*(MARGIN+4), 34, P_BORDER);
  spr.setTextSize(2); spr.setTextColor(P_BORDER, P_BG);
  int16_t iw = ip.length()*12;
  spr.setCursor((SCREEN_W-iw)/2, 136); spr.print(ip.c_str());
  hLine(170, P_BORDER);
  bigCentred(178, "IP Sent via Bluetooth", 1, P_BORDER);
  bigCentred(192, "BT Stopping...", 1, P_BORDER);
  sprPush();
}

// ═══════════════════════════════════════════════════════════════════════════
//  BT WIFI PROVISIONING  —  main flow
//
//  Returns true if new credentials were received and WiFi connected.
//  Can be called from setup() (boot prompt) or from watchdog (auto-trigger).
//
//  showPrompt = true  → show "PRESS SET" countdown (boot-time)
//  showPrompt = false → skip prompt, go straight to BT waiting (watchdog)
// ═══════════════════════════════════════════════════════════════════════════
void startBtProvisioning(bool showPrompt) {
  // ── Step 1: Optional boot prompt (press SET within timeout) ───────────
  if (showPrompt) {
    Serial.println("[BT-PROV] Showing boot prompt — press SET to provision...");
    unsigned long promptStart = millis();
    bool setPressed = false;
    while (millis() - promptStart < BT_PROV_TIMEOUT_MS) {
      feedWDT();
      int secsLeft = (int)((BT_PROV_TIMEOUT_MS - (millis() - promptStart)) / 1000) + 1;
      drawBtProvisionPrompt(secsLeft);
      if (pressed(BTN_SET)) { setPressed = true; break; }
      delay(50);
    }
    if (!setPressed) {
      Serial.println("[BT-PROV] Timeout — no SET press. Booting normally.");
      return;
    }
    buzzerBeep(100);
    Serial.println("[BT-PROV] SET pressed — starting Bluetooth provisioning.");
  } else {
    Serial.println("[BT-PROV] Auto-triggered — starting Bluetooth provisioning.");
  }

  // ── Step 2: Start Bluetooth ───────────────────────────────────────────
  loadBtName();
  WiFi.mode(WIFI_OFF);
  delay(100);
  String provName = btDeviceName + "-WiFi-Setup";
  SerialBT.begin(provName.c_str());
  Serial.printf("[BT-PROV] Bluetooth started as '%s'\n", provName.c_str());

  btProvOldCommsMode = commsMode;
  // Keep commsMode as MODE_WIFI so header shows WiFi symbol
  // and handleWifiWatchdog() continues to process WIFI= commands
  
  btProvisioningActive = true;
  wifiState = WIFI_BT_PROV_MODE;
}

// ═══════════════════════════════════════════════════════════════════════════
//  WIFI WATCHDOG
// ═══════════════════════════════════════════════════════════════════════════
void handleWifiWatchdog() {
  if (commsMode != MODE_WIFI && !btProvisioningActive) return;
  if (manualMode) return;
  switch (wifiState) {
    case WIFI_OK:
      if (WiFi.status() != WL_CONNECTED) {
        wifiConnected = false; wifiRetryCount = 0; lastWifiRetryAttempt = 0;
        wifiState = WIFI_RETRYING;
        Serial.println("[WiFi] Connection lost — retrying silently.");
      }
      break;
    case WIFI_RETRYING:
      if (millis()-lastWifiRetryAttempt >= WIFI_RETRY_INTERVAL_MS) {
        wifiRetryCount++; lastWifiRetryAttempt = millis();
        Serial.printf("[WiFi] Retry %d/%d...\n", wifiRetryCount, WIFI_MAX_RETRY);
        if (connectWifi(wifiSSID, wifiPassword, 10000)) {
          server.stop(); startHttpServer(); wifiState = WIFI_OK;
          syncRtcFromNtp(); wifiRetryCount = 0; break;
        }
        if (wifiRetryCount >= WIFI_MAX_RETRY) {
          Serial.println("[WiFi] Max retries reached — starting BT provisioning.");
          startBtProvisioning(false);
        }
      }
      break;
    case WIFI_BT_PROV_MODE:
      if (SerialBT.available()) {
        String msg = SerialBT.readStringUntil('\n');
        msg.trim();
        Serial.printf("[BT-PROV] Received: '%s'\n", msg.c_str());

        if (msg.startsWith("WIFI=")) {
          String payload = msg.substring(5);
          int commaIdx = payload.indexOf(',');
          if (commaIdx > 0) {
            String newSsid = payload.substring(0, commaIdx);
            String newPass = payload.substring(commaIdx + 1);
            newSsid.trim(); newPass.trim();
            if (!newSsid.isEmpty()) {
              SerialBT.println("CONNECTING...");
              commsMode = btProvOldCommsMode; // Restore to WIFI mode for header symbol
              Serial.printf("[BT-PROV] Connecting WiFi SSID='%s'...\n", newSsid.c_str());
              
              if (connectWifi(newSsid, newPass, 15000)) {
                deviceIP = WiFi.localIP().toString();
                wifiConnected = true;
                saveWifiCreds(newSsid, newPass);
                wifiSSID = newSsid;
                wifiPassword = newPass;
                Serial.printf("[BT-PROV] WiFi connected! IP=%s\n", deviceIP.c_str());
                
                String ipMsg = "IP=" + deviceIP;
                SerialBT.println(ipMsg);
                drawBtProvisionSuccess(deviceIP);
                delay(2000);  // give BT time to flush
                SerialBT.end();
                
                btProvisioningActive = false;
                wifiState = WIFI_OK;
                server.stop(); startHttpServer(); syncRtcFromNtp(); wifiRetryCount = 0;
                // Wait for SET button release so a lingering press doesn't
                // trigger the 2 s hold → manual-mode entry in loop()
                while (pressed(BTN_SET)) { feedWDT(); delay(50); }
                break;
              } else {
                Serial.println("[BT-PROV] WiFi connection failed.");
                SerialBT.println("FAIL");
                drawWifiFailed();
                delay(1500);
                // Return to waiting for new creds
                // Keep commsMode as MODE_WIFI (WiFi setup mode)
                WiFi.mode(WIFI_OFF);
                break;
              }
            }
          }
          SerialBT.println("ERR:format WIFI=ssid,password");
        } else {
          SerialBT.println("ERR:send WIFI=ssid,password");
        }
      }
      break;
  }
}

void handleDailyNtpSync() {
  if (commsMode != MODE_WIFI) return;
  if (wifiState != WIFI_OK) return;
  if (manualMode) return;
  if (digitalRead(RELAY_TANK) == RELAY_ON) return;
  if (millis()-lastNtpSyncMs >= NTP_DAILY_INTERVAL_MS) syncRtcFromNtp();
}

// ═══════════════════════════════════════════════════════════════════════════
//  FAULT GUARD
// ═══════════════════════════════════════════════════════════════════════════
bool anyFaultActive() { return (RT1 == -999) || maxCoilError || coilNotWorkingError; }

void enforceSafeRelay() {
  if (anyFaultActive()) {
    digitalWrite(RELAY_TANK, RELAY_OFF);
    slotRelayActive[0] = false; slotRelayActive[1] = false;
  }
}

// ═══════════════════════════════════════════════════════════════════════════
//  [C5] ERROR CODE HELPERS
// ═══════════════════════════════════════════════════════════════════════════
uint8_t getActiveErrorMask() {
  uint8_t mask = ERR_NONE;
  if (RT1 == -999)         mask |= ERR_SENSOR_FAULT;
  if (maxCoilError)        mask |= ERR_MAX_COIL;
  if (coilNotWorkingError) mask |= ERR_COIL_NO_WORK;
  return mask;
}

String buildErrorJson() {
  uint8_t mask = getActiveErrorMask();
  StaticJsonDocument<320> doc;
  doc["type"] = "error";
  doc["mask"] = mask;
  JsonArray codes = doc.createNestedArray("errors");
  JsonArray descs = doc.createNestedArray("desc");
  if (mask & ERR_SENSOR_FAULT) { codes.add("E1"); descs.add("Sensor Fault"); }
  if (mask & ERR_MAX_COIL)     { codes.add("E2"); descs.add("Max Coil On Time"); }
  if (mask & ERR_COIL_NO_WORK) { codes.add("E3"); descs.add("Coil Not Working"); }
  doc["tankTemp"] = (RT1 == -999) ? "ERR" : String(RT1);
  DateTime now = rtc.now();
  char ts[22];
  snprintf(ts, sizeof(ts), "%02d:%02d %02d/%02d/%04d",
           now.hour(), now.minute(), now.day(), now.month(), now.year());
  doc["timestamp"] = ts;
  String out; serializeJson(doc, out);
  return out;
}

void broadcastErrors() {
  if (getActiveErrorMask() == ERR_NONE) return;
  unsigned long now = millis();
  if (now - lastErrorBroadcast < ERROR_BROADCAST_INTERVAL_MS) return;
  lastErrorBroadcast = now;
  String pkt = buildErrorJson();
  Serial.println("[ERROR] " + pkt);
  if (commsMode == MODE_BLE && SerialBT.hasClient()) {
    SerialBT.println(pkt);
  }
}

// ═══════════════════════════════════════════════════════════════════════════
//  [C6]  SLOT-SCREEN LOCK MANAGER
//
//  Call once per loop pass (after slotRelayActive[] has been updated).
//  Updates:
//    slotScreenLocked      — true when any slot is running
//    lockedSlotScreenIndex — which screen index to lock to (2 or 3)
//    slotBrowsing          — true while user is browsing away
//
//  Returns the screen index that should be displayed.
//  The caller (drawScreen) uses this value instead of currentScreenIndex
//  when the lock is active and the user is not browsing.
// ═══════════════════════════════════════════════════════════════════════════
void updateSlotScreenLock(bool slot0Active, bool slot1Active) {
  bool anySlotRunning = slot0Active || slot1Active;

  if (anySlotRunning) {
    // Determine which slot screen to lock to (prefer the one that is actually running)
    if (slot0Active) lockedSlotScreenIndex = 2;      // morning slot screen
    else             lockedSlotScreenIndex = 3;      // evening slot screen

    if (!slotScreenLocked) {
      // Slot just became active — snap immediately to the slot screen
      slotScreenLocked    = true;
      slotBrowsing        = false;
      currentScreenIndex  = lockedSlotScreenIndex;
      screenCycleTime     = millis();  // reset auto-cycle timer (won't fire anyway)
      Serial.printf("[LOCK] Slot running — screen locked to index %d\n", lockedSlotScreenIndex);
    }

    // While locked: check if the user is still browsing away
    if (slotBrowsing) {
      if (millis() - slotBrowseLastPress >= SLOT_BROWSE_TIMEOUT_MS) {
        // Idle timeout expired — snap back to the running slot screen
        slotBrowsing       = false;
        currentScreenIndex = lockedSlotScreenIndex;
        Serial.println("[LOCK] Browse timeout — snapped back to slot screen.");
      }
    }

  } else {
    // No slot running — release the lock and restore normal auto-cycling
    if (slotScreenLocked) {
      slotScreenLocked = false;
      slotBrowsing     = false;
      screenCycleTime  = millis();  // restart the auto-cycle timer fresh
      Serial.println("[LOCK] Slot stopped — screen lock released, auto-cycle resumed.");
    }
  }
}

// ═══════════════════════════════════════════════════════════════════════════
//  drawScreen()  —  selects and renders the current auto-cycling screen
//
//  [C6] When a slot is running the auto-cycle is suspended.
//       UP/DOWN navigate freely; 10 s idle snaps back to the slot screen.
//       When no slot is running, normal 3 s auto-cycling resumes.
// ═══════════════════════════════════════════════════════════════════════════
void drawScreen(bool slot0Active, bool slot1Active) {
  int h, m; getTime(h, m);

  // ── [C6] Update slot-lock state ──────────────────────────────────────────
  updateSlotScreenLock(slot0Active, slot1Active);

  if (!slotScreenLocked) {
    // ── Normal auto-cycling (no slot running) ─────────────────────────────
    if (millis()-screenCycleTime > 3000) {
      currentScreenIndex = (currentScreenIndex + 1) % TOTAL_SCREENS;
      if (currentScreenIndex == 5 && !btProvisioningActive) {
        currentScreenIndex = (currentScreenIndex + 1) % TOTAL_SCREENS;
      }
      screenCycleTime = millis();
    }
  }
  // When slot-locked, currentScreenIndex is managed by updateSlotScreenLock()
  // and the UP/DOWN handler in loop().

  switch (currentScreenIndex) {
    case 0: drawDateTimeScreen(h, m);      break;
    case 1: drawTempScreen();              break;
    case 2: drawSlot1Screen(slot0Active);  break;
    case 3: drawSlot2Screen(slot1Active);  break;
    case 4: drawAmbientScreen();           break;
    case 5: drawBtProvisionWaiting();      break;
  }
}

// ═══════════════════════════════════════════════════════════════════════════
//  SETUP
// ═══════════════════════════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  Serial.println("\n[BOOT] ========================================");
  Serial.println("[BOOT] Nuetech Controller v2.2 (WiFi+BLE)");
  Serial.println("[BOOT] ========================================");

  pinMode(BTN_SET,    INPUT_PULLUP);
  pinMode(BTN_UP,     INPUT_PULLUP);
  pinMode(BTN_DOWN,   INPUT_PULLUP);
  pinMode(RELAY_TANK, OUTPUT);
  digitalWrite(RELAY_TANK, RELAY_OFF);

  ledcAttach(BUZZER_PIN, BUZZER_FREQ, BUZZER_RES);
  buzzerOff();
  analogSetAttenuation(ADC_11db);

  tft.init();
  tft.setRotation(3);
  tft.fillScreen(TFT_BLACK);

  spr.setColorDepth(1);
  if (spr.createSprite(SCREEN_W, SCREEN_H)) {
    spr.setBitmapColor(TFT_WHITE, TFT_BLACK);
    sprReady = true;
    Serial.printf("[SPR] 1-bit mono sprite OK  heap: %u\n", ESP.getFreeHeap());
  } else {
    sprReady = false;
    Serial.println("[SPR] WARNING: sprite creation failed.");
  }

  Wire.begin(21, 22);
  Wire.setClock(100000);
  delay(50);

  if (!rtc.begin()) {
    Serial.println("[RTC] Not found!");
  } else {
    if (rtc.lostPower()) {
      rtc.adjust(DateTime(2026, 1, 1, 12, 0, 0));
      Serial.println("[RTC] Lost power — default set.");
    } else {
      DateTime now = rtc.now();
      Serial.printf("[RTC] %02d:%02d  %04d-%02d-%02d\n",
                    now.hour(), now.minute(), now.year(), now.month(), now.day());
    }
  }

  // ── EEPROM (AT24C32 on DS3231 module) ──────────────────────────────────
  if (eepromPresent()) {
    eepromFound = true;
    initLogHeader();
    Serial.println("[EEPROM] AT24C32 found at 0x50 — log storage ready.");
  } else {
    eepromFound = false;
    Serial.println("[EEPROM] AT24C32 not found at 0x50.");
  }

  esp_log_level_set("i2c.master", ESP_LOG_NONE);
  bool ahtFound = aht.begin();
  esp_log_level_set("i2c.master", ESP_LOG_ERROR);
  if (ahtFound) {
    ahtPresent = true; updateAHT20();
    Serial.printf("[AHT20] T=%.1f  H=%.1f%%\n", ahtTemp, ahtHumidity);
  } else {
    ahtPresent = false; ahtOk = false;
    Serial.println("[AHT20] Not found.");
  }

  loadCommsMode();
  loadBtName();
  Serial.printf("[MODE] Communications mode: %s\n", commsMode==MODE_BLE ? "BLE" : "WiFi");
  loadAll();
  loadWifiCreds();
  lastCoilSaveTime = millis();
  targetNextTick   = millis();

  showBootSplash();

  // ── BT WiFi provisioning prompt at boot ─────────────────────────────────
  // Show "PRESS SET" for 5 s.  If pressed → BT provision flow runs.
  if (commsMode == MODE_WIFI) {
    startBtProvisioning(true);
  }

  if (commsMode == MODE_BLE) {
    WiFi.mode(WIFI_OFF);
    SerialBT.begin(btDeviceName.c_str());
    Serial.printf("[BT] Bluetooth started as '%s'\n", btDeviceName.c_str());
    Serial.println("[BT] Send AT+NAME=<newname> via Serial Monitor to rename.");
  } else {
    if (btProvisioningActive) {
      Serial.println("[BOOT] Entering BT WiFi provisioning mode.");
      // State is already WIFI_BT_PROV_MODE, watchdog handles the rest
    } else {
      WiFi.mode(WIFI_STA);
      loadWifiCreds();
      Serial.printf("[BOOT] Connecting WiFi SSID='%s'...\n", wifiSSID.c_str());
      if (!wifiSSID.isEmpty() && connectWifi(wifiSSID, wifiPassword, 15000)) {
        startHttpServer(); wifiState = WIFI_OK;
        syncRtcFromNtp();
        Serial.println("[BOOT] WiFi OK. HTTP server started.");
      } else {
        wifiRetryCount = 0; lastWifiRetryAttempt = millis(); wifiState = WIFI_RETRYING;
        Serial.println("[BOOT] WiFi failed at boot — will retry silently.");
      }
    }
  }

  Serial.printf("[BOOT] Done. Free heap: %u\n", ESP.getFreeHeap());
}

// ═══════════════════════════════════════════════════════════════════════════
//  LOOP
// ═══════════════════════════════════════════════════════════════════════════
void loop() {
  feedWDT();

  // ── Serial Monitor AT commands (AT+NAME=xxx) ──────────────────────────────
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.startsWith("AT+NAME=")) {
      String newName = line.substring(8);
      newName.trim();
      if (newName.length() > 0 && newName.length() <= BT_NAME_MAX_LEN) {
        btDeviceName = newName;
        saveBtName();
        Serial.printf("[BT] Name changed to '%s'\n", btDeviceName.c_str());
        if (commsMode == MODE_BLE) {
          SerialBT.end();
          delay(200);
          SerialBT.begin(btDeviceName.c_str());
          Serial.printf("[BT] Bluetooth restarted as '%s'\n", btDeviceName.c_str());
          showPopup("BT Name Changed", btDeviceName.c_str());
          unsigned long s = millis(); while (millis()-s < 1500) { feedWDT(); }
        } else {
          Serial.println("[BT] Name saved. Will apply on next BLE boot.");
        }
      } else {
        Serial.printf("[BT] Invalid name. Must be 1-%d chars.\n", BT_NAME_MAX_LEN);
      }
    } else if (line.startsWith("AT+NAME?")) {
      Serial.printf("[BT] Current name: '%s'\n", btDeviceName.c_str());
    }
  }

  // ── Network handling ───────────────────────────────────────────────────────
  if (commsMode == MODE_WIFI) {
    handleWifiWatchdog();
    if (wifiState == WIFI_OK) server.handleClient();
    handleDailyNtpSync();
  } else {
    readBluetooth();
  }

  // ── Sensor reads ──────────────────────────────────────────────────────────
  static unsigned long lastAhtUpdate = 0;
  if (millis()-lastAhtUpdate > 5000) { lastAhtUpdate = millis(); updateAHT20(); }

  RT1 = readNTC(NTC_TANK);

  // ── Safety ────────────────────────────────────────────────────────────────
  enforceSafeRelay();
  broadcastErrors();
  handleBuzzer();

  // ── RTC / scheduling ──────────────────────────────────────────────────────
  int h, m; getTime(h, m);
  checkDailyReset();

  if (!postInitDone && timeIsSet) {
    pinMode(BTN_SET,   INPUT_PULLUP);
    pinMode(BTN_UP,    INPUT_PULLUP);
    pinMode(BTN_DOWN,  INPUT_PULLUP);
    postInitDone = true;
  }

  // ── E1 : Sensor fault ─────────────────────────────────────────────────────
  if (RT1 == -999) {
    slotScreenLocked = false; slotBrowsing = false;  // release lock on fault
    digitalWrite(RELAY_TANK, RELAY_OFF);
    slotRelayActive[0] = false; slotRelayActive[1] = false;
    coilEffectivenessActive = false;
    drawSensorError(); delay(100); return;
  }

  // ── E2 : Max coil on-time ─────────────────────────────────────────────────
  if (maxCoilError) {
    slotScreenLocked = false; slotBrowsing = false;
    digitalWrite(RELAY_TANK, RELAY_OFF);
    slotRelayActive[0] = false; slotRelayActive[1] = false;
    if (millis()-lastRelayCheck > 100) { updateCoilOnTime(false); lastRelayCheck = millis(); }
    static unsigned long maxCoilHoldStart = 0;
    if (pressed(BTN_SET)) {
      if (maxCoilHoldStart == 0) maxCoilHoldStart = millis();
      if (millis()-maxCoilHoldStart >= 5000UL) {
        coilOnTime = 0; maxCoilError = false; coilErrorAcknowledged = false;
        coilEffectivenessActive = false; tempAtCoilStart = 0;
        relayOffTimerActive = false; coilWasOn = false; maxCoilHoldStart = 0;
        saveAll();
        showPopup("MAX COIL", "Reset Done!");
        unsigned long s = millis(); while (millis()-s < 1500) { feedWDT(); }
        return;
      }
    } else { maxCoilHoldStart = 0; }
    drawMaxCoilError(); delay(100); return;
  }

  // ── E3 : Coil not working ─────────────────────────────────────────────────
  if (coilNotWorkingError) {
    slotScreenLocked = false; slotBrowsing = false;
    digitalWrite(RELAY_TANK, RELAY_OFF);
    slotRelayActive[0] = false; slotRelayActive[1] = false;
    drawCoilNotWorkingError();
    handleCoilErrorClearHold();
    if (!coilNotWorkingError) return;
    delay(100); return;
  }

  // ── Notification popup ────────────────────────────────────────────────────
  if (notifPopupActive && (millis()-notifPopupTime < 1500)) {
    enforceSafeRelay();
    showPopup(notifPopupMsg.c_str(), notifPopupMsg2.c_str());
    delay(100); return;
  } else if (notifPopupActive) { notifPopupActive = false; }

  // ── Hysteresis update ─────────────────────────────────────────────────────
  updateHysteresis();

  // ── Manual mode entry (hold SET ≥ 2 s) ───────────────────────────────────
  static bool holdActive = false;
  if (pressed(BTN_SET)) {
    if (!holdActive) { holdActive = true; setHoldStart = millis(); }
    // Guard: if a blocking call happened (connectWifi, syncNtp, etc.) the
    // hold timer may be stale.  Require a fresh continuous press — if more
    // than 3 s elapsed since the hold started, restart the timer.
    if (millis()-setHoldStart > 3000) { setHoldStart = millis(); }
    if (!manualMode && (millis()-setHoldStart >= 2000)) {
      beginScreen("< MANUAL >", P_BORDER);
      bigCentred(80,  "MANUAL", 4, P_BORDER);
      bigCentred(130, "MODE",   3, P_BORDER);
      sprPush(); delay(800);
      manualMode = true; editItem = 0; editField = 0;
      editActive = false; pinState = PIN_IDLE;
      manualEntryTime = millis(); holdActive = false; postInitDone = false;
      // Release slot-lock while in manual mode
      slotScreenLocked = false; slotBrowsing = false;
      Serial.println("[MANUAL] Entered.");
    }
  } else { holdActive = false; }

  // ── Manual mode handler ───────────────────────────────────────────────────
  if (manualMode) {
    enforceSafeRelay();
    drawManual();
    if (pressed(BTN_SET) || pressed(BTN_UP) || pressed(BTN_DOWN))
      manualEntryTime = millis();
    else if (millis()-manualEntryTime > 8000) {
      manualMode = false; editActive = false;
      pinState = PIN_IDLE; pinEnteredVal = 0; postInitDone = false;
      // Restore slot lock state on manual exit (will be re-evaluated next loop)
      slotBrowsing = false;
      // Revert to last-saved values — discard any invalid uncommitted edits
      loadAll();
      Serial.println("[MANUAL] Timeout — exiting. Reverted unsaved changes.");
      delay(100); return;
    }

    if (pinState == PIN_ENTERING) {
      if (buttonPressed(BTN_UP))   { pinEnteredVal++; manualEntryTime = millis(); }
      if (buttonPressed(BTN_DOWN)) { pinEnteredVal = 0; manualEntryTime = millis(); }
      if (buttonPressed(BTN_SET)) {
        if (pinEnteredVal == PIN_SECRET) {
          pinState = PIN_GRANTED; editActive = true; editField = 0;
          pinEnteredVal = 0; manualEntryTime = millis();
        } else {
          pinEnteredVal = 0; pinState = PIN_IDLE;
          showPopup("Wrong PIN", "Access Denied");
          unsigned long s = millis(); while (millis()-s < 1200) { feedWDT(); }
          manualEntryTime = millis();
        }
      }
      return;
    }

    if (!editActive) {
      if (buttonPressed(BTN_UP))   { editItem = (editItem+1) % MANUAL_ITEMS; manualEntryTime = millis(); }
      if (buttonPressed(BTN_DOWN)) { editItem = (editItem==0) ? MANUAL_ITEMS-1 : editItem-1; manualEntryTime = millis(); }
      if (buttonPressed(BTN_SET)) {
        if (editItem == 5 || editItem == 6) { /* read-only */ }
        else if (editItem == 7 || editItem == 8 || editItem == 9) { pinState = PIN_ENTERING; pinEnteredVal = 0; pinTarget = editItem; }
        else if (editItem == 12) {
          saveAll();
          commsMode = (commsMode == MODE_BLE) ? MODE_WIFI : MODE_BLE;
          saveCommsMode();
          const char* newModeName = (commsMode==MODE_BLE) ? "Bluetooth" : "WiFi";
          Serial.printf("[MODE] Switching to %s — restarting...\n", newModeName);
          showPopup("Mode Switching!", newModeName);
          unsigned long s = millis(); while (millis()-s < 1800) { feedWDT(); }
          ESP.restart();
        }
        else { editActive = true; editField = 0; }
        manualEntryTime = millis();
      }
      return;
    }

    if (editItem == 0) {
      if (buttonPressed(BTN_UP))   { ST1++; if (ST1>TEMP_MAX) ST1=TEMP_MIN; tempHysteresisReady=(RT1<ST1); manualEntryTime=millis(); }
      if (buttonPressed(BTN_DOWN)) { ST1--; if (ST1<TEMP_MIN) ST1=TEMP_MAX; tempHysteresisReady=(RT1<ST1); manualEntryTime=millis(); }
      if (buttonPressed(BTN_SET))  { editActive=false; manualEntryTime=millis(); }

    } else if (editItem >= 1 && editItem <= 4) {
      int i = (editItem-1)/2; bool start = (editItem%2) == 1;
      int &H = start ? S_H[i] : E_H[i];
      int &M = start ? S_M[i] : E_M[i];
      if (buttonPressed(BTN_SET)) {
        if (editField == 1) {
          if (start) {
            // Auto-set end time = start + 30 min when start is confirmed
            int newEndMin = H*60 + M + MIN_SLOT_GAP_MINS;
            int newEH = newEndMin / 60;
            int newEM = newEndMin % 60;
            int maxH = slotMaxHour(i);
            if (newEH > maxH) { newEH = maxH; newEM = 59; }
            E_H[i] = newEH;
            E_M[i] = newEM;
            editActive = false; editItem = (editItem+1) % MANUAL_ITEMS;
          } else {
            // Validate: minimum 30 min gap between start and end
            int gap = (E_H[i]*60+E_M[i]) - (S_H[i]*60+S_M[i]);
            if (gap < MIN_SLOT_GAP_MINS) {
              showPopup("Need 30 min", "Gap START to END");
              unsigned long s = millis(); while (millis()-s < 1200) { feedWDT(); }
              editField = 0;  // go back to hour editing
            } else {
              editActive = false; editItem = (editItem+1) % MANUAL_ITEMS;
            }
          }
        }
        else { editField = 1; }
        manualEntryTime = millis();
      }
      if (editField == 0) {
        int hMin = slotMinHour(i), hMax = slotMaxHour(i);
        if (buttonPressed(BTN_UP))   { H = constrain(H+1, hMin, hMax); manualEntryTime=millis(); }
        if (buttonPressed(BTN_DOWN)) { H = constrain(H-1, hMin, hMax); manualEntryTime=millis(); }
      } else {
        if (buttonPressed(BTN_UP))   { M = (M+1)%60;  manualEntryTime=millis(); }
        if (buttonPressed(BTN_DOWN)) { M = (M+59)%60; manualEntryTime=millis(); }
      }

    } else if (editItem == 7) {
      if (buttonPressed(BTN_UP))   { coilEffCheckMinutes=constrain(coilEffCheckMinutes+1,5,120); coilEffCheckDurationMs=(unsigned long)coilEffCheckMinutes*60000UL; manualEntryTime=millis(); }
      if (buttonPressed(BTN_DOWN)) { coilEffCheckMinutes=constrain(coilEffCheckMinutes-1,5,120); coilEffCheckDurationMs=(unsigned long)coilEffCheckMinutes*60000UL; manualEntryTime=millis(); }
      if (buttonPressed(BTN_SET))  { editItem=8; manualEntryTime=millis(); }

    } else if (editItem == 8) {
      if (buttonPressed(BTN_UP))   { coilEffMinRiseDeg=constrain(coilEffMinRiseDeg+1,1,10); manualEntryTime=millis(); }
      if (buttonPressed(BTN_DOWN)) { coilEffMinRiseDeg=constrain(coilEffMinRiseDeg-1,1,10); manualEntryTime=millis(); }
      if (buttonPressed(BTN_SET))  { editActive=false; pinState=PIN_IDLE; manualEntryTime=millis(); }

    } else if (editItem == 9) {
      if (buttonPressed(BTN_UP))   { preHeatSteps=constrain(preHeatSteps+1,0,4); manualEntryTime=millis(); }
      if (buttonPressed(BTN_DOWN)) { preHeatSteps=constrain(preHeatSteps-1,0,4); manualEntryTime=millis(); }
      if (buttonPressed(BTN_SET))  { editActive=false; pinState=PIN_IDLE; manualEntryTime=millis(); }

    } else if (editItem == 10) {
      if (buttonPressed(BTN_UP) || buttonPressed(BTN_DOWN)) { slotEnabled[0] = !slotEnabled[0]; manualEntryTime=millis(); }
      if (buttonPressed(BTN_SET)) { editActive=false; manualEntryTime=millis(); }

    } else if (editItem == 11) {
      if (buttonPressed(BTN_UP) || buttonPressed(BTN_DOWN)) { slotEnabled[1] = !slotEnabled[1]; manualEntryTime=millis(); }
      if (buttonPressed(BTN_SET)) { editActive=false; manualEntryTime=millis(); }

    } else {
      if (buttonPressed(BTN_SET)) { editActive=false; manualEntryTime=millis(); }
    }
    // Only save if all enabled slots have valid schedules
    bool slotsOk = true;
    for (int i = 0; i < 2; i++) {
      if (slotEnabled[i] && !isValidSchedule(i)) { slotsOk = false; break; }
    }
    if (slotsOk) saveAll();
    return;
  }

  // ── 1 Hz slot accumulation timer ──────────────────────────────────────────
  if (millis() >= targetNextTick) {
    if (millis()-lastTick >= 900) {
      lastTick = millis();
      for (int i = 0; i < 2; i++) {
        if (slotRelayActive[i]) slotDailyOnTime[i]++;
      }
      // Track peak tank temperature per slot for daily log
      if (RT1 != -999) {
        if (slotRelayActive[0] && RT1 > mornPeakTemp) mornPeakTemp = RT1;
        if (slotRelayActive[1] && RT1 > evePeakTemp) evePeakTemp = RT1;
      }
      // Accumulate error flags throughout the day
      dayErrorAccum |= getActiveErrorMask();
      targetNextTick += 1000UL; saveAll();
    }
  }

  // ── Slot relay decision (evaluated every 1 s) ─────────────────────────────
  if (millis()-lastRelayUpdate > 1000) {
    for (int i = 0; i < 2; i++) {
      if (!isValidSchedule(i)) { slotRelayActive[i] = false; continue; }
      bool inSlot  = (activeSlot(h, m)==i || preheatSlot(h, m)==i);
      bool tempOK  = (RT1 != -999 && tempHysteresisReady);
      bool shouldRun = inSlot && tempOK && !maxCoilError && !coilNotWorkingError;
      slotRelayActive[i] = shouldRun;
    }
    lastRelayUpdate = millis();
  }

  // ── Relay output ──────────────────────────────────────────────────────────
  bool relayOn = slotRelayActive[0] || slotRelayActive[1];
  if (RT1 == -999 || maxCoilError || coilNotWorkingError) relayOn = false;
  digitalWrite(RELAY_TANK, relayOn ? RELAY_ON : RELAY_OFF);

  // ── Coil tracking ─────────────────────────────────────────────────────────
  if (millis()-lastRelayCheck > 100) {
    bool rState = (digitalRead(RELAY_TANK) == RELAY_ON);
    updateCoilOnTime(rState && RT1 != -999);
    updateCoilEffectivenessCheck(rState && RT1 != -999);
    lastRelayCheck = millis();
  }

  // ── Determine active slot states for screen logic ──────────────────────────
  int act = activeSlot(h, m), pre = preheatSlot(h, m);
  bool slot0Active = (act==0 || pre==0);
  bool slot1Active = (act==1 || pre==1);

  // ── [C6] UP/DOWN browsing while slot-screen is locked ─────────────────────
  //
  //  When a slot is running and the screen is locked:
  //    • UP   → advance to the next screen (wraps)
  //    • DOWN → go back to the previous screen
  //    • Any press resets the 10 s browse-idle timer
  //    • SET is NOT consumed here (it is reserved for manual-mode entry)
  //
  //  When no slot is running (normal mode) UP/DOWN do nothing in this block
  //  so they remain available for future use without conflict.
  // ─────────────────────────────────────────────────────────────────────────
  if (slotScreenLocked) {
    if (buttonPressed(BTN_UP)) {
      slotBrowsing        = true;
      slotBrowseLastPress = millis();
      currentScreenIndex = (currentScreenIndex + 1) % TOTAL_SCREENS;
      if (currentScreenIndex == 5 && !btProvisioningActive) {
        currentScreenIndex = (currentScreenIndex + 1) % TOTAL_SCREENS;
      }
    }
    if (buttonPressed(BTN_DOWN)) {
      slotBrowsing        = true;
      slotBrowseLastPress = millis();
      currentScreenIndex = (currentScreenIndex - 1 + TOTAL_SCREENS) % TOTAL_SCREENS;
      if (currentScreenIndex == 5 && !btProvisioningActive) {
        currentScreenIndex = (currentScreenIndex - 1 + TOTAL_SCREENS) % TOTAL_SCREENS;
      }
    }
  }

  // ── Draw normal cycling screen ────────────────────────────────────────────
  drawScreen(slot0Active, slot1Active);
  delay(100);
}
////////////