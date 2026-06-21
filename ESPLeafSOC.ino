// ESPLeafSOC.ino
// ==============
// ESP32 / LilyGo T-CAN485 refactor of Paul Kennett's LeafSOCdisplay
// Original project: https://github.com/PaulKennett/LeafSOCdisplay
// This fork:        https://github.com/Mozzie-AU/ESPLeafSOC
//
// Authors:
//   Ray (Mozzie-AU) - hardware build, testing, project direction
//   Claude (Anthropic AI) - ESP32 refactor, code architecture assistance
//
// Key changes from original Arduino Nano version:
//   - ESP32 native TWAI CAN controller replaces external MCP2515 SPI module
//   - OLED retains SPI interface (hardware jumpered on existing installs)
//   - Settings (page, km/kWh, pack type) via WiFi web portal replaces HVAC button UI
//   - Settings stored in ESP32 NVS (Preferences) replaces Arduino EEPROM
//   - SOC% is read directly from the BMS (CAN 0x55B) - no pack type/maxGids needed
//   - kWh and range remain GIDS-based (CAN 0x5BC) for finer resolution
//   - WS2812 RGB LED provides power/CAN receive diagnostics
//   - Only CAN messages 0x5BC (GIDS) and 0x55B (SOC%) are processed
//
// Hardware: LilyGo T-CAN485 (confirmed from LilyGO pinout diagram)
//   CAN TX:  GPIO27 (required by TWAI driver but not driven - listen-only mode)
//   CAN RX:  GPIO26
//   CAN SE:  GPIO23 (SN65HVD231 silent/enable - must be driven LOW to enable RX)
//   WS2812:  GPIO4  (onboard RGB LED)
//   OLED SPI (hardware SPI, 12-pin IO header):
//     CLK:   GPIO33
//     MOSI:  GPIO32
//     CS:    GPIO25
//     DC:    GPIO18
//     RST:   GPIO5 (IO05) - GPIO35 is input-only on ESP32
//
// Libraries required:
//   U8g2          - OLED display (https://github.com/olikraus/u8g2)
//   ESPAsyncWebServer + AsyncTCP - WiFi config portal
//   Preferences   - NVS storage (built into ESP32 Arduino core)
//   FastLED       - WS2812 LED
//   driver/twai.h - CAN bus (built into ESP32 Arduino core)
//
// Changelog:
//   v01 - Initial ESP32/T-CAN485 skeleton. I2C OLED (superseded)
//   v02 - Switch OLED to hardware SPI. Dual constructor (SH1106/SSD1306).
//         Added Claude contribution note. SPI.begin() fix for custom pins.
//   v03 - Runtime display rotation (0/180) in web portal and NVS.
//         Vertical "km" label on page 1 to save horizontal space.
//         Smaller font for splash screen / page 4.
//         Test mode in web portal - cycle through display pages.
//         RST pin confirmed GPIO5 (GPIO35 is input-only on ESP32).
//   v04 - WiFi portal stays alive while client connected, 60s timeout after last disconnect.
//         mDNS added - portal accessible at http://espleafsoc.local
//         Splash screen line spacing tightened to fit github URL in 64px.
//   v05 - Page 1 restored to Paul's original layout with logisoso16 fonts for SOC%/kWh.
//         Vertical "km" stacked beside range number on page 1.
//         LINE1-4 defines added, shifted up 6px to avoid defective bottom pixel rows.
//         battery_small.h added for page 1 outline graphic.
//   v06 - Complete page layout rewrite based on actual display testing.
//         Page 1: fixed overlap between range number, battery graphic, SOC%, kWh.
//         Page 2: battery_large confirmed 128x41px, SOC% inside, % smaller font,
//                 kWh below at y=56.
//         Page 3: Range/km labels top, large number centred at y=54.
//   v07 - Page 1: SOC% and kWh side by side on single bottom line (y=62).
//         Page 2: fixed duplicate % bug, kWh dropped to y=62.
//         Page 3: range number dropped to y=62.
//         Full 64px vertical restored (second OLED has no defective bottom rows).
//   v08 - Fixed test mode display artifacts (thread safety).
//         updateDisplay() no longer called from async web handler.
//         displayNeedsUpdate flag used instead - main loop handles display updates.
//         testMode loop update rate limited to 100ms to avoid hammering display.
//   v09 - Page 2: % symbol switched to 6x10 small font, repositioned to x=90,y=28
//         to sit cleanly inside battery outline without overlap or glyph artifacts.
//         (Ray's manual refinements: page 1 SOC% repositioned to x=5,y=62 inside
//         battery outline footprint; page 2 duplicate % print calls commented out.)
//   v10 - Removed auto-learn maxGids stub - simplified to pack type default only
//         until T-2CAN dual-bus SoH (0x5B3) integration is implemented.
//         maxGids now fixed at pack type preset, adjustable only via web portal
//         pack type selection. WH_PER_GID clarified as usable-energy figure (75),
//         not stored-capacity figure (80) - see comment at definition.
//         Future: T-2CAN board (ESP32-S3, dual CAN) will read 0x5B3 SoH from
//         Car-CAN and compute maxGids = packTypeFullGids * (SoH/100), allowing
//         the displayed SOC% to track real battery health as the pack ages.
//   v11 - MaxGIDS field on web portal changed from read-only to editable.
//         Manual override applied when pack type is unchanged on save.
//         Selecting a different pack type still resets MaxGIDS to that preset,
//         overriding any manually entered value from before the change.
//   v12 - Added page 5: Diagnostics. Shows rawGids, maxGids, rawSoc/SocPct,
//         GidsPct, kWh, range, time since last CAN message, and pack type -
//         all on the OLED for bench testing without needing Serial Monitor.
//         Test mode and display page selector both extended to cover 5 pages.
//   v13 - Switched primary displayed SOC% (pages 1 and 2) from GidsPct to SocPct.
//         Live testing showed GidsPct stuck at 100% while car's own BMS reported
//         91.2% via 0x55B (rawSoc) - GIDS-based % cannot self-correct for actual
//         pack SoH (measured 81.2% via LeafSpy) without reading 0x5B3 (T-2CAN
//         project). The BMS already solves this problem, so we now trust it
//         directly for the displayed percentage. kWh and range remain GIDS-based
//         since GIDS still gives finer resolution for those. GidsPct retained
//         and still shown on diagnostics page 5 for comparison.
//         maxGids/pack type selection in web portal is now informational only
//         for kWh/range, no longer drives the primary SOC% figure - portal
//         simplification deferred to a later version.
//   v14 - Removed maxGids, packType, and GidsPct entirely. BMS-reported SOC%
//         (CAN 0x55B) is now the only displayed percentage, with no pack type
//         selection, presets, or manual MaxGIDS override needed. kWh and range
//         remain GIDS-based (CAN 0x5BC). Web portal simplified to: display page,
//         km/kWh, display rotation. NVS keys for pack type/maxGids no longer
//         written; any previously stored values are simply ignored.
//   v15 - OLED driver chip (SH1106/SSD1306) now selectable from the web portal
//         instead of a code recompile + reflash. Both U8G2 display objects are
//         instantiated; "u8g2" is now a U8G2* base-class pointer set in setup()
//         to whichever object matches the saved NVS setting. All u8g2.xxx()
//         call sites changed to u8g2->xxx() to support this. Driver choice is
//         saved to NVS; changing it triggers ESP.restart() since the active
//         object can only be selected during setup(), not swapped live.

// ============================================================
// TODO:
//   1. Verify GIDS decode bit-shift for 0x5BC matches your Leaf model year
//   2. Implement T-2CAN dual-bus SoH (0x5B3) integration once board arrives -
//      maxGids = packTypeFullGids * (SoH/100), replacing fixed preset
//   3. Verify WH_PER_GID (75 vs 80) against live data once CAN testing begins
// ============================================================

#include <Arduino.h>
#include <Preferences.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <ESPmDNS.h>
#include <U8g2lib.h>
#include <SPI.h>
#include <FastLED.h>
#include "driver/twai.h"
#include "battery_large.h"
#include "battery_solid.h"
#include "battery_small.h"

// ------------------------------------------------------------
// Version
// ------------------------------------------------------------
#define VERSION   "ESPLeafSOC v15"
#define DATE      "June 2026"
#define AUTHOR    "Mozzie-AU"

// ------------------------------------------------------------
// Display line positions (Y pixel, U8G2 baseline)
// Kept 6px clear of bottom to avoid defective pixel rows on some modules
// ------------------------------------------------------------
#define LINE1  16
#define LINE2  36
#define LINE3  54
#define LINE4  58

// ------------------------------------------------------------
// Hardware pin definitions (confirmed from LilyGO T-CAN485 pinout diagram)
// ------------------------------------------------------------
#define CAN_TX_PIN      27    // Required by TWAI driver - not actively driven (listen-only)
#define CAN_RX_PIN      26    // CAN RX
#define CAN_SE_PIN      23    // SN65HVD231 silent/enable - drive LOW to enable transceiver
#define OLED_CLK_PIN    33    // SPI clock  (12-pin IO header)
#define OLED_MOSI_PIN   32    // SPI data   (12-pin IO header)
#define OLED_CS_PIN     25    // SPI chip select (12-pin IO header)
#define OLED_DC_PIN     18    // Data/command    (12-pin IO header)
#define OLED_RST_PIN    5     // Changed from 35 - GPIO35 is input-only on ESP32
#define WS2812_PIN      4     // Onboard WS2812B RGB LED
#define WS2812_COUNT    1

// ------------------------------------------------------------
// CAN / Leaf constants
// ------------------------------------------------------------
#define GIDS_TURTLE     8         // GIDS at which Leaf enters turtle mode
// Wh per GID - community-estimated, not an official Nissan figure.
// Two values commonly quoted, measuring different things:
//   ~80 Wh/GID = stored/charged energy (matches LeafSpy "Wh", OVMS default)
//   ~75 Wh/GID = usable/discharge energy (round-trip losses + reserve)
// We use 75 here since kWh/range are meant to reflect real-world usable range,
// not nameplate stored capacity. Revisit if test data suggests otherwise.
#define WH_PER_GID      75.0F

// GIDS_TURTLE kept for potential future use in kWh/range calculations.
// Pack type presets and maxGids removed in v14 - SOC% now comes directly
// from the BMS (CAN 0x55B), making pack-size-based capacity guessing redundant.

// ------------------------------------------------------------
// WiFi AP settings
// ------------------------------------------------------------
#define WIFI_SSID       "ESPLeafSOC"
#define WIFI_PASSWORD   "leafsoc1"    // minimum 8 chars for WPA2
#define WIFI_PORTAL_TIMEOUT_S  60     // seconds AP stays active on boot

// ------------------------------------------------------------
// NVS key names (Preferences namespace "leafsoc")
// ------------------------------------------------------------
#define NVS_NAMESPACE   "leafsoc"
#define NVS_PAGE        "page"
#define NVS_KM_PER_KWH  "kmkwh"
#define NVS_ROTATION    "rotation"
#define NVS_OLED_DRIVER "oleddrv"

// ------------------------------------------------------------
// Display constructor
// ------------------------------------------------------------
// Hardware SPI - CLK=33, MOSI=32, CS=25, DC=18, RST=35
// U8G2_R0 = no rotation, U8G2_R2 = 180 degrees (match your physical mount)
//
// Display rotation is set at runtime from NVS via u8g2->setDisplayRotation()
// Constructors always use U8G2_R0 - rotation applied in setup() after loadSettings()
//
// Driver chip (SH1106 / SSD1306) is also selected at runtime from NVS, chosen
// via the web portal. Both display objects exist; u8g2 is a pointer to the
// common U8G2 base class, set to whichever object setup() decides to use.
// Changing driver type in the portal triggers a reboot to take effect.

U8G2_SH1106_128X64_NONAME_F_4W_HW_SPI u8g2_sh1106(U8G2_R0,
  /* cs=*/ OLED_CS_PIN, /* dc=*/ OLED_DC_PIN, /* reset=*/ OLED_RST_PIN);

U8G2_SSD1306_128X64_NONAME_F_4W_HW_SPI u8g2_ssd1306(U8G2_R0,
  /* cs=*/ OLED_CS_PIN, /* dc=*/ OLED_DC_PIN, /* reset=*/ OLED_RST_PIN);

// Pointer to whichever display object is active - set in setup() from NVS.
// All draw functions call u8g2->method() instead of u8g2->method().
U8G2* u8g2 = &u8g2_sh1106;

// ------------------------------------------------------------
// LED
// ------------------------------------------------------------
CRGB leds[WS2812_COUNT];

// LED state machine
enum LedState {
  LED_BOOT,         // Blue slow pulse - booting / WiFi AP active
  LED_CAN_OK,       // Green brief flash - CAN data receiving
  LED_NO_CAN,       // Red slow blink - no CAN data (timeout)
  LED_SAVED         // White double flash - settings saved
};
LedState ledState = LED_BOOT;
unsigned long ledLastUpdate = 0;

// ------------------------------------------------------------
// Runtime variables
// ------------------------------------------------------------
uint16_t rawGids     = 0;
uint16_t rawGids2    = 0;    // previous value - detect changes
uint16_t rawSoc      = 0;
// GidsPct removed in v14 - SOC% now comes directly from BMS (rawSoc/SocPct below)
float    SocPct      = 0.0F; // SOC% direct from BMS (cross-check)
float    kWh         = 0.0F; // energy remaining
int      range       = 0;    // estimated range km

unsigned long lastCanRxTime = 0;
#define CAN_TIMEOUT_MS  5000  // ms without CAN data before LED goes red

// ------------------------------------------------------------
// Settings (loaded from NVS on boot)
// ------------------------------------------------------------
Preferences prefs;

int      displayPage    = 1;
float    kmPerKwh       = 6.4F;
// packType and maxGids removed in v14 - no longer needed for SOC% calculation
int      displayRotation = 2;     // 0 = normal, 2 = 180 degrees (U8G2_R0 / U8G2_R2)
int      oledDriver      = 0;     // 0 = SH1106 (default, Keyestudio), 1 = SSD1306 (Seeedstudio)

// Test mode - cycles display pages for layout checking (not saved to NVS)
bool     testMode          = false;
int      testPage          = 1;
bool     displayNeedsUpdate = false;   // set by web handlers, actioned by main loop
unsigned long testLastUpdate = 0;      // rate limit test mode display updates

// ------------------------------------------------------------
// Web server
// ------------------------------------------------------------
AsyncWebServer server(80);
bool wifiActive = false;
unsigned long wifiStartTime = 0;

// ------------------------------------------------------------
// Forward declarations
// ------------------------------------------------------------
void loadSettings();
void saveSettings();
void initCAN();
void initWifi();
void stopWifi();
void handleWebRequests();
void processCanMessage(uint32_t id, uint8_t* data);
void updateDisplay();
void updateLed();
void drawPage1();
void drawPage2();
void drawPage3();
void drawPage4();
void drawPage5();

// ============================================================
// SETUP
// ============================================================
void setup() {
  Serial.begin(115200);
  Serial.println(VERSION);

  // LED init - show blue immediately on power up
  FastLED.addLeds<WS2812B, WS2812_PIN, GRB>(leds, WS2812_COUNT);
  FastLED.setBrightness(40);
  leds[0] = CRGB::Blue;
  FastLED.show();

  // Enable CAN transceiver - SN65HVD231 SE pin must be LOW to enable RX
  // If this pin floats high the transceiver goes silent and nothing is received
  pinMode(CAN_SE_PIN, OUTPUT);
  digitalWrite(CAN_SE_PIN, LOW);

  // Load settings from NVS
  loadSettings();

  // Select OLED driver object based on saved setting (default SH1106)
  u8g2 = (oledDriver == 1) ? (U8G2*)&u8g2_ssd1306 : (U8G2*)&u8g2_sh1106;

  // OLED init - hardware SPI on custom pins
  SPI.begin(OLED_CLK_PIN, -1, OLED_MOSI_PIN, OLED_CS_PIN);
  u8g2->begin();
  // Apply rotation from NVS setting (0=normal, 2=180 degrees)
  u8g2->setDisplayRotation(displayRotation == 2 ? U8G2_R2 : U8G2_R0);

  // Splash screen - 12px line spacing fits 5 lines within 64px height
  u8g2->clearBuffer();
  u8g2->setFont(u8g2_font_6x10_tr);
  u8g2->setCursor(0, 10);  u8g2->print(VERSION);
  u8g2->setCursor(0, 22);  u8g2->print("Original: Paul Kennett");
  u8g2->setCursor(0, 34);  u8g2->print(DATE);
  u8g2->setCursor(0, 46);  u8g2->print(AUTHOR);
  u8g2->setCursor(0, 58);  u8g2->print("github.com/Mozzie-AU");
  u8g2->sendBuffer();

  // Start WiFi config portal
  initWifi();

  // Start CAN
  initCAN();

  delay(1000);
}

// ============================================================
// MAIN LOOP
// ============================================================
void loop() {
  // Close WiFi portal after timeout - but only when no clients connected
  if (wifiActive) {
    if (WiFi.softAPgetStationNum() > 0) {
      // Client connected - reset timeout
      wifiStartTime = millis();
    } else if (millis() - wifiStartTime > WIFI_PORTAL_TIMEOUT_S * 1000UL) {
      stopWifi();
    }
  }

  // Read CAN messages
  twai_message_t message;
  if (twai_receive(&message, pdMS_TO_TICKS(10)) == ESP_OK) {
    if (!(message.rtr)) {
      processCanMessage(message.identifier, message.data);
    }
  }

  // Update display:
  //   - when GIDS value changes (normal operation)
  //   - when web handler sets displayNeedsUpdate flag (test mode / settings saved)
  //   - in test mode, refresh at 100ms rate so page shows promptly after Next Page tap
  unsigned long now = millis();
  bool timeToUpdate = (rawGids != rawGids2)
                   || displayNeedsUpdate
                   || (testMode && (now - testLastUpdate > 100));
  if (timeToUpdate) {
    displayNeedsUpdate = false;
    testLastUpdate = now;
    updateDisplay();
    rawGids2 = rawGids;
  }

  // Update LED state machine
  updateLed();
}

// ============================================================
// SETTINGS - NVS load/save
// ============================================================
void loadSettings() {
  prefs.begin(NVS_NAMESPACE, false);

  displayPage = prefs.getInt(NVS_PAGE, 1);
  if (displayPage < 1 || displayPage > 5) displayPage = 1;

  kmPerKwh = prefs.getFloat(NVS_KM_PER_KWH, 6.4F);
  if (kmPerKwh < 1.0F || kmPerKwh > 20.0F) kmPerKwh = 6.4F;

  // Load display rotation (0=normal, 2=180 degrees) - default 2 for existing installs
  displayRotation = prefs.getInt(NVS_ROTATION, 2);
  if (displayRotation != 0 && displayRotation != 2) displayRotation = 2;

  // Load OLED driver type (0=SH1106, 1=SSD1306)
  oledDriver = prefs.getInt(NVS_OLED_DRIVER, 0);
  if (oledDriver != 0 && oledDriver != 1) oledDriver = 0;

  prefs.end();

  Serial.printf("Settings loaded: page=%d kmPerKwh=%.1f rotation=%d oledDriver=%d\n",
                displayPage, kmPerKwh, displayRotation, oledDriver);
}

void saveSettings() {
  prefs.begin(NVS_NAMESPACE, false);
  prefs.putInt(NVS_PAGE, displayPage);
  prefs.putFloat(NVS_KM_PER_KWH, kmPerKwh);
  prefs.putInt(NVS_ROTATION, displayRotation);
  prefs.putInt(NVS_OLED_DRIVER, oledDriver);
  prefs.end();
  // Apply new rotation immediately without reboot
  u8g2->setDisplayRotation(displayRotation == 2 ? U8G2_R2 : U8G2_R0);
  ledState = LED_SAVED;
  Serial.println("Settings saved to NVS");
}

// ============================================================
// CAN - init and message processing
// ============================================================
void initCAN() {
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
    (gpio_num_t)CAN_TX_PIN,
    (gpio_num_t)CAN_RX_PIN,
    TWAI_MODE_LISTEN_ONLY   // receive only - does not ACK or transmit
  );
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
    Serial.println("ERROR: TWAI driver install failed");
    return;
  }
  if (twai_start() != ESP_OK) {
    Serial.println("ERROR: TWAI start failed");
    return;
  }
  Serial.println("TWAI CAN started (listen-only, 500kbps)");
}

void processCanMessage(uint32_t id, uint8_t* data) {
  if (id == 0x5BC) {
    // GIDS - raw battery capacity (500ms interval)
    rawGids = (data[0] << 2) | (data[1] >> 6);

    // SOC% no longer calculated from GIDS (see SocPct from 0x55B below) -
    // GIDS is retained purely for kWh and range, which benefit from its
    // finer resolution compared to the BMS's 0.1% SOC steps.
    kWh   = ((float)rawGids * WH_PER_GID) / 1000.0F;
    range = (int)(kmPerKwh * ((rawGids - GIDS_TURTLE) * WH_PER_GID) / 1000.0F);

    lastCanRxTime = millis();
    ledState = LED_CAN_OK;

  } else if (id == 0x55B) {
    // SOC% direct from BMS (100ms interval) - primary displayed value.
    // The BMS already accounts for actual pack State of Health, which a
    // GIDS-based calculation cannot do without a separate SoH reading
    // (CAN 0x5B3, Car-CAN bus - see T-2CAN project notes).
    rawSoc  = (data[0] << 2) | (data[1] >> 6);
    SocPct  = rawSoc / 10.0F;
  }
}

// ============================================================
// WIFI - config portal
// ============================================================
void initWifi() {
  WiFi.softAP(WIFI_SSID, WIFI_PASSWORD);
  Serial.printf("WiFi AP started: SSID=%s IP=%s\n",
                WIFI_SSID, WiFi.softAPIP().toString().c_str());

  // Start mDNS - portal accessible at http://espleafsoc.local
  if (MDNS.begin("espleafsoc")) {
    Serial.println("mDNS started: http://espleafsoc.local");
    MDNS.addService("http", "tcp", 80);
  } else {
    Serial.println("mDNS start failed - use 192.168.4.1");
  }

  // Serve config page
  server.on("/", HTTP_GET, [](AsyncWebServerRequest* request) {
    String html = "<!DOCTYPE html><html><head>"
      "<meta name='viewport' content='width=device-width,initial-scale=1'>"
      "<title>ESPLeafSOC Config</title>"
      "<style>body{font-family:sans-serif;max-width:400px;margin:20px auto;padding:0 10px}"
      "h2{color:#2a6;}label{display:block;margin-top:12px;font-weight:bold}"
      "select,input{width:100%;padding:6px;margin-top:4px;font-size:1em}"
      "input[type=submit]{background:#2a6;color:#fff;border:none;padding:12px;"
      "margin-top:20px;cursor:pointer;border-radius:4px}"
      ".info{color:#666;font-size:0.85em;margin-top:4px}</style></head><body>"
      "<h2>ESPLeafSOC Settings</h2>"
      "<form action='/save' method='POST'>"

      "<label>Display Page</label>"
      "<select name='page'>"
      "<option value='1'" + String(displayPage==1?" selected":"") + ">1 - Range + SOC% + kWh</option>"
      "<option value='2'" + String(displayPage==2?" selected":"") + ">2 - Large SOC% + kWh</option>"
      "<option value='3'" + String(displayPage==3?" selected":"") + ">3 - Range only</option>"
      "<option value='4'" + String(displayPage==4?" selected":"") + ">4 - Version info</option>"
      "<option value='5'" + String(displayPage==5?" selected":"") + ">5 - Diagnostics</option>"
      "</select>"

      "<label>km per kWh</label>"
      "<input type='number' name='kmkwh' min='1' max='20' step='0.1' value='"
      + String(kmPerKwh, 1) + "'>"
      "<div class='info'>Your average efficiency. Used to calculate range estimate.</div>"

      "<label>Display Rotation</label>"
      "<select name='rotation'>"
      "<option value='2'" + String(displayRotation==2?" selected":"") + ">180° (default - existing installs)</option>"
      "<option value='0'" + String(displayRotation==0?" selected":"") + ">0° (normal)</option>"
      "</select>"
      "<div class='info'>Match your physical display mount orientation.</div>"

      "<label>OLED Driver Chip</label>"
      "<select name='oleddrv'>"
      "<option value='0'" + String(oledDriver==0?" selected":"") + ">SH1106 (Keyestudio - default)</option>"
      "<option value='1'" + String(oledDriver==1?" selected":"") + ">SSD1306 (Seeedstudio)</option>"
      "</select>"
      "<div class='info'>Match your physical OLED module. "
      "Changing this restarts the device to apply.</div>"

      "<input type='submit' value='Save Settings'>"
      "</form>"

      "<hr style='margin-top:24px'>"
      "<h3>Display Test Mode</h3>"
      "<p class='info'>Cycle through display pages to check layout. "
      "Test mode is active while WiFi portal is open.</p>"
      "<form action='/test' method='POST' style='display:inline'>"
      "<button style='background:#a62;color:#fff;border:none;padding:10px 20px;"
      "border-radius:4px;cursor:pointer;font-size:1em'>Next Page (now: "
      + String(testMode ? testPage : displayPage) + ")</button>"
      "</form>"
      "<form action='/testoff' method='POST' style='display:inline;margin-left:10px'>"
      "<button style='background:#666;color:#fff;border:none;padding:10px 20px;"
      "border-radius:4px;cursor:pointer;font-size:1em'>Exit Test</button>"
      "</form>"
      "</body></html>";
    request->send(200, "text/html", html);
  });

  server.on("/save", HTTP_POST, [](AsyncWebServerRequest* request) {
    if (request->hasParam("page", true))
      displayPage = request->getParam("page", true)->value().toInt();
    if (request->hasParam("kmkwh", true))
      kmPerKwh = request->getParam("kmkwh", true)->value().toFloat();
    if (request->hasParam("rotation", true)) {
      displayRotation = request->getParam("rotation", true)->value().toInt();
      if (displayRotation != 0 && displayRotation != 2) displayRotation = 2;
    }
    bool driverChanged = false;
    if (request->hasParam("oleddrv", true)) {
      int newDriver = request->getParam("oleddrv", true)->value().toInt();
      if (newDriver != 0 && newDriver != 1) newDriver = 0;
      if (newDriver != oledDriver) {
        oledDriver = newDriver;
        driverChanged = true;
      }
    }
    saveSettings();
    displayNeedsUpdate = true;       // main loop handles display refresh

    if (driverChanged) {
      // OLED driver class can't be swapped without re-running setup() -
      // restart the device so it boots using the newly selected driver.
      request->send(200, "text/html",
        "<html><body style='font-family:sans-serif;max-width:400px;margin:20px auto'>"
        "<h2 style='color:#a62'>Restarting...</h2>"
        "<p>OLED driver changed. Device is restarting to apply this - "
        "give it a few seconds, then reconnect to the WiFi portal if needed.</p>"
        "</body></html>");
      delay(500);   // let the response actually get sent before rebooting
      ESP.restart();
      return;
    }

    request->send(200, "text/html",
      "<html><body style='font-family:sans-serif;max-width:400px;margin:20px auto'>"
      "<h2 style='color:#2a6'>Settings Saved</h2>"
      "<p>Device will use new settings immediately.</p>"
      "<a href='/'>Back</a></body></html>");
  });

  // Test mode - advance to next page
  server.on("/test", HTTP_POST, [](AsyncWebServerRequest* request) {
    testMode = true;
    testPage = (testPage % 5) + 1;  // cycle 1->2->3->4->5->1
    displayNeedsUpdate = true;       // main loop handles actual display update
    request->send(200, "text/html",
      "<html><body style='font-family:sans-serif;max-width:400px;margin:20px auto'>"
      "<h2 style='color:#a62'>Test Mode - Page " + String(testPage) + "</h2>"
      "<p>Display now showing page " + String(testPage) + ".</p>"
      "<a href='/'>Back</a></body></html>");
  });

  // Exit test mode
  server.on("/testoff", HTTP_POST, [](AsyncWebServerRequest* request) {
    testMode = false;
    testPage = 1;
    displayNeedsUpdate = true;       // main loop handles actual display update
    request->send(200, "text/html",
      "<html><body style='font-family:sans-serif;max-width:400px;margin:20px auto'>"
      "<h2 style='color:#2a6'>Test Mode Off</h2>"
      "<p>Display returning to page " + String(displayPage) + ".</p>"
      "<a href='/'>Back</a></body></html>");
  });

  server.begin();
  wifiActive = true;
  wifiStartTime = millis();
}

void stopWifi() {
  server.end();
  MDNS.end();
  WiFi.softAPdisconnect(true);
  wifiActive = false;
  Serial.println("WiFi AP stopped");
  if (ledState == LED_BOOT) ledState = LED_NO_CAN;
}

// ============================================================
// DISPLAY
// ============================================================
void updateDisplay() {
  int page = testMode ? testPage : displayPage;
  switch (page) {
    case 1: drawPage1(); break;
    case 2: drawPage2(); break;
    case 3: drawPage3(); break;
    case 4: drawPage4(); break;
    case 5: drawPage5(); break;
    default: drawPage1(); break;
  }
}

void drawPage1() {
  // Layout (128x64 display, full height used):
  //   "Range" label top left, large range number, vertical "km" right edge,
  //   battery_small outline bottom left, SOC% inside battery footprint (x=5),
  //   kWh alongside at x=62, both on bottom line y=62.
  char buf[8];
  u8g2->clearBuffer();

  // "Range" label top left - medium font
  u8g2->setFont(u8g2_font_logisoso16_tr);
  u8g2->setCursor(0, 34);
  u8g2->print("Range");

  // Large range number - centre top area
  u8g2->setFont(u8g2_font_logisoso32_tn);
  u8g2->setCursor(50, 38);
  if (rawGids != 0) {
    dtostrf(range, 3, 0, buf);
    u8g2->print(buf);
  } else {
    u8g2->print("---");
  }

  // "km" stacked vertically right edge
  u8g2->setFont(u8g2_font_logisoso16_tr);
  u8g2->setCursor(111, 20);
  u8g2->print("k");
  u8g2->setCursor(111, 36);
  u8g2->print("m");

  // Battery outline graphic bottom left (56x24 starting at y=40)
  u8g2->drawXBM(0, 40, 56, 24, battery_small_bits);

  // SOC% and kWh side by side on bottom line, SOC% inside battery outline footprint
  // SOC% now sourced directly from BMS (rawSoc/SocPct via CAN 0x55B) rather than
  // calculated from GIDS - the BMS already accounts for actual pack SoH, whereas
  // our GIDS-based calculation could not without knowing real current capacity.
  u8g2->setFont(u8g2_font_logisoso16_tr);
  u8g2->setCursor(5, 62);
  if (rawSoc != 0) {
    dtostrf(SocPct, 3, 0, buf);
    u8g2->print(buf);
    u8g2->print("%");
  } else {
    u8g2->print("---%");
  }
  u8g2->setFont(u8g2_font_logisoso16_tr);
  u8g2->setCursor(62, 62);
  if (rawGids != 0) {
    dtostrf(kWh, 4, 1, buf);
    u8g2->print(buf);
    u8g2->print("kWh");
  } else {
    u8g2->print("--.-kWh");
  }

  u8g2->sendBuffer();
}

void drawPage2() {
  // battery_large is 128x41px - draws from y=0 to y=40
  // SOC% number centred inside battery outline (% intentionally suppressed -
  // see commented print calls below, Ray's choice to avoid clutter/overlap)
  // kWh below battery graphic at y=62, full 64px available
  char buf[8];
  u8g2->clearBuffer();

  // Battery large outline graphic - full width, top of screen
  u8g2->drawXBMP(0, 0, bitmap_width, bitmap_height, battery_large_bits);

  // SOC% inside battery - logisoso26 numerals
  // Sourced directly from BMS (rawSoc/SocPct via CAN 0x55B) - see page 1 comment.
  // Battery interior roughly x=10 to x=118, y=4 to y=37
  u8g2->setFont(u8g2_font_logisoso26_tn);
  u8g2->setCursor(28, 32);
  if (rawSoc != 0) {
    dtostrf(SocPct, 3, 0, buf);
    u8g2->print(buf);
    u8g2->setFont(u8g2_font_6x10_tr);
    u8g2->setCursor(90, 28);
    //u8g2->print("%");
  } else {
    u8g2->print("---");
    u8g2->setFont(u8g2_font_6x10_tr);
    u8g2->setCursor(90, 28);
    //u8g2->print("%");
  }

  // kWh below battery - medium font, y=62 full 64px available
  u8g2->setFont(u8g2_font_logisoso16_tr);
  u8g2->setCursor(20, 62);
  if (rawGids != 0) {
    dtostrf(kWh, 4, 1, buf);
    u8g2->print(buf);
    u8g2->print(" kWh");
  } else {
    u8g2->print(" --.- kWh");
  }
  u8g2->sendBuffer();
}

void drawPage3() {
  // Range only - large number centre screen
  // "Range" top left, "km" top right, large number centre
  char buf[8];
  u8g2->clearBuffer();

  // "Range" top left, "km" top right - medium font, baseline y=16
  u8g2->setFont(u8g2_font_logisoso16_tr);
  u8g2->setCursor(0, 16);
  u8g2->print("Range");
  u8g2->setCursor(103, 16);
  u8g2->print("km");

  // Large range number - full 64px available, baseline y=62
  u8g2->setFont(u8g2_font_logisoso32_tn);
  u8g2->setCursor(44, 62);
  if (rawGids != 0) {
    dtostrf(range, 3, 0, buf);
    u8g2->print(buf);
  } else {
    u8g2->print("---");
  }
  u8g2->sendBuffer();
}

void drawPage4() {
  // Version info - small font, 12px spacing fits 5 lines in 64px
  u8g2->clearBuffer();
  u8g2->setFont(u8g2_font_6x10_tr);
  u8g2->setCursor(0, 10);  u8g2->print(VERSION);
  u8g2->setCursor(0, 22);  u8g2->print("Paul Kennett (original)");
  u8g2->setCursor(0, 34);  u8g2->print(DATE);
  u8g2->setCursor(0, 46);  u8g2->print(AUTHOR);
  u8g2->setCursor(0, 58);  u8g2->print("github.com/Mozzie-AU");
  u8g2->sendBuffer();
}

void drawPage5() {
  // Diagnostics page - raw values for bench testing.
  // Small font, lines at 10px spacing fit comfortably in 64px.
  char buf[16];
  u8g2->clearBuffer();
  u8g2->setFont(u8g2_font_6x10_tr);

  u8g2->setCursor(0, 9);
  snprintf(buf, sizeof(buf), "rawGids: %u", rawGids);
  u8g2->print(buf);

  u8g2->setCursor(0, 19);
  snprintf(buf, sizeof(buf), "rawSoc: %u", rawSoc);
  u8g2->print(buf);

  u8g2->setCursor(0, 29);
  snprintf(buf, sizeof(buf), "SOC(BMS): %.1f%%", SocPct);
  u8g2->print(buf);

  u8g2->setCursor(0, 39);
  snprintf(buf, sizeof(buf), "kWh: %.2f", kWh);
  u8g2->print(buf);

  u8g2->setCursor(0, 49);
  snprintf(buf, sizeof(buf), "Range: %d km", range);
  u8g2->print(buf);

  u8g2->setCursor(0, 59);
  unsigned long secsSinceRx = lastCanRxTime > 0 ? (millis() - lastCanRxTime) / 1000 : 0;
  snprintf(buf, sizeof(buf), "CAN age: %lus", secsSinceRx);
  u8g2->print(buf);

  u8g2->sendBuffer();
}

// ============================================================
// LED state machine
// ============================================================
void updateLed() {
  unsigned long now = millis();

  // Check for CAN timeout
  if (lastCanRxTime > 0 && (now - lastCanRxTime > CAN_TIMEOUT_MS)) {
    if (ledState == LED_CAN_OK) ledState = LED_NO_CAN;
  }

  switch (ledState) {
    case LED_BOOT: {
      // Blue slow pulse using sine wave
      uint8_t brightness = (uint8_t)(127.5F + 127.5F * sin(now / 500.0F));
      leds[0] = CRGB(0, 0, brightness);
      FastLED.show();
      break;
    }
    case LED_CAN_OK: {
      // Brief green flash, then off
      if (now - ledLastUpdate < 80) {
        leds[0] = CRGB::Green;
      } else {
        leds[0] = CRGB::Black;
      }
      FastLED.show();
      ledLastUpdate = now;
      ledState = LED_NO_CAN;  // return to timeout watch after flash
      break;
    }
    case LED_NO_CAN: {
      // Red slow blink 1Hz
      leds[0] = ((now / 500) % 2 == 0) ? CRGB::Red : CRGB::Black;
      FastLED.show();
      break;
    }
    case LED_SAVED: {
      // White double flash then return to previous state
      static uint8_t flashCount = 0;
      if (now - ledLastUpdate > 150) {
        leds[0] = (flashCount % 2 == 0) ? CRGB::White : CRGB::Black;
        FastLED.show();
        flashCount++;
        ledLastUpdate = now;
        if (flashCount >= 4) {
          flashCount = 0;
          ledState = (lastCanRxTime > 0) ? LED_NO_CAN : LED_BOOT;
        }
      }
      break;
    }
  }
}
