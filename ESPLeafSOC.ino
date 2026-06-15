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
//   - uint16_t maxGids replaces byte - supports 30/40/62kWh packs
//   - Auto-learn maxGids retained and extended for pack ageing compensation
//   - Pack type preset seeds maxGids on first boot for immediate accuracy
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

// ============================================================
// TODO:
//   1. Verify GIDS decode bit-shift for 0x5BC matches your Leaf model year
//   2. Implement auto-learn MaxGids update logic in processCanMessage()
//   3. Research and verify exact MaxGids values for 30/40/62kWh packs
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
#define VERSION   "ESPLeafSOC v05"
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
#define WH_PER_GID      75.0F    // Wh per GID (from Leaf hacking community)

// Battery pack type presets - seeds maxGids on first boot
// Values are approximate full-charge GIDS for a new pack.
// Auto-learn will reduce these as the pack ages.
// Source: Leaf community (mnl.li/wiki, MyNissanLeaf forums) - verify/refine
#define PACK_24KWH_GIDS   281
#define PACK_30KWH_GIDS   340
#define PACK_40KWH_GIDS   395
#define PACK_62KWH_GIDS   536

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
#define NVS_PACK_TYPE   "packtype"
#define NVS_MAX_GIDS    "maxgids"
#define NVS_ROTATION    "rotation"

// ------------------------------------------------------------
// Display constructor
// ------------------------------------------------------------
// Hardware SPI - CLK=33, MOSI=32, CS=25, DC=18, RST=35
// U8G2_R0 = no rotation, U8G2_R2 = 180 degrees (match your physical mount)
//
// Uncomment the line matching your display's driver chip.
// Existing Keyestudio installs: SH1106 (default)
// Seeedstudio and some others:  SSD1306

// Display rotation is now set at runtime from NVS via u8g2.setDisplayRotation()
// Constructor always uses U8G2_R0 - rotation applied in setup() after loadSettings()
//
// Uncomment the line matching your display's driver chip.
// Existing Keyestudio installs: SH1106 (default)
// Seeedstudio and some others:  SSD1306

// SH1106 128x64 SPI - Keyestudio KS0056 and compatible (DEFAULT)
U8G2_SH1106_128X64_NONAME_F_4W_HW_SPI u8g2(U8G2_R0,
  /* cs=*/ OLED_CS_PIN, /* dc=*/ OLED_DC_PIN, /* reset=*/ OLED_RST_PIN);

// SSD1306 128x64 SPI - Seeedstudio and compatible (uncomment if needed)
// U8G2_SSD1306_128X64_NONAME_F_4W_HW_SPI u8g2(U8G2_R0,
//   /* cs=*/ OLED_CS_PIN, /* dc=*/ OLED_DC_PIN, /* reset=*/ OLED_RST_PIN);

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
float    GidsPct     = 0.0F; // SOC% derived from GIDS (primary display value)
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
int      packType       = 1;       // 1=24kWh, 2=30kWh, 3=40kWh, 4=62kWh
uint16_t maxGids        = PACK_24KWH_GIDS;  // overwritten from NVS on boot
int      displayRotation = 2;     // 0 = normal, 2 = 180 degrees (U8G2_R0 / U8G2_R2)

// Test mode - cycles display pages for layout checking (not saved to NVS)
bool     testMode       = false;
int      testPage       = 1;

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

  // OLED init - hardware SPI on custom pins
  SPI.begin(OLED_CLK_PIN, -1, OLED_MOSI_PIN, OLED_CS_PIN);
  u8g2.begin();
  // Apply rotation from NVS setting (0=normal, 2=180 degrees)
  u8g2.setDisplayRotation(displayRotation == 2 ? U8G2_R2 : U8G2_R0);

  // Splash screen - 12px line spacing fits 5 lines within 64px height
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x10_tr);
  u8g2.setCursor(0, 10);  u8g2.print(VERSION);
  u8g2.setCursor(0, 22);  u8g2.print("Original: Paul Kennett");
  u8g2.setCursor(0, 34);  u8g2.print(DATE);
  u8g2.setCursor(0, 46);  u8g2.print(AUTHOR);
  u8g2.setCursor(0, 58);  u8g2.print("github.com/Mozzie-AU");
  u8g2.sendBuffer();

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

  // Update display when GIDS changes, or in test mode always update
  if (rawGids != rawGids2 || testMode) {
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
  if (displayPage < 1 || displayPage > 4) displayPage = 1;

  kmPerKwh = prefs.getFloat(NVS_KM_PER_KWH, 6.4F);
  if (kmPerKwh < 1.0F || kmPerKwh > 20.0F) kmPerKwh = 6.4F;

  packType = prefs.getInt(NVS_PACK_TYPE, 1);
  if (packType < 1 || packType > 4) packType = 1;

  // Determine seed maxGids from pack type if not yet learned
  uint16_t seedGids;
  switch (packType) {
    case 2:  seedGids = PACK_30KWH_GIDS; break;
    case 3:  seedGids = PACK_40KWH_GIDS; break;
    case 4:  seedGids = PACK_62KWH_GIDS; break;
    default: seedGids = PACK_24KWH_GIDS; break;
  }

  // Load learned maxGids - if not set yet, use seed value
  maxGids = prefs.getUShort(NVS_MAX_GIDS, seedGids);

  // Safety check - if stored value is wildly out of range, reset to seed
  if (maxGids < 100 || maxGids > 600) maxGids = seedGids;

  // Load display rotation (0=normal, 2=180 degrees) - default 2 for existing installs
  displayRotation = prefs.getInt(NVS_ROTATION, 2);
  if (displayRotation != 0 && displayRotation != 2) displayRotation = 2;

  prefs.end();

  Serial.printf("Settings loaded: page=%d kmPerKwh=%.1f packType=%d maxGids=%d rotation=%d\n",
                displayPage, kmPerKwh, packType, maxGids, displayRotation);
}

void saveSettings() {
  prefs.begin(NVS_NAMESPACE, false);
  prefs.putInt(NVS_PAGE, displayPage);
  prefs.putFloat(NVS_KM_PER_KWH, kmPerKwh);
  prefs.putInt(NVS_PACK_TYPE, packType);
  prefs.putUShort(NVS_MAX_GIDS, maxGids);
  prefs.putInt(NVS_ROTATION, displayRotation);
  prefs.end();
  // Apply new rotation immediately without reboot
  u8g2.setDisplayRotation(displayRotation == 2 ? U8G2_R2 : U8G2_R0);
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

    // Auto-learn maxGids: only update downward from seed (tracks pack ageing)
    // Only update if rawGids is a plausible full-charge reading (above 95% of current max)
    if (rawGids > 0 && rawGids < maxGids && rawGids > (maxGids * 95 / 100)) {
      // TODO: add boot-startup logic here - only update maxGids from first
      // reading after power-on (like Paul's InitialGids approach) to avoid
      // mid-drive partial readings corrupting the learned value
    }

    // Recalculate derived values
    GidsPct = ((float)(rawGids - GIDS_TURTLE) / (float)(maxGids - GIDS_TURTLE)) * 100.0F;
    GidsPct = constrain(GidsPct, 0.0F, 100.0F);
    kWh     = ((float)rawGids * WH_PER_GID) / 1000.0F;
    range   = (int)(kmPerKwh * ((rawGids - GIDS_TURTLE) * WH_PER_GID) / 1000.0F);

    lastCanRxTime = millis();
    ledState = LED_CAN_OK;

  } else if (id == 0x55B) {
    // SOC% direct from BMS (100ms interval) - cross-check only
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
      "</select>"

      "<label>km per kWh</label>"
      "<input type='number' name='kmkwh' min='1' max='20' step='0.1' value='"
      + String(kmPerKwh, 1) + "'>"
      "<div class='info'>Your average efficiency. Used to calculate range estimate.</div>"

      "<label>Battery Pack Type</label>"
      "<select name='packtype'>"
      "<option value='1'" + String(packType==1?" selected":"") + ">24 kWh (Gen 1 - 2011-2012)</option>"
      "<option value='2'" + String(packType==2?" selected":"") + ">30 kWh (Gen 2 - 2016)</option>"
      "<option value='3'" + String(packType==3?" selected":"") + ">40 kWh (Gen 3 - 2018+)</option>"
      "<option value='4'" + String(packType==4?" selected":"") + ">62 kWh (Gen 3 e+)</option>"
      "</select>"
      "<div class='info'>Sets initial MaxGIDS. Auto-learn will refine this over time.</div>"

      "<label>Display Rotation</label>"
      "<select name='rotation'>"
      "<option value='2'" + String(displayRotation==2?" selected":"") + ">180° (default - existing installs)</option>"
      "<option value='0'" + String(displayRotation==0?" selected":"") + ">0° (normal)</option>"
      "</select>"
      "<div class='info'>Match your physical display mount orientation.</div>"

      "<div class='info' style='margin-top:16px'>Current learned MaxGIDS: <b>"
      + String(maxGids) + "</b></div>"

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
    if (request->hasParam("packtype", true)) {
      int newPackType = request->getParam("packtype", true)->value().toInt();
      if (newPackType != packType) {
        // Pack type changed - reseed maxGids from preset
        packType = newPackType;
        switch (packType) {
          case 2: maxGids = PACK_30KWH_GIDS; break;
          case 3: maxGids = PACK_40KWH_GIDS; break;
          case 4: maxGids = PACK_62KWH_GIDS; break;
          default: maxGids = PACK_24KWH_GIDS; break;
        }
      }
    }
    if (request->hasParam("rotation", true)) {
      displayRotation = request->getParam("rotation", true)->value().toInt();
      if (displayRotation != 0 && displayRotation != 2) displayRotation = 2;
    }
    saveSettings();
    request->send(200, "text/html",
      "<html><body style='font-family:sans-serif;max-width:400px;margin:20px auto'>"
      "<h2 style='color:#2a6'>Settings Saved</h2>"
      "<p>Device will use new settings immediately.</p>"
      "<a href='/'>Back</a></body></html>");
  });

  // Test mode - advance to next page
  server.on("/test", HTTP_POST, [](AsyncWebServerRequest* request) {
    testMode = true;
    testPage = (testPage % 4) + 1;  // cycle 1->2->3->4->1
    updateDisplay();
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
    updateDisplay();
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
    default: drawPage1(); break;
  }
}

void drawPage1() {
  // Range + SOC% + kWh
  // Based on Paul Kennett's original layout with these changes:
  //   - battery_small (outline) instead of battery_solid
  //   - "km" drawn vertically beside range number to save horizontal space
  //   - LINE3 raised to avoid defective bottom pixel rows
  char buf[8];
  u8g2.clearBuffer();

  // Battery outline graphic - bottom left
  u8g2.drawXBM(0, 40, 56, 24, battery_small_bits);

  // Large range number - shifted right slightly to make room for vertical km
  u8g2.setFont(u8g2_font_logisoso32_tn);
  u8g2.setCursor(45, 38);
  if (rawGids != 0) {
    dtostrf(range, 3, 0, buf);
    u8g2.print(buf);
  } else {
    u8g2.print(" --");
  }

  // "Range" label - medium font top left
  u8g2.setFont(u8g2_font_logisoso16_tr);
  u8g2.setCursor(0, LINE1);
  u8g2.print("Range");

  // "km" stacked vertically - medium font, right of range number
  u8g2.setFont(u8g2_font_6x10_tr);
  u8g2.setCursor(118, 20);
  u8g2.print("k");
  u8g2.setCursor(118, 32);
  u8g2.print("m");

  // SOC% - medium font, bottom centre
  u8g2.setFont(u8g2_font_logisoso16_tr);
  u8g2.setCursor(58, LINE3);
  if (rawGids != 0) {
    dtostrf(GidsPct, 3, 0, buf);
    u8g2.print(buf);
  } else {
    u8g2.print(" --");
  }
  u8g2.print("%");

  // kWh - medium font, bottom right
  u8g2.setCursor(kWh >= 10 ? 58 : 68, LINE4);
  dtostrf(kWh, 3, 1, buf);
  u8g2.print(buf);
  u8g2.print("kWh");

  u8g2.sendBuffer();
}

void drawPage2() {
  // Large SOC% inside battery graphic + kWh
  char buf[8];
  u8g2.clearBuffer();
  u8g2.drawXBMP(0, 0, bitmap_width, bitmap_height, battery_large_bits);
  u8g2.setFont(u8g2_font_logisoso26_tn);
  u8g2.setCursor(41, 33);
  if (rawGids != 0) {
    dtostrf(GidsPct, 3, 0, buf); u8g2.print(buf);
  } else {
    u8g2.print(" --");
  }
  u8g2.setFont(u8g2_font_logisoso16_tr);
  u8g2.setCursor(30, LINE3);  // raised from 64 to LINE3=54
  if (rawGids != 0) {
    dtostrf(kWh, 3, 1, buf); u8g2.print(buf); u8g2.print(" kWh");
  } else {
    u8g2.print("-- kWh");
  }
  u8g2.sendBuffer();
}

void drawPage3() {
  // Range only - large centred display
  char buf[8];
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_logisoso32_tn);
  u8g2.setCursor(39, 44);
  if (rawGids != 0) {
    dtostrf(range, 3, 0, buf); u8g2.print(buf);
  } else {
    u8g2.print(" --");
  }
  u8g2.setFont(u8g2_font_logisoso16_tr);
  u8g2.setCursor(0, LINE2);    u8g2.print("Range");
  u8g2.setCursor(103, LINE2);  u8g2.print("km");
  u8g2.sendBuffer();
}

void drawPage4() {
  // Version info - small font, 12px spacing fits 5 lines in 64px
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x10_tr);
  u8g2.setCursor(0, 10);  u8g2.print(VERSION);
  u8g2.setCursor(0, 22);  u8g2.print("Paul Kennett (original)");
  u8g2.setCursor(0, 34);  u8g2.print(DATE);
  u8g2.setCursor(0, 46);  u8g2.print(AUTHOR);
  u8g2.setCursor(0, 58);  u8g2.print("github.com/Mozzie-AU");
  u8g2.sendBuffer();
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
