/*
  TiTo Combined Health & Environment Monitor
  ============================================
  Sensors:
    - MAX30102 / MAX30105  → Heart Rate & SpO2 (I2C)
    - DS18B20              → Temperature (1-Wire)
    - YF-S401               → Air Flow (pulse counter)

  Wiring - MAX3010x (I2C):
    VIN  → 3.3V    |  GND → GND
    SDA  → GPIO 21 |  SCL → GPIO 22

  Wiring - DS18B20 (1-Wire):
    VDD  → 3.3V  |  GND → GND
    DATA → GPIO 4  (+ 4.7 kΩ pull-up to 3.3V)

  Wiring - YF-S401:
    Red    → 5V
    Black  → GND
    Yellow → GPIO 27  (INPUT_PULLUP)

  Wiring - Relay Module (Active LOW):
    VCC → 5V   |  GND → GND
    IN  → GPIO 19
    (LOW = relay ON, HIGH = relay OFF)

    Relay behaviour is now driven by the active operating mode:
      Mode 1 (Monitoring)       → relay ON  (continuous)
      Mode 2 (Nebulizer)        → relay OFF (continuous)
      Mode 3 (Ventilation)      → relay OFF (continuous)
      Mode 4 (Controlled Vent)  → relay auto-toggles ON 3s / OFF 3s
      Emergency STOP            → relay ON  (same as Monitoring)
    Manual override is still available from the web dashboard (/relay
    endpoint) or Serial ('1'=ON, '0'=OFF, 'a'=AUTO), but note: in Modes
    1-3 the relay state is re-asserted every loop() pass, so a manual
    override there is overwritten almost immediately. In Mode 4, a
    manual override pauses the 3s auto-toggle until 'a'/AUTO is sent
    again or the mode is re-entered.

  Libraries (Arduino IDE → Manage Libraries):
    • SparkFun MAX3010x Pulse and Proximity Sensor Library
    • OneWire  (Paul Stoffregen)
    • DallasTemperature  (Miles Burton)

  WiFi Access Point
    SSID:     TiTo
    Password: 12345678
    URL:      http://192.168.4.1

  Value adjustments applied:
    • Displayed Heart Rate  = (measured HR − 35), capped at 105 BPM
    • Displayed Air Flow    = min(measured_value / 5, 15)  [L/min]
    • Temperature shown in both °C and °F
    • Web page refresh: 120 ms  (was 200 ms)
    • Patient condition panel derived from HR + SpO2 + Temp combined

  ── Mode buttons (4-button metal switch panel) ──────────────────────
    Four 16mm metal push buttons with independent 12V built-in LEDs.
    The 12V LEDs are wired to an EXTERNAL 12V supply — the ESP32 only
    ever reads the dry switch contacts (NO ↔ C), never any 12V line.
      Button 1 (Mode 1 / Monitoring)      → GPIO 26  (NO), C → GND
      Button 2 (Mode 2 / Nebulizer)       → GPIO 25  (NO), C → GND
      Button 3 (Mode 3 / Ventilation)     → GPIO 33  (NO), C → GND
      Button 4 (Mode 4 / Controlled Vent) → GPIO 32  (NO), C → GND
    All four use INPUT_PULLUP, active-LOW (pressed = LOW). NC left open.
    Debounce/edge-detection (buttonPressed()) is UNCHANGED for these 4.

  ── Emergency STOP input — UPDATED to a maintained ON/OFF toggle switch ──
    The STOP input is NO LONGER a momentary push button. It is now a
    2-pin ON/OFF toggle switch wired between GPIO14 and GND.
      Toggle Switch (Emergency)  → GPIO 14, other leg → GND
    GPIO14 is still configured INPUT_PULLUP (unchanged), so:
      Switch OFF → GPIO14 reads HIGH → emergencyStopActive = false
      Switch ON  → GPIO14 reads LOW  → emergencyStopActive = true
    Because this is a maintained switch (not momentary), it is READ
    CONTINUOUSLY every loop() pass with a plain digitalRead() — there is
    NO debounce and NO edge-detection (buttonPressed()) applied to it.
    See handleEmergencySwitch() below for the full logic.
*/

#include <Wire.h>
#include "MAX30105.h"
#include "spo2_algorithm.h"
#include <WiFi.h>
#include <WebServer.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <LiquidCrystal_I2C.h>   // I2C backpack (PCF8574) — Sketch > Include Library > Manage Libraries, search "LiquidCrystal I2C" by Frank de Brabander

// ── Pin definitions ──────────────────────────────────────────────
#define I2C_SDA        21
#define I2C_SCL        22
#define ONE_WIRE_BUS    4
#define FLOW_SENSOR_PIN 27
#define RELAY_PIN      19   // Relay module — active LOW (LOW = ON, HIGH = OFF)

// ── Mode buttons (INPUT_PULLUP, active LOW) ─────────────────────────
// 16mm metal push buttons, dry-contact NO/C wired to GND. 12V LEDs run
// on a separate external supply and are NOT connected to the ESP32.
#define BTN_MODE1  26   // Mode 1 -> Monitoring
#define BTN_MODE2  25   // Mode 2 -> Nebulizer
#define BTN_MODE3  33   // Mode 3 -> Ventilation
#define BTN_MODE4  32   // Mode 4 -> Controlled Ventilation
#define BTN_STOP   14   // Emergency input — now a maintained ON/OFF toggle switch
                         // (was a momentary push button). Wiring/pin/pull-up
                         // unchanged: switch leg -> GPIO14, other leg -> GND.

// ── Buzzer (bare piezo, no driver module — driven with tone()/noTone()) ──
#define BUZZER_PIN 13

// ── LCD I2C addresses ────────────────────────────────────────────
// Default PCF8574 backpacks are usually 0x27 (some are 0x3F). Since you
// have TWO LCDs on the SAME I2C bus, they MUST have different addresses —
// change one module's address via its A0/A1/A2 solder jumpers, then
// confirm both addresses with an I2C-scanner sketch before relying on this.
#define LCD1_ADDR 0x27
#define LCD2_ADDR 0x26

// ── MAX3010x ─────────────────────────────────────────────────────
MAX30105 particleSensor;
bool sensorFound = false;

#define BUFFER_LENGTH      100
#define SAMPLES_PER_UPDATE   4   // refill 4 samples → recalculate ~every 40 ms

uint32_t irBuffer[BUFFER_LENGTH];
uint32_t redBuffer[BUFFER_LENGTH];

int32_t  spo2           = 0;
int8_t   validSPO2      = 0;
int32_t  heartRate      = 0;
int8_t   validHeartRate = 0;

// ── DS18B20 ──────────────────────────────────────────────────────
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature tempSensor(&oneWire);

float         temperatureC           = 0.0;
bool          tempConversionRequested = false;
unsigned long tempRequestTime        = 0;

// ── YF-S401 flow sensor ──────────────────────────────────────────
volatile unsigned long pulseCount = 0;
unsigned long lastCalcTime        = 0;
float rawFlowRate                 = 0.0;
float totalLiters                 = 0.0;

const float calibrationFactor = 98.0;   // ~98 pulses / litre for YF-S401

void IRAM_ATTR pulseCounter() { pulseCount++; }

// ── Relay (active-LOW module) ─────────────────────────────────────
//   - LOW = relay ON, HIGH = relay OFF.
//   - Relay state is now driven by the active operating mode
//     (see runModeTasks() and relayUpdate() below):
//       Mode 1 (Monitoring)      → ON continuously
//       Mode 2 (Nebulizer)       → OFF continuously
//       Mode 3 (Ventilation)     → OFF continuously
//       Mode 4 (Controlled Vent) → auto-toggle ON 3s / OFF 3s
//   - relayAutoMode still exists so Mode 4's toggle can be paused via
//     manual web/Serial override ('a'/AUTO resumes it).
//   - Emergency STOP forces the relay ON (same as Monitoring) and drops
//     out of auto mode, so it does not silently start toggling again
//     while stopped.
bool          relayState      = false;    // true = ON
bool          relayAutoMode   = true;     // true = Mode 4 auto-toggles every 3s
unsigned long relayLastToggle = 0;
const unsigned long RELAY_TOGGLE_INTERVAL = 3000;   // 3 seconds (Mode 4 only)

// ملاحظة: الريلاي هنا من نوع Active LOW
// يعني LOW = تشغيل (ON)  و  HIGH = إيقاف (OFF)
void applyRelay(bool state) {
  relayState = state;
  digitalWrite(RELAY_PIN, relayState ? LOW : HIGH);
}

// ── WiFi / Web ───────────────────────────────────────────────────
const char* ssid     = "TiTo";
const char* password = "12345678";
WebServer server(80);

// ── LCD objects (share RS + D4-D7, separate Enable pins) ────────────
// LCD1 (Mode):     line0 = active mode name | line1 = status (STOPPED / running)
// LCD2 (Vitals):   line0 = SpO2 + Heart Rate | line1 = Flow + Temperature
LiquidCrystal_I2C lcd1(LCD1_ADDR, 16, 2);
LiquidCrystal_I2C lcd2(LCD2_ADDR, 16, 2);

// ── Operating modes (plain constants — avoids Arduino's auto-prototype
//    ordering bug that happens with custom enum/struct types) ────────
// Renumbered 1-4 to match the physical Mode 1/2/3/4 button panel.
// STOPPED is 0 so an un-set/cleared state never collides with a real mode.
#define MODE_STOPPED         0
#define MODE_MONITORING      1   // Mode 1
#define MODE_NEBULIZER       2   // Mode 2
#define MODE_VENTILATION     3   // Mode 3
#define MODE_CONTROLLED_VENT 4   // Mode 4

uint8_t currentMode = MODE_MONITORING;   // defaults to Mode 1 (Monitoring) at boot
bool emergencyStopActive  = false;

const char* modeName(uint8_t m) {
  switch (m) {
    case MODE_MONITORING:      return "Monitoring";
    case MODE_NEBULIZER:       return "Nebulizer";
    case MODE_VENTILATION:     return "Ventilation";
    case MODE_CONTROLLED_VENT: return "Ctrl.Vent";
    case MODE_STOPPED:         return "STOPPED";
    default:                   return "?";
  }
}

// ── Button debounce state — parallel arrays (built-in types only) ───
// NOTE: BTN_STOP/IDX_STOP are kept in these arrays only so the array
// sizes/indices don't shift. The emergency switch itself is NO LONGER
// read through buttonPressed()/this debounce logic — see
// handleEmergencySwitch() further down, which reads BTN_STOP directly
// with digitalRead() every loop pass instead.
#define IDX_MODE1  0
#define IDX_MODE2  1
#define IDX_MODE3  2
#define IDX_MODE4  3
#define IDX_STOP   4
#define NUM_BUTTONS 5

const uint8_t buttonPins[NUM_BUTTONS] = {
  BTN_MODE1, BTN_MODE2, BTN_MODE3, BTN_MODE4, BTN_STOP
};
bool          btnLastReading[NUM_BUTTONS]    = { HIGH, HIGH, HIGH, HIGH, HIGH };
bool          btnStableState[NUM_BUTTONS]    = { HIGH, HIGH, HIGH, HIGH, HIGH };
unsigned long btnLastChangeTime[NUM_BUTTONS] = { 0, 0, 0, 0, 0 };

const unsigned long DEBOUNCE_MS = 150;   // 150ms software debounce/state-lock

// returns true exactly once, on the frame the button transitions HIGH → LOW (press)
// UNCHANGED — still used for Mode 1-4 momentary buttons only.
bool buttonPressed(uint8_t idx) {
  bool reading = digitalRead(buttonPins[idx]);
  if (reading != btnLastReading[idx]) {
    btnLastChangeTime[idx] = millis();
    btnLastReading[idx]    = reading;
  }
  bool justPressed = false;
  if ((millis() - btnLastChangeTime[idx]) > DEBOUNCE_MS && reading != btnStableState[idx]) {
    btnStableState[idx] = reading;
    if (btnStableState[idx] == LOW) justPressed = true;   // active-LOW press edge
  }
  return justPressed;
}

// ════════════════════════════════════════════════════════════════
//  HTML Dashboard
// ════════════════════════════════════════════════════════════════
const char htmlPage[] PROGMEM = R"SRSSHTML(
<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1" />
  <title>SRSS Smart Respiratory Support System</title>
  <style>
    :root {
      color-scheme: dark;
      --bg: #070a10;
      --body-glow-a: rgba(36, 215, 223, .16);
      --body-glow-b: rgba(55, 213, 106, .11);
      --phone-bg: #080c13;
      --panel: #111722;
      --panel-2: #151b27;
      --line: #263041;
      --muted: #9aa4b5;
      --text: #f4f7fb;
      --green: #37d56a;
      --cyan: #24d7df;
      --blue: #4da1ff;
      --red: #ff4f64;
      --amber: #f3a62d;
      --purple: #8c5cff;
      --shadow: 0 20px 60px rgba(0, 0, 0, .42);
      font-family: Inter, ui-sans-serif, system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
    }

    body.light-theme {
      color-scheme: light;
      --bg: #eef3f7;
      --body-glow-a: rgba(36, 215, 223, .18);
      --body-glow-b: rgba(55, 213, 106, .14);
      --phone-bg: #f8fbff;
      --panel: #ffffff;
      --panel-2: #eef3f9;
      --line: #ccd7e5;
      --muted: #66758a;
      --text: #101824;
      --shadow: 0 20px 54px rgba(30, 48, 70, .18);
    }

    * { box-sizing: border-box; }

    body {
      margin: 0;
      font-size: 14px;
      min-height: 100vh;
      background:
        radial-gradient(circle at 18% 0%, var(--body-glow-a), transparent 28rem),
        radial-gradient(circle at 86% 12%, var(--body-glow-b), transparent 24rem),
        var(--bg);
      color: var(--text);
      display: grid;
      place-items: center;
      padding: 12px;
    }

    .phone {
      width: min(100%, 360px);
      height: min(800px, calc(100vh - 24px));
      border: 1px solid color-mix(in srgb, var(--line) 70%, transparent);
      border-radius: 24px;
      overflow: hidden;
      background: var(--phone-bg);
      box-shadow: var(--shadow);
      position: relative;
      isolation: isolate;
    }

    .screen {
      height: 100%;
      display: none;
      flex-direction: column;
    }

    .screen.active { display: flex; }

    .topbar {
      height: 52px;
      display: grid;
      grid-template-columns: 42px 1fr 42px;
      align-items: center;
      padding: 0 12px;
    }

    .title {
      text-align: center;
      font-weight: 800;
      font-size: 15px;
    }

    .iconbtn {
      width: 34px;
      height: 34px;
      border: 0;
      background: transparent;
      color: #dfe7f3;
      border-radius: 12px;
      display: inline-grid;
      place-items: center;
      cursor: pointer;
      transition: background .18s ease, color .18s ease;
    }

    .iconbtn:hover { background: rgba(255, 255, 255, .07); color: white; }

    .content {
      flex: 1;
      overflow: auto;
      padding: 0 16px 82px;
      scrollbar-width: none;
    }

    .content::-webkit-scrollbar { display: none; }

    .connect {
      justify-content: flex-start;
      overflow: auto;
      padding: 18px 18px 28px;
      gap: 16px;
      scrollbar-width: none;
    }

    .connect::-webkit-scrollbar { display: none; }

    .brand {
      text-align: center;
      display: grid;
      justify-items: center;
      gap: 7px;
      flex: 0 0 auto;
    }

    .logo {
      width: 96px;
      height: 96px;
      filter: drop-shadow(0 16px 30px rgba(36, 215, 223, .24));
    }

    .brand h1 {
      margin: 0;
      font-size: 36px;
      line-height: 1;
      letter-spacing: 4px;
    }

    .brand p {
      margin: 0;
      color: #c1c9d6;
      line-height: 1.35;
      font-size: 15px;
    }

    .panel {
      background: linear-gradient(180deg, rgba(255, 255, 255, .055), rgba(255, 255, 255, .025));
      border: 1px solid rgba(255, 255, 255, .08);
      border-radius: 16px;
      padding: 18px;
    }

    .form-card {
      width: 100%;
      display: grid;
      gap: 12px;
      flex: 0 0 auto;
    }

    .field {
      display: grid;
      gap: 8px;
    }

    label {
      color: #cbd3df;
      font-size: 13px;
      font-weight: 700;
    }

    select,
    input,
    .readonly {
      width: 100%;
      height: 44px;
      border: 0;
      border-radius: 10px;
      background: var(--panel-2);
      color: var(--text);
      padding: 0 14px;
      font-weight: 750;
      outline: 1px solid transparent;
    }

    select:focus,
    input:focus { outline-color: rgba(36, 215, 223, .55); }

    .readonly {
      display: flex;
      align-items: center;
      justify-content: space-between;
    }

    .connected-pill,
    .primary {
      height: 46px;
      border: 0;
      border-radius: 10px;
      background: linear-gradient(135deg, #2ac86a, #42e474);
      color: #082111;
      font-weight: 850;
      font-size: 15px;
      cursor: pointer;
      display: inline-flex;
      align-items: center;
      justify-content: center;
      gap: 10px;
    }

    .connected-pill {
      color: white;
      background: linear-gradient(135deg, #1f8d4a, #257743);
      justify-content: flex-start;
      padding: 0 14px;
    }

    .connected-pill.waiting {
      background: linear-gradient(135deg, #66501d, #8d671c);
    }

    .connected-pill.offline {
      background: linear-gradient(135deg, #7c2633, #9b2e40);
    }

    .hint {
      color: var(--muted);
      font-size: 12px;
      line-height: 1.45;
      margin: -4px 0 0;
    }

    .message {
      min-height: 20px;
      color: var(--muted);
      font-size: 12px;
      line-height: 1.45;
    }

    .device-banner {
      border: 1px solid rgba(55, 213, 106, .36);
      background: linear-gradient(135deg, rgba(55, 213, 106, .18), rgba(36, 215, 223, .06));
      border-radius: 14px;
      padding: 16px;
      display: flex;
      align-items: center;
      justify-content: space-between;
      margin-bottom: 18px;
    }

    .timer-card {
      display: grid;
      grid-template-columns: 1fr auto;
      align-items: center;
      gap: 12px;
      min-height: 68px;
      margin: 16px 0;
      border: 1px solid color-mix(in srgb, var(--line) 60%, transparent);
      border-radius: 14px;
      padding: 14px 16px;
      background: linear-gradient(135deg, color-mix(in srgb, var(--panel) 92%, transparent), color-mix(in srgb, var(--panel-2) 70%, transparent));
    }

    .timer-value {
      font-size: 30px;
      line-height: 1;
      font-weight: 900;
      letter-spacing: 0;
      color: var(--cyan);
      font-variant-numeric: tabular-nums;
    }

    .session-controls {
      grid-column: 1 / -1;
      display: grid;
      grid-template-columns: repeat(3, 1fr);
      gap: 8px;
    }

    .session-btn {
      height: 38px;
      border: 0;
      border-radius: 999px;
      color: #fff;
      font-size: 12px;
      font-weight: 900;
      cursor: pointer;
      display: inline-flex;
      align-items: center;
      justify-content: center;
      gap: 6px;
      transition: transform .15s ease, opacity .15s ease;
    }

    .session-btn:active { transform: scale(.98); }
    .session-btn.start { background: linear-gradient(135deg, #20b95f, #39d978); }
    .session-btn.pause { background: linear-gradient(135deg, #596273, #7c8798); }
    .session-btn.stop { background: linear-gradient(135deg, #f05a49, #ff745f); }
    .session-btn[disabled] {
      opacity: .45;
      cursor: not-allowed;
    }

    .green { color: var(--green); }
    .cyan { color: var(--cyan); }
    .blue { color: var(--blue); }
    .red { color: var(--red); }
    .amber { color: var(--amber); }

    .small {
      color: var(--muted);
      font-size: 12px;
      font-weight: 650;
    }

    .mode-chip {
      width: fit-content;
      margin: 8px auto 18px;
      height: 46px;
      padding: 0 18px;
      border-radius: 999px;
      background: var(--panel-2);
      display: inline-flex;
      align-items: center;
      gap: 9px;
      font-weight: 850;
      box-shadow: inset 0 0 0 1px rgba(255, 255, 255, .06);
    }

    .metrics {
      display: grid;
      grid-template-columns: repeat(2, minmax(0, 1fr));
      gap: 12px;
    }

    .metric {
      min-height: 132px;
      border-radius: 12px;
      padding: 14px;
      background: linear-gradient(180deg, color-mix(in srgb, var(--panel) 92%, transparent), color-mix(in srgb, var(--phone-bg) 96%, transparent));
      border: 1px solid var(--line);
      display: flex;
      flex-direction: column;
      justify-content: space-between;
    }

    .metric.spo2 { border-color: rgba(36, 215, 223, .46); }
    .metric.hr { border-color: rgba(55, 213, 106, .42); }
    .metric.air { border-color: rgba(77, 161, 255, .38); }
    .metric.temp { border-color: rgba(243, 166, 45, .42); }
    .metric.abnormal { border-color: rgba(255, 79, 100, .72); }

    .metric .value {
      font-size: 36px;
      line-height: .9;
      font-weight: 900;
      letter-spacing: 0;
    }

    .metric .unit {
      font-size: 13px;
      font-weight: 850;
      margin-left: 2px;
    }

    .status-card,
    .list-card,
    .chart-card,
    .alert-card {
      background: linear-gradient(180deg, color-mix(in srgb, var(--panel) 94%, transparent), color-mix(in srgb, var(--phone-bg) 94%, transparent));
      border: 1px solid color-mix(in srgb, var(--line) 60%, transparent);
      border-radius: 14px;
      padding: 16px;
    }

    .status-card {
      margin-top: 16px;
      display: flex;
      gap: 13px;
      align-items: center;
    }

    .shield {
      width: 42px;
      height: 42px;
      border-radius: 12px;
      display: grid;
      place-items: center;
      background: rgba(55, 213, 106, .16);
      color: var(--green);
    }

    .tabs {
      height: 46px;
      margin-bottom: 14px;
      border-radius: 999px;
      background: var(--panel-2);
      display: grid;
      grid-template-columns: repeat(4, 1fr);
      padding: 3px;
      gap: 3px;
    }

    .tab {
      border: 0;
      border-radius: 999px;
      background: transparent;
      color: #a8b1c0;
      font-weight: 800;
      cursor: pointer;
    }

    .tab.active {
      color: white;
      background: linear-gradient(135deg, #1fa7ad, #27c5cc);
    }

    .chart-card {
      margin-bottom: 14px;
      overflow: hidden;
    }

    .chart-head {
      display: flex;
      justify-content: space-between;
      align-items: baseline;
      gap: 12px;
      margin-bottom: 8px;
      font-weight: 850;
    }

    canvas {
      width: 100%;
      height: 124px;
      display: block;
    }

    .mode-list,
    .history-list,
    .settings-list {
      display: grid;
      gap: 12px;
    }

    .theme-toggle {
      display: grid;
      grid-template-columns: 1fr 1fr;
      width: 136px;
      height: 34px;
      padding: 3px;
      gap: 3px;
      border-radius: 999px;
      background: var(--panel-2);
      border: 1px solid color-mix(in srgb, var(--line) 70%, transparent);
    }

    .theme-option {
      border: 0;
      border-radius: 999px;
      background: transparent;
      color: var(--muted);
      font-size: 12px;
      font-weight: 850;
      cursor: pointer;
    }

    .theme-option.active {
      color: #07120d;
      background: var(--green);
    }

    .mode-option {
      min-height: 92px;
      display: grid;
      grid-template-columns: 58px 1fr 30px;
      align-items: center;
      gap: 12px;
      border-radius: 14px;
      padding: 14px;
      background: color-mix(in srgb, var(--panel) 72%, transparent);
      border: 1px solid color-mix(in srgb, var(--line) 56%, transparent);
      cursor: pointer;
    }

    .mode-option.active { border-color: rgba(55, 213, 106, .58); background: rgba(55, 213, 106, .08); }
    .mode-option.emergency { border-color: rgba(255, 79, 100, .25); background: rgba(255, 79, 100, .08); }

    .round-icon {
      width: 52px;
      height: 52px;
      border-radius: 50%;
      display: grid;
      place-items: center;
      color: white;
      font-weight: 900;
      font-size: 23px;
    }

    .check {
      width: 22px;
      height: 22px;
      border-radius: 50%;
      border: 2px solid #5d6878;
      display: grid;
      place-items: center;
      color: #07100a;
      font-size: 13px;
      font-weight: 900;
    }

    .mode-option.active .check {
      border: 0;
      background: var(--green);
    }

    .history-main {
      display: grid;
      grid-template-columns: 1fr auto;
      gap: 12px;
      align-items: start;
      margin-bottom: 12px;
    }

    .averages {
      display: grid;
      gap: 18px;
      margin: 18px 0 20px;
    }

    .row {
      display: flex;
      align-items: center;
      justify-content: space-between;
      gap: 12px;
      min-height: 36px;
    }

    .row strong { font-size: 14px; }

    .sensor-list {
      margin-top: 18px;
      display: grid;
      gap: 10px;
    }

    .bottom-nav {
      position: absolute;
      left: 0;
      right: 0;
      bottom: 0;
      height: 72px;
      border-top: 1px solid rgba(255, 255, 255, .1);
      background: color-mix(in srgb, var(--phone-bg) 92%, transparent);
      backdrop-filter: blur(18px);
      display: grid;
      grid-template-columns: repeat(5, 1fr);
      padding-bottom: 8px;
      z-index: 4;
    }

    .navitem {
      border: 0;
      background: transparent;
      color: #8f98a8;
      display: grid;
      justify-items: center;
      align-content: center;
      gap: 4px;
      font-size: 11px;
      font-weight: 800;
      cursor: pointer;
    }

    .navitem.active { color: var(--green); }

    .drawer {
      position: absolute;
      inset: 0;
      z-index: 10;
      display: none;
    }

    .drawer.open { display: block; }

    .scrim {
      position: absolute;
      inset: 0;
      background: rgba(0, 0, 0, .48);
    }

    .drawer-panel {
      position: absolute;
      top: 0;
      bottom: 0;
      left: 0;
      width: 82%;
      background: var(--phone-bg);
      border-right: 1px solid rgba(255, 255, 255, .08);
      padding: 58px 22px 24px;
      box-shadow: 20px 0 60px rgba(0, 0, 0, .46);
    }

    .drawer-brand {
      display: flex;
      gap: 13px;
      align-items: center;
      margin-bottom: 34px;
    }

    .drawer-brand .logo { width: 70px; height: 70px; }

    .drawer-link {
      width: 100%;
      min-height: 50px;
      border: 0;
      border-radius: 10px;
      background: transparent;
      color: var(--text);
      display: flex;
      align-items: center;
      gap: 14px;
      padding: 0 16px;
      font-weight: 850;
      cursor: pointer;
    }

    .drawer-link.active { background: rgba(55, 213, 106, .15); color: var(--green); }
    .drawer-link.danger { color: var(--red); margin-top: 24px; }

    .drawer-sep {
      height: 1px;
      background: rgba(255, 255, 255, .08);
      margin: 24px 12px;
    }

    svg { display: block; }

    @media (max-width: 440px) {
      body { padding: 0; }
      .phone {
        width: 100vw;
        height: 100vh;
        border-radius: 0;
        border: 0;
      }
      .connect {
        padding: 12px 16px 24px;
        gap: 12px;
      }
      .logo {
        width: 82px;
        height: 82px;
      }
      .brand h1 { font-size: 32px; }
      .brand p { font-size: 14px; }
      .panel { padding: 16px; }
      .form-card { gap: 10px; }
      select,
      input,
      .readonly { height: 42px; }
      .connected-pill,
      .primary { height: 44px; }
      .content { padding-inline: 16px; }
      .metric { min-height: 136px; padding: 14px; }
      .metric .value { font-size: 34px; }
    }
  </style>
</head>
<body>
  <main class="phone" aria-label="SRSS mobile application">
    <section class="screen connect" id="connect-screen">
      <div class="brand">
        <div class="logo" aria-hidden="true"></div>
        <h1>SRSS</h1>
        <p>Smart Respiratory<br>Support System</p>
      </div>
      <div class="panel form-card">
        <h2 style="margin:0;font-size:18px;">Connect to Device</h2>
        <div class="field">
          <label for="device-name">Device Name</label>
          <select id="device-name">
            <option>ESP32-SRSS</option>
            <option>ESP32-SRSS-LAB</option>
          </select>
        </div>
        <div class="field">
          <label for="wifi-ssid">Network Name</label>
          <input id="wifi-ssid" autocomplete="off" placeholder="Enter Wi-Fi / ESP32 AP name" value="TiTo" />
          <p class="hint">Connect your phone or laptop to this network first if the ESP32 is working as an access point.</p>
        </div>
        <div class="field">
          <label for="wifi-password">Network Password</label>
          <input id="wifi-password" type="password" autocomplete="current-password" placeholder="Enter password, e.g. 12345678" />
        </div>
        <div class="field">
          <label>Connection Status</label>
          <div class="connected-pill offline" id="connection-status"><span data-icon="wifi"></span><span id="connection-status-text">Not connected</span></div>
        </div>
        <div class="field">
          <label for="device-ip">Device IP Address</label>
          <input id="device-ip" inputmode="decimal" value="192.168.4.1" />
        </div>
        <button class="primary" id="connect-btn"><span data-icon="link"></span>Connect Device</button>
        <div class="message" id="connect-message">Readings endpoint: <strong>/data</strong></div>
      </div>
    </section>

    <section class="screen active" id="app-screen">
      <header class="topbar">
        <button class="iconbtn" id="menu-btn" aria-label="Open menu"><span data-icon="menu"></span></button>
        <div class="title" id="page-title">Dashboard</div>
        <button class="iconbtn" aria-label="More options"><span data-icon="more"></span></button>
      </header>

      <div class="content" id="page-dashboard">
        <div class="device-banner">
          <div>
            <strong class="green" id="banner-status">Device Connected</strong>
            <div class="small green">● <span id="banner-device">ESP32-SRSS</span> · <span id="banner-ip">192.168.4.1</span></div>
          </div>
          <span class="green" data-icon="wifi"></span>
        </div>
        <div class="small">Current Mode</div>
        <div style="text-align:center;"><div class="mode-chip"><span class="green" data-icon="wifi"></span><span id="current-mode-label">Monitoring</span><span class="cyan" data-icon="pulse"></span></div></div>
        <div class="metrics">
          <article class="metric spo2">
            <div class="small"><span class="cyan" data-icon="droplet"></span><br>SpO₂</div>
            <div><span class="value cyan" id="spo2-value">98</span><span class="unit cyan" id="spo2-unit">%</span><div class="small" id="spo2-status">Normal</div></div>
          </article>
          <article class="metric hr">
            <div class="small"><span class="green" id="heart-icon" data-icon="heart"></span><br>Heart Rate</div>
            <div><span class="value green" id="heart-value">75</span><span class="unit green" id="heart-unit">BPM</span><div class="small" id="heart-status">Normal</div></div>
          </article>
          <article class="metric air">
            <div class="small"><span class="blue" data-icon="air"></span><br>Air Flow</div>
            <div><span class="value blue" id="air-value">10</span><span class="unit blue" id="air-unit">L/min</span><div class="small" id="air-status">Normal</div></div>
          </article>
          <article class="metric temp">
            <div class="small"><span class="amber" data-icon="thermo"></span><br>Temperature</div>
            <div><span class="value amber" id="temp-value">36.5</span><span class="unit amber" id="temp-unit">°C</span><div class="small" id="temp-status">Normal</div></div>
          </article>
        </div>
        <div class="timer-card">
          <div>
            <strong>Session Timer</strong>
            <div class="small" id="session-state-label">Session stopped</div>
          </div>
          <div class="timer-value" id="session-timer">00:00</div>
          <div class="session-controls">
            <button class="session-btn start" id="session-start-btn" type="button"><span data-icon="play"></span>Start</button>
            <button class="session-btn pause" id="session-pause-btn" type="button" disabled><span data-icon="pause"></span>Pause</button>
            <button class="session-btn stop" id="session-stop-btn" type="button" disabled><span data-icon="stop"></span>Stop</button>
          </div>
        </div>
        <div class="status-card"><div class="shield"><span data-icon="shield"></span></div><div><strong>System Status</strong><div class="green"><strong>Normal</strong></div></div></div>
      </div>

      <div class="content" id="page-live" hidden>
        <div class="tabs" role="tablist">
          <button class="tab active" data-chart-tab="spo2">SpO₂</button>
          <button class="tab" data-chart-tab="heart">Heart Rate</button>
          <button class="tab" data-chart-tab="air">Air Flow</button>
          <button class="tab" data-chart-tab="temp">Temp</button>
        </div>
        <article class="chart-card" data-chart-card="spo2">
          <div class="chart-head"><span>SpO₂ (%)</span><span class="cyan"><span id="live-spo2">98</span>%</span></div>
          <canvas id="chart-spo2" width="640" height="248"></canvas>
        </article>
        <article class="chart-card" data-chart-card="heart">
          <div class="chart-head"><span>Heart Rate (BPM)</span><span class="green" id="live-heart-wrap"><span id="live-heart">75</span> <small>BPM</small></span></div>
          <canvas id="chart-heart" width="640" height="248"></canvas>
        </article>
        <article class="chart-card" data-chart-card="air">
          <div class="chart-head"><span>Air Flow (L/min)</span><span class="blue"><span id="live-air">10</span> <small>L/min</small></span></div>
          <canvas id="chart-air" width="640" height="248"></canvas>
        </article>
      </div>

      <div class="content" id="page-modes" hidden>
        <div class="mode-list" id="mode-list"></div>
      </div>

      <div class="content" id="page-history" hidden>
        <div class="list-card">
          <div class="history-main">
            <div><strong><span class="cyan" data-icon="calendar"></span> Session 01</strong></div>
            <span class="small">08 July 2026</span>
          </div>
          <div class="row"><span class="small">Start Time<br><strong style="color:#cfd6e2">09:00 AM</strong></span><span class="small">Duration<br><strong style="color:#cfd6e2">30 min</strong></span></div>
        </div>
        <h3 style="font-size:16px;margin:20px 0 12px;">Average Values</h3>
        <div class="list-card averages">
          <div class="row"><strong>SpO₂</strong><strong class="cyan">97 %</strong></div>
          <div class="row"><strong>Heart Rate</strong><strong class="red">78 BPM</strong></div>
          <div class="row"><strong>Air Flow</strong><strong class="blue">11 L/min</strong></div>
          <div class="row"><strong>Temperature</strong><strong class="amber">36.6 °C</strong></div>
        </div>
        <div class="history-list">
          <div class="list-card row"><span><span class="green" data-icon="calendar"></span> <strong>Session 02</strong><br><span class="small">25 min</span></span><span class="small">07 July 2026 ›</span></div>
          <div class="list-card row"><span><span class="green" data-icon="calendar"></span> <strong>Session 03</strong><br><span class="small">95 min</span></span><span class="small">06 July 2026 ›</span></div>
        </div>
      </div>

      <div class="content" id="page-alerts" hidden>
        <div class="alert-card" style="border-color:rgba(55,213,106,.45);background:linear-gradient(135deg,rgba(55,213,106,.18),rgba(21,27,39,.95));display:flex;gap:16px;align-items:center;">
          <div class="shield" style="width:58px;height:58px;"><span data-icon="shield"></span></div>
          <div><strong>System Status</strong><div class="green" style="font-size:18px;font-weight:900;">Normal</div><div class="small">All systems are working properly.</div></div>
        </div>
        <h3 style="font-size:16px;margin:22px 0 12px;">Active Alerts</h3>
        <div class="alert-card" style="min-height:118px;display:grid;place-items:center;text-align:center;">
          <div><span class="green" data-icon="checkcircle"></span><br><strong class="small" style="font-size:16px;">No Active Alerts</strong><div class="small">Everything is under control.</div></div>
        </div>
        <h3 style="font-size:16px;margin:22px 0 12px;">Sensor Status</h3>
        <div class="sensor-list list-card">
          <div class="row"><strong><span class="green" data-icon="sensor"></span> MAX30102 Sensor</strong><strong class="green">Connected</strong></div>
          <div class="row"><strong><span class="green" data-icon="thermo"></span> Temperature Sensor</strong><strong class="green">Connected</strong></div>
          <div class="row"><strong><span class="green" data-icon="droplet"></span> Flow Sensor</strong><strong class="green">Connected</strong></div>
        </div>
      </div>

      <div class="content" id="page-settings" hidden>
        <h3 style="font-size:15px;color:var(--muted);margin:0 0 10px;">Device</h3>
        <div class="settings-list list-card">
          <div class="row"><span>Device Name</span><strong>ESP32-SRSS ›</strong></div>
          <div class="row"><span>WiFi Status</span><strong class="green">Connected</strong></div>
          <div class="row"><span>IP Address</span><strong>192.168.4.1</strong></div>
        </div>
        <h3 style="font-size:15px;color:var(--muted);margin:20px 0 10px;">Alarm Limits</h3>
        <div class="settings-list list-card">
          <div class="row"><span>SpO₂ Minimum</span><strong class="cyan">90 % ›</strong></div>
          <div class="row"><span>Heart Rate Max</span><strong class="red">120 BPM ›</strong></div>
          <div class="row"><span>Temperature Max</span><strong class="amber">38.0 °C ›</strong></div>
        </div>
        <h3 style="font-size:15px;color:var(--muted);margin:20px 0 10px;">General</h3>
        <div class="settings-list list-card">
          <div class="row"><span>Theme</span><span class="theme-toggle" aria-label="Theme selector"><button class="theme-option active" data-theme="dark">Dark</button><button class="theme-option" data-theme="light">Light</button></span></div>
          <div class="row"><span>Units</span><strong>Metric (°C, L/min) ›</strong></div>
          <div class="row"><span>About System</span><strong>v1.0.0 ›</strong></div>
        </div>
      </div>

      <nav class="bottom-nav" aria-label="Main navigation">
        <button class="navitem active" data-page="dashboard"><span data-icon="home"></span>Home</button>
        <button class="navitem" data-page="history"><span data-icon="calendar"></span>History</button>
        <button class="navitem" data-page="modes"><span data-icon="modes"></span>Modes</button>
        <button class="navitem" data-page="alerts"><span data-icon="bell"></span>Alerts</button>
        <button class="navitem" data-page="settings"><span data-icon="gear"></span>Settings</button>
      </nav>
    </section>

    <aside class="drawer" id="drawer" aria-hidden="true">
      <div class="scrim" id="scrim"></div>
      <div class="drawer-panel">
        <div class="drawer-brand"><div class="logo" aria-hidden="true"></div><div><h2 style="margin:0;font-size:24px;">SRSS</h2><div class="small">Smart Respiratory<br>Support System</div></div></div>
        <button class="drawer-link active" data-drawer-page="dashboard"><span data-icon="dashboard"></span>Dashboard</button>
        <button class="drawer-link" data-drawer-page="live"><span data-icon="chart"></span>Live Data</button>
        <button class="drawer-link" data-drawer-page="modes"><span data-icon="modes"></span>Operating Modes</button>
        <button class="drawer-link" data-drawer-page="history"><span data-icon="calendar"></span>History</button>
        <button class="drawer-link" data-drawer-page="alerts"><span data-icon="bell"></span>Alerts</button>
        <button class="drawer-link" data-drawer-page="settings"><span data-icon="gear"></span>Settings</button>
        <div class="drawer-sep"></div>
        <button class="drawer-link danger" id="disconnect-btn"><span data-icon="unlink"></span>Disconnect Device</button>
      </div>
    </aside>
  </main>

  <script>
    const iconSvg = {
      menu: '<svg width="22" height="22" viewBox="0 0 24 24" fill="none"><path d="M4 7h16M4 12h16M4 17h16" stroke="currentColor" stroke-width="2" stroke-linecap="round"/></svg>',
      more: '<svg width="22" height="22" viewBox="0 0 24 24" fill="none"><path d="M12 5.5v.01M12 12v.01M12 18.5v.01" stroke="currentColor" stroke-width="3" stroke-linecap="round"/></svg>',
      wifi: '<svg width="20" height="20" viewBox="0 0 24 24" fill="none"><path d="M5 10a10 10 0 0 1 14 0M8 13a6 6 0 0 1 8 0M11 16a2 2 0 0 1 2 0" stroke="currentColor" stroke-width="2" stroke-linecap="round"/><circle cx="12" cy="19" r="1.5" fill="currentColor"/></svg>',
      pulse: '<svg width="21" height="21" viewBox="0 0 24 24" fill="none"><path d="M3 13h4l2-7 4 13 2-6h6" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"/></svg>',
      droplet: '<svg width="22" height="22" viewBox="0 0 24 24" fill="none"><path d="M12 3s7 7.2 7 12a7 7 0 0 1-14 0c0-4.8 7-12 7-12Z" fill="currentColor" opacity=".95"/></svg>',
      heart: '<svg width="22" height="22" viewBox="0 0 24 24" fill="currentColor"><path d="M12 21s-7.5-4.4-9.4-9.2C1.2 8.2 3.4 5 6.8 5c2 0 3.3 1 4.1 2.2C11.7 6 13 5 15.1 5c3.5 0 5.6 3.2 4.2 6.8C17.5 16.6 12 21 12 21Z"/></svg>',
      air: '<svg width="23" height="23" viewBox="0 0 24 24" fill="none"><path d="M4 8h10a3 3 0 1 0-3-3M3 13h15a3 3 0 1 1-3 3M5 18h6" stroke="currentColor" stroke-width="2" stroke-linecap="round"/></svg>',
      thermo: '<svg width="22" height="22" viewBox="0 0 24 24" fill="none"><path d="M14 14.8V5a2 2 0 1 0-4 0v9.8a4 4 0 1 0 4 0Z" stroke="currentColor" stroke-width="2"/><path d="M12 9v8" stroke="currentColor" stroke-width="2" stroke-linecap="round"/></svg>',
      shield: '<svg width="24" height="24" viewBox="0 0 24 24" fill="currentColor"><path d="M12 2 20 5v6c0 5-3.4 9-8 11-4.6-2-8-6-8-11V5l8-3Zm-1 13.2 5.3-5.3-1.4-1.4L11 12.4 9.1 10.5 7.7 12l3.3 3.2Z"/></svg>',
      link: '<svg width="20" height="20" viewBox="0 0 24 24" fill="none"><path d="M10 14a5 5 0 0 0 7 0l2-2a5 5 0 0 0-7-7l-1 1M14 10a5 5 0 0 0-7 0l-2 2a5 5 0 0 0 7 7l1-1" stroke="currentColor" stroke-width="2" stroke-linecap="round"/></svg>',
      copy: '<svg width="19" height="19" viewBox="0 0 24 24" fill="none"><path d="M8 8h10v12H8zM6 16H5a1 1 0 0 1-1-1V5h10v1" stroke="currentColor" stroke-width="2" stroke-linejoin="round"/></svg>',
      home: '<svg width="20" height="20" viewBox="0 0 24 24" fill="currentColor"><path d="M3 11 12 3l9 8v9a1 1 0 0 1-1 1h-5v-6H9v6H4a1 1 0 0 1-1-1v-9Z"/></svg>',
      calendar: '<svg width="20" height="20" viewBox="0 0 24 24" fill="none"><path d="M7 3v3M17 3v3M4 8h16M5 5h14v15H5z" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"/></svg>',
      modes: '<svg width="20" height="20" viewBox="0 0 24 24" fill="none"><path d="M8 5c3 0 5 3 8 3M8 19c3 0 5-3 8-3M7 5a3 3 0 1 0 0 6 3 3 0 0 0 0-6Zm10 8a3 3 0 1 0 0 6 3 3 0 0 0 0-6Z" stroke="currentColor" stroke-width="2"/></svg>',
      bell: '<svg width="20" height="20" viewBox="0 0 24 24" fill="currentColor"><path d="M12 22a2.7 2.7 0 0 0 2.6-2H9.4a2.7 2.7 0 0 0 2.6 2Zm7-5-1.8-2.1V10a5.2 5.2 0 0 0-4-5V3a1.2 1.2 0 1 0-2.4 0v2a5.2 5.2 0 0 0-4 5v4.9L5 17v1h14v-1Z"/></svg>',
      gear: '<svg width="20" height="20" viewBox="0 0 24 24" fill="none"><path d="M12 15.5a3.5 3.5 0 1 0 0-7 3.5 3.5 0 0 0 0 7Z" stroke="currentColor" stroke-width="2"/><path d="m19 13.5 1.5 1-2 3.5-1.8-.7a7 7 0 0 1-1.7 1l-.3 1.9h-4l-.3-1.9a7 7 0 0 1-1.7-1l-1.8.7-2-3.5 1.5-1a7 7 0 0 1 0-2l-1.5-1 2-3.5 1.8.7a7 7 0 0 1 1.7-1l.3-1.9h4l.3 1.9a7 7 0 0 1 1.7 1l1.8-.7 2 3.5-1.5 1a7 7 0 0 1 0 2Z" stroke="currentColor" stroke-width="2" stroke-linejoin="round"/></svg>',
      checkcircle: '<svg width="24" height="24" viewBox="0 0 24 24" fill="none"><circle cx="12" cy="12" r="9" stroke="currentColor" stroke-width="2"/><path d="m8 12 2.6 2.6L16.5 9" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"/></svg>',
      sensor: '<svg width="20" height="20" viewBox="0 0 24 24" fill="none"><circle cx="12" cy="12" r="3" stroke="currentColor" stroke-width="2"/><path d="M6 12a6 6 0 0 1 12 0M3 12a9 9 0 0 1 18 0" stroke="currentColor" stroke-width="2" stroke-linecap="round"/></svg>',
      dashboard: '<svg width="20" height="20" viewBox="0 0 24 24" fill="none"><path d="M4 13h6V4H4v9Zm10 7h6V4h-6v16ZM4 20h6v-4H4v4Z" stroke="currentColor" stroke-width="2" stroke-linejoin="round"/></svg>',
      chart: '<svg width="20" height="20" viewBox="0 0 24 24" fill="none"><path d="M4 19V5M4 19h16M7 15l3-4 4 2 4-7" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"/></svg>',
      unlink: '<svg width="20" height="20" viewBox="0 0 24 24" fill="none"><path d="m4 4 16 16M9 15a5 5 0 0 0 6.8-.2l1.2-1.2M14.5 9.5a5 5 0 0 0-6.5.4l-1.9 1.9a5 5 0 0 0 5.3 8.2M13 4.2a5 5 0 0 1 4.9 8.1" stroke="currentColor" stroke-width="2" stroke-linecap="round"/></svg>',
      play: '<svg width="15" height="15" viewBox="0 0 24 24" fill="currentColor"><path d="M8 5v14l11-7L8 5Z"/></svg>',
      pause: '<svg width="15" height="15" viewBox="0 0 24 24" fill="currentColor"><path d="M7 5h4v14H7V5Zm6 0h4v14h-4V5Z"/></svg>',
      stop: '<svg width="15" height="15" viewBox="0 0 24 24" fill="currentColor"><path d="M7 7h10v10H7V7Z"/></svg>'
    };

    document.querySelectorAll("[data-icon]").forEach(el => el.innerHTML = iconSvg[el.dataset.icon] || "");
    document.querySelectorAll(".logo").forEach(el => {
      el.innerHTML = '<svg viewBox="0 0 140 140" aria-hidden="true"><defs><linearGradient id="lung" x1="18" y1="120" x2="120" y2="16" gradientUnits="userSpaceOnUse"><stop stop-color="#8d61ff"/><stop offset=".55" stop-color="#27d6e0"/><stop offset="1" stop-color="#35d56d"/></linearGradient></defs><path d="M66 24v40c-12-22-28-28-41-18-18 14-23 50-12 71 13 9 40-3 50-22V65" fill="url(#lung)" opacity=".95"/><path d="M74 24v40c12-22 28-28 41-18 18 14 23 50 12 71-13 9-40-3-50-22V65" fill="url(#lung)" opacity=".95"/><path d="M70 18v52M35 81h17l6-14 9 29 9-19h29" fill="none" stroke="#07101a" stroke-width="7" stroke-linecap="round" stroke-linejoin="round"/><path d="M35 81h17l6-14 9 29 9-19h29" fill="none" stroke="#2ddde0" stroke-width="3" stroke-linecap="round" stroke-linejoin="round"/></svg>';
    });

    const pages = ["dashboard", "live", "modes", "history", "alerts", "settings"];
    const titles = { dashboard: "Dashboard", live: "Live Data", modes: "Operating Modes", history: "History", alerts: "Alerts", settings: "Settings" };
    const state = {
      page: "dashboard",
      mode: "Monitoring",
      theme: localStorage.getItem("srss-theme") || "dark",
      connected: false,
      deviceName: "ESP32-SRSS",
      deviceIp: "192.168.4.1",
      wifiSsid: "TiTo",
      wifiPassword: "",
      pollTimer: null,
      sessionStart: null,
      sessionElapsed: 0,
      sessionRunning: false,
      spo2: 98,
      heart: 75,
      air: 10,
      temp: 36.5,
      series: {
        spo2: Array.from({ length: 46 }, (_, i) => 93 + Math.sin(i / 6) * 1.2 + Math.random() * 2),
        heart: Array.from({ length: 46 }, (_, i) => 76 + Math.sin(i / 5) * 8 + (Math.random() - .5) * 10),
        air: Array.from({ length: 46 }, (_, i) => 10 + Math.sin(i / 4) * 1.3 + Math.random() * 2)
      }
    };

    const normalRanges = {
      spo2: value => value >= 90,
      heart: value => value >= 50 && value <= 120,
      air: value => value >= 5 && value <= 20,
      temp: value => value >= 35 && value <= 38
    };

    function setReadingState(key, normalColor, isNormal) {
      const metric = document.querySelector(`.metric.${key === "heart" ? "hr" : key}`);
      const value = document.getElementById(`${key}-value`);
      const unit = document.getElementById(`${key}-unit`);
      const status = document.getElementById(`${key}-status`);
      const icon = document.getElementById(`${key}-icon`);
      const liveWrap = document.getElementById(`live-${key}-wrap`);
      [value, unit, icon].filter(Boolean).forEach(el => {
        el.classList.remove("cyan", "green", "blue", "amber", "red");
        el.classList.add(isNormal ? normalColor : "red");
      });
      if (liveWrap) {
        liveWrap.classList.remove("cyan", "green", "blue", "amber", "red");
        liveWrap.classList.add(isNormal ? normalColor : "red");
      }
      metric?.classList.toggle("abnormal", !isNormal);
      if (status) {
        status.textContent = isNormal ? "Normal" : "Abnormal";
        status.classList.toggle("red", !isNormal);
      }
    }

    function updateReadingStates() {
      setReadingState("spo2", "cyan", normalRanges.spo2(state.spo2));
      setReadingState("heart", "green", normalRanges.heart(state.heart));
      setReadingState("air", "blue", normalRanges.air(state.air));
      setReadingState("temp", "amber", normalRanges.temp(state.temp));
    }

    function currentSessionElapsed() {
      if (!state.sessionRunning || state.sessionStart === null) return state.sessionElapsed;
      return state.sessionElapsed + Math.floor((Date.now() - state.sessionStart) / 1000);
    }

    function updateSessionControls() {
      const label = document.getElementById("session-state-label");
      const start = document.getElementById("session-start-btn");
      const pause = document.getElementById("session-pause-btn");
      const stop = document.getElementById("session-stop-btn");
      const hasTime = currentSessionElapsed() > 0;
      if (label) label.textContent = state.sessionRunning ? "Session running" : hasTime ? "Session paused" : "Session stopped";
      if (start) start.disabled = state.sessionRunning;
      if (pause) pause.disabled = !state.sessionRunning;
      if (stop) stop.disabled = !state.sessionRunning && !hasTime;
    }

    function updateSessionTimer() {
      const elapsed = currentSessionElapsed();
      const minutes = String(Math.floor(elapsed / 60)).padStart(2, "0");
      const seconds = String(elapsed % 60).padStart(2, "0");
      const node = document.getElementById("session-timer");
      if (node) node.textContent = `${minutes}:${seconds}`;
      updateSessionControls();
    }

    function startSession() {
      if (state.sessionRunning) return;
      state.sessionStart = Date.now();
      state.sessionRunning = true;
      updateSessionTimer();
    }

    function pauseSession() {
      if (!state.sessionRunning) return;
      state.sessionElapsed = currentSessionElapsed();
      state.sessionStart = null;
      state.sessionRunning = false;
      updateSessionTimer();
    }

    function stopSession() {
      state.sessionElapsed = 0;
      state.sessionStart = null;
      state.sessionRunning = false;
      updateSessionTimer();
    }

    function applyTheme(theme) {
      state.theme = theme === "light" ? "light" : "dark";
      document.body.classList.toggle("light-theme", state.theme === "light");
      localStorage.setItem("srss-theme", state.theme);
      document.querySelectorAll(".theme-option").forEach(btn => {
        btn.classList.toggle("active", btn.dataset.theme === state.theme);
      });
      if (state.page === "live") drawCharts();
    }

    function endpoint(path) {
      if (location.protocol.startsWith("http")) return path;
      const cleanIp = state.deviceIp.replace(/^https?:\/\//, "").replace(/\/.*$/, "").trim();
      return `http://${cleanIp}${path}`;
    }

    function setConnectionStatus(status, message) {
      const pill = document.getElementById("connection-status");
      const label = document.getElementById("connection-status-text");
      pill.classList.toggle("waiting", status === "waiting");
      pill.classList.toggle("offline", status === "offline");
      label.textContent = status === "connected" ? "Connected" : status === "waiting" ? "Connecting..." : "Not connected";
      document.getElementById("connect-message").innerHTML = message;
    }

    function applyReading(data) {
      const spo2 = Number(data.spo2 ?? data.SpO2 ?? data.oxygen ?? data.oxygenSaturation);
      const heart = Number(data.heartRate ?? data.heart ?? data.bpm ?? data.hr);
      const air = Number(data.airFlow ?? data.airflow ?? data.flow ?? data.air);
      const temp = Number(data.temperature ?? data.temp ?? data.bodyTemp);
      if (Number.isFinite(spo2)) state.spo2 = Math.max(0, Math.min(100, spo2));
      if (Number.isFinite(heart)) state.heart = Math.max(0, heart);
      if (Number.isFinite(air)) state.air = Math.max(0, air);
      if (Number.isFinite(temp)) state.temp = temp;
      pushCurrentValues();
    }

    function normalizeModeName(value) {
      const text = String(value || "").toLowerCase().replace(/[_-]/g, " ").trim();
      if (text.includes("nebulizer")) return "Nebulizer";
      if (text.includes("controlled") || text.includes("ctrl")) return "Controlled Vent";
      if (text.includes("ventilation") || text === "vent") return "Ventilation";
      if (text.includes("stop") || text.includes("emergency")) return "Emergency Stop";
      return "Monitoring";
    }

    function modeApiValue(mode) {
      return {
        "Monitoring": "monitoring",
        "Nebulizer": "nebulizer",
        "Ventilation": "ventilation",
        "Controlled Vent": "controlled",
        "Emergency Stop": "stop"
      }[mode] || "monitoring";
    }

    function setModeUi(mode) {
      state.mode = normalizeModeName(mode);
      document.getElementById("current-mode-label").textContent = state.mode;
      document.querySelectorAll(".mode-option").forEach(item => {
        const active = item.dataset.mode === state.mode;
        item.classList.toggle("active", active);
        item.querySelector(".check").textContent = active ? "✓" : "";
      });
      const bannerStatus = document.getElementById("banner-status");
      if (bannerStatus) bannerStatus.textContent = state.mode === "Emergency Stop" ? "Emergency Stop Active" : "Device Connected";
    }

    function applyStatus(data) {
      if (!data || typeof data !== "object") return;
      if (data.mode || data.modeName || data.currentMode) setModeUi(data.mode || data.modeName || data.currentMode);
      if (typeof data.connected === "boolean") state.connected = data.connected || state.connected;
      if (data.alarm || data.emergencyStop) setModeUi("Emergency Stop");
    }

    async function fetchJsonWithTimeout(url, options = {}, timeoutMs = 3000) {
      const controller = new AbortController();
      const timeout = setTimeout(() => controller.abort(), timeoutMs);
      try {
        const response = await fetch(url, { ...options, signal: controller.signal });
        if (!response.ok) throw new Error(`HTTP ${response.status}`);
        return await response.json();
      } finally {
        clearTimeout(timeout);
      }
    }

    async function sendWifiCredentials() {
      if (!state.wifiSsid && !state.wifiPassword) return;
      try {
        await fetchJsonWithTimeout(endpoint("/wifi"), {
          method: "POST",
          headers: { "Content-Type": "application/json" },
          body: JSON.stringify({ ssid: state.wifiSsid, password: state.wifiPassword })
        }, 2500);
      } catch (error) {
        console.info("Wi-Fi provisioning endpoint is not available:", error.message);
      }
    }

    async function readDeviceOnce() {
      const data = await fetchJsonWithTimeout(endpoint("/data"), { cache: "no-store" }, 3500);
      applyReading(data);
      return data;
    }

    async function readStatusOnce() {
      const data = await fetchJsonWithTimeout(endpoint("/status"), { cache: "no-store" }, 2500);
      applyStatus(data);
      return data;
    }

    async function setDeviceMode(mode) {
      setModeUi(mode);
      if (!state.connected && !location.protocol.startsWith("http")) return;
      try {
        const value = encodeURIComponent(modeApiValue(mode));
        const data = await fetchJsonWithTimeout(endpoint(`/mode?value=${value}`), { method: "POST", cache: "no-store" }, 2500);
        applyStatus(data);
      } catch (error) {
        console.info("Mode endpoint is not available:", error.message);
      }
    }

    async function connectToDevice() {
      state.deviceName = document.getElementById("device-name").value;
      state.wifiSsid = document.getElementById("wifi-ssid").value.trim();
      state.wifiPassword = document.getElementById("wifi-password").value;
      state.deviceIp = document.getElementById("device-ip").value.trim() || "192.168.4.1";
      document.getElementById("banner-device").textContent = state.deviceName;
      document.getElementById("banner-ip").textContent = state.deviceIp;
      setConnectionStatus("waiting", `Trying <strong>${endpoint("/data")}</strong>...`);
      document.getElementById("connect-btn").disabled = true;
      try {
        await sendWifiCredentials();
        await readDeviceOnce();
        try { await readStatusOnce(); } catch (error) { console.info("Status endpoint is not available:", error.message); }
        state.connected = true;
        stopSession();
        startSession();
        setConnectionStatus("connected", `Connected to <strong>${state.deviceIp}</strong>. Reading live data from <strong>/data</strong>.`);
        document.getElementById("connect-screen").classList.remove("active");
        document.getElementById("app-screen").classList.add("active");
        setPage("dashboard");
        if (state.pollTimer) clearInterval(state.pollTimer);
        state.pollTimer = setInterval(pollDeviceReadings, 1400);
      } catch (error) {
        state.connected = false;
        setConnectionStatus("offline", `Could not read <strong>${endpoint("/data")}</strong>. Make sure you are connected to <strong>${state.wifiSsid || "the ESP32 network"}</strong> and the ESP32 sends CORS headers.`);
      } finally {
        document.getElementById("connect-btn").disabled = false;
      }
    }

    async function pollDeviceReadings() {
      if (!state.connected) return;
      try {
        await readDeviceOnce();
        try { await readStatusOnce(); } catch (error) { console.info("Status endpoint is not available:", error.message); }
      } catch (error) {
        state.connected = false;
        clearInterval(state.pollTimer);
        state.pollTimer = null;
        document.getElementById("banner-status").textContent = "Device Disconnected";
        alert("Lost connection to the ESP32 readings endpoint.");
      }
    }

    const modeData = [
      { name: "Monitoring", desc: "Monitor real-time vital signs and system parameters.", color: "var(--green)", icon: "pulse" },
      { name: "Nebulizer", desc: "Deliver medication as mist for respiratory care.", color: "var(--blue)", icon: "droplet" },
      { name: "Ventilation", desc: "Provide assisted ventilation support.", color: "var(--purple)", icon: "modes" },
      { name: "Controlled Vent", desc: "Automatic breath control and support.", color: "var(--amber)", icon: "gear" },
      { name: "Emergency Stop", desc: "Immediately stop all outputs and airflow.", color: "var(--red)", icon: "unlink", emergency: true }
    ];

    const modeList = document.getElementById("mode-list");
    modeData.forEach(item => {
      const node = document.createElement("button");
      node.className = "mode-option" + (item.name === state.mode ? " active" : "") + (item.emergency ? " emergency" : "");
      node.dataset.mode = item.name;
      node.innerHTML = `<div class="round-icon" style="background:${item.color}">${iconSvg[item.icon]}</div><div style="text-align:left"><strong ${item.emergency ? 'class="red"' : ""}>${item.name}</strong><div class="small">${item.desc}</div></div><span class="check">${item.name === state.mode ? "✓" : ""}</span>`;
      modeList.appendChild(node);
    });

    function setPage(page) {
      state.page = page;
      pages.forEach(name => {
        document.getElementById(`page-${name}`).hidden = name !== page;
      });
      document.getElementById("page-title").textContent = titles[page];
      document.querySelectorAll(".navitem").forEach(btn => btn.classList.toggle("active", btn.dataset.page === page));
      document.querySelectorAll(".drawer-link[data-drawer-page]").forEach(btn => btn.classList.toggle("active", btn.dataset.drawerPage === page));
      closeDrawer();
      if (page === "live") drawCharts();
    }

    function openDrawer() {
      document.getElementById("drawer").classList.add("open");
      document.getElementById("drawer").setAttribute("aria-hidden", "false");
    }

    function closeDrawer() {
      document.getElementById("drawer").classList.remove("open");
      document.getElementById("drawer").setAttribute("aria-hidden", "true");
    }

    document.getElementById("connect-btn").addEventListener("click", connectToDevice);
    document.getElementById("disconnect-btn").addEventListener("click", () => {
      state.connected = false;
      if (state.pollTimer) clearInterval(state.pollTimer);
      state.pollTimer = null;
      closeDrawer();
      document.getElementById("app-screen").classList.remove("active");
      document.getElementById("connect-screen").classList.add("active");
      setConnectionStatus("offline", "Disconnected. Enter network details and connect again.");
      stopSession();
    });
    document.getElementById("menu-btn").addEventListener("click", openDrawer);
    document.getElementById("scrim").addEventListener("click", closeDrawer);
    document.querySelectorAll(".navitem").forEach(btn => btn.addEventListener("click", () => setPage(btn.dataset.page)));
    document.querySelectorAll(".drawer-link[data-drawer-page]").forEach(btn => btn.addEventListener("click", () => setPage(btn.dataset.drawerPage)));
    document.querySelectorAll(".theme-option").forEach(btn => btn.addEventListener("click", () => applyTheme(btn.dataset.theme)));
    document.getElementById("session-start-btn").addEventListener("click", startSession);
    document.getElementById("session-pause-btn").addEventListener("click", pauseSession);
    document.getElementById("session-stop-btn").addEventListener("click", stopSession);

    document.querySelectorAll(".mode-option").forEach(btn => {
      btn.addEventListener("click", () => setDeviceMode(btn.dataset.mode));
    });

    document.querySelectorAll(".tab").forEach(tab => {
      tab.addEventListener("click", () => {
        document.querySelectorAll(".tab").forEach(t => t.classList.toggle("active", t === tab));
        const key = tab.dataset.chartTab;
        document.querySelectorAll("[data-chart-card]").forEach(card => {
          card.style.display = (key === "temp" || card.dataset.chartCard === key) ? "" : "none";
        });
        if (key === "temp") document.querySelectorAll("[data-chart-card]").forEach(card => card.style.display = "");
      });
    });

    function updateValues() {
      if (state.connected) return;
      state.spo2 = Math.max(92, Math.min(99, state.spo2 + (Math.random() - .45) * .9));
      state.heart = Math.max(62, Math.min(96, state.heart + (Math.random() - .48) * 4));
      state.air = Math.max(7, Math.min(15, state.air + (Math.random() - .5) * 1.1));
      state.temp = Math.max(36.1, Math.min(37.4, state.temp + (Math.random() - .5) * .12));
      pushCurrentValues();
    }

    function pushCurrentValues() {
      state.series.spo2.push(state.spo2); state.series.spo2.shift();
      state.series.heart.push(state.heart); state.series.heart.shift();
      state.series.air.push(state.air); state.series.air.shift();
      document.getElementById("spo2-value").textContent = Math.round(state.spo2);
      document.getElementById("heart-value").textContent = Math.round(state.heart);
      document.getElementById("air-value").textContent = Math.round(state.air);
      document.getElementById("temp-value").textContent = state.temp.toFixed(1);
      updateReadingStates();
      document.getElementById("live-spo2").textContent = Math.round(state.spo2);
      document.getElementById("live-heart").textContent = Math.round(state.heart);
      document.getElementById("live-air").textContent = Math.round(state.air);
      if (state.page === "live") drawCharts();
    }

    function drawChart(id, values, color, fill, min, max) {
      const canvas = document.getElementById(id);
      const ctx = canvas.getContext("2d");
      const w = canvas.width;
      const h = canvas.height;
      ctx.clearRect(0, 0, w, h);
      ctx.strokeStyle = "rgba(255,255,255,.07)";
      ctx.lineWidth = 1;
      for (let i = 1; i < 4; i++) {
        const y = (h / 4) * i;
        ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(w, y); ctx.stroke();
      }
      const pts = values.map((v, i) => ({
        x: i / (values.length - 1) * w,
        y: h - ((v - min) / (max - min)) * (h - 26) - 13
      }));
      ctx.beginPath();
      pts.forEach((p, i) => i ? ctx.lineTo(p.x, p.y) : ctx.moveTo(p.x, p.y));
      ctx.lineTo(w, h); ctx.lineTo(0, h); ctx.closePath();
      const gradient = ctx.createLinearGradient(0, 0, 0, h);
      gradient.addColorStop(0, fill);
      gradient.addColorStop(1, "rgba(0,0,0,0)");
      ctx.fillStyle = gradient;
      ctx.fill();
      ctx.beginPath();
      pts.forEach((p, i) => i ? ctx.lineTo(p.x, p.y) : ctx.moveTo(p.x, p.y));
      ctx.strokeStyle = color;
      ctx.lineWidth = 4;
      ctx.lineJoin = "round";
      ctx.lineCap = "round";
      ctx.stroke();
    }

    function drawCharts() {
      drawChart("chart-spo2", state.series.spo2, "#24d7df", "rgba(36,215,223,.2)", 88, 100);
      const heartNormal = normalRanges.heart(state.heart);
      drawChart("chart-heart", state.series.heart, heartNormal ? "#37d56a" : "#ff4f64", heartNormal ? "rgba(55,213,106,.18)" : "rgba(255,79,100,.19)", 45, 125);
      drawChart("chart-air", state.series.air, "#4da1ff", "rgba(77,161,255,.2)", 0, 22);
    }

    applyTheme(state.theme);
    updateReadingStates();
    updateSessionTimer();
    drawCharts();
    if (location.protocol.startsWith("http")) {
      state.connected = true;
      state.deviceIp = location.host || "192.168.4.1";
      document.getElementById("banner-ip").textContent = state.deviceIp;
      state.pollTimer = setInterval(pollDeviceReadings, 1400);
      pollDeviceReadings();
    }
    setInterval(updateValues, 1400);
    setInterval(updateSessionTimer, 1000);
  </script>
</body>
</html>

)SRSSHTML";

// ════════════════════════════════════════════════════════════════
//  Web handlers
// ════════════════════════════════════════════════════════════════
void handleRoot() {
  server.send_P(200, "text/html", htmlPage);
}

void handleData() {
  // Heart rate: subtract 35, then cap at 105 BPM
  int32_t displayHR = 0;
  if (validHeartRate) {
    displayHR = heartRate - 35;
    if (displayHR < 0)   displayHR = 0;
    if (displayHR > 105) displayHR = 105;
  }

  // Air flow: raw / 5, max 15 L/min
  float displayFlow = rawFlowRate / 5.0f;
  if (displayFlow > 15.0f) displayFlow = 15.0f;

  String json = "{";
  json += "\"hr\":"          + String(displayHR)       + ",";
  json += "\"hrValid\":"     + String(validHeartRate)  + ",";
  json += "\"spo2\":"        + String(spo2)            + ",";
  json += "\"spo2Valid\":"   + String(validSPO2)       + ",";
  json += "\"temp\":"        + String(temperatureC, 1) + ",";
  json += "\"flow\":"        + String(displayFlow, 2)  + ",";
  json += "\"totalLiters\":" + String(totalLiters, 3);
  json += "}";
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "application/json", json);
}


void sendApiHeaders() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.sendHeader("Access-Control-Allow-Methods", "GET,POST,OPTIONS");
  server.sendHeader("Access-Control-Allow-Headers", "Content-Type");
}

bool setModeFromText(String requested) {
  requested.toLowerCase();
  requested.replace("_", " ");
  requested.replace("-", " ");
  requested.trim();

  if (requested == "monitoring" || requested == "monitor") {
    emergencyStopActive = false;
    currentMode = MODE_MONITORING;
    relayAutoMode = false;
    applyRelay(true);         // relay ON in Monitoring, same as Nebulizer/Ventilation
    noTone(BUZZER_PIN);
    startBeep(2000, 80);
    Serial.println("Mode -> Monitoring (web)");
    return true;
  }
  if (requested == "nebulizer" || requested == "nebulizer therapy") {
    emergencyStopActive = false;
    currentMode = MODE_NEBULIZER;
    startBeep(2000, 80);
    Serial.println("Mode -> Nebulizer Therapy (web)");
    return true;
  }
  if (requested == "ventilation" || requested == "vent") {
    emergencyStopActive = false;
    currentMode = MODE_VENTILATION;
    startBeep(2000, 80);
    Serial.println("Mode -> Ventilation (web)");
    return true;
  }
  if (requested == "controlled" || requested == "controlled vent" || requested == "controlled ventilation" || requested == "ctrl vent" || requested == "ctrl.vent") {
    emergencyStopActive = false;
    currentMode = MODE_CONTROLLED_VENT;
    relayAutoMode   = true;      // (re)start the 3s ON / 3s OFF cycle
    relayLastToggle = millis();
    applyRelay(true);            // begin the cycle in the ON phase
    startBeep(2000, 80);
    Serial.println("Mode -> Controlled Ventilation (web)");
    return true;
  }
  if (requested == "stop" || requested == "emergency" || requested == "emergency stop") {
    // NOTE: this web-triggered STOP still works exactly as before, but since
    // the physical toggle switch is now polled continuously in
    // handleEmergencySwitch(), if the hardware switch is currently OFF it
    // will flip emergencyStopActive back to false again on the very next
    // loop() pass. The hardware switch is the authoritative source of
    // truth for the emergency state, same as a real safety interlock.
    emergencyStopActive = true;
    currentMode = MODE_STOPPED;
    startBeep(4000, 300);
    applyRelay(true);         // relay ON during Emergency Stop, same as Nebulizer/Ventilation
    relayAutoMode = false;    // hold this state until operator re-selects a mode
    Serial.println("!!! EMERGENCY STOP FROM WEB !!! Relay ON.");
    return true;
  }
  return false;
}

void handleStatus() {
  String json = "{";
  json += "\"connected\":true,";
  json += "\"alarm\":"; json += emergencyStopActive ? "true" : "false"; json += ",";
  json += "\"emergencyStop\":"; json += emergencyStopActive ? "true" : "false"; json += ",";
  json += "\"modeId\":"; json += String(currentMode); json += ",";
  json += "\"mode\":\""; json += modeName(currentMode); json += "\",";
  json += "\"relayOn\":"; json += relayState ? "true" : "false"; json += ",";
  json += "\"relayAutoMode\":"; json += relayAutoMode ? "true" : "false"; json += ",";
  json += "\"wifiSsid\":\"TiTo\",";
  json += "\"ip\":\""; json += WiFi.softAPIP().toString(); json += "\",";
  json += "\"max30102\":"; json += sensorFound ? "true" : "false";
  json += "}";
  sendApiHeaders();
  server.send(200, "application/json", json);
}

void handleMode() {
  String requested = server.arg("value");
  if (requested.length() == 0) requested = server.arg("mode");
  if (requested.length() == 0) requested = server.arg("plain");
  if (requested.length() == 0) {
    sendApiHeaders();
    server.send(400, "application/json", "{\"ok\":false,\"error\":\"missing mode value\"}");
    return;
  }

  if (!setModeFromText(requested)) {
    sendApiHeaders();
    server.send(400, "application/json", "{\"ok\":false,\"error\":\"invalid mode\"}");
    return;
  }

  String json = "{";
  json += "\"ok\":true,";
  json += "\"modeId\":"; json += String(currentMode); json += ",";
  json += "\"mode\":\""; json += modeName(currentMode); json += "\",";
  json += "\"emergencyStop\":"; json += emergencyStopActive ? "true" : "false";
  json += "}";
  sendApiHeaders();
  server.send(200, "application/json", json);
}

// Relay endpoint — manual override. In Modes 1-3 the relay state is
// re-asserted every loop() pass by runModeTasks(), so a manual override
// there is overwritten almost immediately. In Mode 4, ON/OFF here pause
// the 3s auto-toggle until 'auto' is sent again (or the mode is re-entered).
void handleRelay() {
  String requested = server.arg("value");
  if (requested.length() == 0) requested = server.arg("plain");
  requested.toLowerCase();
  requested.trim();

  if (requested == "on") {
    relayAutoMode = false;
    applyRelay(true);
    Serial.println("Relay -> ON (web, manual)");
  } else if (requested == "off") {
    relayAutoMode = false;
    applyRelay(false);
    Serial.println("Relay -> OFF (web, manual)");
  } else if (requested == "auto") {
    relayAutoMode = true;
    relayLastToggle = millis();
    Serial.println("Relay -> AUTO mode (web)");
  } else {
    sendApiHeaders();
    server.send(400, "application/json", "{\"ok\":false,\"error\":\"invalid relay value (use on/off/auto)\"}");
    return;
  }

  String json = "{";
  json += "\"ok\":true,";
  json += "\"relayOn\":"; json += relayState ? "true" : "false"; json += ",";
  json += "\"relayAutoMode\":"; json += relayAutoMode ? "true" : "false";
  json += "}";
  sendApiHeaders();
  server.send(200, "application/json", json);
}


// ════════════════════════════════════════════════════════════════
//  Temperature helper
// ════════════════════════════════════════════════════════════════
void updateTemperature() {
  if (tempConversionRequested && (millis() - tempRequestTime >= 200)) {
    float t = tempSensor.getTempCByIndex(0);
    if (t != DEVICE_DISCONNECTED_C) temperatureC = t;
    tempSensor.requestTemperatures();
    tempRequestTime = millis();
  }
}

// ════════════════════════════════════════════════════════════════
//  MAX3010x setup helper
// ════════════════════════════════════════════════════════════════
void setupMAX30105() {
  byte ledBrightness = 60;
  byte sampleAverage = 4;
  byte ledMode       = 2;
  byte sampleRate    = 100;
  int  pulseWidth    = 411;
  int  adcRange      = 4096;
  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange);
}

// ════════════════════════════════════════════════════════════════
//  Buzzer — non-blocking (never uses delay(), safe to call every loop)
// ════════════════════════════════════════════════════════════════
unsigned long buzzerOffAt   = 0;
bool          buzzerOn      = false;
unsigned long alarmLastToggle = 0;
bool          alarmState    = false;

// Fire a single short beep of `freqHz` for `durationMs`. Returns immediately.
void startBeep(unsigned int freqHz, unsigned int durationMs) {
  tone(BUZZER_PIN, freqHz);
  buzzerOn    = true;
  buzzerOffAt = millis() + durationMs;
}

// Call every loop iteration. Stops single beeps on time, and drives a
// continuous on/off alarm pattern for as long as emergencyStopActive is true.
void buzzerUpdate() {
  if (emergencyStopActive) {
    if (millis() - alarmLastToggle >= 150) {
      alarmLastToggle = millis();
      alarmState = !alarmState;
      if (alarmState) tone(BUZZER_PIN, 3000);
      else            noTone(BUZZER_PIN);
    }
    return;
  }

  if (buzzerOn && millis() >= buzzerOffAt) {
    noTone(BUZZER_PIN);
    buzzerOn = false;
  }
}

// ════════════════════════════════════════════════════════════════
//  Relay — non-blocking auto-toggle, MODE 4 (Controlled Ventilation)
//  ONLY. Modes 1-3 set the relay directly (see runModeTasks() below);
//  this function only fires the 3s ON / 3s OFF cycle while
//  currentMode == MODE_CONTROLLED_VENT and relayAutoMode is true (a
//  manual /relay on/off override pauses the cycle until 'auto' is sent
//  again, or the mode is re-entered via the button/web handler).
// ════════════════════════════════════════════════════════════════
void relayUpdate() {
  if (emergencyStopActive) return;   // relay stays forced ON (set by the STOP handler) during a stop
  if (currentMode == MODE_CONTROLLED_VENT && relayAutoMode &&
      (millis() - relayLastToggle >= RELAY_TOGGLE_INTERVAL)) {
    applyRelay(!relayState);
    relayLastToggle = millis();
  }
}

// Optional Serial control for the relay, ported from the original relay
// sketch's handleSerial(): '1' = ON, '0' = OFF, 'a'/'A' = AUTO.
// Any other character (e.g. newline) is ignored.
void relaySerialUpdate() {
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    if (cmd == '1') {
      relayAutoMode = false;
      applyRelay(true);
      Serial.println("Relay ON (via Serial)");
    } else if (cmd == '0') {
      relayAutoMode = false;
      applyRelay(false);
      Serial.println("Relay OFF (via Serial)");
    } else if (cmd == 'a' || cmd == 'A') {
      relayAutoMode = true;
      relayLastToggle = millis();
      Serial.println("Relay AUTO mode (via Serial)");
    }
    // any other char (e.g. \n or \r) is ignored
  }
}

// ════════════════════════════════════════════════════════════════
//  Emergency toggle switch — NEW continuous level-monitoring logic.
//  ─────────────────────────────────────────────────────────────────
//  Replaces the old momentary-button STOP handling. The switch is a
//  maintained ON/OFF toggle wired GPIO14 <-> GND, still read with
//  INPUT_PULLUP, so:
//    digitalRead(BTN_STOP) == LOW  -> switch is ON  -> emergency active
//    digitalRead(BTN_STOP) == HIGH -> switch is OFF -> emergency cleared
//  This is intentionally NOT run through buttonPressed()/debounce —
//  it is called every single loop() pass (including inside the sensor
//  sample-wait loops) and simply mirrors the switch's current physical
//  position into emergencyStopActive. State-change edges (ON entry /
//  OFF entry) are still detected internally (by comparing against
//  emergencyStopActive) purely so the existing beep/relay/log actions
//  fire ONCE per transition, not on every loop pass.
// ════════════════════════════════════════════════════════════════
void handleEmergencySwitch() {
  bool stopSwitchOn = (digitalRead(BTN_STOP) == LOW);   // active-LOW: switch ON = LOW

  if (stopSwitchOn && !emergencyStopActive) {
    // Switch has just been turned ON (or this is the first loop after boot
    // with the switch already ON) -> enter Emergency Stop immediately.
    emergencyStopActive = true;
    currentMode = MODE_STOPPED;
    startBeep(4000, 300);
    applyRelay(true);          // relay ON during Emergency Stop, same as before
    relayAutoMode = false;     // hold this state until the switch is turned OFF
    Serial.println("!!! EMERGENCY STOP (toggle switch ON) !!! Relay ON.");
  } else if (!stopSwitchOn && emergencyStopActive) {
    // Switch has just been turned OFF -> clear Emergency Stop and return to
    // Mode 1 (Monitoring), matching the original panel's recovery behaviour.
    emergencyStopActive = false;
    currentMode = MODE_MONITORING;
    relayAutoMode = false;
    applyRelay(true);          // relay ON in Monitoring, same as Nebulizer/Ventilation
    noTone(BUZZER_PIN);
    startBeep(1500, 100);
    Serial.println("Emergency switch OFF -> cleared, Mode 1 (Monitoring)");
  }
  // else: no change since last loop pass — do nothing (this is what makes
  // the check "continuous" rather than edge-triggered: it simply keeps
  // re-confirming the current state every pass without re-firing actions).
}

// ════════════════════════════════════════════════════════════════
//  4-Button Mode Switching (Mode 1-4 only)
//  Emergency STOP is now handled separately by handleEmergencySwitch()
//  above, using continuous digitalRead() polling instead of
//  buttonPressed()/debounce. Mode 1-4 debounce logic is UNCHANGED.
// ════════════════════════════════════════════════════════════════
void handleButtons() {
  // While the emergency toggle switch is ON, ignore Mode 1-4 button
  // presses entirely — the switch is the sole authority for clearing
  // Emergency Stop (handled in handleEmergencySwitch(), not here).
  if (emergencyStopActive) return;

  // Each mode button only updates currentMode + prints/beeps if that mode
  // isn't already the active one, so holding/re-pressing the already-active
  // button does not repeatedly re-trigger the mode-change action.
  if (buttonPressed(IDX_MODE1)) {
    if (currentMode != MODE_MONITORING) {
      currentMode = MODE_MONITORING;
      relayAutoMode = false;
      applyRelay(true);        // relay ON in Monitoring, same as Nebulizer/Ventilation
      startBeep(2000, 80);
      Serial.print("Mode -> 1 ("); Serial.print(modeName(currentMode)); Serial.println(")");
    } else {
      Serial.println("Mode 1 already active — ignoring re-press");
    }
  } else if (buttonPressed(IDX_MODE2)) {
    if (currentMode != MODE_NEBULIZER) {
      currentMode = MODE_NEBULIZER;
      startBeep(2000, 80);
      Serial.print("Mode -> 2 ("); Serial.print(modeName(currentMode)); Serial.println(")");
    } else {
      Serial.println("Mode 2 already active — ignoring re-press");
    }
  } else if (buttonPressed(IDX_MODE3)) {
    if (currentMode != MODE_VENTILATION) {
      currentMode = MODE_VENTILATION;
      startBeep(2000, 80);
      Serial.print("Mode -> 3 ("); Serial.print(modeName(currentMode)); Serial.println(")");
    } else {
      Serial.println("Mode 3 already active — ignoring re-press");
    }
  } else if (buttonPressed(IDX_MODE4)) {
    if (currentMode != MODE_CONTROLLED_VENT) {
      currentMode = MODE_CONTROLLED_VENT;
      relayAutoMode   = true;      // (re)start the 3s ON / 3s OFF cycle
      relayLastToggle = millis();
      applyRelay(true);            // begin the cycle in the ON phase
      startBeep(2000, 80);
      Serial.print("Mode -> 4 ("); Serial.print(modeName(currentMode)); Serial.println(")");
    } else {
      Serial.println("Mode 4 already active — ignoring re-press");
    }
  }
}

// ════════════════════════════════════════════════════════════════
//  Per-mode task — drives the relay to match the active mode every
//  loop() pass:
//    Mode 1 (Monitoring)      → relay ON
//    Mode 2 (Nebulizer)       → relay OFF
//    Mode 3 (Ventilation)     → relay OFF
//    Mode 4 (Controlled Vent) → left alone here; relayUpdate() drives
//                                the 3s ON / 3s OFF auto-toggle instead
//    Emergency Stop           → relay ON (forced by handleEmergencySwitch(),
//                                not by this function)
// ════════════════════════════════════════════════════════════════
void runModeTasks() {
  if (emergencyStopActive) return;   // no mode tasks while stopped

  switch (currentMode) {
    case MODE_MONITORING:
      if (!relayState) applyRelay(true);   // relay ON continuously in Monitoring
      break;

    case MODE_NEBULIZER:
    case MODE_VENTILATION:
      if (relayState) applyRelay(false);   // relay OFF continuously
      break;

    case MODE_CONTROLLED_VENT:
      // Relay handled by relayUpdate()'s 3s ON / 3s OFF auto-toggle.
      break;

    default:
      break;
  }
}

// ════════════════════════════════════════════════════════════════
//  LCDs: LCD1 = active Mode + run status | LCD2 = all sensor values
// ════════════════════════════════════════════════════════════════
void updateLCDs() {
  static unsigned long lastLCDUpdate = 0;
  if (millis() - lastLCDUpdate < 400) return;   // refresh ~2.5x/sec, avoid I2C flooding
  lastLCDUpdate = millis();

  if (emergencyStopActive) {
    // LCD1: mode/status display
    lcd1.setCursor(0, 0); lcd1.print("** STOPPED **   ");
    lcd1.setCursor(0, 1); lcd1.print("Turn switch OFF ");
    // LCD2: sensor values still shown even while stopped, so the operator
    // can keep an eye on vitals during the fault condition.
    updateLCD2Vitals();
    return;
  }

  // LCD1: active mode name (line0) + run status (line1)
  char modeLine[17], statusLine[17];
  snprintf(modeLine,   sizeof(modeLine),   "Mode: %-10s", modeName(currentMode));
  snprintf(statusLine, sizeof(statusLine), "Status: Running ");
  lcd1.setCursor(0, 0); lcd1.print(modeLine);
  lcd1.setCursor(0, 1); lcd1.print(statusLine);

  // LCD2: all sensor values together
  updateLCD2Vitals();
}

// LCD2: SpO2 + Heart Rate on line0, Flow + Temperature on line1.
// Pulled into its own function so it can be called both from the normal
// path and from the emergency-stop branch above without duplicating code.
void updateLCD2Vitals() {
  char line0[17], line1[17];

  int32_t dispHR = validHeartRate ? constrain(heartRate - 35, 0, 105) : 0;
  if (validSPO2 && validHeartRate) {
    snprintf(line0, sizeof(line0), "S:%3d%% H:%3dbpm", (int)spo2, (int)dispHR);
  } else if (validSPO2) {
    snprintf(line0, sizeof(line0), "S:%3d%% H:--     ", (int)spo2);
  } else if (validHeartRate) {
    snprintf(line0, sizeof(line0), "S:--   H:%3dbpm ", (int)dispHR);
  } else {
    snprintf(line0, sizeof(line0), "S:--   H:--      ");
  }

  float displayFlow = min(rawFlowRate / 5.0f, 15.0f);
  snprintf(line1, sizeof(line1), "F:%4.1f T:%4.1fC", displayFlow, temperatureC);

  lcd2.setCursor(0, 0); lcd2.print(line0);
  lcd2.setCursor(0, 1); lcd2.print(line1);
}

// ════════════════════════════════════════════════════════════════
//  SETUP
// ════════════════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  delay(1000);

  WiFi.softAP(ssid, password);
  IPAddress ip = WiFi.softAPIP();
  Serial.println("==== TiTo Combined Monitor ====");
  Serial.print("SSID: ");     Serial.println(ssid);
  Serial.print("URL:  http://"); Serial.println(ip);

  server.on("/",       handleRoot);
  server.on("/data",   handleData);
  server.on("/status", handleStatus);
  server.on("/mode",   HTTP_GET,  handleMode);
  server.on("/mode",   HTTP_POST, handleMode);
  server.on("/relay",  HTTP_GET,  handleRelay);
  server.on("/relay",  HTTP_POST, handleRelay);
  server.begin();
  Serial.println("Web server started");

  Wire.begin(I2C_SDA, I2C_SCL);

  tempSensor.begin();
  tempSensor.setResolution(10);
  tempSensor.setWaitForConversion(false);
  tempSensor.requestTemperatures();
  tempRequestTime          = millis();
  tempConversionRequested  = true;
  Serial.println("DS18B20 initialised");

  Serial.println("Initialising MAX3010x...");
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("MAX3010x not found — retrying every 3 s");
    sensorFound = false;
  } else {
    sensorFound = true;
    Serial.println("MAX3010x found!");
    setupMAX30105();
  }

  pinMode(FLOW_SENSOR_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(FLOW_SENSOR_PIN), pulseCounter, FALLING);
  lastCalcTime = millis();
  Serial.println("YF-S401 flow sensor initialised");

  // 4 mode buttons (momentary, debounced) + emergency toggle switch
  // (maintained ON/OFF, read continuously — no debounce). Pin config for
  // BTN_STOP is UNCHANGED (still INPUT_PULLUP), only how it's read differs.
  pinMode(BTN_MODE1, INPUT_PULLUP);
  pinMode(BTN_MODE2, INPUT_PULLUP);
  pinMode(BTN_MODE3, INPUT_PULLUP);
  pinMode(BTN_MODE4, INPUT_PULLUP);
  pinMode(BTN_STOP,  INPUT_PULLUP);
  Serial.println("Mode buttons (1-4) initialised; Emergency input configured as toggle switch");
  Serial.print("Boot mode: "); Serial.println(modeName(currentMode));

  // If the emergency toggle switch happens to already be ON at boot,
  // this call will detect it on the very first pass and immediately
  // enter Emergency Stop before anything else runs in loop().
  handleEmergencySwitch();

  pinMode(BUZZER_PIN, OUTPUT);
  noTone(BUZZER_PIN);
  Serial.println("Buzzer initialised");

  // Relay module (active LOW). Boots ON — Mode 1 (Monitoring) is the
  // default boot mode and keeps the relay ON via runModeTasks(), same
  // as Nebulizer/Ventilation.
  pinMode(RELAY_PIN, OUTPUT);
  applyRelay(true);
  relayAutoMode   = false;
  relayLastToggle = millis();
  Serial.println("Relay module initialised (active LOW, mode-driven)");

// LCDs (I2C — share SDA/SCL bus with MAX30102)
  lcd1.init();
  lcd1.backlight();
  lcd1.setCursor(0, 0); lcd1.print("SRSS booting...");

  lcd2.init();
  lcd2.backlight();
  lcd2.setCursor(0, 0); lcd2.print("SRSS booting...");
  Serial.println("LCDs initialised");
}

// ════════════════════════════════════════════════════════════════
//  LOOP
// ════════════════════════════════════════════════════════════════
void loop() {
  server.handleClient();
  updateTemperature();
  handleEmergencySwitch();   // NEW: continuous level-read of the toggle switch, every pass
  handleButtons();           // Mode 1-4 only now (STOP removed from here)
  runModeTasks();
  buzzerUpdate();
  relayUpdate();
  relaySerialUpdate();
  updateLCDs();

  // Flow rate calculation (every 1 s)
  if (millis() - lastCalcTime >= 1000) {
    detachInterrupt(digitalPinToInterrupt(FLOW_SENSOR_PIN));
    float lps    = pulseCount / calibrationFactor;
    rawFlowRate  = lps * 60.0f;
    totalLiters += lps;
    Serial.print("Raw Flow: "); Serial.print(rawFlowRate, 2);
    Serial.print(" L/min | Total: "); Serial.print(totalLiters, 3); Serial.println(" L");
    pulseCount   = 0;
    lastCalcTime = millis();
    attachInterrupt(digitalPinToInterrupt(FLOW_SENSOR_PIN), pulseCounter, FALLING);
  }

  // MAX3010x absent: show zeros and retry every 3 s
  if (!sensorFound) {
    heartRate = 0; validHeartRate = 0;
    spo2      = 0; validSPO2      = 0;
    static unsigned long lastRetry = 0;
    if (millis() - lastRetry > 3000) {
      lastRetry = millis();
      Serial.println("Retrying MAX3010x…");
      if (particleSensor.begin(Wire, I2C_SPEED_FAST)) {
        sensorFound = true;
        Serial.println("MAX3010x detected!");
        setupMAX30105();
      }
    }
    delay(10);
    return;
  }

  // Fill initial 100-sample buffer
  for (byte i = 0; i < BUFFER_LENGTH; i++) {
    while (particleSensor.available() == false) {
      particleSensor.check();
      server.handleClient();
      updateTemperature();
      handleEmergencySwitch();   // NEW: keep polling the toggle switch while waiting on samples
      handleButtons();
      runModeTasks();
      buzzerUpdate();
      relayUpdate();
      relaySerialUpdate();
      updateLCDs();
    }
    redBuffer[i] = particleSensor.getRed();
    irBuffer[i]  = particleSensor.getIR();
    particleSensor.nextSample();
    server.handleClient();
  }

  // Calculate HR & SpO2
  maxim_heart_rate_and_oxygen_saturation(
    irBuffer, BUFFER_LENGTH, redBuffer,
    &spo2, &validSPO2, &heartRate, &validHeartRate
  );

  // Serial print every 500 ms
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 500) {
    lastPrint = millis();
    int32_t dispHR = validHeartRate ? constrain(heartRate - 35, 0, 105) : 0;
    float   dispFL = min(rawFlowRate / 5.0f, 15.0f);
    float   tempF  = temperatureC * 9.0f / 5.0f + 32.0f;
    Serial.print("HR(disp): "); Serial.print(dispHR);
    Serial.print("  SpO2: ");   Serial.print(spo2);
    Serial.print("  Temp: ");   Serial.print(temperatureC, 1);
    Serial.print(" C / ");      Serial.print(tempF, 1);
    Serial.print(" F  Flow(disp): "); Serial.print(dispFL, 2);
    Serial.println(" L/min");
  }

  // Shift buffer and refill SAMPLES_PER_UPDATE new samples
  for (byte i = SAMPLES_PER_UPDATE; i < BUFFER_LENGTH; i++) {
    redBuffer[i - SAMPLES_PER_UPDATE] = redBuffer[i];
    irBuffer [i - SAMPLES_PER_UPDATE] = irBuffer [i];
  }

  for (byte i = BUFFER_LENGTH - SAMPLES_PER_UPDATE; i < BUFFER_LENGTH; i++) {
    while (particleSensor.available() == false) {
      particleSensor.check();
      server.handleClient();
      updateTemperature();
      handleEmergencySwitch();   // NEW: keep polling the toggle switch while waiting on samples
      handleButtons();
      runModeTasks();
      buzzerUpdate();
      relayUpdate();
      relaySerialUpdate();
      updateLCDs();
      if (millis() - lastCalcTime >= 1000) {
        detachInterrupt(digitalPinToInterrupt(FLOW_SENSOR_PIN));
        float lps    = pulseCount / calibrationFactor;
        rawFlowRate  = lps * 60.0f;
        totalLiters += lps;
        pulseCount   = 0;
        lastCalcTime = millis();
        attachInterrupt(digitalPinToInterrupt(FLOW_SENSOR_PIN), pulseCounter, FALLING);
      }
    }
    redBuffer[i] = particleSensor.getRed();
    irBuffer [i] = particleSensor.getIR();
    particleSensor.nextSample();
    server.handleClient();
  }
}