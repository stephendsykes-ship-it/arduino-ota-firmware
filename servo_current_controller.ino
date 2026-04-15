// ============================================================
//  servo_current_controller.ino
//  Target board : Arduino Uno R4 WiFi
//
//  Hardware:
//    - ACS723 current sensor  → Analog pin A0
//    - EK1690 PWM Servo Shield (PCA9685) → I2C (SDA / SCL)
//    - Continuous servo motor → Shield channel 0
//    - DHT22 sensor 1         → Digital pin 2  (future, stubbed)
//    - DHT22 sensor 2         → Digital pin 3  (future, stubbed)
//
//  Behaviour:
//    Current rises  >= CURRENT_THRESHOLD_A  → servo rotates 360° forward
//    Current falls  <  CURRENT_THRESHOLD_A  → servo rotates 360° reverse
//    Web page (port 80) shows live status; auto-refreshes every 2 s
//
//  Libraries needed (install via Library Manager):
//    * Adafruit PWM Servo Driver Library  (by Adafruit)
//    * Adafruit BusIO                     (dependency of above)
//    * WiFiS3         (installed with "Arduino UNO R4 Boards" board package)
//    * DHT sensor library by Adafruit  (install when adding sensors)
//    * Adafruit Unified Sensor         (dependency of DHT lib)
// ============================================================

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <WiFiS3.h>
#include <OTAUpdate.h>
#include "secrets.h"

// ── Future DHT sensor support (uncomment all DHT lines when ready) ──────────
// #include <DHT.h>
// #define DHT1_PIN      2
// #define DHT2_PIN      3
// #define DHT_TYPE      DHT22
// DHT dht1(DHT1_PIN, DHT_TYPE);
// DHT dht2(DHT2_PIN, DHT_TYPE);
// ────────────────────────────────────────────────────────────────────────────

// ============================================================
//  USER CONFIGURATION  (adjust these values for your hardware)
// ============================================================

// -- ACS723 current sensor (main – circuit under test) ----------------------
const int   CURRENT_SENSOR_PIN  = A0;

// ACS723 sensitivity – pick the line that matches your part number:
//   ACS723LLCTR-05AB  → 0.800 V/A
//   ACS723LLCTR-10AB  → 0.400 V/A   (most common)
//   ACS723LLCTR-20AB  → 0.200 V/A
const float SENSITIVITY_VA      = 0.400;   // volts per amp

// Supply voltage fed to the ACS723 VCC pin (measure with a multimeter)
// NOTE: Arduino Uno R4 ADC reference is 3.3 V, not 5 V
const float SENSOR_VCC          = 3.3;     // volts

// ACS723 quiescent output at 0 A = VCC / 2 for bidirectional variants
const float ZERO_CURRENT_V      = SENSOR_VCC / 2.0;

// -- ACS723 stall sensor (servo motor supply rail) ---------------------------
const int   STALL_SENSOR_PIN    = A1;
const float STALL_SENSITIVITY_VA = 0.800;  // ACS723LLCTR-05AB → 0.800 V/A
// Uses same VCC and zero-current reference as the main sensor

// Hysteresis prevents chattering near the threshold.
// Example: threshold = 1.0 A, hysteresis = 0.05 A
//   Rising  edge fires at >= 1.05 A
//   Falling edge fires at <  0.95 A
const float CURRENT_THRESHOLD_A = 1.0;     // amps  ← change to your value
const float HYSTERESIS_A        = 0.05;    // amps

// Number of ADC samples averaged per reading (reduces noise)
const int   ADC_SAMPLES         = 20;

// -- Continuous servo via EK1690 / PCA9685 PWM shield -----------------------
// I2C address: 0x40 (default – change if address jumpers are soldered)
const uint8_t  PWM_SHIELD_ADDR  = 0x40;

// Shield channel the servo signal wire is connected to (0–15)
const uint8_t  SERVO_CHANNEL    = 0;

// PCA9685 PWM frequency for standard servos
const float    PWM_FREQ_HZ      = 50.0f;   // 50 Hz → 20 ms period

// Pulse widths in microseconds for a continuous-rotation servo:
//   1500 µs = stop,  ~1000 µs = full forward,  ~2000 µs = full reverse
const uint16_t SERVO_STOP_US    = 1500;
const uint16_t SERVO_FORWARD_US = 1000;
const uint16_t SERVO_REVERSE_US = 2000;

// Fine-tune the stop pulse if the servo creeps at idle.
// Positive = shift stop pulse higher (µs), negative = lower.
// Adjust via the GUI TRIM command without reflashing.
int16_t SERVO_STOP_TRIM_US = 0;

// *** CALIBRATION REQUIRED ***
// Measure how long (ms) your specific servo takes to spin exactly one full
// revolution, then set ROTATION_MS to that value.
// Quick guide: run the servo at full speed and time 10 revolutions, divide by 10.
// A typical 60 RPM servo: 60 000 ms / 60 RPM = 1 000 ms per revolution.
// Calibrated: was 1000 ms → 540°; scaled to 667 ms → 360°.
const unsigned long ROTATION_MS = 667;   // milliseconds for ONE 360° turn

// -- Stall detection (testing) -----------------------------------------------
// When enabled, the servo stops immediately if measured current exceeds
// STALL_CURRENT_A after the STALL_IGNORE_MS spin-up grace period.
// Requires the ACS723 to be sensing the servo motor supply current.
// Toggle at runtime:   STALL_ON / STALL_OFF  (serial or web /cmd)
// Adjust threshold:    STALL_THRESH <amps>
// Clear flag:          STALL_CLEAR
const unsigned long STALL_IGNORE_MS = 150;  // ms – ignore stall during motor spin-up surge

// -- Web server --------------------------------------------------------------
const int WEB_SERVER_PORT = 80;

// -- OTA firmware update ------------------------------------------------------
// Increment FW_VERSION, then run push_firmware.ps1 to publish a new release.
// The Arduino polls GitHub every OTA_CHECK_INTERVAL_MS milliseconds.
#define FW_VERSION            7
#define FW_VERSION_STR        "7"
#define OTA_CHECK_INTERVAL_MS (5UL * 60UL * 1000UL)  // 5 minutes

// ============================================================
//  STATE MACHINE
// ============================================================
enum ServoState {
  SERVO_IDLE,
  SERVO_ROTATING_FORWARD,
  SERVO_ROTATING_REVERSE
};

// ============================================================
//  GLOBALS
// ============================================================
Adafruit_PWMServoDriver pwm(PWM_SHIELD_ADDR);
WiFiServer       webServer(WEB_SERVER_PORT);

ServoState       servoState        = SERVO_IDLE;
bool             triggerActive     = false;  // true while current >= threshold
unsigned long    servoMoveStart    = 0;

float            measuredCurrent   = 0.0;

// Placeholders filled in when DHT sensors are wired
float            temp1_C           = NAN;
float            humidity1_pct     = NAN;
float            temp2_C           = NAN;
float            humidity2_pct     = NAN;

// ── Debug / Simulation ─────────────────────────────────────────────────────
bool  simMode    = false;   // true = bypass ADC, use simCurrent
float simCurrent = 0.0f;   // injected current value (amps)

// ── Stall detection ──────────────────────────────────────────────────────────
bool  stallDetectEnabled = false;
bool  stallDetected      = false;
float STALL_CURRENT_A    = 0.5f;   // amps – tune while watching stall_current in GUI
float stallCurrent       = 0.0f;   // latest reading from the stall sensor (A1)

// ── Door State (updated when timed servo rotation completes) ────────────────
enum DoorState { DOOR_UNKNOWN, DOOR_OPEN, DOOR_CLOSED };
DoorState doorState = DOOR_UNKNOWN;

// ── OTA ─────────────────────────────────────────────────────────────────────
OTAUpdate ota;
bool otaCheckRequested = false;

// ============================================================
//  CURRENT READING
// ============================================================
float readCurrentAmps() {
  if (simMode) return simCurrent;   // debug simulation override
  // Use 14-bit ADC resolution for better accuracy on the R4
  analogReadResolution(14);
  long accumulator = 0;
  for (int i = 0; i < ADC_SAMPLES; i++) {
    accumulator += analogRead(CURRENT_SENSOR_PIN);
    delayMicroseconds(200);
  }
  float avgAdc  = (float)accumulator / ADC_SAMPLES;
  float voltage = (avgAdc / 16383.0f) * SENSOR_VCC;
  float current = (voltage - ZERO_CURRENT_V) / SENSITIVITY_VA;
  return current;
}

// Dedicated stall sensor read – always reads the real ADC (not simulated)
float readStallCurrentAmps() {
  analogReadResolution(14);
  long accumulator = 0;
  for (int i = 0; i < ADC_SAMPLES; i++) {
    accumulator += analogRead(STALL_SENSOR_PIN);
    delayMicroseconds(200);
  }
  float avgAdc  = (float)accumulator / ADC_SAMPLES;
  float voltage = (avgAdc / 16383.0f) * SENSOR_VCC;
  return (voltage - ZERO_CURRENT_V) / STALL_SENSITIVITY_VA;
}

// ============================================================
//  SERVO CONTROL
// ============================================================
void startServo(uint16_t pulseUs) {
  pwm.writeMicroseconds(SERVO_CHANNEL, pulseUs);
  servoMoveStart = millis();
  stallDetected  = false;           // clear stall flag on each new rotation
  digitalWrite(LED_BUILTIN, HIGH);  // LED on while motor runs
}

void stopServo() {
  int16_t stopUs = (int16_t)SERVO_STOP_US + SERVO_STOP_TRIM_US;
  pwm.writeMicroseconds(SERVO_CHANNEL, (uint16_t)stopUs);
}

// Call every loop iteration – handles timed stop without blocking
void updateServo() {
  if (servoState == SERVO_IDLE) return;

  // Blink LED at 4 Hz while motor is running
  if ((millis() / 125) % 2 == 0) digitalWrite(LED_BUILTIN, HIGH);
  else                            digitalWrite(LED_BUILTIN, LOW);

  // Stall detection – reads dedicated ACS723 on A1, fires after grace period
  if (stallDetectEnabled && (millis() - servoMoveStart >= STALL_IGNORE_MS)) {
    if (stallCurrent >= STALL_CURRENT_A) {
      stopServo();
      servoState    = SERVO_IDLE;
      stallDetected = true;
      digitalWrite(LED_BUILTIN, LOW);
      Serial.print(F("[Stall] Detected at "));
      Serial.print(stallCurrent, 3);
      Serial.println(F(" A (A1) – servo stopped."));
      return;
    }
  }

  if (millis() - servoMoveStart >= ROTATION_MS) {
    if (servoState == SERVO_ROTATING_FORWARD) doorState = DOOR_OPEN;
    if (servoState == SERVO_ROTATING_REVERSE) doorState = DOOR_CLOSED;
    stopServo();
    servoState = SERVO_IDLE;
    digitalWrite(LED_BUILTIN, LOW);   // LED off when motor stops
    Serial.println("[Servo] Rotation complete – stopped.");
  }
}

// ============================================================
//  THRESHOLD LOGIC
// ============================================================
void updateThresholdLogic() {
  // Only re-evaluate when the servo is not mid-rotation
  if (servoState != SERVO_IDLE) return;

  float risingEdge  = CURRENT_THRESHOLD_A + HYSTERESIS_A;
  float fallingEdge = CURRENT_THRESHOLD_A - HYSTERESIS_A;

  if (!triggerActive && measuredCurrent >= risingEdge) {
    triggerActive = true;
    servoState    = SERVO_ROTATING_FORWARD;
    startServo(SERVO_FORWARD_US);
    Serial.print("[Trigger] Current ABOVE threshold (");
    Serial.print(measuredCurrent, 3);
    Serial.println(" A) – rotating FORWARD 360°");
  }
  else if (triggerActive && measuredCurrent < fallingEdge) {
    triggerActive = false;
    servoState    = SERVO_ROTATING_REVERSE;
    startServo(SERVO_REVERSE_US);
    Serial.print("[Trigger] Current BELOW threshold (");
    Serial.print(measuredCurrent, 3);
    Serial.println(" A) – rotating REVERSE 360°");
  }
}

// ============================================================
//  WEB PAGE BUILDER
// ============================================================
String servoStateLabel() {
  switch (servoState) {
    case SERVO_ROTATING_FORWARD: return "Rotating Forward";
    case SERVO_ROTATING_REVERSE: return "Rotating Reverse";
    default:                      return "Idle";
  }
}

const char* doorStateLabel() {
  switch (doorState) {
    case DOOR_OPEN:   return "Open";
    case DOOR_CLOSED: return "Closed";
    default:          return "Unknown";
  }
}

// ============================================================
//  STATUS JSON  (shared by Serial + HTTP)
// ============================================================
void writeStatus(Print& out) {
  out.print(F("{\"current\":"));
  out.print(measuredCurrent, 3);
  out.print(F(",\"servo\":\""));
  out.print(servoStateLabel());
  out.print(F("\",\"trigger\":"));
  out.print(triggerActive ? F("true") : F("false"));
  out.print(F(",\"trim\":"));
  out.print(SERVO_STOP_TRIM_US);
  out.print(F(",\"sim\":"));
  out.print(simMode ? F("true") : F("false"));
  out.print(F(",\"threshold\":"));
  out.print(CURRENT_THRESHOLD_A, 3);
  out.print(F(",\"hysteresis\":"));
  out.print(HYSTERESIS_A, 3);
  out.print(F(",\"door\":\""));
  out.print(doorStateLabel());
  out.print(F("\",\"ip\":\""));
  if (WiFi.status() == WL_CONNECTED) out.print(WiFi.localIP());
  else                                out.print(F("none"));
  out.print(F("\",\"stall\":"));
  out.print(stallDetected ? F("true") : F("false"));
  out.print(F(",\"stall_detect\":"));
  out.print(stallDetectEnabled ? F("true") : F("false"));
  out.print(F(",\"stall_thresh\":"));
  out.print(STALL_CURRENT_A, 3);
  out.print(F(",\"stall_current\":"));
  out.print(stallCurrent, 3);
  out.print(F(",\"fw\":"));
  out.print(FW_VERSION);
  out.println(F("}")); 
}

// ============================================================
//  HTTP HELPERS
// ============================================================
String getQueryParam(const String& query, const String& key) {
  String search = key + "=";
  int idx = query.indexOf(search);
  if (idx < 0) return "";
  int start = idx + search.length();
  int end   = query.indexOf('&', start);
  if (end < 0) end = query.length();
  // decode '+' as space (not needed here but safe)
  String val = query.substring(start, end);
  val.replace("+", " ");
  return val;
}

void sendHTTPOK(WiFiClient& client, const __FlashStringHelper* ct) {
  client.println(F("HTTP/1.1 200 OK"));
  client.print(F("Content-Type: "));
  client.println(ct);
  client.println(F("Connection: close"));
  client.println(F("Cache-Control: no-cache"));
  client.println();
}

void handleStatusRequest(WiFiClient& client) {
  sendHTTPOK(client, F("application/json"));
  writeStatus(client);
}

void handleCommandRequest(WiFiClient& client, const String& query) {
  String c = getQueryParam(query, "c");
  String v = getQueryParam(query, "v");

  if (c == "SIM") {
    simCurrent = v.toFloat();
    simMode    = true;
  } else if (c == "SIM_OFF") {
    simMode = false;
  } else if (c == "FWD") {
    servoState = SERVO_ROTATING_FORWARD;
    startServo(SERVO_FORWARD_US);
  } else if (c == "REV") {
    servoState = SERVO_ROTATING_REVERSE;
    startServo(SERVO_REVERSE_US);
  } else if (c == "STOP") {
    stopServo();
    servoState = SERVO_IDLE;
  } else if (c == "TRIM") {
    SERVO_STOP_TRIM_US = (int16_t)v.toInt();
    stopServo();
  } else if (c == "STALL_ON") {
    stallDetectEnabled = true;
    stallDetected      = false;
  } else if (c == "STALL_OFF") {
    stallDetectEnabled = false;
  } else if (c == "STALL_THRESH") {
    STALL_CURRENT_A = v.toFloat();
  } else if (c == "STALL_CLEAR") {
    stallDetected = false;
  } else if (c == "OTA") {
    otaCheckRequested = true;
    // Respond immediately; check runs on next loop iteration
    sendHTTPOK(client, F("application/json"));
    client.println(F("{\"msg\":\"OTA check scheduled\"}"));
    return;
  }

  // Respond with current status so the page can update immediately
  sendHTTPOK(client, F("application/json"));
  writeStatus(client);
}

void streamHTMLPage(WiFiClient& client) {
  sendHTTPOK(client, F("text/html; charset=utf-8"));
  // Stream page in chunks – all strings in flash via F()
  client.print(F("<!DOCTYPE html><html lang='en'><head>"
    "<meta charset='UTF-8'>"
    "<meta name='viewport' content='width=device-width,initial-scale=1'>"
    "<title>Servo Controller</title>"
    "<style>"
    "*{box-sizing:border-box}body{margin:0;padding:12px;font-family:Arial,sans-serif;"
    "background:#0d1117;color:#c9d1d9}h1{color:#58a6ff;text-align:center;margin:0 0 12px}"
    ".card{background:#161b22;border:1px solid #30363d;border-radius:8px;padding:12px;margin:8px 0}"
    ".card h2{margin:0 0 6px;font-size:.8em;color:#8b949e;text-transform:uppercase;letter-spacing:.05em}"
    ".val{font-size:1.8em;font-weight:bold}.sub{font-size:.82em;color:#8b949e;margin-top:2px}"
    ".ok{color:#3fb950}.warn{color:#d29922}.err{color:#f85149}.blue{color:#58a6ff}"
    "button{background:#21262d;color:#c9d1d9;border:1px solid #30363d;border-radius:6px;"
    "padding:7px 14px;cursor:pointer;font-size:.88em;margin:3px}"
    "button:active{opacity:.7}"
    ".bg{background:#1f4a2e;color:#3fb950;border-color:#3fb950}"
    ".br{background:#3d1515;color:#f85149;border-color:#f85149}"
    ".bb{background:#1a3a5c;color:#58a6ff;border-color:#58a6ff}"
    ".by{background:#2d2000;color:#d29922;border-color:#d29922}"
    "input[type=range]{width:100%;accent-color:#58a6ff;margin:4px 0}"
    "input[type=number]{background:#21262d;color:#c9d1d9;border:1px solid #30363d;"
    "border-radius:4px;padding:4px 6px;width:72px}"
    ".row{display:flex;gap:6px;align-items:center;flex-wrap:wrap;margin-top:6px}"
    "footer{text-align:center;color:#484f58;font-size:.72em;margin-top:14px}"
    "</style></head><body>"
    "<h1>Servo Controller</h1>"));

  client.print(F(
    "<div class='card'><h2>Current</h2>"
    "<div class='val' id='cur'>-</div>"
    "<div class='sub' id='thr'></div></div>"
    "<div class='card'><h2>Servo State</h2>"
    "<div class='val' id='srv'>-</div>"
    "<div class='sub' id='trg'></div></div>"
    "<div class='card'><h2>Mode</h2>"
    "<div id='mod'>-</div>"
    "<div class='sub' id='trm'></div></div>"
    "<div class='card'><h2>Manual Servo</h2><div class='row'>"
    "<button class='bb' onclick=\"c('REV')\">&#9664; Reverse</button>"
    "<button class='br' onclick=\"c('STOP')\">&#9632; Stop</button>"
    "<button class='bg' onclick=\"c('FWD')\">Forward &#9654;</button>"
    "</div></div>"));

  client.print(F(
    "<div class='card'><h2>Simulate Current</h2>"
    "<input type='range' id='ss' min='0' max='5' step='0.01' value='0' oninput=\"sm(this.value)\">"
    "<div class='row'>"
    "<input type='number' id='sv' min='0' max='5' step='0.01' value='0.00' onchange=\"sv2(this.value)\">"
    "<span style='color:#8b949e'>A</span>"
    "<button class='bb' onclick='se()'>&#9654; Enable Sim</button>"
    "<button class='br' onclick=\"c('SIM_OFF')\">&#10005; Disable</button>"
    "</div>"
    "<div style='font-size:.82em;color:#8b949e;margin-top:8px'>Quick presets:</div>"
    "<div class='row'>"
    "<button onclick='p(0)'>0.00</button>"
    "<button onclick='p(0.5)'>0.50</button>"
    "<button onclick='p(1.0)'>1.00</button>"
    "<button onclick='p(1.1)'>1.10</button>"
    "<button onclick='p(2.0)'>2.00</button>"
    "</div></div>"));

  client.print(F(
    "<div class='card'><h2>Stop Pulse Trim</h2>"
    "<input type='range' id='ts' min='-100' max='100' step='1' value='0' oninput=\"tm(this.value)\">"
    "<div class='row'>"
    "<input type='number' id='tv' min='-100' max='100' value='0'>"
    "<span style='color:#8b949e'>&micro;s</span>"
    "<button class='by' onclick='at()'>Apply</button>"
    "</div></div>"
    "<div class='card'><h2>Firmware Update (OTA)</h2>"
    "<div class='row'>"
    "<button class='bb' onclick=\"ota()\">&#8635; Check for Update</button>"
    "<span id='otamsg' style='font-size:.82em;color:#8b949e'></span>"
    "</div>"
    "<div style='font-size:.78em;color:#484f58;margin-top:4px'>v" FW_VERSION_STR " &bull; Polls GitHub every 5 min</div>"
    "</div>"
    "<footer>Arduino Uno R4 WiFi &bull; Standalone</footer>"));

  client.print(F("<script>"
    "var sc=0;"
    "function c(cmd,val){"
      "var u='/cmd?c='+cmd;"
      "if(val!==undefined)u+='&v='+val;"
      "fetch(u).then(r=>r.json()).then(upd).catch(()=>{});}"
    "function sm(v){sc=parseFloat(v);document.getElementById('sv').value=parseFloat(v).toFixed(2);}"
    "function sv2(v){sc=Math.min(5,Math.max(0,parseFloat(v)||0));"
      "document.getElementById('ss').value=sc;}"
    "function se(){c('SIM',sc.toFixed(2));}"
    "function p(v){sc=v;document.getElementById('ss').value=v;"
      "document.getElementById('sv').value=v.toFixed(2);c('SIM',v.toFixed(2));}"
    "function tm(v){document.getElementById('tv').value=v;}"
    "function at(){c('TRIM',document.getElementById('tv').value);}"
    "function upd(d){"
      "var a=d.current,e=document.getElementById('cur');"
      "e.textContent=(a>=0?'+':'')+a.toFixed(3)+' A';"
      "e.className='val '+(Math.abs(a)>=d.threshold?'err':'ok');"
      "document.getElementById('thr').textContent="
        "'Threshold: '+d.threshold.toFixed(2)+' A  \u00b1'+d.hysteresis.toFixed(3)+' A';"
      "var s=document.getElementById('srv');"
      "s.textContent=d.servo;"
      "s.className='val '+(d.servo==='Idle'?'ok':'blue');"
      "var t=document.getElementById('trg');"
      "t.textContent='Trigger: '+(d.trigger?'ABOVE threshold':'BELOW threshold');"
      "t.className=d.trigger?'err':'ok';"
      "var m=document.getElementById('mod');"
      "m.textContent=d.sim?'\u25cf SIMULATED':'\u25cf LIVE (real ADC)';"
      "m.className=d.sim?'warn':'ok';"
      "document.getElementById('trm').textContent="
        "'Stop trim: '+(d.trim>=0?'+':'')+d.trim+'\u00b5s';"
      "document.getElementById('ts').value=d.trim;"
      "document.getElementById('tv').value=d.trim;}"
    "function poll(){fetch('/status').then(r=>r.json()).then(upd)"
      ".catch(()=>{}).finally(()=>setTimeout(poll,500));}"
    "function ota(){"
      "document.getElementById('otamsg').textContent='Checking...';"
      "fetch('/cmd?c=OTA').then(()=>{"
        "document.getElementById('otamsg').textContent='Requested. Watch serial for progress.';"
      "}).catch(()=>{"
        "document.getElementById('otamsg').textContent='Request failed.';"
      "});}"
    "poll();"
    "</script></body></html>"));
}

// ============================================================
//  WEB SERVER HANDLER
// ============================================================
void handleWebClients() {
  WiFiClient client = webServer.available();
  if (!client) return;

  // Read request line
  String requestLine = "";
  unsigned long deadline = millis() + 1000;
  while (client.connected() && millis() < deadline) {
    if (client.available()) {
      requestLine = client.readStringUntil('\n');
      requestLine.trim();
      break;
    }
  }

  // Drain remaining headers
  deadline = millis() + 1000;
  while (client.connected() && millis() < deadline) {
    if (client.available()) {
      String line = client.readStringUntil('\n');
      if (line == "\r" || line.length() == 0) break;
    }
  }

  // Parse path and query string from "GET /path?query HTTP/1.1"
  String path  = "/";
  String query = "";
  int sp1 = requestLine.indexOf(' ');
  int sp2 = requestLine.indexOf(' ', sp1 + 1);
  if (sp1 >= 0 && sp2 > sp1) {
    String full = requestLine.substring(sp1 + 1, sp2);
    int qm = full.indexOf('?');
    if (qm >= 0) { path = full.substring(0, qm); query = full.substring(qm + 1); }
    else         { path = full; }
  }

  if      (path == "/status") handleStatusRequest(client);
  else if (path == "/cmd")   handleCommandRequest(client, query);
  else                        streamHTMLPage(client);

  client.flush();
  delay(2);
  client.stop();
}

// ============================================================
//  SERIAL STATUS  (JSON heartbeat to USB serial / Python GUI)
// ============================================================
void printStatus() { writeStatus(Serial); }

// ============================================================
//  SERIAL COMMAND PARSER  (USB serial / Python GUI)
// ============================================================
void parseSerialCommands() {
  if (!Serial.available()) return;
  String cmd = Serial.readStringUntil('\n');
  cmd.trim();
  if (cmd.length() == 0) return;

  if (cmd.startsWith("SIM ")) {
    simCurrent = cmd.substring(4).toFloat();
    simMode    = true;
  } else if (cmd.startsWith("TRIM ")) {
    SERVO_STOP_TRIM_US = (int16_t)cmd.substring(5).toInt();
    stopServo();
  } else if (cmd == "SIM_OFF") {
    simMode = false;
  } else if (cmd == "FWD") {
    servoState = SERVO_ROTATING_FORWARD;
    startServo(SERVO_FORWARD_US);
  } else if (cmd == "REV") {
    servoState = SERVO_ROTATING_REVERSE;
    startServo(SERVO_REVERSE_US);
  } else if (cmd == "STOP") {
    stopServo();
    servoState = SERVO_IDLE;
  } else if (cmd == "STALL_ON") {
    stallDetectEnabled = true;
    stallDetected      = false;
  } else if (cmd == "STALL_OFF") {
    stallDetectEnabled = false;
  } else if (cmd.startsWith("STALL_THRESH ")) {
    STALL_CURRENT_A = cmd.substring(13).toFloat();
  } else if (cmd == "STALL_CLEAR") {
    stallDetected = false;
  } else if (cmd == "WIFI") {
    printWiFiStatus();
    return;   // skip status echo
  } else if (cmd == "OTA") {
    checkForOTAUpdate();
    return;
  }
  // Always echo back current status after any command
  writeStatus(Serial);
}
// ============================================================
void printWiFiStatus() {
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print(F("[WiFi] Connected! IP: "));
    Serial.println(WiFi.localIP());
  } else {
    Serial.println(F("[WiFi] NOT connected."));
  }
}

// ============================================================
//  SETUP
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== Servo Current Controller (Arduino Uno R4 WiFi) ===");

  // PWM shield – initialise and centre the servo before anything moves
  Wire.begin();
  pwm.begin();
  pwm.setPWMFreq(PWM_FREQ_HZ);
  delay(10);   // let oscillator settle before writing pulses
  stopServo();
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  Serial.println("[Servo] PWM shield initialised – servo stopped.");

  // DHT sensors (uncomment when wired)
  // dht1.begin();
  // dht2.begin();

  // WiFi
  Serial.print("[WiFi] Connecting to: ");
  Serial.println(WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  int tries = 0;
  while (WiFi.status() != WL_CONNECTED && tries < 40) {
    delay(500);
    Serial.print('.');
    tries++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println();
    Serial.println(F("============================================"));
    Serial.print(F("[WiFi] Connected! IP: "));
    Serial.println(WiFi.localIP());
    Serial.print(F("[WiFi] Open  http://"));
    Serial.print(WiFi.localIP());
    Serial.println(F("/  in your browser."));
    Serial.println(F("============================================"));
    webServer.begin();
    Serial.println(F("[Web] Server started on port 80."));
  } else {
    Serial.println(F("\n[WiFi] Connection failed – running without web server."));
    Serial.println(F("[WiFi] Will retry automatically in loop."));
  }

  Serial.println(F("[System] Ready. Send 'WIFI' over serial to reprint IP.\n"));
}

// ============================================================
//  OTA FIRMWARE UPDATE
// ============================================================
void checkForOTAUpdate() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println(F("[OTA] Skipped - WiFi not connected."));
    return;
  }
  Serial.println(F("[OTA] Checking for firmware update..."));

  // DNS check before attempting SSL connection
  IPAddress resolvedIP;
  if (WiFi.hostByName("raw.githubusercontent.com", resolvedIP) != 1) {
    Serial.println(F("[OTA] DNS failed – cannot resolve raw.githubusercontent.com"));
    Serial.println(F("[OTA] Check hotspot has internet access and DNS is reachable."));
    return;
  }
  Serial.print(F("[OTA] DNS OK – raw.githubusercontent.com -> "));
  Serial.println(resolvedIP);

  // Step 1: fetch ota/version.txt from GitHub
  WiFiSSLClient ssl;
  ssl.setTimeout(10000);
  if (!ssl.connect("raw.githubusercontent.com", 443)) {
    Serial.print(F("[OTA] HTTPS connect failed to "));
    Serial.println(resolvedIP);
    Serial.println(F("[OTA] Hotspot may be blocking port 443, or SSL handshake timed out."));
    return;
  }
  ssl.print(F("GET /" OTA_GH_USER "/" OTA_GH_REPO "/" OTA_GH_BRANCH "/ota/version.txt"
              " HTTP/1.1\r\nHost: raw.githubusercontent.com\r\nConnection: close\r\n\r\n"));

  unsigned long t = millis();
  bool inBody = false;
  String versionStr;
  while (ssl.connected() && millis() - t < 8000UL) {
    if (ssl.available()) {
      String line = ssl.readStringUntil('\n');
      if (!inBody) {
        line.trim();
        if (line.length() == 0) inBody = true;
      } else {
        line.trim();
        if (line.length() > 0) { versionStr = line; break; }
      }
    }
  }
  ssl.stop();

  if (versionStr.length() == 0) {
    Serial.println(F("[OTA] Could not read version.txt - check OTA_GH_USER/REPO in secrets.h"));
    return;
  }

  int remoteVer = versionStr.toInt();
  Serial.print(F("[OTA] Local FW_VERSION="));
  Serial.print(FW_VERSION);
  Serial.print(F("  Remote="));
  Serial.println(remoteVer);

  if (remoteVer <= FW_VERSION) {
    Serial.println(F("[OTA] Firmware is up to date."));
    return;
  }

  // Step 2: download firmware binary
  Serial.print(F("[OTA] New version available! Downloading v"));
  Serial.println(remoteVer);

  char fwUrl[300];
  snprintf(fwUrl, sizeof(fwUrl),
    "https://raw.githubusercontent.com/" OTA_GH_USER "/" OTA_GH_REPO "/" OTA_GH_BRANCH "/ota/firmware.bin");

  int ret = ota.begin("/update.bin");
  if (ret != OTAUpdate::OTA_ERROR_NONE) {
    Serial.print(F("[OTA] begin() error: ")); Serial.println(ret); return;
  }

  int dlSize = ota.download(fwUrl, "/update.bin");
  if (dlSize <= 0) {
    Serial.print(F("[OTA] download() error: ")); Serial.println(dlSize);
    Serial.println(F("[OTA] Ensure ota/firmware.bin exists in your GitHub repo."));
    return;
  }
  Serial.print(F("[OTA] Downloaded ")); Serial.print(dlSize); Serial.println(F(" bytes. Verifying..."));

  // Step 3: verify (checks the firmware integrity)
  ret = ota.verify();
  if (ret != OTAUpdate::OTA_ERROR_NONE) {
    Serial.print(F("[OTA] verify() error: ")); Serial.println(ret);
    Serial.println(F("[OTA] Check that push_firmware.ps1 was used to publish the binary."));
    return;
  }

  // Step 4: apply (board auto-resets after this)
  Serial.println(F("[OTA] Verified OK. Applying update - board will restart..."));
  ret = ota.update("/update.bin");
  if (ret != OTAUpdate::OTA_ERROR_NONE) {
    Serial.print(F("[OTA] update() error: ")); Serial.println(ret);
  }
}

// ============================================================
//  GOOGLE SHEETS LOGGING  (HTTPS via Google Apps Script)
// ============================================================

// Format a float for URL parameters; writes "N/A" when value is NaN.
static void fmtFloat(char* buf, size_t sz, float val, uint8_t dp) {
  if (isnan(val) || isinf(val)) { strncpy(buf, "N/A", sz); buf[sz - 1] = '\0'; return; }
  char fmt[8];
  snprintf(fmt, sizeof(fmt), "%%.%uf", dp);
  snprintf(buf, sz, fmt, val);
}

bool logToGoogleSheets(float t1, float h1, float t2, float h2, float cur, const char* door) {
  char vt1[8], vh1[8], vt2[8], vh2[8], vcur[12];
  fmtFloat(vt1,  sizeof(vt1),  t1,  1);
  fmtFloat(vh1,  sizeof(vh1),  h1,  1);
  fmtFloat(vt2,  sizeof(vt2),  t2,  1);
  fmtFloat(vh2,  sizeof(vh2),  h2,  1);
  fmtFloat(vcur, sizeof(vcur), cur, 3);

  char path[320];
  snprintf(path, sizeof(path),
    "/macros/s/" SHEETS_SCRIPT_ID "/exec"
    "?t1=%s&h1=%s&t2=%s&h2=%s&cur=%s&door=%s",
    vt1, vh1, vt2, vh2, vcur, door);

  Serial.print(F("[Sheets] Logging row..."));

  // Step 1: Connect to Apps Script URL (will redirect)
  WiFiSSLClient ssl;
  if (!ssl.connect("script.google.com", 443)) {
    Serial.println(F(" HTTPS connect failed."));
    return false;
  }
  ssl.print(F("GET "));
  ssl.print(path);
  ssl.println(F(" HTTP/1.1"));
  ssl.println(F("Host: script.google.com"));
  ssl.println(F("User-Agent: ArduinoR4"));
  ssl.println(F("Connection: close"));
  ssl.println();

  // Step 2: Read response headers, find Location: redirect
  char locBuf[384];
  locBuf[0] = '\0';
  unsigned long deadline = millis() + 8000UL;
  while (ssl.connected() && millis() < deadline) {
    if (ssl.available()) {
      String line = ssl.readStringUntil('\n');
      line.trim();
      if (line.startsWith("Location: ")) {
        strncpy(locBuf, line.c_str() + 10, sizeof(locBuf) - 1);
        locBuf[sizeof(locBuf) - 1] = '\0';
        break;
      }
      if (line.length() == 0) break;
    }
  }
  ssl.stop();

  if (strlen(locBuf) == 0) {
    // Google may respond 200 directly on some deployments
    Serial.println(F(" No redirect (check Apps Script logs)."));
    return false;
  }

  // Step 3: Parse redirect URL  "https://<host><path>"
  if (strncmp(locBuf, "https://", 8) != 0) {
    Serial.println(F(" Unexpected redirect format."));
    return false;
  }
  char* urlBody  = locBuf + 8;
  char* slashPtr = strchr(urlBody, '/');
  if (!slashPtr) { Serial.println(F(" Bad redirect URL.")); return false; }

  char redirectHost[128];
  size_t hostLen = slashPtr - urlBody;
  if (hostLen >= sizeof(redirectHost)) hostLen = sizeof(redirectHost) - 1;
  memcpy(redirectHost, urlBody, hostLen);
  redirectHost[hostLen] = '\0';
  const char* redirectPath = slashPtr;

  // Step 4: Follow redirect
  WiFiSSLClient ssl2;
  if (!ssl2.connect(redirectHost, 443)) {
    Serial.println(F(" Redirect connect failed."));
    return false;
  }
  ssl2.print(F("GET "));
  ssl2.print(redirectPath);
  ssl2.println(F(" HTTP/1.1"));
  ssl2.print(F("Host: "));
  ssl2.println(redirectHost);
  ssl2.println(F("User-Agent: ArduinoR4"));
  ssl2.println(F("Connection: close"));
  ssl2.println();

  // Drain response
  deadline = millis() + 8000UL;
  while (ssl2.connected() && millis() < deadline) {
    while (ssl2.available()) ssl2.read();
  }
  ssl2.stop();

  Serial.println(F(" OK."));
  return true;
}

// ============================================================
//  WIFI RECONNECTION
// ============================================================
bool wifiWasConnected = false;

void maintainWiFi() {
  bool connected = (WiFi.status() == WL_CONNECTED);
  if (connected && !wifiWasConnected) {
    // Just (re)connected
    Serial.println(F("============================================"));
    Serial.print(F("[WiFi] Connected! IP: "));
    Serial.println(WiFi.localIP());
    Serial.print(F("[WiFi] Open  http://"));
    Serial.print(WiFi.localIP());
    Serial.println(F("/  in your browser."));
    Serial.println(F("============================================"));
    webServer.begin();
    Serial.println(F("[Web] Server (re)started on port 80."));
  } else if (!connected && wifiWasConnected) {
    Serial.println(F("[WiFi] Connection lost – reconnecting..."));
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  } else if (!connected && !wifiWasConnected) {
    // Still trying – retry every 10 s
    static unsigned long lastRetry = 0;
    if (millis() - lastRetry >= 10000) {
      lastRetry = millis();
      Serial.println(F("[WiFi] Retrying connection..."));
      WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    }
  }
  wifiWasConnected = connected;
}

// ============================================================
//  MAIN LOOP
// ============================================================
void loop() {
  // 0. Maintain WiFi connection and web server
  maintainWiFi();

  // 1. Process incoming debug/GUI commands
  parseSerialCommands();

  // 2. Read sensors
  measuredCurrent = readCurrentAmps();
  stallCurrent    = readStallCurrentAmps();   // dedicated ACS723 on A1

  // 3. Update DHT readings (uncomment when sensors are wired)
  // temp1_C      = dht1.readTemperature();
  // humidity1_pct = dht1.readHumidity();
  // temp2_C      = dht2.readTemperature();
  // humidity2_pct = dht2.readHumidity();

  // 4. Evaluate threshold and command servo if needed
  updateThresholdLogic();

  // 5. Check if timed rotation is complete
  updateServo();

  // 6. Accumulate readings; log 1-minute averages to Google Sheets
  {
    static float accT1  = 0, accH1  = 0, accT2  = 0, accH2  = 0, accCur = 0;
    static int   cntT1  = 0, cntH1  = 0, cntT2  = 0, cntH2  = 0, cntCur = 0;
    static unsigned long lastLog = 0;

    if (!isnan(temp1_C))       { accT1  += temp1_C;       cntT1++; }
    if (!isnan(humidity1_pct)) { accH1  += humidity1_pct; cntH1++; }
    if (!isnan(temp2_C))       { accT2  += temp2_C;       cntT2++; }
    if (!isnan(humidity2_pct)) { accH2  += humidity2_pct; cntH2++; }
    accCur += measuredCurrent; cntCur++;

    if (millis() - lastLog >= 60000UL) {
      lastLog = millis();
      if (WiFi.status() == WL_CONNECTED && cntCur > 0) {
        logToGoogleSheets(
          cntT1 > 0 ? accT1 / cntT1 : NAN,
          cntH1 > 0 ? accH1 / cntH1 : NAN,
          cntT2 > 0 ? accT2 / cntT2 : NAN,
          cntH2 > 0 ? accH2 / cntH2 : NAN,
          accCur / cntCur,
          doorStateLabel()
        );
      }
      accT1 = accH1 = accT2 = accH2 = accCur = 0;
      cntT1 = cntH1 = cntT2 = cntH2 = cntCur = 0;
    }
  }

  // 7. Serve any pending web request
  handleWebClients();

  // 8. Send JSON status to serial (consumed by Python GUI)
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint >= 500) {
    lastPrint = millis();
    printStatus();
  }

  // 9. Periodic OTA check (or on-demand from web/serial)
  {
    static unsigned long lastOtaCheck = 0;
    if (otaCheckRequested ||
        (WiFi.status() == WL_CONNECTED && millis() - lastOtaCheck >= OTA_CHECK_INTERVAL_MS)) {
      lastOtaCheck = millis();
      otaCheckRequested = false;
      checkForOTAUpdate();
    }
  }

  delay(10);   // short yield; keeps web server responsive
}
