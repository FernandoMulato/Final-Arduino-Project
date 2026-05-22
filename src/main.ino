/*
 * ACCESS CONTROL AND SECURITY SYSTEM
 * Arquitectura Computacional — Arduino Mega (ATmega2560)
 *
 * FSM of 6 states: IDLE, OPEN, BLOCKED, INTRUSION_MONITOR,
 * ENV_MONITOR, ALARM
 *
 * Libraries: StateMachineLib, AsyncTaskLib, Keypad, Servo,
 *            LiquidCrystal, Average, EEPROM
 * Board: Arduino Mega (ATmega2560) — EEPROM: 4KB
 *
 * Transitions:
 *   IDLE --(correct PIN + role in window)--> OPEN
 *   IDLE --(3 failed attempts)--> BLOCKED
 *   OPEN --(2s unlock expired)--> IDLE
 *   BLOCKED --(5s expired)--> IDLE
 *   ENV_MONITOR --(threshold triggered)--> ALARM
 *   ENV_MONITOR --(4s timeout, no event)--> IDLE
 *   INTRUSION_MONITOR --(hall/mic triggered)--> ALARM
 *   INTRUSION_MONITOR --(2s timeout, no event)--> IDLE
 *   ALARM --(2s, <3 in 12s)--> INTRUSION_MONITOR
 *   ALARM --(3 events in 12s)--> extended block -> INTRUSION_MONITOR
 */

// ============================================================================
// INCLUDES
// ============================================================================
#include <Arduino.h>
#include <StateMachineLib.h>
#include <EEPROM.h>
#include <Keypad.h>
#include <Servo.h>
#include <LiquidCrystal.h>
#include <RunningAverage.h>
#include <AsyncTaskLib.h>

// ============================================================================
// PIN DEFINITIONS
// ============================================================================
// Keypad 4x4 matrix (rows D2-D5, cols D6-D9)
constexpr uint8_t ROW_PINS[4] = {2, 3, 4, 5};
constexpr uint8_t COL_PINS[4] = {6, 7, 8, 9};

// Actuators
constexpr uint8_t PIN_LED    = 10;   // Red LED
constexpr uint8_t PIN_BUZZER = 11;   // Piezo buzzer
constexpr uint8_t PIN_SERVO  = 12;   // Servo lock

// LCD 16x2 in 4-bit parallel mode
constexpr uint8_t LCD_RS = 22, LCD_EN = 23;
constexpr uint8_t LCD_D4 = 24, LCD_D5 = 25, LCD_D6 = 26, LCD_D7 = 27;

// Analog sensors (A0-A3 on Mega)
constexpr uint8_t PIN_MIC  = A0;   // Sound sensor KY-037
constexpr uint8_t PIN_NTC  = A1;   // Temperature KY-013
constexpr uint8_t PIN_LDR  = A2;   // Photoresistor KY-018
constexpr uint8_t PIN_HALL = A3;   // Hall sensor KY-035

// ============================================================================
// TIMING CONSTANTS (ms) — Per PRD
// ============================================================================
constexpr unsigned long T_UNLOCK      = 2000;   // Servo unlock
constexpr unsigned long T_LOCKOUT     = 5000;   // Block (PRD: 5s)
constexpr unsigned long T_ENV_MONITOR = 4000;   // Environmental (PRD: 4s)
constexpr unsigned long T_ALARM       = 2000;   // Alarm (PRD: 2s)
constexpr unsigned long T_INTRUSION   = 2000;   // Intrusion window
constexpr unsigned long T_PIN_TIMEOUT = 10000;  // PIN input timeout
constexpr unsigned long T_TRIPLE      = 12000;  // Triple alarm window

// LED blink patterns — Per PRD
constexpr unsigned long BLK_ON  = 300;   // BLOCKED on
constexpr unsigned long BLK_OFF = 700;   // BLOCKED off
constexpr unsigned long ALM_ON  = 100;   // ALARM on
constexpr unsigned long ALM_OFF = 500;   // ALARM off

// ============================================================================
// SENSOR THRESHOLDS
// ============================================================================
constexpr float TEMP_LOW     = 20.0f;
constexpr float TEMP_HIGH    = 50.0f;
constexpr int   LIGHT_MIN    = 100;
constexpr int   SOUND_HIGH   = 800;
constexpr int   HALL_OPEN    = 512;

// ============================================================================
// PIN CONSTANTS
// ============================================================================
constexpr uint8_t PIN_MIN_LEN  = 4;
constexpr uint8_t PIN_MAX_LEN  = 6;
constexpr uint8_t PIN_MAX_USES = 4;
constexpr uint8_t PIN_HIST_LEN = 4;   // Number of previous PINs tracked

// Steinhart-Hart coefficients for NTC thermistor
constexpr float R1  = 10000.0f;
constexpr float C1  = 0.001129148f;
constexpr float C2  = 0.000234125f;
constexpr float C3  = 0.0000000876741f;

// Average samples
constexpr uint8_t AVG_SAMPLES = 5;

// ============================================================================
// EEPROM LAYOUT (4KB on ATmega2560)
// Each user record = 24 bytes
// 10 users x 24 = 240 bytes total
// ============================================================================
constexpr uint8_t EEP_MAGIC       = 0;
constexpr uint8_t EEP_USER_COUNT  = 1;
constexpr uint8_t EEP_USERS_START = 2;
constexpr uint8_t EEP_USER_SIZE   = 24;
constexpr uint8_t EEP_MAGIC_VAL   = 0xA5;
constexpr uint8_t MAX_USERS       = 10;

// Offsets within each 24-byte user record
constexpr uint8_t OFF_PIN      = 0;   // 4 bytes
constexpr uint8_t OFF_ROLE     = 4;   // 1 byte
constexpr uint8_t OFF_USES     = 5;   // 1 byte
constexpr uint8_t OFF_ACTIVE   = 6;   // 1 byte
constexpr uint8_t OFF_HIST_IDX = 7;   // 1 byte
constexpr uint8_t OFF_HIST     = 8;   // 16 bytes (4 x 4-byte PINs)

// ============================================================================
// ENUMERATIONS
// ============================================================================
enum State : uint8_t {
  S_IDLE,
  S_OPEN,
  S_BLOCKED,
  S_INTRUSION_MONITOR,
  S_ENV_MONITOR,
  S_ALARM
};

enum Trigger : uint8_t {
  TRIG_NONE,
  TRIG_AUTH_OK,       // PIN + role window valid
  TRIG_LOCKOUT,       // 3 failed attempts
  TRIG_ENV_ALARM,     // Temperature or light threshold
  TRIG_INTRUSION      // Hall or mic detection
};

// ============================================================================
// ROLE DEFINITIONS — Per PRD section 6.1
// ============================================================================
enum Role : uint8_t {
  ROLE_SECURITY   = 1,
  ROLE_OPERATOR    = 2,
  ROLE_COORDINATOR = 3,
  ROLE_MANAGER     = 4
};

constexpr const char* ROLE_NAMES[5] = {
  "", "Security", "Operator", "Coordinator", "Manager"
};

// ============================================================================
// GLOBAL OBJECTS
// ============================================================================
// FSM: 6 states, 10 transitions
StateMachine fsm(6, 10);
State currentState = S_IDLE;

// Keypad 4x4
constexpr uint8_t KP_ROWS = 4, KP_COLS = 4;
char keyMap[KP_ROWS][KP_COLS] = {
  {'1','2','3','A'}, {'4','5','6','B'},
  {'7','8','9','C'}, {'*','0','#','D'}
};
Keypad keypad = Keypad(makeKeymap(keyMap),
  (byte*)ROW_PINS, (byte*)COL_PINS, KP_ROWS, KP_COLS);

// Servo
Servo doorServo;
constexpr uint8_t SRV_LOCKED   = 0;
constexpr uint8_t SRV_UNLOCKED = 90;

// LCD
LiquidCrystal lcd(LCD_RS, LCD_EN, LCD_D4, LCD_D5, LCD_D6, LCD_D7);

// Average objects for sensor smoothing
RunningAverage tempAvg(AVG_SAMPLES);
RunningAverage lightAvg(AVG_SAMPLES);

// ============================================================================
// GLOBAL STATE
// ============================================================================
// PIN entry
char pinBuf[PIN_MAX_LEN + 1] = {0};
uint8_t pinLen = 0;
unsigned long pinStartTime = 0;

// Auth
uint8_t failCount = 0;
uint8_t alarmCount = 0;
unsigned long firstAlarmTime = 0;

// State timing
unsigned long stateEntryTime = 0;

// Transition trigger (set by events, consumed by FSM)
Trigger trig = TRIG_NONE;

// Sensor data
int hallVal = 0, micVal = 0;
float temperature = 0.0f;
int lightLevel = 0;

// Blink state (used in updateState for LED patterns)
unsigned long lastBlink = 0;
bool ledOn = false;
unsigned long blinkOnMs = 0, blinkOffMs = 0;
bool blinkActive = false;

// Menu / PIN change
bool menuActive = false;
uint8_t menuStep = 0;        // 0=old PIN, 1=new PIN, 2=confirm
char menuBuf[PIN_MAX_LEN + 1] = {0};
uint8_t menuBufLen = 0;
uint8_t menuUserIdx = 0xFF;  // User changing PIN

// PIN rotation flag — set when uses >= 4
bool pinChangeRequired = false;
uint8_t pinChangeUserIdx = 0xFF;

// LCD update
unsigned long lastLcdUpdate = 0;
constexpr unsigned long LCD_INTERVAL = 250;

// ============================================================================
// FORWARD DECLARATIONS
// ============================================================================
void updateDisplay();
void processInput();
void updateState();

void onEnterIdle();
void onLeaveIdle();
void onEnterOpen();
void onLeaveOpen();
void onEnterBlocked();
void onLeaveBlocked();
void onEnterIntrusionMonitor();
void onLeaveIntrusionMonitor();
void onEnterEnvMonitor();
void onLeaveEnvMonitor();
void onEnterAlarm();
void onLeaveAlarm();

// ============================================================================
// EEPROM FUNCTIONS
// ============================================================================
void initEEPROM() {
  if (EEPROM.read(EEP_MAGIC) != EEP_MAGIC_VAL) {
    EEPROM.write(EEP_MAGIC, EEP_MAGIC_VAL);
    EEPROM.write(EEP_USER_COUNT, 0);
  }
}

uint8_t userCount() {
  return EEPROM.read(EEP_USER_COUNT);
}

void setUserCount(uint8_t c) {
  EEPROM.write(EEP_USER_COUNT, c);
}

// Read a user record from EEPROM at given index
void loadUser(uint8_t idx, char pin[5], uint8_t& role, uint8_t& uses,
              bool& active, uint8_t& histIdx, char history[4][4]) {
  uint16_t addr = EEP_USERS_START + idx * EEP_USER_SIZE;
  for (uint8_t i = 0; i < 4; i++) pin[i] = EEPROM.read(addr + OFF_PIN + i);
  pin[4] = '\0';
  role    = EEPROM.read(addr + OFF_ROLE);
  uses    = EEPROM.read(addr + OFF_USES);
  active  = EEPROM.read(addr + OFF_ACTIVE) != 0;
  histIdx = EEPROM.read(addr + OFF_HIST_IDX);
  for (uint8_t h = 0; h < PIN_HIST_LEN; h++)
    for (uint8_t i = 0; i < 4; i++)
      history[h][i] = EEPROM.read(addr + OFF_HIST + h * 4 + i);
  if (pin[0] < '0' || pin[0] > '9') active = false;
}

// Save a user record to EEPROM at given index
void saveUser(uint8_t idx, const char pin[5], uint8_t role, uint8_t uses,
              bool active, uint8_t histIdx, const char history[4][4]) {
  uint16_t addr = EEP_USERS_START + idx * EEP_USER_SIZE;
  for (uint8_t i = 0; i < 4; i++) EEPROM.update(addr + OFF_PIN + i, pin[i]);
  EEPROM.update(addr + OFF_ROLE, role);
  EEPROM.update(addr + OFF_USES, uses);
  EEPROM.update(addr + OFF_ACTIVE, active ? 1 : 0);
  EEPROM.update(addr + OFF_HIST_IDX, histIdx);
  for (uint8_t h = 0; h < PIN_HIST_LEN; h++)
    for (uint8_t i = 0; i < 4; i++)
      EEPROM.update(addr + OFF_HIST + h * 4 + i, history[h][i]);
}

// ============================================================================
// ACCESS VALIDATION
// ============================================================================
// Find user index by PIN. Returns 0xFF if not found.
uint8_t findUserByPin(const char* pin) {
  uint8_t n = userCount();
  for (uint8_t i = 0; i < n; i++) {
    char sp[5]; uint8_t r, u, hi; bool a; char hist[4][4];
    loadUser(i, sp, r, u, a, hi, hist);
    if (!a) continue;
    bool match = true;
    for (uint8_t j = 0; j < 4; j++) { if (pin[j] != sp[j]) { match = false; break; } }
    if (match) return i;
  }
  return 0xFF;
}

// Check if new PIN differs from current PIN and all history entries
bool pinIsUnique(uint8_t userIdx, const char* newPin) {
  char sp[5]; uint8_t r, u, hi; bool a; char hist[4][4];
  loadUser(userIdx, sp, r, u, a, hi, hist);

  // Check against current PIN
  bool same = true;
  for (uint8_t j = 0; j < 4; j++) { if (newPin[j] != sp[j]) { same = false; break; } }
  if (same) return false;

  // Check against history
  for (uint8_t h = 0; h < PIN_HIST_LEN; h++) {
    if (hist[h][0] < '0' || hist[h][0] > '9') continue;  // empty slot
    same = true;
    for (uint8_t j = 0; j < 4; j++) { if (newPin[j] != hist[h][j]) { same = false; break; } }
    if (same) return false;
  }
  return true;
}

// Rotate PIN: push current PIN into history, store new PIN, reset uses
void rotatePin(uint8_t userIdx, const char* newPin) {
  char sp[5]; uint8_t r, u, hi; bool a; char hist[4][4];
  loadUser(userIdx, sp, r, u, a, hi, hist);

  // Push current PIN into history at histIdx
  for (uint8_t i = 0; i < 4; i++) hist[hi][i] = sp[i];
  hi = (hi + 1) % PIN_HIST_LEN;

  // Save new PIN with uses=0
  saveUser(userIdx, newPin, r, 0, true, hi, (const char(*)[4])hist);
}

// Validate PIN + role + time window. Returns true if access granted.
// If uses reaches PIN_MAX_USES, still grants but sets pinChangeRequired.
bool validateAccess(const char* pin) {
  uint8_t idx = findUserByPin(pin);
  if (idx == 0xFF) {
    Serial.println(F("[AUTH] PIN not found"));
    return false;
  }

  char sp[5]; uint8_t role, uses, hi; bool a; char hist[4][4];
  loadUser(idx, sp, role, uses, a, hi, hist);

  // Check role time window (always true in demo, structure ready for RTC)
  if (role < 1 || role > 4) {
    Serial.println(F("[AUTH] Invalid role"));
    return false;
  }

  // Grant access, increment uses
  uses++;
  if (uses >= PIN_MAX_USES) {
    // Force PIN change on next attempt
    pinChangeRequired = true;
    pinChangeUserIdx = idx;
    saveUser(idx, sp, role, uses, true, hi, (const char(*)[4])hist);
    Serial.println(F("[AUTH] PIN expired: change required after this access"));
  } else {
    saveUser(idx, sp, role, uses, true, hi, (const char(*)[4])hist);
  }

  Serial.print(F("[AUTH] Granted: user "));
  Serial.print(idx);
  Serial.print(F(" role "));
  Serial.println(ROLE_NAMES[role]);
  return true;
}

// ============================================================================
// ACTUATOR HELPERS
// ============================================================================
void setLED(bool on)     { digitalWrite(PIN_LED, on ? HIGH : LOW); }
void setBuzzer(bool on)  { digitalWrite(PIN_BUZZER, on ? HIGH : LOW); }
void unlockDoor()        { doorServo.write(SRV_UNLOCKED); }
void lockDoor()          { doorServo.write(SRV_LOCKED); }

// ============================================================================
// LCD DISPLAY — Per PRD section 8.1
// ============================================================================
void updateDisplay() {
  lcd.clear();
  lcd.setCursor(0, 0);
  switch (currentState) {
    case S_IDLE: {
      if (menuActive) {
        lcd.print(F("CHANGE PIN"));
        lcd.setCursor(0, 1);
        if (menuStep == 0) lcd.print(F("Old PIN:"));
        else if (menuStep == 1) lcd.print(F("New PIN:"));
        else lcd.print(F("Confirm:"));
        for (uint8_t i = 0; i < menuBufLen; i++) lcd.print('*');
        for (uint8_t i = menuBufLen; i < PIN_MAX_LEN; i++) lcd.print('-');
      } else if (pinLen > 0) {
        lcd.print(F("IDLE"));
        lcd.setCursor(0, 1);
        for (uint8_t i = 0; i < PIN_MAX_LEN; i++) lcd.print(i < pinLen ? '*' : '-');
        lcd.print(F(" #=ok"));
      } else {
        lcd.print(F("IDLE"));
        lcd.setCursor(0, 1);
        lcd.print(F("Enter PIN:"));
      }
      break;
    }
    case S_OPEN: {
      lcd.print(F("ACCESS GRANTED"));
      unsigned long elapsed = millis() - stateEntryTime;
      unsigned long rem = (elapsed < T_UNLOCK) ? (T_UNLOCK - elapsed) / 1000 : 0;
      lcd.setCursor(0, 1);
      lcd.print(F("OPEN "));
      lcd.print(rem);
      lcd.print('s');
      break;
    }
    case S_BLOCKED:
      lcd.print(F("BLOCKED"));
      lcd.setCursor(0, 1);
      lcd.print(F("Wait 5s..."));
      break;
    case S_ENV_MONITOR: {
      lcd.print(F("MONITOR ENV"));
      lcd.setCursor(0, 1);
      lcd.print(F("T:"));
      lcd.print((int)temperature);
      lcd.print(F("C L:"));
      lcd.print(lightLevel);
      break;
    }
    case S_INTRUSION_MONITOR:
      lcd.print(F("MONITOR SEC"));
      break;
    case S_ALARM:
      lcd.print(F("!!! ALARM !!!"));
      lcd.setCursor(0, 1);
      lcd.print(F("Intrusion!"));
      break;
  }
}

// ============================================================================
// FSM STATE HANDLERS
// ============================================================================
void onEnterIdle() {
  currentState = S_IDLE;
  failCount = 0;
  pinLen = 0; memset(pinBuf, 0, sizeof(pinBuf));
  trig = TRIG_NONE;
  menuActive = false; menuStep = 0; menuBufLen = 0; menuUserIdx = 0xFF;
  blinkActive = false; setLED(LOW); setBuzzer(false);
  lockDoor();
  lastLcdUpdate = 0;  // Force LCD update
  Serial.println(F("[STATE] IDLE — System ready"));
}

void onLeaveIdle() {
  pinLen = 0; memset(pinBuf, 0, sizeof(pinBuf));
  menuActive = false;
}

void onEnterOpen() {
  currentState = S_OPEN;
  trig = TRIG_NONE;
  stateEntryTime = millis();
  unlockDoor();
  Serial.println(F("[STATE] OPEN — Door unlocked (2s)"));
}

void onLeaveOpen() {
  lockDoor();
  Serial.println(F("[STATE] OPEN — Door locked"));
}

void onEnterBlocked() {
  currentState = S_BLOCKED;
  trig = TRIG_NONE;
  stateEntryTime = millis();
  blinkActive = true; blinkOnMs = BLK_ON; blinkOffMs = BLK_OFF;
  lastBlink = millis(); ledOn = false;
  Serial.println(F("[STATE] BLOCKED — 3 failed attempts, 5s block"));
}

void onLeaveBlocked() {
  blinkActive = false; setLED(LOW);
}

void onEnterIntrusionMonitor() {
  currentState = S_INTRUSION_MONITOR;
  trig = TRIG_NONE;
  stateEntryTime = millis();
  Serial.println(F("[STATE] INTRUSION_MONITOR — Watching..."));
}

void onLeaveIntrusionMonitor() {}

void onEnterEnvMonitor() {
  currentState = S_ENV_MONITOR;
  trig = TRIG_NONE;
  stateEntryTime = millis();
  Serial.println(F("[STATE] ENV_MONITOR — Temp + light"));
}

void onLeaveEnvMonitor() {}

void onEnterAlarm() {
  currentState = S_ALARM;
  trig = TRIG_NONE;
  stateEntryTime = millis();
  setBuzzer(true);
  blinkActive = true; blinkOnMs = ALM_ON; blinkOffMs = ALM_OFF;
  lastBlink = millis(); ledOn = false;
  Serial.println(F("[STATE] ALARM — Intrusion!"));
}

void onLeaveAlarm() {
  setBuzzer(false);
  blinkActive = false; setLED(LOW);
}

// ============================================================================
// KEYPAD INPUT PROCESSING
// ============================================================================
void handleMenuKey(char key) {
  if (key == '*') {
    // Cancel menu
    menuActive = false; menuStep = 0; menuBufLen = 0; menuUserIdx = 0xFF;
    pinLen = 0; memset(pinBuf, 0, sizeof(pinBuf));
    Serial.println(F("[MENU] Cancelled"));
    return;
  }

  if (key == '#') {
    if (menuStep == 0) {
      // Old PIN entered — validate
      menuBuf[menuBufLen] = '\0';
      if (menuBufLen < PIN_MIN_LEN) {
        Serial.println(F("[MENU] PIN too short"));
        menuBufLen = 0;
        return;
      }
      uint8_t idx = findUserByPin(menuBuf);
      if (idx == 0xFF) {
        Serial.println(F("[MENU] Wrong PIN"));
        menuBufLen = 0;
        return;
      }
      menuUserIdx = idx;
      menuStep = 1; menuBufLen = 0; memset(menuBuf, 0, sizeof(menuBuf));
      Serial.println(F("[MENU] Enter new PIN (4-6 digits):"));
    } else if (menuStep == 1) {
      // New PIN entered — check length
      menuBuf[menuBufLen] = '\0';
      if (menuBufLen < PIN_MIN_LEN) {
        Serial.println(F("[MENU] Too short (min 4)"));
        menuBufLen = 0;
        return;
      }
      // Check uniqueness
      if (!pinIsUnique(menuUserIdx, menuBuf)) {
        Serial.println(F("[MENU] PIN was used before. Choose another."));
        menuBufLen = 0;
        return;
      }
      // Save new PIN
      rotatePin(menuUserIdx, menuBuf);
      pinChangeRequired = false;
      pinChangeUserIdx = 0xFF;
      Serial.println(F("[MENU] PIN changed successfully!"));
      menuActive = false; menuStep = 0; menuBufLen = 0;
    }
    return;
  }

  if (key >= '0' && key <= '9') {
    if (menuBufLen < PIN_MAX_LEN) {
      menuBuf[menuBufLen++] = key;
    }
  }
}

void handlePinEntry(char key) {
  if (key >= '0' && key <= '9') {
    if (pinLen < PIN_MAX_LEN) {
      pinBuf[pinLen++] = key;
      pinStartTime = millis();
    }
  } else if (key == '#') {
    if (pinLen == 0) {
      // Open PIN change menu
      menuActive = true; menuStep = 0; menuBufLen = 0;
      memset(menuBuf, 0, sizeof(menuBuf));
      Serial.println(F("[MENU] Enter current PIN:"));
    } else if (pinLen >= PIN_MIN_LEN) {
      pinBuf[pinLen] = '\0';

      // Check if PIN change is required for this user
      if (pinChangeRequired) {
        uint8_t idx = findUserByPin(pinBuf);
        if (idx != 0xFF && idx == pinChangeUserIdx) {
          Serial.println(F("[AUTH] PIN expired. Change via menu (# at IDLE)."));
          pinLen = 0; memset(pinBuf, 0, sizeof(pinBuf));
          return;
        }
      }

      if (validateAccess(pinBuf)) {
        trig = TRIG_AUTH_OK;
      } else {
        failCount++;
        Serial.print(F("[AUTH] Failed attempt "));
        Serial.println(failCount);
        if (failCount >= 3) {
          trig = TRIG_LOCKOUT;
        } else {
          pinLen = 0;
          memset(pinBuf, 0, sizeof(pinBuf));
        }
      }
    }
  } else if (key == '*') {
    pinLen = 0; memset(pinBuf, 0, sizeof(pinBuf));
    Serial.println(F("[INPUT] Cancelled"));
  }
}

void processInput() {
  char key = keypad.getKey();

  if (currentState == S_IDLE) {
    if (key != NO_KEY) {
      if (menuActive) {
        handleMenuKey(key);
      } else {
        handlePinEntry(key);
      }
    }

    // Check PIN timeout
    if (pinLen > 0 && (millis() - pinStartTime) >= T_PIN_TIMEOUT) {
      Serial.println(F("[INPUT] PIN timeout"));
      pinLen = 0; memset(pinBuf, 0, sizeof(pinBuf));
    }
  }
}

// ============================================================================
// STATE UPDATE — Per-loop logic
// ============================================================================
void updateState() {
  unsigned long now = millis();

  // --- State timing (millis-based) ---
  if (trig == TRIG_NONE) {
    unsigned long elapsed = now - stateEntryTime;
    switch (currentState) {
      case S_OPEN:
        if (elapsed >= T_UNLOCK) trig = TRIG_AUTH_OK;  // reuse to exit
        break;
      case S_BLOCKED:
        if (elapsed >= T_LOCKOUT) trig = TRIG_AUTH_OK;
        break;
      case S_ENV_MONITOR:
        if (elapsed >= T_ENV_MONITOR) trig = TRIG_AUTH_OK;
        break;
      case S_INTRUSION_MONITOR:
        if (elapsed >= T_INTRUSION) trig = TRIG_AUTH_OK;
        break;
      case S_ALARM:
        if (elapsed >= T_ALARM) trig = TRIG_AUTH_OK;
        break;
      default: break;
    }
  }

  // --- Sensor reading (AsyncTaskLib driven) ---
  // AsyncTask sensorTask handles analog reads in setup/loop Update
  // Temperature calculated from averaged raw NTC values
  if (tempAvg.getCount() > 0) {
    float avgRaw = tempAvg.getAverage();
    if (avgRaw <= 0) avgRaw = 1;
    float R2 = R1 * (1023.0f / avgRaw - 1.0f);
    float logR2 = log(R2);
    temperature = (1.0f / (C1 + C2 * logR2 + C3 * logR2 * logR2 * logR2)) - 273.15f;
  }
  if (lightAvg.getCount() > 0) {
    lightLevel = (int)lightAvg.getAverage();
  }

  // --- State-specific sensor monitoring ---
  switch (currentState) {
    case S_ENV_MONITOR:
      if ((temperature > 0 && temperature < TEMP_LOW) ||
          temperature > TEMP_HIGH ||
          (lightLevel > 0 && lightLevel < LIGHT_MIN)) {
        trig = TRIG_ENV_ALARM;
        Serial.print(F("[ENV] Threshold: T="));
        Serial.print(temperature); Serial.print(F(" L=")); Serial.println(lightLevel);
      }
      break;

    case S_INTRUSION_MONITOR:
      if (hallVal > HALL_OPEN || micVal > SOUND_HIGH) {
        trig = TRIG_INTRUSION;
        Serial.println(F("[INTRUSION] Detected!"));
        // Only trigger once per INTRUSION state
      }
      break;

    case S_ALARM:
      // Triple alarm detection within T_TRIPLE window
      if (hallVal > HALL_OPEN || micVal > SOUND_HIGH) {
        alarmCount++;
        if (alarmCount == 1) firstAlarmTime = now;
        Serial.print(F("[ALARM] Event #"));
        Serial.println(alarmCount);
        if (alarmCount >= 3 && (now - firstAlarmTime) < T_TRIPLE) {
          Serial.println(F("[ALARM] Triple event!"));
          // Reset the alarm timer for extended block
          stateEntryTime = now;
          alarmCount = 0;
        }
      }
      break;

    default: break;
  }

  // --- LED blink pattern ---
  if (blinkActive) {
    unsigned long interval = ledOn ? blinkOffMs : blinkOnMs;
    if (now - lastBlink >= interval) {
      lastBlink = now;
      ledOn = !ledOn;
      setLED(ledOn);
    }
  }

  // --- LCD refresh (AsyncTaskLib driven in loop) ---
  if (now - lastLcdUpdate >= LCD_INTERVAL) {
    lastLcdUpdate = now;
    updateDisplay();
  }
}

// ============================================================================
// FSM CONFIGURATION
// ============================================================================
void setupFSM() {
  // State entry handlers
  fsm.SetOnEntering(S_IDLE, onEnterIdle);
  fsm.SetOnEntering(S_OPEN, onEnterOpen);
  fsm.SetOnEntering(S_BLOCKED, onEnterBlocked);
  fsm.SetOnEntering(S_INTRUSION_MONITOR, onEnterIntrusionMonitor);
  fsm.SetOnEntering(S_ENV_MONITOR, onEnterEnvMonitor);
  fsm.SetOnEntering(S_ALARM, onEnterAlarm);

  // State exit handlers
  fsm.SetOnLeaving(S_IDLE, onLeaveIdle);
  fsm.SetOnLeaving(S_OPEN, onLeaveOpen);
  fsm.SetOnLeaving(S_BLOCKED, onLeaveBlocked);
  fsm.SetOnLeaving(S_INTRUSION_MONITOR, onLeaveIntrusionMonitor);
  fsm.SetOnLeaving(S_ENV_MONITOR, onLeaveEnvMonitor);
  fsm.SetOnLeaving(S_ALARM, onLeaveAlarm);

  // Transitions
  // Auth OK (also used as "state timer done" for timed states)
  fsm.AddTransition(S_IDLE, S_OPEN, []() { return trig == TRIG_AUTH_OK; });

  // 3 failed attempts
  fsm.AddTransition(S_IDLE, S_BLOCKED, []() { return trig == TRIG_LOCKOUT; });

  // Timed states: any non-event trigger means time expired
  auto timedDone = []() { return trig == TRIG_AUTH_OK; };
  fsm.AddTransition(S_OPEN, S_IDLE, timedDone);
  fsm.AddTransition(S_BLOCKED, S_IDLE, timedDone);
  fsm.AddTransition(S_ENV_MONITOR, S_IDLE, timedDone);
  fsm.AddTransition(S_INTRUSION_MONITOR, S_IDLE, timedDone);

  // Threshold/intrusion events
  fsm.AddTransition(S_ENV_MONITOR, S_ALARM, []() { return trig == TRIG_ENV_ALARM; });
  fsm.AddTransition(S_INTRUSION_MONITOR, S_ALARM, []() { return trig == TRIG_INTRUSION; });

  // Alarm -> intrusion monitoring when timer expires
  fsm.AddTransition(S_ALARM, S_INTRUSION_MONITOR, timedDone);
}

// ============================================================================
// ASYNCTASKLIB TASKS
// ============================================================================
// Sensor averaging task: reads analog sensors every SENSOR_INTERVAL ms
constexpr unsigned long SENSOR_INTERVAL = 200;
AsyncTask sensorTask(SENSOR_INTERVAL, true, []() {
  int ntc = analogRead(PIN_NTC);
  int ldr = analogRead(PIN_LDR);
  hallVal = analogRead(PIN_HALL);
  micVal  = analogRead(PIN_MIC);

  // Add to running averages
  tempAvg.add(ntc);
  lightAvg.add(ldr);
});

// ============================================================================
// SETUP
// ============================================================================
void setup() {
  Serial.begin(9600);
  randomSeed(analogRead(PIN_MIC));

  // Pin modes
  pinMode(PIN_LED, OUTPUT); digitalWrite(PIN_LED, LOW);
  pinMode(PIN_BUZZER, OUTPUT); digitalWrite(PIN_BUZZER, LOW);

  // Servo
  doorServo.attach(PIN_SERVO);
  doorServo.write(SRV_LOCKED);

  // LCD
  lcd.begin(16, 2);
  lcd.print(F("Access Control"));
  lcd.setCursor(0, 1);
  lcd.print(F("Starting..."));

  // EEPROM
  initEEPROM();

  // Start AsyncTask sensor averaging
  sensorTask.Start();

  // FSM
  setupFSM();
  fsm.SetState(S_IDLE, false, true);

  Serial.println(F("=== ACCESS CONTROL & SECURITY ==="));
  Serial.println(F("Keys: [0-9]=digit [#]=ok [*]=cancel"));
  Serial.println(F("From IDLE: # with no digits = change PIN"));
}

// ============================================================================
// LOOP
// ============================================================================
void loop() {
  processInput();
  updateState();

  // Update AsyncTaskLib periodic tasks
  sensorTask.Update();

  // Update FSM (state transition checks)
  fsm.Update();
}
