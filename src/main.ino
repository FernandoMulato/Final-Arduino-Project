/**
 * @file main.ino
 * @brief Access Control and Security System for Arduino Mega.
 *
 * @details
 * Academic project for Arquitectura Computacional. Implements a 6-state
 * Finite State Machine (FSM) with the following states:
 * @code{.cpp}
 * IDLE -> (correct PIN + role) -> OPEN -> (2s) -> IDLE
 * IDLE -> (3 failures) -> BLOCKED -> (5s) -> IDLE
 * ENV_MONITOR -> (threshold) -> ALARM -> (2s) -> INTRUSION_MONITOR
 * ENV_MONITOR -> (4s no event) -> IDLE
 * INTRUSION_MONITOR -> (hall/mic) -> ALARM (triple rearm 12s)
 * INTRUSION_MONITOR -> (2s no event) -> IDLE
 * ALARM -> (3 in 12s) -> extended block -> INTRUSION_MONITOR
 * @endcode
 *
 * @par Hardware
 * Board: Arduino Mega (ATmega2560)
 * - Lock: Servo motor (D12, PWM)
 * - Display: LCD 16x2 parallel (D22-D27)
 * - Door sensor: Reed switch on D21 (INT2, interrupt-driven)
 * - Mic: KY-037 on A0, NTC: KY-013 on A1, LDR: KY-018 on A2
 * - Keypad: 4x4 matrix (D2-D9)
 *
 * @par Memory
 * SRAM: 741B (9.0%), Flash: 17.4KB (6.9%), EEPROM: 4KB
 *
 * @par EEPROM Layout
 * 10 users x 24 bytes = 240 bytes
 * User record: PIN[4] + role + uses + active + histIdx + history[4][4]
 *
 * @par Libraries
 * StateMachineLib, AsyncTaskLib, Keypad 3.1.1, Servo,
 * LiquidCrystal, RunningAverage, EEPROM
 *
 * @author Arquitectura Computacional — Universidad del Cauca
 * @date 2026
 */

// ============================================================================
// @name Includes
// @brief System and library includes
// @{
// ============================================================================

#include <Arduino.h>
#include <StateMachineLib.h>
#include <EEPROM.h>
#include <Keypad.h>
#include <Servo.h>
#include <LiquidCrystal.h>
#include <RunningAverage.h>
#include <AsyncTaskLib.h>

/** @} */

// ============================================================================
// @name Pin Definitions
// @brief Hardware pin assignments for the ATmega2560
// @{
// ============================================================================

/** @brief Keypad row pins (4x4 matrix). */
constexpr uint8_t ROW_PINS[4] = {2, 3, 4, 5};
/** @brief Keypad column pins. */
constexpr uint8_t COL_PINS[4] = {6, 7, 8, 9};

/** @brief Red LED indicator (alarm/block patterns). */
constexpr uint8_t PIN_LED    = 10;
/** @brief Piezo buzzer (alarm). */
constexpr uint8_t PIN_BUZZER = 11;
/** @brief Servo motor (door lock, PWM). */
constexpr uint8_t PIN_SERVO  = 12;

/** @brief LCD RS pin. */
constexpr uint8_t LCD_RS = 22;
/** @brief LCD Enable pin. */
constexpr uint8_t LCD_EN = 23;
/** @brief LCD data pin D4 (4-bit mode). */
constexpr uint8_t LCD_D4 = 24;
/** @brief LCD data pin D5. */
constexpr uint8_t LCD_D5 = 25;
/** @brief LCD data pin D6. */
constexpr uint8_t LCD_D6 = 26;
/** @brief LCD data pin D7. */
constexpr uint8_t LCD_D7 = 27;

/**
 * @brief Sound sensor analog input.
 * @details KY-037 microphone module, threshold > 800.
 */
constexpr uint8_t PIN_MIC  = A0;
/**
 * @brief NTC thermistor analog input.
 * @details KY-013, Steinhart-Hart equation for temperature.
 */
constexpr uint8_t PIN_NTC  = A1;
/** @brief Photoresistor analog input (KY-018). */
constexpr uint8_t PIN_LDR  = A2;

/**
 * @brief Door reed switch (interrupt-driven).
 * @details
 * Connected to D21 (INT2 on ATmega2560). Uses INPUT_PULLUP:
 * - Magnet present (door closed) = LOW
 * - Magnet absent (door open) = HIGH
 * ISR fires on CHANGE for instant detection.
 *
 * @see doorISR()
 */
constexpr uint8_t PIN_HALL_DOOR = 21;

/** @} */

// ============================================================================
// @name Timing Constants (ms)
// @brief State duration and LED pattern timings per PRD
// @{
// ============================================================================

/** @brief Servo unlock duration. */
constexpr unsigned long T_UNLOCK      = 2000;
/** @brief BLOCKED state block duration (PRD: 5s). */
constexpr unsigned long T_LOCKOUT     = 5000;
/** @brief ENV_MONITOR watch window (PRD: 4s). */
constexpr unsigned long T_ENV_MONITOR = 4000;
/** @brief ALARM active duration (PRD: 2s). */
constexpr unsigned long T_ALARM       = 2000;
/** @brief INTRUSION_MONITOR watch window. */
constexpr unsigned long T_INTRUSION   = 2000;
/** @brief PIN input timeout. */
constexpr unsigned long T_PIN_TIMEOUT = 10000;
/**
 * @brief Triple alarm detection window.
 * @details 3 alarm events within 12s trigger extended block.
 */
constexpr unsigned long T_TRIPLE      = 12000;

/** @brief BLOCKED LED on time. */
constexpr unsigned long BLK_ON  = 300;
/** @brief BLOCKED LED off time. */
constexpr unsigned long BLK_OFF = 700;
/** @brief ALARM LED on time. */
constexpr unsigned long ALM_ON  = 100;
/** @brief ALARM LED off time. */
constexpr unsigned long ALM_OFF = 500;

/** @} */

// ============================================================================
// @name Sensor Thresholds
// @brief Analog trigger levels for environmental monitoring
// @{
// ============================================================================

/** @brief Low temperature threshold (°C). */
constexpr float TEMP_LOW     = 20.0f;
/** @brief High temperature threshold (°C). */
constexpr float TEMP_HIGH    = 50.0f;
/** @brief Minimum light level (ADC units). */
constexpr int   LIGHT_MIN    = 100;
/** @brief Loud sound threshold (ADC units). */
constexpr int   SOUND_HIGH   = 800;

/** @} */

// ============================================================================
// @name PIN Policy Constants
// @brief Password policy configuration
// @{
// ============================================================================

/** @brief Minimum PIN digits. */
constexpr uint8_t PIN_MIN_LEN  = 4;
/** @brief Maximum PIN digits. */
constexpr uint8_t PIN_MAX_LEN  = 6;
/**
 * @brief Max uses before forced rotation.
 * @details After 4 successful accesses, user must change PIN.
 */
constexpr uint8_t PIN_MAX_USES = 4;
/** @brief Number of previous PINs tracked (circular buffer). */
constexpr uint8_t PIN_HIST_LEN = 4;

/** @} */

// ============================================================================
// @name Steinhart-Hart Coefficients
// @brief NTC thermistor temperature conversion constants
// @{
// ============================================================================

/** @brief Reference resistor value (10k Ohm). */
constexpr float R1  = 10000.0f;
/** @brief Steinhart-Hart coefficient A. */
constexpr float C1  = 0.001129148f;
/** @brief Steinhart-Hart coefficient B. */
constexpr float C2  = 0.000234125f;
/** @brief Steinhart-Hart coefficient C. */
constexpr float C3  = 0.0000000876741f;

/** @} */

/** @brief Running average sample count for sensor smoothing. */
constexpr uint8_t AVG_SAMPLES = 5;

// ============================================================================
// @name EEPROM Layout
// @brief Memory map for persistent user storage (4KB ATmega2560)
// @{
// ============================================================================

/**
 * @brief EEPROM layout:
 * @code{.txt}
 * 0x00: Magic byte (0xA5)
 * 0x01: User count
 * 0x02..0xF1: Users (10 x 24 bytes)
 * @endcode
 * Each user record = 24 bytes:
 * @code{.txt}
 * [PIN 4B][role 1B][uses 1B][active 1B][histIdx 1B][history 16B]
 * @endcode
 */

/** @brief EEPROM magic number address. */
constexpr uint8_t EEP_MAGIC       = 0;
/** @brief EEPROM user count address. */
constexpr uint8_t EEP_USER_COUNT  = 1;
/** @brief EEPROM first user record address. */
constexpr uint8_t EEP_USERS_START = 2;
/** @brief Each user record size (bytes). */
constexpr uint8_t EEP_USER_SIZE   = 24;
/** @brief Magic value to detect initialized EEPROM. */
constexpr uint8_t EEP_MAGIC_VAL   = 0xA5;
/** @brief Maximum number of stored users. */
constexpr uint8_t MAX_USERS       = 10;

/** @brief PIN offset within user record (4 bytes). */
constexpr uint8_t OFF_PIN      = 0;
/** @brief Role offset (1 byte). */
constexpr uint8_t OFF_ROLE     = 4;
/** @brief Uses counter offset (1 byte). */
constexpr uint8_t OFF_USES     = 5;
/** @brief Active flag offset (1 byte). */
constexpr uint8_t OFF_ACTIVE   = 6;
/** @brief History index offset (1 byte). */
constexpr uint8_t OFF_HIST_IDX = 7;
/**
 * @brief History buffer offset (16 bytes).
 * @details 4 previous PINs x 4 bytes each, circular.
 */
constexpr uint8_t OFF_HIST     = 8;

/** @} */

// ============================================================================
// @name Enumerations
// @brief FSM states, triggers, and role definitions
// @{
// ============================================================================

/**
 * @brief FSM states for the access control system.
 * @details 6-state finite state machine.
 */
enum State : uint8_t {
  S_IDLE,               /**< Idle. Waits for keypad input or menu. */
  S_OPEN,               /**< Access granted. Servo unlocked (2s). */
  S_BLOCKED,            /**< 3 failed attempts. LED blink (5s). */
  S_INTRUSION_MONITOR,  /**< Watching door + mic for intrusion. */
  S_ENV_MONITOR,        /**< Monitoring temperature + light. */
  S_ALARM               /**< Buzzer + LED alarm active. */
};

/**
 * @brief FSM transition triggers.
 * @details Set by events in `updateState()`, consumed by `StateMachineLib`.
 */
enum Trigger : uint8_t {
  TRIG_NONE,        /**< No pending trigger. */
  TRIG_AUTH_OK,     /**< Auth success OR state timer expired. */
  TRIG_LOCKOUT,     /**< 3 failed attempts reached. */
  TRIG_ENV_ALARM,   /**< Temperature/light threshold violated. */
  TRIG_INTRUSION    /**< Door or sound intrusion detected. */
};

/**
 * @brief User role access levels.
 * @details Per PRD section 6.1: Security, Operator, Coordinator, Manager.
 */
enum Role : uint8_t {
  ROLE_SECURITY   = 1, /**< Access to surveillance zones. */
  ROLE_OPERATOR   = 2, /**< Access to production/work zones. */
  ROLE_COORDINATOR = 3,  /**< Extended access + meeting rooms. */
  ROLE_MANAGER    = 4   /**< Full access + user management. */
};

/** @brief Human-readable role names (indexed by Role value). */
constexpr const char* ROLE_NAMES[5] = {
  "", "Security", "Operator", "Coordinator", "Manager"
};

/** @} */

// ============================================================================
// @name Global Objects
// @brief FSM, peripherals, and sensor instances
// @{
// ============================================================================

/**
 * @brief Finite State Machine instance.
 * @details 6 states, 10 transitions using StateMachineLib.
 */
StateMachine fsm(6, 10);

/** @brief Current FSM state, cached for display and logic. */
State currentState = S_IDLE;

/** @brief Keypad matrix dimensions. */
constexpr uint8_t KP_ROWS = 4, KP_COLS = 4;

/**
 * @brief Keypad character map (4x4).
 * @code{.txt}
 * ┌───┬───┬───┬───┐
 * │ 1 │ 2 │ 3 │ A │
 * ├───┼───┼───┼───┤
 * │ 4 │ 5 │ 6 │ B │
 * ├───┼───┼───┼───┤
 * │ 7 │ 8 │ 9 │ C │
 * ├───┼───┼───┼───┤
 * │ * │ 0 │ # │ D │
 * └───┴───┴───┴───┘
 * @endcode
 */
char keyMap[KP_ROWS][KP_COLS] = {
  {'1','2','3','A'}, {'4','5','6','B'},
  {'7','8','9','C'}, {'*','0','#','D'}
};

/** @brief Keypad library instance. */
Keypad keypad = Keypad(makeKeymap(keyMap),
  (byte*)ROW_PINS, (byte*)COL_PINS, KP_ROWS, KP_COLS);

/** @brief Servo motor for door lock. */
Servo doorServo;
/** @brief Servo lock position (degrees). */
constexpr uint8_t SRV_LOCKED   = 0;
/** @brief Servo unlock position (degrees). */
constexpr uint8_t SRV_UNLOCKED = 90;

/** @brief LCD 16x2 display (parallel 4-bit mode). */
LiquidCrystal lcd(LCD_RS, LCD_EN, LCD_D4, LCD_D5, LCD_D6, LCD_D7);

/** @brief Temperature running average (5 samples). */
RunningAverage tempAvg(AVG_SAMPLES);
/** @brief Light sensor running average (5 samples). */
RunningAverage lightAvg(AVG_SAMPLES);

/** @} */

// ============================================================================
// @name Global State Variables
// @brief Runtime state: buffers, counters, flags
// @{
// ============================================================================

/** @brief Current PIN entry buffer. */
char pinBuf[PIN_MAX_LEN + 1] = {0};
/** @brief Number of digits currently entered. */
uint8_t pinLen = 0;
/** @brief Timestamp of last keypress (for timeout detection). */
unsigned long pinStartTime = 0;

/** @brief Consecutive failed auth attempts. */
uint8_t failCount = 0;
/**
 * @brief Alarm event counter for triple detection.
 * @details Incremented on door OPEN events (interrupt) and loud sound (polled).
 * Cleared on state entry.
 */
uint8_t alarmCount = 0;
/**
 * @brief Timestamp of first alarm event in the T_TRIPLE window.
 * @see T_TRIPLE
 */
unsigned long firstAlarmTime = 0;

/** @brief Timestamp when the current FSM state was entered. */
unsigned long stateEntryTime = 0;

/**
 * @brief Pending FSM transition trigger.
 * @details Set by events in `updateState()`, consumed by `fsm.Update()`.
 * Critical: reset to TRIG_NONE on each state entry.
 */
Trigger trig = TRIG_NONE;

/**
 * @brief Door state change flag, set by ISR.
 * @details Volatile because it's modified in interrupt context.
 * - INTRUSION_MONITOR uses `hallDoorOpen` (level check), clears this flag.
 * - ALARM consumes this flag (edge-triggered) for door OPEN events only.
 * - Top of `updateState()` reads this flag but does NOT clear it.
 *
 * @see doorISR()
 */
volatile bool doorChanged = false;
/**
 * @brief Current door state.
 * @details `true` if door is open (pin HIGH, no magnet).
 * Updated from `doorChanged` at the top of `updateState()`.
 */
bool hallDoorOpen = false;
/** @brief Microphone raw ADC value (polled every 200ms). */
int micVal = 0;
/** @brief Current temperature in Celsius (computed from NTC average). */
float temperature = 0.0f;
/** @brief Current light level (ADC, averaged). */
int lightLevel = 0;

/** @brief Last LED blink toggle timestamp. */
unsigned long lastBlink = 0;
/** @brief Current LED state (on/off) for blink patterns. */
bool ledOn = false;
/** @brief Blink ON duration for current state. */
unsigned long blinkOnMs = 0;
/** @brief Blink OFF duration for current state. */
unsigned long blinkOffMs = 0;
/** @brief Whether blink pattern is active. */
bool blinkActive = false;

/** @brief Whether PIN change menu is active (within IDLE state). */
bool menuActive = false;
/** @brief Menu step: 0=old PIN, 1=new PIN, 2=confirm. */
uint8_t menuStep = 0;
/** @brief Menu PIN entry buffer. */
char menuBuf[PIN_MAX_LEN + 1] = {0};
/** @brief Menu buffer digit count. */
uint8_t menuBufLen = 0;
/** @brief User index changing PIN (0xFF = none). */
uint8_t menuUserIdx = 0xFF;

/** @brief Force PIN change flag, set when uses >= 4. */
bool pinChangeRequired = false;
/** @brief User index requiring PIN change (0xFF = none). */
uint8_t pinChangeUserIdx = 0xFF;

/** @brief Last LCD refresh timestamp. */
unsigned long lastLcdUpdate = 0;
/** @brief LCD refresh interval (ms). */
constexpr unsigned long LCD_INTERVAL = 250;

/** @} */

// ============================================================================
// @name Interrupt Service Routines
// @brief Hardware interrupt handlers
// @{
// ============================================================================

/**
 * @brief Door reed switch ISR.
 * @details
 * Fires on any CHANGE of D21 (INT2). Minimal ISR: only sets a volatile
 * flag. The main loop reads and processes the flag in `updateState()`.
 *
 * @par ISR Safety
 * - No `digitalRead()` inside ISR (too slow for interrupt context).
 * - No `Serial.print()` inside ISR (blocking).
 * - No `millis()` inside ISR (non-deterministic).
 *
 * @see doorChanged
 * @see PIN_HALL_DOOR
 */
void doorISR() {
  doorChanged = true;
}

/** @} */

// ============================================================================
// @name Forward Declarations
// @brief Function prototypes called from FSM handlers and main loop
// @{
// ============================================================================

/**
 * @brief Update the LCD display with current state info.
 * @details Called every LCD_INTERVAL ms from `updateState()`.
 * Shows state name, PIN entry (masked), sensor data, or alarm messages.
 */
void updateDisplay();

/**
 * @brief Read and process keypad input.
 * @details Only active in S_IDLE state. Routes to menu or PIN entry handlers.
 */
void processInput();

/**
 * @brief Main per-loop state logic.
 * @details
 * Processes interrupt flags, state timers, sensor thresholds,
 * LED blink patterns, LCD refresh. Sets `trig` for FSM transitions.
 */
void updateState();

/** @brief S_IDLE state entry handler. */
void onEnterIdle();
/** @brief S_IDLE state exit handler. */
void onLeaveIdle();
/** @brief S_OPEN state entry handler (unlocks door). */
void onEnterOpen();
/** @brief S_OPEN state exit handler (locks door). */
void onLeaveOpen();
/** @brief S_BLOCKED state entry handler (starts LED blink). */
void onEnterBlocked();
/** @brief S_BLOCKED state exit handler (stops LED blink). */
void onLeaveBlocked();
/** @brief S_INTRUSION_MONITOR state entry handler. */
void onEnterIntrusionMonitor();
/** @brief S_INTRUSION_MONITOR state exit handler. */
void onLeaveIntrusionMonitor();
/** @brief S_ENV_MONITOR state entry handler. */
void onEnterEnvMonitor();
/** @brief S_ENV_MONITOR state exit handler. */
void onLeaveEnvMonitor();
/** @brief S_ALARM state entry handler (activates buzzer + blink). */
void onEnterAlarm();
/** @brief S_ALARM state exit handler (deactivates buzzer + LED). */
void onLeaveAlarm();

/** @} */

// ============================================================================
// @name EEPROM Functions
// @brief Persistent memory read/write for user profiles
// @{
// ============================================================================

/**
 * @brief Initialize EEPROM with magic number if not already set.
 * @details
 * Writes EEP_MAGIC_VAL at address 0 if not present. Sets user count to 0.
 * Idempotent — safe to call on every boot.
 */
void initEEPROM() {
  if (EEPROM.read(EEP_MAGIC) != EEP_MAGIC_VAL) {
    EEPROM.write(EEP_MAGIC, EEP_MAGIC_VAL);
    EEPROM.write(EEP_USER_COUNT, 0);
  }
}

/**
 * @brief Get the number of registered users.
 * @return User count (0–10).
 */
uint8_t userCount() {
  return EEPROM.read(EEP_USER_COUNT);
}

/**
 * @brief Set the number of registered users.
 * @param [in] c  New user count.
 */
void setUserCount(uint8_t c) {
  EEPROM.write(EEP_USER_COUNT, c);
}

/**
 * @brief Load a user record from EEPROM.
 * @details
 * Reads the 24-byte user record at the given index.
 * Validates the loaded PIN: if first byte is not a digit, marks as inactive.
 *
 * @param [in]  idx      User index (0..MAX_USERS-1).
 * @param [out] pin      Destination PIN buffer (5 bytes, null-terminated).
 * @param [out] role     User role (1-4).
 * @param [out] uses     Access use counter.
 * @param [out] active   Whether the user account is enabled.
 * @param [out] histIdx  Circular history buffer index.
 * @param [out] history  4x4 history buffer (4 previous PINs).
 */
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

/**
 * @brief Save a user record to EEPROM.
 * @details
 * Uses `EEPROM.update()` for all writes to minimize wear on the EEPROM.
 * `update()` only writes if the value has changed.
 *
 * @param [in] idx      User index (0..MAX_USERS-1).
 * @param [in] pin      PIN buffer (5 bytes, null-terminated).
 * @param [in] role     User role (1-4).
 * @param [in] uses     Access use counter.
 * @param [in] active   Whether the user account is enabled.
 * @param [in] histIdx  Circular history buffer index.
 * @param [in] history  4x4 history buffer (4 previous PINs).
 */
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

/** @} */

// ============================================================================
// @name Access Validation
// @brief PIN lookup, uniqueness checks, and rotation
// @{
// ============================================================================

/**
 * @brief Find a user by PIN.
 * @details Iterates over all registered users and compares the PIN.
 * Skips inactive users.
 *
 * @param [in] pin  4-digit PIN string.
 * @return User index (0..MAX_USERS-1), or 0xFF if not found.
 *
 * @retval 0..9  User index.
 * @retval 0xFF  PIN not found or user inactive.
 */
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

/**
 * @brief Check if a new PIN is unique (not current or in history).
 * @details
 * Compares against the user's current PIN and all 4 history entries.
 * Prevents PIN reuse per security policy.
 *
 * @param [in] userIdx  User index.
 * @param [in] newPin   Proposed new PIN.
 * @return `true` if the PIN is unique, `false` if it was used before.
 *
 * @retval true  PIN is acceptable (not in current or history).
 * @retval false PIN matches current or a previous PIN.
 */
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

/**
 * @brief Rotate PIN: push current PIN into history, store new PIN.
 * @details
 * Implements the circular history buffer: stores the current PIN at
 * `hist[histIdx]`, advances the index, and saves the new PIN with
 * `uses = 0`.
 *
 * @param [in] userIdx  User index.
 * @param [in] newPin   New PIN to assign.
 */
void rotatePin(uint8_t userIdx, const char* newPin) {
  char sp[5]; uint8_t r, u, hi; bool a; char hist[4][4];
  loadUser(userIdx, sp, r, u, a, hi, hist);

  // Push current PIN into history at histIdx
  for (uint8_t i = 0; i < 4; i++) hist[hi][i] = sp[i];
  hi = (hi + 1) % PIN_HIST_LEN;

  // Save new PIN with uses=0
  saveUser(userIdx, newPin, r, 0, true, hi, (const char(*)[4])hist);
}

/**
 * @brief Validate a PIN and grant access if valid.
 * @details
 * Steps:
 * 1. Find user by PIN.
 * 2. Validate role range (1-4).
 * 3. Increment use counter.
 * 4. If uses >= PIN_MAX_USES, set `pinChangeRequired` flag.
 *
 * @param [in] pin  4-digit PIN string.
 * @return `true` if access granted, `false` otherwise.
 *
 * @retval true  PIN valid, access granted.
 * @retval false PIN not found or invalid role.
 */
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

/** @} */

// ============================================================================
// @name Actuator Helpers
// @brief Low-level hardware control wrappers
// @{
// ============================================================================

/** @brief Set red LED state. @param [in] on  `true` = ON, `false` = OFF. */
void setLED(bool on)     { digitalWrite(PIN_LED, on ? HIGH : LOW); }
/** @brief Set buzzer state. @param [in] on  `true` = ON, `false` = OFF. */
void setBuzzer(bool on)  { digitalWrite(PIN_BUZZER, on ? HIGH : LOW); }
/** @brief Unlock the door (servo to 90°). */
void unlockDoor()        { doorServo.write(SRV_UNLOCKED); }
/** @brief Lock the door (servo to 0°). */
void lockDoor()          { doorServo.write(SRV_LOCKED); }

/** @} */

// ============================================================================
// @name LCD Display
// @brief Per-state LCD output (16x2 character display)
// @{
// ============================================================================

/**
 * @brief Update the LCD with current state information.
 * @details
 * Called every LCD_INTERVAL ms. Clears display and shows:
 * - S_IDLE: "IDLE" + PIN entry (masked) or menu prompts
 * - S_OPEN: "ACCESS GRANTED" + countdown
 * - S_BLOCKED: "BLOCKED" + wait message
 * - S_ENV_MONITOR: temperature and light readings
 * - S_INTRUSION_MONITOR: "MONITOR SEC"
 * - S_ALARM: "!!! ALARM !!!" + intrusion message
 *
 * @note
 * Uses `F()` macro for all string literals to store them in flash
 * (PROGMEM) instead of SRAM.
 *
 * @see T_LCD_INTERVAL
 */
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

/** @} */

// ============================================================================
// @name FSM State Handlers
// @brief StateMachineLib on-enter and on-leave callbacks
// @{
// ============================================================================

/**
 * @brief S_IDLE state entry handler.
 * @details
 * Resets counters, clears buffers, turns off actuators, forces LCD refresh.
 * Sets the system to a clean idle state, ready for PIN entry.
 */
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

/**
 * @brief S_IDLE state exit handler.
 * @details Clears the PIN buffer and deactivates menu mode.
 */
void onLeaveIdle() {
  pinLen = 0; memset(pinBuf, 0, sizeof(pinBuf));
  menuActive = false;
}

/**
 * @brief S_OPEN state entry handler.
 * @details Unlocks the servo door lock and starts the 2-second countdown.
 */
void onEnterOpen() {
  currentState = S_OPEN;
  trig = TRIG_NONE;
  stateEntryTime = millis();
  unlockDoor();
  Serial.println(F("[STATE] OPEN — Door unlocked (2s)"));
}

/**
 * @brief S_OPEN state exit handler.
 * @details Locks the door servo and logs the event.
 */
void onLeaveOpen() {
  lockDoor();
  Serial.println(F("[STATE] OPEN — Door locked"));
}

/**
 * @brief S_BLOCKED state entry handler.
 * @details
 * Activates the slow LED blink pattern (300ms ON / 700ms OFF)
 * for the 5-second block duration.
 */
void onEnterBlocked() {
  currentState = S_BLOCKED;
  trig = TRIG_NONE;
  stateEntryTime = millis();
  blinkActive = true; blinkOnMs = BLK_ON; blinkOffMs = BLK_OFF;
  lastBlink = millis(); ledOn = false;
  Serial.println(F("[STATE] BLOCKED — 3 failed attempts, 5s block"));
}

/**
 * @brief S_BLOCKED state exit handler.
 * @details Stops the LED blink pattern and turns the LED off.
 */
void onLeaveBlocked() {
  blinkActive = false; setLED(LOW);
}

/**
 * @brief S_INTRUSION_MONITOR state entry handler.
 * @details Begins the 2-second watch window for door/sound intrusion.
 */
void onEnterIntrusionMonitor() {
  currentState = S_INTRUSION_MONITOR;
  trig = TRIG_NONE;
  stateEntryTime = millis();
  Serial.println(F("[STATE] INTRUSION_MONITOR — Watching..."));
}

/** @brief S_INTRUSION_MONITOR state exit handler (noop). */
void onLeaveIntrusionMonitor() {}

/**
 * @brief S_ENV_MONITOR state entry handler.
 * @details Begins the 4-second environmental monitoring window.
 */
void onEnterEnvMonitor() {
  currentState = S_ENV_MONITOR;
  trig = TRIG_NONE;
  stateEntryTime = millis();
  Serial.println(F("[STATE] ENV_MONITOR — Temp + light"));
}

/** @brief S_ENV_MONITOR state exit handler (noop). */
void onLeaveEnvMonitor() {}

/**
 * @brief S_ALARM state entry handler.
 * @details
 * Activates the buzzer (continuous) and fast LED blink (100ms ON / 500ms OFF)
 * for the 2-second alarm duration.
 */
void onEnterAlarm() {
  currentState = S_ALARM;
  trig = TRIG_NONE;
  stateEntryTime = millis();
  setBuzzer(true);
  blinkActive = true; blinkOnMs = ALM_ON; blinkOffMs = ALM_OFF;
  lastBlink = millis(); ledOn = false;
  Serial.println(F("[STATE] ALARM — Intrusion!"));
}

/**
 * @brief S_ALARM state exit handler.
 * @details Deactivates buzzer and LED, clears blink pattern.
 */
void onLeaveAlarm() {
  setBuzzer(false);
  blinkActive = false; setLED(LOW);
}

/** @} */

// ============================================================================
// @name Keypad Input Processing
// @brief PIN entry and menu navigation handlers
// @{
// ============================================================================

/**
 * @brief Handle keypad input during PIN change menu.
 * @details
 * Menu flow:
 * - Step 0: Enter current PIN for validation.
 * - Step 1: Enter new PIN (min 4, max 6 digits, check uniqueness).
 * - `*` key cancels at any step.
 * - `#` key confirms each step.
 *
 * @param [in] key  Keypad character ('0'-'9', '#', '*').
 */
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

/**
 * @brief Handle keypad input for PIN authentication.
 * @details
 * Collects digits until `#` confirms. On confirmation:
 * - If PIN is empty (pinLen == 0): opens the PIN change menu.
 * - If PIN length >= 4: validates via `validateAccess()`.
 * - On success: sets `trig = TRIG_AUTH_OK`.
 * - On failure: increments `failCount`. At >= 3: sets `TRIG_LOCKOUT`.
 * - `*` clears the entry buffer at any time.
 *
 * @param [in] key  Keypad character ('0'-'9', '#', '*').
 */
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

/**
 * @brief Main keypad input dispatcher.
 * @details
 * Only processes input in S_IDLE state. Reads one key from the keypad
 * library and routes to `handlePinEntry()` or `handleMenuKey()` depending on
 * whether the menu is active. Also checks the PIN input timeout (10s).
 */
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

/** @} */

// ============================================================================
// @name State Update
// @brief Core per-loop logic: interrupt processing, sensor thresholds, timing
// @{
// ============================================================================

/**
 * @brief Main per-loop state update logic.
 * @details
 * Called on every `loop()` iteration. Executes in order:
 *
 * 1. **Interrupt flag processing**: Reads `doorChanged` (set by ISR)
 *    and updates `hallDoorOpen` with the current pin state. Does NOT
 *    clear `doorChanged` — state-specific handlers consume it.
 *
 * 2. **State timing**: For each timed state (OPEN, BLOCKED, etc.),
 *    checks if the elapsed time exceeds the duration and sets
 *    `trig = TRIG_AUTH_OK` to trigger a transition.
 *
 * 3. **Sensor calculation**: Computes temperature (Steinhart-Hart)
 *    and light level from RunningAverage data.
 *
 * 4. **State-specific monitoring**:
 *    - ENV_MONITOR: checks temperature and light thresholds.
 *    - INTRUSION_MONITOR: checks door state (`hallDoorOpen`) and mic level.
 *    - ALARM: counts new door events (edge-triggered via `doorChanged`)
 *      and sound events (polled), detects triple-alarm condition.
 *
 * 5. **LED blink**: Updates the red LED according to the active blink
 *    pattern (BLOCKED or ALARM).
 *
 * 6. **LCD refresh**: Calls `updateDisplay()` at LCD_INTERVAL rate.
 *
 * @note
 * Door events in ALARM use edge-triggered detection (via `doorChanged`),
 * while INTRUSION_MONITOR uses level-based detection (`hallDoorOpen`).
 * This ensures ALARM only counts physical door OPEN events, not the
 * steady-state "door open" condition.
 */
void updateState() {
  unsigned long now = millis();

  // --- Process interrupt flags ---
  // Door ISR fires on CHANGE; update hallDoorOpen with current state.
  // NOTE: we READ doorChanged but do NOT clear it here — state-specific
  // handlers (e.g. ALARM) consume it for edge-triggered event counting.
  if (doorChanged) {
    hallDoorOpen = digitalRead(PIN_HALL_DOOR) == HIGH;
  }

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
      // Door state updated via interrupt (hallDoorOpen), mic via polling
      if (hallDoorOpen || micVal > SOUND_HIGH) {
        trig = TRIG_INTRUSION;
        if (hallDoorOpen) {
          doorChanged = false;  // Consume event so ALARM doesn't double-count
          Serial.println(F("[INTRUSION] Door opened!"));
        }
        if (micVal > SOUND_HIGH) Serial.println(F("[INTRUSION] Sound detected!"));
      }
      break;

    case S_ALARM:
      // Triple alarm detection within T_TRIPLE window
      // doorChanged is edge-triggered (ISR fires on CHANGE), so only
      // counts DOOR events when the door physically changes state
      if (doorChanged) {
        doorChanged = false;
        hallDoorOpen = digitalRead(PIN_HALL_DOOR) == HIGH;
        if (hallDoorOpen) {
          alarmCount++;
          if (alarmCount == 1) firstAlarmTime = now;
          Serial.print(F("[ALARM] Door event #"));
          Serial.println(alarmCount);
        }
      }
      // Mic is polled (level-based)
      if (micVal > SOUND_HIGH) {
        alarmCount++;
        if (alarmCount == 1) firstAlarmTime = now;
        Serial.print(F("[ALARM] Sound event #"));
        Serial.println(alarmCount);
      }
      if (alarmCount >= 3 && (now - firstAlarmTime) < T_TRIPLE) {
        Serial.println(F("[ALARM] Triple event!"));
        // Reset the alarm timer for extended block
        stateEntryTime = now;
        alarmCount = 0;
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

/** @} */

// ============================================================================
// @name FSM Configuration
// @brief StateMachineLib transition wiring
// @{
// ============================================================================

/**
 * @brief Configure the finite state machine.
 * @details
 * Registers all state entry/exit callbacks and establishes the 10
 * transitions between the 6 states. Transitions use lambda functions
 * that check the global `trig` variable.
 *
 * Transition table:
 * @code{.txt}
 * S_IDLE --[TRIG_AUTH_OK]--> S_OPEN
 * S_IDLE --[TRIG_LOCKOUT]--> S_BLOCKED
 * S_OPEN --[timer]--> S_IDLE
 * S_BLOCKED --[timer]--> S_IDLE
 * S_ENV_MONITOR --[TRIG_ENV_ALARM]--> S_ALARM
 * S_ENV_MONITOR --[timer]--> S_IDLE
 * S_INTRUSION_MONITOR --[TRIG_INTRUSION]--> S_ALARM
 * S_INTRUSION_MONITOR --[timer]--> S_IDLE
 * S_ALARM --[timer]--> S_INTRUSION_MONITOR
 * @endcode
 *
 * @note
 * `TRIG_AUTH_OK` is reused as the "timer expired" signal for timed states,
 * since auth success and timer expiry are mutually exclusive per state.
 *
 * @see Trigger
 */
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

/** @} */

// ============================================================================
// @name AsyncTask Configuration
// @brief Periodic sensor averaging task
// @{
// ============================================================================

/**
 * @brief Sensor read interval (ms).
 * @details AsyncTaskLib timer fires every 200ms to read analog sensors.
 */
constexpr unsigned long SENSOR_INTERVAL = 200;

/**
 * @brief Periodic sensor averaging task.
 * @details
 * Reads analog sensors and adds values to RunningAverage objects.
 * Runs every `SENSOR_INTERVAL` ms, auto-resetting.
 *
 * Sensors read:
 * - NTC thermistor (temperature, A1)
 * - LDR (light, A2)
 * - Microphone (sound, A0)
 *
 * @note
 * Door sensor (reed switch) is NOT read here — it uses hardware interrupt
 * on D21 (INT2) for instant detection.
 *
 * @see SENSOR_INTERVAL
 * @see tempAvg
 * @see lightAvg
 */
AsyncTask sensorTask(SENSOR_INTERVAL, true, []() {
  int ntc = analogRead(PIN_NTC);
  int ldr = analogRead(PIN_LDR);
  micVal  = analogRead(PIN_MIC);

  // Add to running averages
  tempAvg.add(ntc);
  lightAvg.add(ldr);
});

/** @} */

// ============================================================================
// @name System Initialization
// @brief Arduino setup() function
// @{
// ============================================================================

/**
 * @brief Arduino setup — system initialization.
 * @details
 * Initializes in order:
 * 1. Serial monitor (9600 baud).
 * 2. Output pins (LED, buzzer) to safe OFF state.
 * 3. Door reed switch on D21 with INPUT_PULLUP + attachInterrupt.
 * 4. Servo motor, locked position.
 * 5. LCD display (16x2, 4-bit parallel).
 * 6. EEPROM initialization (magic number check).
 * 7. AsyncTask sensor averaging task.
 * 8. FSM configuration and initial state (S_IDLE).
 *
 * After setup, the system is in S_IDLE state waiting for keypad input.
 */
void setup() {
  Serial.begin(9600);
  randomSeed(analogRead(PIN_MIC));

  // Pin modes
  pinMode(PIN_LED, OUTPUT); digitalWrite(PIN_LED, LOW);
  pinMode(PIN_BUZZER, OUTPUT); digitalWrite(PIN_BUZZER, LOW);

  // Door interrupt: reed switch on D21 (INT2), INPUT_PULLUP
  // Normally closed with magnet (door shut) = LOW
  // Open with no magnet (door opened) = HIGH
  pinMode(PIN_HALL_DOOR, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_HALL_DOOR), doorISR, CHANGE);

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

/** @} */

// ============================================================================
// @name Main Loop
// @brief Arduino loop() — execution entry point
// @{
// ============================================================================

/**
 * @brief Arduino main loop.
 * @details
 * Executes every frame:
 * 1. `processInput()`: Read and handle keypad input.
 * 2. `updateState()`: Process interrupts, timers, sensors, blink, LCD.
 * 3. `sensorTask.Update()`: Run async sensor averaging tasks.
 * 4. `fsm.Update()`: Evaluate FSM transitions.
 *
 * @note
 * All timing uses `millis()` for non-blocking operation. No `delay()`
 * calls outside of keypad debounce (handled by Keypad library).
 *
 * @section Loop Timing
 * - Keypad: checked every loop iteration.
 * - Sensors: read every 200ms (AsyncTaskLib).
 * - LCD: refresh every 250ms.
 * - State transitions: on trigger or timer expiry.
 * - Door events: interrupt-driven (microsecond response).
 */
void loop() {
  processInput();
  updateState();

  // Update AsyncTaskLib periodic tasks
  sensorTask.Update();

  // Update FSM (state transition checks)
  fsm.Update();
}

/** @} */
