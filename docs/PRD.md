# PRD — Access Control and Security System with Environmental Monitoring
**Course:** Arquitectura Computacional
**Faculty:** Electronic Engineering and Telecommunications
**University:** Universidad del Cauca
**Implementation:** v4_main.ino — Arduino Mega (ATmega2560)

---

## 1. Overview

The project consists of designing and implementing an embedded access control system with a smart lock for a company. The system manages personnel entry via numeric keypad, administers user profiles and roles, controls failed attempts, activates alarms for security events, and monitors environmental conditions (temperature, light, sound, magnetic field) to ensure thermal comfort and environmental safety.

The system is implemented on the **ATmega2560** microcontroller and organized as a **Finite State Machine (FSM)** with 6 states using StateMachineLib.

---

## 2. Objectives

- Control physical access to the facility via numeric keypad authentication.
- Manage user profiles with different physical access levels based on role.
- Detect and respond to intrusion events and unauthorized access.
- Monitor environmental conditions and act to maintain comfort.
- Store users, keys, and history persistently in EEPROM.
- Force key rotation after 4 uses, preventing reuse of previous keys.
- Detect triple alarm events (3 in 12s) and extend security response.

---

## 3. Hardware Pin Mapping

| Component | Pins | Function |
|---|---|---|
| ATmega2560 | — | Main microcontroller |
| Keypad 4x4 | Rows: D29,D31,D33,D35 / Cols: D37,D39,D41,D43 | Numeric entry + menu navigation |
| Servo Motor | D10 (PWM) | Lock mechanism (0° locked, 90° unlocked) |
| Buzzer (Piezo) | D9 | Alarm tone via tone() — 1kHz square wave |
| RGB LED | R=A3, G=A4, B=A5 | Status indicator (digital out on analog pins) |
| LCD 16x2 | RS=12, EN=11, D4=5, D5=4, D6=3, D7=2 | Status display (parallel 4-bit mode) |
| Reed Switch | D21 (INT2) | Door open/close detection via hardware interrupt |
| Sound Sensor KY-037 | A0 | Intrusion detection via microphone |
| NTC Thermistor KY-013 | A1 | Ambient temperature reading |
| Photoresistor KY-018 | A2 | Ambient light level reading (inverted: 1023-ADC) |
| EEPROM | Internal (4KB) | Persistent storage for users, keys, roles, history |

---

## 4. Software and Libraries

- **Language:** C/C++ (Arduino framework on ATmega2560)
- **StateMachineLib 1.0.0:** Finite State Machine (6 states, 11 transitions)
- **AsyncTaskLib 1.0.0:** Non-blocking periodic sensor reads (200ms interval)
- **RunningAverage 0.4.9:** 5-sample sliding window for NTC + LDR smoothing
- **EEPROM.h:** Persistent read/write of user profiles (10 users × 24 bytes)
- **Servo.h:** Servo motor control for the lock mechanism
- **LiquidCrystal.h:** LCD 16x2 control (parallel 4-bit)
- **Keypad 3.1.1:** Matrix keypad driver (4×4)

---

## 5. Finite State Machine (FSM)

The system operates with **6 states** and **11 transitions**:

### 5.1 States

| # | State (code) | Description |
|---|---|---|
| 1 | **S_IDLE** | Idle — waits for keypad input. LCD shows "IDLE / Enter PIN:". PIN entry, menu, or debug keys processed here. |
| 2 | **S_OPEN** | Access granted. Servo unlocks (90°, 2s). LCD shows "ACCESS GRANTED / {Role} {countdown}s". LED green. |
| 3 | **S_BLOCKED** | 3 failed attempts. Buzzer beep (200ms) + LED red blink 300/700ms (5s). LCD: "BLOCKED / Wait 5s..." |
| 4 | **S_INTRUSION_MONITOR** | Monitors door (reed switch D21 via INT2) and microphone (KY-037, ADC > 800). LCD: "MONITOR SEC / Door: OPEN/CLOSED". LED blue. |
| 5 | **S_ENV_MONITOR** | Monitors temperature (KY-013) and light (KY-018) with RunningAverage. LCD: "MONITOR ENV / T:{temp}C L:{light}". LED blue. |
| 6 | **S_ALARM** | Active alarm. Buzzer continuous 1kHz + LED red fast blink 100/500ms. LCD: "!!! ALARM !!! / Event #{N}". Triple alarm detection active. |

### 5.2 Complete Transition Table

| From | Condition | To |
|---|---|---|
| S_IDLE | Correct PIN + role validated | S_OPEN |
| S_IDLE | 3 failed attempts (failCount >= 3) | S_BLOCKED |
| S_IDLE | Key 'A' (debug) | S_ENV_MONITOR |
| S_IDLE | Key 'B' (debug) | S_INTRUSION_MONITOR |
| S_OPEN | Timer expired (T_UNLOCK = 2s) | S_IDLE |
| S_BLOCKED | Timer expired (T_LOCKOUT = 5s) | S_IDLE |
| S_ENV_MONITOR | Temp < 20°C OR Temp > 50°C OR Light > 900 (ADC) | S_ALARM |
| S_ENV_MONITOR | Light < 100 (ADC) — low light condition | S_IDLE |
| S_ENV_MONITOR | Timer expired (T_ENV_MONITOR = 4s) | S_IDLE |
| S_INTRUSION_MONITOR | Door opens (hallDoorOpen) OR Sound > 800 (ADC) | S_ALARM |
| S_INTRUSION_MONITOR | Timer expired (T_INTRUSION = 2s, no events) | S_IDLE |
| S_ALARM | Timer expired (T_ALARM = 2s) | S_INTRUSION_MONITOR |
| S_ALARM | 3 events in 12s (triple alarm) | Extended ALARM (timer reset) |

### 5.3 LED and Buzzer Timing

| State | Red LED | Green | Blue | Buzzer |
|---|---|---|---|---|
| S_IDLE | OFF | OFF | OFF | OFF |
| S_OPEN | OFF | ON | OFF | OFF |
| S_BLOCKED | Blink 300/700ms | OFF | OFF | 200ms beep (on entry) |
| S_INTRUSION_MONITOR | OFF | OFF | ON | OFF |
| S_ENV_MONITOR | OFF | OFF | ON | OFF |
| S_ALARM | Blink 100/500ms | OFF | OFF | ON (continuous 1kHz) |

---

## 6. User and Role Management

### 6.1 Roles and Physical Access Levels

| Role | Value | Physical Access |
|---|---|---|
| Security | 1 | Access to surveillance zones and main entrance |
| Operator | 2 | Access to production/work zones |
| Coordinator | 3 | Access to operational zones + meeting rooms |
| Manager | 4 | Full access to all areas |

### 6.2 Password Policy

- **Digits:** 4–6 digits per PIN.
- **Expiration:** Each PIN expires after **4 uses**; the system forces the user to change it.
- **History:** 4 previous PINs stored in EEPROM (circular buffer). New PIN cannot match any previous PIN.
- **Timeout:** PIN entry times out after 10 seconds of inactivity (buffer cleared).
- **Change Menu:** Accessible from S_IDLE by pressing `#` with no digits entered → Old PIN → New PIN → Confirm.

### 6.3 PIN Change Flow

1. From IDLE, press `#` (no digits) → enters menu mode.
2. **Step 0:** Enter current PIN for identity verification.
3. **Step 1:** Enter new PIN (4–6 digits).
4. `pinIsUnique()` checks new PIN against current PIN + 4 history entries.
5. `rotatePin()` saves old PIN to history, stores new PIN with uses=0.
6. **Confirmation:** LCD shows "PIN changed! / OK" for 1.5s, then returns to IDLE.
7. Press `*` at any step to cancel and return to IDLE.

### 6.4 Seed Users (first boot)

| PIN | Role |
|---|---|
| 1234 | Manager (4) |
| 5678 | Operator (2) |
| 9999 | Security (1) |

### 6.5 EEPROM Layout

```
0x00: Magic byte (0xA5) — initialization marker
0x01: User count
0x02–0xF1: 10 users × 24 bytes
  [PIN 4B][role 1B][uses 1B][active 1B][histIdx 1B][history 16B]
  history = 4 previous PINs × 4 bytes (circular buffer)
```

---

## 7. Environmental Monitoring

The **S_ENV_MONITOR** state uses **RunningAverage Library** (5 samples) to average sensor readings and avoid false activations.

| Variable | Sensor | Action Threshold |
|---|---|---|
| Temperature | KY-013 (A1) | < 20°C OR > 50°C → alarm |
| Light | KY-018 (A2) | < 100 ADC → return to IDLE (low light); > 900 ADC → alarm |
| Sound | KY-037 (A0) | > 800 ADC → intrusion detection |
| Door | Reed switch D21 (INT2) | CHANGE interrupt → immediate door open detection |

The sensor averaging task runs every **200ms** via AsyncTaskLib, reading all three analog sensors and feeding RunningAverage objects.

Temperature calculation uses the **Steinhart-Hart equation** with voltage divider inversion (1023 - ADC), clamped to sane values (25°C on error).

---

## 8. Display Output

### 8.1 LCD 16x2 (RS=12, EN=11, D4=5, D5=4, D6=3, D7=2)

| State | Line 1 | Line 2 |
|---|---|---|
| S_IDLE | `IDLE` | `Enter PIN:` or `**** #=ok` (4–6 masked digits) |
| S_IDLE (menu) | `CHANGE PIN` | `Old PIN:----` / `New PIN:----` / `Confirm:----` |
| S_OPEN | `ACCESS GRANTED` | `{Role} {N}s` (countdown) |
| S_BLOCKED | `BLOCKED` | `Wait 5s...` |
| S_ENV_MONITOR | `MONITOR ENV` | `T:{temp}C L:{light}` |
| S_INTRUSION_MONITOR | `MONITOR SEC` | `Door: OPEN` or `Door: CLOSED` |
| S_ALARM | `!!! ALARM !!!` | `Event #N` (alarm count) |

### 8.2 Serial Monitor (9600 baud)

- State transitions: `[STATE] IDLE — System ready`, `[STATE] ALARM — Event #2`
- Authentication: `[AUTH] Granted: user 1 role Operator`, `[AUTH] Failed attempt 2`
- Sensor debug every 500ms: `T=24.5C L=512 MIC=120 DOOR=CLOSED`
- Menu flow: `[MENU] Enter new PIN (4-6 digits):`, `[MENU] PIN changed successfully!`
- Input events: `[INPUT] Entering ENV_MONITOR`, `[INPUT] PIN timeout`

---

## 9. Triple Alarm Detection

The system implements a **triple alarm** security mechanism to prevent alarm fatigue:

- Each transition to S_ALARM increments `alarmCount` in `onEnterAlarm()`.
- The first alarm event records `firstAlarmTime = millis()`.
- If `alarmCount >= 3` **and** `(now - firstAlarmTime) < T_TRIPLE (12s)`:
  - Reset `alarmCount = 0`.
  - Reset `stateEntryTime = now` (extends alarm duration).
  - Buzzer and LED continue uninterrupted.
- This prevents rapid cycling between ALARM and INTRUSION_MONITOR after 3 intrusion events in quick succession.
- Normal 2s ALARM→INTRUSION_MONITOR cycle resumes when events are spaced >12s apart.

---

## 10. Functional Requirements

| ID | Requirement |
|---|---|
| RF-01 | The system accepts numeric keys of 4–6 digits via 4x4 matrix keypad. |
| RF-02 | The system validates the key against the value stored in EEPROM. |
| RF-03 | Allows maximum 3 failed attempts before activating S_BLOCKED. |
| RF-04 | Activates buzzer (1kHz) and red LED blink on alarm or block. |
| RF-05 | The servo motor unlocks for 2s (90°) and returns to S_IDLE. |
| RF-06 | The system automatically returns to S_IDLE after any completed action or timer. |
| RF-07 | Access time windows are defined per role in code (hardcoded). |
| RF-08 | Reed switch on D21 (INT2) monitors door via hardware interrupt (ISR + flag). |
| RF-09 | The system manages user registration and role assignment stored in EEPROM (10 max). |
| RF-10 | Keys are forcibly renewed every 4 uses; no reuse allowed (4-history buffer). |
| RF-11 | Sound sensor KY-037 detects intrusion events via noise threshold (ADC > 800). |
| RF-12 | Sensors KY-013 and KY-018 monitor temperature and light with RunningAverage (5 samples). |
| RF-13 | LCD shows status, masked PIN entry, countdown, environmental info, and alarm events. |
| RF-14 | Serial monitor logs all system events at 9600 baud. |
| RF-15 | 3 consecutive alarms in less than 12s activate extended alarm (triple alarm detection). |
| RF-16 | Roles define different physical access levels (Security/Operator/Coordinator/Manager). |
| RF-17 | PIN entry times out after 10 seconds of inactivity. |
| RF-18 | LCD updates every 250ms, sensors read every 200ms (AsyncTaskLib). |

---

## 11. Non-Functional Requirements

| ID | Requirement |
|---|---|
| NFR-01 | System must respond to keypad input within 200ms. |
| NFR-02 | Code is organized by FSM states with section comments. |
| NFR-03 | Code is documented with Doxygen-style comments. |
| NFR-04 | EEPROM uses `update()` instead of `write()` to minimize wear. |
| NFR-05 | RunningAverage library smooths noisy analog sensor readings. |
| NFR-06 | All timing uses `millis()` — no `delay()` except keypad library internal debounce. |
| NFR-07 | All string literals use `F()` macro for PROGMEM storage. |

---

## 12. Deliverables

| Deliverable | Description |
|---|---|
| Functional physical prototype | Assembled and operational hardware on ATmega2560 |
| Documented source code | Single-file C/C++ (1727 lines) with Doxygen comments |
| UML diagrams | Class diagram + 4 sequence diagrams (docs/UML/) |
| Written report | Document per instructor-provided format |

---

## 13. Pending Items

- Specific time windows per role (exact values to hardcode).
- Potentiometer contrast adjustment for LCD.
- Report format (pending delivery by instructor).

---

## Appendix A — Debug Keys

From S_IDLE, special keypad keys enable manual test transitions:

| Key | Action |
|---|---|
| `A` | Force transition to S_ENV_MONITOR |
| `B` | Force transition to S_INTRUSION_MONITOR |
| `#` (no digits) | Enter PIN change menu |
| `*` | Cancel current input / menu |
| `#` (with digits ≥4) | Confirm PIN entry |

## Appendix B — Keypad Layout

```
┌───┬───┬───┬───┐
│ 1 │ 2 │ 3 │ A │  (A = debug ENV_MONITOR)
├───┼───┼───┼───┤
│ 4 │ 5 │ 6 │ B │  (B = debug INTRUSION_MONITOR)
├───┼───┼───┼───┤
│ 7 │ 8 │ 9 │ C │
├───┼───┼───┼───┤
│ * │ 0 │ # │ D │  (* = cancel, # = confirm)
└───┴───┴───┴───┘
```
