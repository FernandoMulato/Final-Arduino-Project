# PRD — Access Control and Security System with Environmental Monitoring
**Course:** Arquitectura Computacional
**Faculty:** Electronic Engineering and Telecommunications
**University:** Universidad del Cauca

---

## 1. Overview

The project consists of designing and implementing an embedded access control system with a smart lock for a company. The system manages personnel entry via numeric keypad, administers user profiles and roles, controls failed attempts, activates alarms for security events, and monitors environmental conditions (temperature, light, sound, magnetic field) to ensure thermal comfort and environmental safety.

The system is implemented on the **ATmega2560** microcontroller and organized as a **Finite State Machine (FSM)** with 6 main states.

---

## 2. Objectives

- Control physical access to the facility via numeric keypad authentication.
- Manage user profiles with different physical access levels based on role.
- Detect and respond to intrusion events and unauthorized access.
- Monitor environmental conditions and act to maintain comfort.
- Store users, keys, and schedules persistently in EEPROM.
- Force key rotation after 4 uses, preventing reuse of previous keys.

---

## 3. Hardware

| Component | System Function |
|---|---|
| ATmega2560 | Main microcontroller |
| LCD Keypad Shield | Status display and numeric keypad input (replaces RFID) |
| Servo Motor | Simulates lock mechanism (opens/closes door) |
| Buzzer | Audible alarm for excessive attempts or intrusion |
| LED | Visual status indicator (red flashing per mode) |
| Sound Sensor KY-037 | Sound/intrusion detection via microphone |
| Magnetic Reed Switch (D21) | Door status monitoring via interrupt (D21, INT2) |
| Analog Temperature Sensor KY-013 | Ambient temperature reading |
| Photoresistor Module KY-018 | Ambient light level reading |
| Potentiometer | LCD contrast adjustment |
| EEPROM (internal ATmega2560) | Persistent storage for users, keys, roles, and schedules |

> **Note:** The RFID module present in the original FSM diagram was replaced by the LCD Keypad Shield's keypad as the authentication method.

---

## 4. Software and Libraries

- **Language:** C/C++ (Arduino framework on ATmega2560)
- **Average Library:** Running average calculation for analog sensor readings (temperature, light) to smooth values and avoid noisy readings.
- **EEPROM.h:** Persistent read/write of users, passwords, and schedules.
- **Servo.h:** Servo motor control for the lock mechanism.
- **LiquidCrystal.h:** LCD Keypad Shield control.

---

## 5. Finite State Machine (FSM)

The system operates with **6 states** and the following transitions:

### 5.1 States

| # | State | Description |
|---|---|---|
| 1 | **INICIO** | Initial state. Waits for keypad PIN entry. Shows prompt on LCD. |
| 2 | **CONFIG** | Correct PIN. Validates role, schedule, and profile. Activates servo (lock open for programmed time). Automatically returns to INICIO. |
| 3 | **BLOQUEO** | Activated after 3 failed attempts. System temporarily blocked (5 s). Red LED ON=300 ms / OFF=700 ms. |
| 4 | **MONITOR INTRUSOS** | Monitors microphone (KY-037) and Hall sensor (KY-035). Detects door opening without authorization or suspicious sound. |
| 5 | **MONITOR AMBIENTAL** | Monitors temperature (KY-013) and light (KY-018). Acts if temperature < 20°C or light < 100. |
| 6 | **ALARMA** | Activated on intrusion or 3 consecutive alarms in less than 12 s. Buzzer ON. Red LED ON=100 ms / OFF=500 ms. Configurable duration. |

### 5.2 Main Transitions

| From | Condition | To |
|---|---|---|
| INICIO | Correct PIN + valid role/schedule | CONFIG |
| INICIO | Wrong PIN (< 3 attempts) | INICIO (increments counter) |
| INICIO | 3 failed attempts | BLOQUEO |
| INICIO | `#` / `*` key | Menu actions (PIN change, management) |
| CONFIG | Unlock time expired (2 s) | INICIO |
| BLOQUEO | Block time expired (5 s) | INICIO |
| MONITOR INTRUSOS | Intrusion detected (door/microphone) | ALARMA |
| MONITOR INTRUSOS | 3 times without event | INICIO |
| MONITOR AMBIENTAL | Temp < 20°C or Light < 100 | Comfort action (actuator) |
| MONITOR AMBIENTAL | Time expired (4 s) | INICIO |
| ALARMA | 3 consecutive alarms < 12 s | System blocked (extended BLOQUEO) |
| ALARMA | Alarm time expired (2 s) | INICIO |

### 5.3 LED and Buzzer Timing

| State | Red LED | Buzzer |
|---|---|---|
| BLOQUEO | ON=300 ms, OFF=700 ms | OFF |
| ALARMA | ON=100 ms, OFF=500 ms | ON (continuous) |

---

## 6. User and Role Management

### 6.1 Roles and Physical Access Levels

| Role | Level | Physical Access |
|---|---|---|
| Security | 1 | Access to surveillance zones and main entrance |
| Operator | 2 | Access to production/work zones |
| Coordinator | 3 | Access to operational zones + meeting rooms |
| Manager | 4 | Full access to all areas |

> Different roles enable the servo (lock) for different physical zones of the system.

### 6.2 Password Policy

- Numeric key of **minimum 4 digits**.
- Each key expires after **4 uses**; the system forces the user to change it.
- The new key **cannot be the same** as any previous key (history stored in EEPROM).
- Key change is managed through the menu accessible via the `#` / `*` keys on the LCD Keypad.

### 6.3 Time Schedules

- Allowed access time slots per role are **defined in code (hardcoded)**.
- If a user attempts access outside their time window, access is denied even if the key is correct.

### 6.4 EEPROM Storage

- Users, keys (with history), roles, and time schedules are stored in the internal EEPROM of the ATmega2560.
- The exact number of storable users **depends on the EEPROM memory map** (4 KB on ATmega2560).

---

## 7. Environmental Monitoring

The **MONITOR AMBIENTAL** state uses the **RunningAverage Library** to average sensor readings and avoid false activations due to electrical noise.

| Variable | Sensor | Action Threshold |
|---|---|---|
| Temperature | KY-013 | < 20°C → activate heating/ventilation |
| Light | KY-018 | < 100 (ADC units) → activate lighting |
| Sound | KY-037 | Configurable threshold → intrusion detection |
| Magnetic field (door) | Reed switch D21 (INT2) | Interrupt (CHANGE) → immediate door open detection |

> The number of people present may influence thermal comfort thresholds (to be defined during implementation).

---

## 8. Display

System information is displayed simultaneously on **two channels**:

### 8.1 LCD Keypad Shield (16x2)

| Line | Content |
|---|---|
| Line 1 | System status: `IDLE`, `OPEN`, `ERR`, `ALR`, `BLK` |
| Line 2 | Entered digits (masked: `****`), countdown, error messages, environmental data |

LCD contrast is adjusted via the **potentiometer**.

### 8.2 Serial Monitor (Serial.print)

- Complete event log: access attempts, state changes, sensor readings, alerts.
- Useful for debugging and verification during development.

---

## 9. Functional Requirements

| ID | Requirement |
|---|---|
| RF-01 | The system accepts numeric keys of minimum 4 digits via LCD Keypad. |
| RF-02 | The system validates the key against the value stored in EEPROM. |
| RF-03 | Allows maximum 3 failed attempts before activating BLOQUEO. |
| RF-04 | Activates audible alarm (buzzer) and visual (fast red LED) on intrusion or block. |
| RF-05 | The servo motor unlocks the lock for a programmed time (2 s) and returns to INICIO state. |
| RF-06 | The system automatically returns to INICIO state after any completed action. |
| RF-07 | Access time windows are defined per role in code. |
| RF-08 | The reed switch on D21 (INT2) monitors door status via hardware interrupt (attachInterrupt). |
| RF-09 | The system manages user registration and role assignment stored in EEPROM. |
| RF-10 | Keys are forcibly renewed every 4 uses; no reuse allowed. |
| RF-11 | The sound sensor (KY-037) detects intrusion events via noise. |
| RF-12 | Sensors KY-013 and KY-018 monitor temperature and light with averaging (RunningAverage Library). |
| RF-13 | The LCD shows status, masked digits, countdown, and environmental information. |
| RF-14 | The serial monitor logs all system events. |
| RF-15 | 3 consecutive alarms in less than 12 s activate extended system block. |
| RF-16 | Roles define different physical access levels to company zones. |

---

## 10. Non-Functional Requirements

| ID | Requirement |
|---|---|
| NFR-01 | The system must respond to keypad input within 200 ms. |
| NFR-02 | Code must be modularized by FSM states. |
| NFR-03 | Code must be documented with comments in each function. |
| NFR-04 | EEPROM must not be written unnecessarily to preserve its lifespan. |
| NFR-05 | The system must be robust against noisy analog sensor readings (use RunningAverage Library). |

---

## 11. Deliverables

| Deliverable | Description |
|---|---|
| Functional physical prototype | Assembled and operational hardware on ATmega2560 |
| Documented source code | Modular C/C++ code with comments, delivered per course format |
| Written report | Document per instructor-provided format (pending attachment) |

---

## 12. Pending Items

- Maximum number of storable users in EEPROM (depends on memory map).
- Specific time windows per role (exact values to hardcode).
- Exact sound threshold for intrusion detection (KY-037).
- Number of people that modifies environmental comfort thresholds.
- Report format (pending delivery by instructor).
