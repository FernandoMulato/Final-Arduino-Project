# Access Control and Security System

> **Arquitectura Computacional** — Arduino Mega (ATmega2560)
> Academic project: smart lock access control system with
> environmental monitoring and intrusion detection.

---

## System Overview

The system implements a 6-state Finite State Machine (FSM) that manages
entry via 4x4 numeric keypad, validates credentials stored in EEPROM,
controls a servo-based door lock, detects intrusion via hall and sound
sensors, activates audiovisual alarms, and monitors environmental
conditions (temperature, light) with sensor averaging.

**Technical Specifications:**

| Parameter | Value |
|-----------|-------|
| Microcontroller | ATmega2560 (Arduino Mega) |
| Frequency | 16 MHz |
| SRAM | 8 KB (725 B used — 8.9%) |
| Flash | 248 KB (16.5 KB used — 6.5%) |
| EEPROM | 4 KB |
| Language | C++ (Arduino framework) |
| Libraries | StateMachineLib, AsyncTaskLib, Keypad 3.1.1, Servo, LiquidCrystal, RunningAverage |
| Files | 1 (`src/main.ino`, ~560 lines) |

---

## System Architecture

### Class Diagram

The following diagram shows the complete system structure: data,
state machine, peripherals, sensors, actuators, and the central controller.

| System Classes |
|----------------|
| ![Classes](docs/UML/Clases.png) |

**Package Legend:**
| Color | Meaning |
|-------|---------|
| 🟡 Yellow `#FFFACD` | Data structures (User) |
| 🔵 Blue `#B3D9FF` | State machine (State, Trigger, StateMachine) |
| 🔷 Light Blue `#D4E6F1` | Peripherals (Keypad, EEPROM, LiquidCrystal, AsyncTask) |
| 🟢 Green `#D5F5E3` | Sensors (NTC, LDR, Hall, Microphone) |
| 🔴 Pink `#FADBD8` | Actuators (Servo, LED, Buzzer) |
| 🟥 Red `#FFCCCC` | Central controller (System) |

---

## Finite State Machine (FSM)

The FSM is the architectural core. It implements **6 states** with
**9 transitions** using `StateMachineLib`.

### States

| # | State | Description | Timing |
|---|-------|-------------|--------|
| 1 | `S_INICIO` | Idle. Waits for keypad input or menu trigger. | PIN timeout 10s |
| 2 | `S_CONFIG` | Valid PIN + role in window. Servo unlocked. | 2s |
| 3 | `S_BLOQUEO` | 3 failed attempts. Red LED 300/700ms. | 5s |
| 4 | `S_MONITOR_INTRUSOS` | Hall + mic monitoring. | 2s |
| 5 | `S_MONITOR_AMBIENTAL` | Temp + light monitoring (RunningAverage). | 4s |
| 6 | `S_ALARMA` | Buzzer ON. Red LED 100/500ms. | 2s + triple rearm |

### Main Transitions

```
INICIO --(correct PIN + role in window)--> CONFIG --(2s)--> INICIO
INICIO --(3 failed attempts)--> BLOQUEO --(5s)--> INICIO
MONITOR_AMBIENTAL --(threshold)--> ALARMA --(2s)--> MONITOR_INTRUSOS
MONITOR_AMBIENTAL --(4s no event)--> INICIO
MONITOR_INTRUSOS --(hall/mic)--> ALARMA (triple rearm 12s)
MONITOR_INTRUSOS --(2s no event)--> INICIO
INICIO --(# without digits)--> Menu (PIN change, within INICIO)
```

---

## Sequence Diagrams

### Successful Entry (Happy Path)

Correct PIN + role in window → servo unlock (2s) → return to idle.

| Entry Sequence |
|----------------|
| ![EntrySuccess](docs/UML/IngresoExitoso.png) |

**Flow:**
1. User presses digits → stored in `pinBuf[]`
2. Confirms with `#` → validated via `findUserByPin()`
3. Role time window checked via `isInTimeWindow()`
4. Auth OK → transition to `S_CONFIG` → servo unlocked 2s
5. Timer expires → return to `S_INICIO`

---

### Intrusion and Alarm

Hall or microphone trigger → ALARMA (2s, buzzer + LED) → MONITOR_INTRUSOS → triple rearm.

| Alarm Sequence |
|----------------|
| ![IntrusionAlarm](docs/UML/IntrusionAlarma.png) |

**Flow:**
1. `hallVal > 512` (door open) or `micVal > 800` (sound)
2. Transition to `S_ALARMA` → buzzer ON + LED flash (100/500ms)
3. Each new detection increments `alarmCount`
4. 3 events within 12s (`T_TRIPLE`) → alarm timer reset
5. Timer expires → transition to `S_MONITOR_INTRUSOS` (2s)
6. No new events → return to `S_INICIO`

---

### Blocked State

3 wrong PINs → BLOQUEO (5s, LED 300/700ms) → restore.

| Block Sequence |
|----------------|
| ![BlockAttempts](docs/UML/BloqueoIntentos.png) |

**Flow:**
1. Each wrong PIN increments `failCount`
2. At 3 → `TRIG_LOCKOUT` → transition to `S_BLOQUEO`
3. LED blinks 300ms ON / 700ms OFF for 5s
4. Timer expires → return to `S_INICIO` with `failCount = 0`

---

### PIN Change Menu

From INICIO, press `#` with no digits → menu → change PIN with history check.

| Config Sequence |
|-----------------|
| ![ConfigUser](docs/UML/ConfigUsuario.png) |

**Flow:**
1. From `S_INICIO`, press `#` with empty pin → menu mode activated
2. Step 0: Enter current PIN → validated
3. Step 1: Enter new PIN (4-6 digits) → checked against history
4. New PIN saved → `rotatePin()` pushes old PIN to history, resets uses
5. `*` cancels at any step

---

## Code Structure

`src/main.ino` is organized by sections:

| Section | Lines | Content |
|---------|-------|---------|
| Header | 1-24 | Comments, description, transition diagram |
| Includes | 26-36 | Libraries: Arduino, StateMachineLib, AsyncTaskLib, Keypad, Servo, LiquidCrystal, RunningAverage, EEPROM |
| Pin Defs | 38-60 | Pin assignments (D2-D12, D22-D27, A0-A3) |
| Timings | 62-79 | Constants: T_UNLOCK, T_LOCKOUT, T_ENV_MONITOR, T_ALARM, etc. |
| Thresholds | 81-89 | TEMP_LOW/HIGH, LIGHT_MIN, SOUND_HIGH, HALL_OPEN |
| PIN/Pin History | 91-105 | Min/max digits, max uses, Steinhart-Hart, history length |
| EEPROM Layout | 107-122 | Address map: magic, count, users (24 bytes each, 10 max) |
| Enums | 124-145 | State (6), Trigger (4), Role (4) |
| Time Windows | 147-165 | Hardcoded access schedules per role |
| Global Objects | 167-195 | FSM, Keypad, Servo, LCD, RunningAverages, AsyncTask |
| State Variables | 197-228 | pinBuf, failCount, alarmCount, blink state, menu state |
| EEPROM Funcs | 232-285 | initEEPROM, loadUser, saveUser |
| Access Validation | 287-355 | findUserByPin, pinIsUnique, rotatePin, validateAccess |
| Actuator Helpers | 357-365 | setLED, setBuzzer, unlockDoor, lockDoor |
| LCD Display | 367-419 | updateDisplay with per-state formatting |
| FSM Handlers | 421-490 | onEnter/onLeave for each state |
| Input Processing | 492-559 | handleMenuKey, handlePinEntry, processInput |
| State Update | 561-619 | State timing, sensor monitoring, LED blink, LCD refresh |
| FSM Setup | 621-670 | setupFSM with all transitions |
| AsyncTask | 672-690 | Sensor averaging task (200ms) |
| setup() | 692-717 | Initialization: pins, servo, LCD, EEPROM, tasks, FSM |
| loop() | 719-728 | processInput → updateState → sensorTask.Update() → fsm.Update() |

---

## Pin Assignment

| Pin | Function | Type | Notes |
|-----|----------|------|-------|
| D2-D5 | Keypad Rows (4x4) | INPUT_PULLUP | Matrix rows |
| D6-D9 | Keypad Columns | INPUT | Matrix columns |
| D10 | RED_LED | OUTPUT | Alarm/block pattern |
| D11 | BUZZER | OUTPUT | Piezo, on in ALARMA |
| D12 | SERVO | OUTPUT | PWM, door lock |
| D22 | LCD_RS | OUTPUT | LCD register select |
| D23 | LCD_EN | OUTPUT | LCD enable |
| D24-D27 | LCD_D4-D7 | OUTPUT | LCD data lines (4-bit) |
| A0 | MICROPHONE | INPUT | KY-037 sound |
| A1 | NTC_THERMISTOR | INPUT | KY-013 temp |
| A2 | LDR | INPUT | KY-018 light |
| A3 | HALL | INPUT | KY-035 door sensor |

### Keypad Map

```
┌───┬───┬───┬───┐
│ 1 │ 2 │ 3 │ A │
├───┼───┼───┼───┤
│ 4 │ 5 │ 6 │ B │
├───┼───┼───┼───┤
│ 7 │ 8 │ 9 │ C │
├───┼───┼───┼───┤
│ * │ 0 │ # │ D │
└───┴───┴───┴───┘
```

**Functional keys:**
- `#` — Confirm PIN / Enter menu (no digits)
- `*` — Cancel / Exit menu
- `0-9` — PIN digits

---

## EEPROM Layout (4KB)

| Address | Content | Size |
|---------|---------|------|
| `0x00` | Magic number (`0xA5`) | 1 byte |
| `0x01` | User count | 1 byte |
| `0x02` - `0xF1` | Users (10 × 24 bytes) | 240 bytes |

**User record structure (24 bytes):**
```
[PIN 4B][role 1B][uses 1B][active 1B][histIdx 1B][history 16B]
```
- `role`: 1=Seguridad, 2=Operario, 3=Coordinador, 4=Gerente
- `uses`: counter, max 4 before PIN rotation
- `histIdx`: current index in circular history buffer
- `history`: 4 previous PINs (4 bytes each)

---

## Timing Constants

| Constant | Value | Purpose |
|----------|-------|---------|
| `T_UNLOCK` | 2 s | Servo unlock duration |
| `T_LOCKOUT` | 5 s | Blocked state duration |
| `T_ENV_MONITOR` | 4 s | Environmental monitor window |
| `T_ALARM` | 2 s | Alarm duration |
| `T_INTRUSION` | 2 s | Intrusion monitor window |
| `T_PIN_TIMEOUT` | 10 s | PIN input timeout |
| `T_TRIPLE` | 12 s | Triple alarm rearm window |
| `SENSOR_INTERVAL` | 200 ms | AsyncTask sensor read interval |
| `LCD_INTERVAL` | 250 ms | LCD refresh rate |

### LED Blink Patterns (Red LED)

| State | On | Off | Pattern |
|-------|----|-----|---------|
| `S_BLOQUEO` | 300 ms | 700 ms | Slow flash |
| `S_ALARMA` | 100 ms | 500 ms | Fast flash |

---

## Sensor Thresholds

| Sensor | Pin | Threshold | Normal | Alarm |
|--------|-----|-----------|--------|-------|
| Temperature (NTC) | A1 | < 20°C / > 50°C | 20-50°C | Outside range |
| Light (LDR) | A2 | < 100 ADC | ≥ 100 | < 100 |
| Hall (door) | A3 | > 512 ADC | ≤ 512 | > 512 (open) |
| Microphone | A0 | > 800 ADC | ≤ 800 | > 800 (sound) |

All analog sensors are read via `AsyncTaskLib` at 200ms intervals.
Temperature and light use `RunningAverage` library (5 samples) for smoothing.

---

## Security

### PIN Rotation Policy

Each user has a `uses` counter incremented on each successful entry.
When uses reaches 4:
1. Access is still granted for the 4th use
2. `pinChangeRequired` flag is set
3. The next authentication attempt with the same PIN is rejected
4. User must change PIN via the menu (`#` from IDLE with no digits)
5. New PIN is checked against current PIN + 4 previous PINs in history
6. Old PIN is pushed into circular history buffer on successful change

### Failed Attempt Policy

- Maximum 3 consecutive failed attempts
- At threshold: 5-second block (S_BLOQUEO) with LED 300/700ms
- After block: counter resets to 0

### User Roles

| Code | Role | Access Level |
|------|------|--------------|
| 1 | Seguridad | Full access |
| 2 | Operario | Production area (time-limited) |
| 3 | Coordinador | Extended access |
| 4 | Gerente | Full access + user management |

Time windows are hardcoded as `constexpr` tables. With an RTC module,
`isInTimeWindow()` validates access per role schedule.

---

## Dependencies

| Library | Version | Source | Purpose |
|---------|---------|--------|---------|
| StateMachineLib | 1.0.0 | Local (`lib/`) | Finite state machine |
| AsyncTaskLib | 1.0.0 | luisllamasbinaburo | Non-blocking sensor timing |
| Keypad | 3.1.1 | chris--a | 4x4 matrix keypad driver |
| RunningAverage | 0.4.9 | robtillaart | Analog sensor smoothing |
| Servo | 1.3.0 | arduino-libraries | Servo motor lock control |
| LiquidCrystal | 1.0.7 | arduino-libraries | 16x2 LCD display |
| EEPROM | built-in | Arduino | Persistent storage |

---

## Compilation and Build

```bash
scripts/build.sh build       # Compile
scripts/build.sh upload      # Compile + upload to board
scripts/build.sh run         # Upload + serial monitor
scripts/build.sh monitor     # Serial monitor (9600 baud)
scripts/build.sh clean       # Clean build files
scripts/build.sh size        # Show memory usage
scripts/build.sh deps        # Install dependencies
scripts/build.sh full        # clean + deps + build
scripts/build.sh -v build    # Verbose
```

**Memory usage (Arduino Mega):**
```
RAM:    725 bytes (8.9%) of 8192
Flash:  16494 bytes (6.5%) of 253952
```

---

## Project Files

```
Arduino-Project/
├── src/
│   └── main.ino              # Full implementation (~560 lines)
├── lib/
│   └── StateMachineLib/       # Local FSM library
├── spec/
│   ├── ARQ_Proyecto.pptx.pdf  # Course spec
│   └── fsm_arqB.drawio.pdf    # Original FSM diagram
├── docs/
│   ├── examples/              # Sensor example sketches
│   ├── PRD.md                 # Product Requirements Document
│   └── UML/                   # Generated diagrams
│       ├── Clases.png
│       ├── IngresoExitoso.png
│       ├── IntrusionAlarma.png
│       ├── BloqueoIntentos.png
│       ├── ConfigUsuario.png
│       ├── clases.puml
│       ├── secuencia-ingreso.puml
│       ├── secuencia-alarma.puml
│       ├── secuencia-bloqueo.puml
│       └── secuencia-config.puml
├── openspec/                  # Project specs and documentation
├── scripts/
│   └── build.sh              # Build script
├── platformio.ini             # PlatformIO config
├── AGENTS.md                  # Project guidelines and conventions
└── README.md                  # This file
```

---

## System Lifecycle

```
                    ┌──────────┐
                    │  INICIO  │ <────────────────────────┐
                    └────┬─────┘                          │
                         │ correct PIN + role             │
                         v                                │
                    ┌──────────┐                          │
                    │  CONFIG  │── 2s ─────────────────────┤
                    └──────────┘                          │
                                                          │
                    ┌──────────┐                          │
         ┌────────>│ BLOQUEO  │── 5s ─────────────────────┤
         │         └──────────┘                          │
         │                                                │
         │         ┌──────────────────┐                   │
         │         │ MONITOR AMBIENTAL│── 4s ─────────────┤
         │         └────────┬─────────┘                   │
         │                  │ threshold                   │
         │                  v                             │
         │         ┌──────────┐                           │
         │         │  ALARMA  │── 2s ─────────────────────┘
         │         └─────┬────┘         ┌──────────────────┐
         │               │ triple       │                  │
         │               v              v                  │
         │         ┌──────────────────┐                    │
         │         │ MONITOR INTRUSOS │── 2s ──────────────┘
         │         └───────┬──────────┘
         │                 │ hall/mic
         │                 v
         │         ┌──────────┐
         └─────────│  ALARMA  │ (rearm)
                   └──────────┘
```

---

*Document generated for Arquitectura Computacional — 2026*
