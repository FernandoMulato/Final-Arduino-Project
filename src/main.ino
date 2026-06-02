/**
 * @file v4_main.ino
 * @brief Sistema de Control de Acceso y Seguridad para Arduino Mega.
 *
 * @details
 * Versión v4 — Implementación hardware completa con StateMachineLib.
 * Proyecto académico para Arquitectura Computacional. Implementa una
 * Máquina de Estados Finita (FSM) de 6 estados con StateMachineLib.
 *
 * @par Transiciones
 * @code{.txt}
 * IDLE -> (PIN correcto + rol) -> OPEN -> (2s) -> IDLE
 * IDLE -> (3 fallos) -> BLOCKED -> (5s) -> IDLE
 * IDLE -> (tecla A debug) -> ENV_MONITOR
 * IDLE -> (tecla B debug) -> INTRUSION_MONITOR
 * ENV_MONITOR -> (umbral temp/light) -> ALARM -> (2s) -> INTRUSION_MONITOR
 * ENV_MONITOR -> (luz baja) -> IDLE
 * INTRUSION_MONITOR -> (hall/mic) -> ALARM (triple rearme 12s)
 * INTRUSION_MONITOR -> (2s sin evento) -> IDLE
 * ALARM -> (3 en 12s) -> bloqueo extendido -> INTRUSION_MONITOR
 * @endcode
 *
 * @par Hardware
 * Placa: Arduino Mega (ATmega2560)
 * - Cerradura: Servo motor (D13, PWM)
 * - Display: LCD 16x2 paralelo (RS=12, EN=11, D4=5, D5=4, D6=3, D7=2)
 * - LED RGB: R=A3, G=A4, B=A5
 * - Zumbador: D9 (tone())
 * - Sensor puerta: Reed switch en D21 (INT2, por interrupción)
 * - Micrófono: KY-037 en A0, NTC: KY-013 en A1, LDR: KY-018 en A2
 * - Teclado: Matriz 4x4 (filas D29,D31,D33,D35 / columnas D37,D39,D41,D43)
 * - RFID: MFRC522 (SS=53, RST=49)
 *
 * @par Memoria
 * SRAM: 846B (10.3%), Flash: 23KB (9.1%), EEPROM: 4KB interna
 *
 * @par Layout EEPROM
 * 10 usuarios x 24 bytes = 240 bytes
 * Registro de usuario: PIN[4] + rol + usos + activo + histIdx + historial[4][4]
 *
 * @par Librerías
 * StateMachineLib, AsyncTaskLib, Keypad 3.1.1, Servo,
 * LiquidCrystal, RunningAverage, EEPROM, MFRC522
 *
 * @author Arquitectura Computacional — Universidad del Cauca
 * @date 2026
 */

// ============================================================================
// @name Includes
// @brief Includes del sistema y librerías
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
#include <SPI.h>
#include <MFRC522.h>

/** @} */

// ============================================================================
// @name Definiciones de Pines
// @brief Asignación de pines hardware para el ATmega2560
// @{
// ============================================================================

/**
 * @brief Pines de filas del teclado (matriz 4x4).
 * @attention NO usar const/constexpr. La librería Keypad almacena el puntero
 * a este array. En AVR, const global va a FLASH y el puntero dereferencia
 * basura desde SRAM. Mantener en SRAM para que Keypad funcione.
 */
uint8_t ROW_PINS[4] = { 29, 31, 33, 35 };
/**
 * @brief Pines de columnas del teclado.
 * @attention Ídem ROW_PINS — debe estar en SRAM, no en FLASH.
 */
uint8_t COL_PINS[4] = { 37, 39, 41, 43 };

/** @brief Canal rojo del LED RGB (patrones de alarma/bloqueo). */
#define PIN_LED_R A3
/** @brief Canal verde del LED RGB (acceso concedido). */
#define PIN_LED_G A4
/** @brief Canal azul del LED RGB (monitoreo). */
#define PIN_LED_B A5

/** @brief Zumbador piezoeléctrico (alarma, mediante tone()). */
#define PIN_BUZZER 9
/** @brief Servo motor (cerradura de puerta, PWM). */
#define PIN_SERVO 13

/** @brief Pin RS del LCD. */
#define LCD_RS 12
/** @brief Pin Enable del LCD. */
#define LCD_EN 11
/** @brief Pin de datos D4 del LCD (modo 4 bits). */
#define LCD_D4 5
/** @brief Pin de datos D5 del LCD. */
#define LCD_D5 4
/** @brief Pin de datos D6 del LCD. */
#define LCD_D6 3
/** @brief Pin de datos D7 del LCD. */
#define LCD_D7 2

/** @brief Entrada analógica del sensor de sonido (KY-037). */
#define PIN_MIC A0
/** @brief Entrada analógica del termistor NTC (KY-013). */
#define PIN_NTC A1
/** @brief Entrada analógica de la fotorresistencia (KY-018). */
#define PIN_LDR A2

/**
 * @brief Reed switch de puerta (por interrupción).
 * @details
 * Conectado a D21 (INT2 en ATmega2560). Usa INPUT_PULLUP:
 * - Imán presente (puerta cerrada) = LOW
 * - Imán ausente (puerta abierta) = HIGH
 * La ISR dispara en CHANGE para detección instantánea.
 */
#define PIN_HALL_DOOR 21

#define PIN_RFID_SS 53
#define PIN_RFID_RST 49

/** @} */

// ============================================================================
// @name Constantes de Temporización (ms)
// @brief Duración de estados y tiempos de patrones LED según PRD
// @{
// ============================================================================

/** @brief Duración de desbloqueo del servo. */
#define T_UNLOCK 2000UL
/** @brief Duración del estado BLOCKED (PRD: 5s). */
#define T_LOCKOUT 5000UL
/** @brief Ventana de monitoreo ENV_MONITOR (PRD: 4s). */
#define T_ENV_MONITOR 10000UL
/** @brief Duración activa de ALARM (PRD: 2s). */
#define T_ALARM 2000UL
/** @brief Ventana de monitoreo INTRUSION_MONITOR. */
#define T_INTRUSION 10000UL
/** @brief Timeout de ingreso de PIN. */
#define T_PIN_TIMEOUT 10000UL
/**
 * @brief Ventana de detección de alarma triple.
 * @details 3 eventos de alarma en 12s disparan bloqueo extendido.
 */
#define T_TRIPLE 12000UL

/** @brief Tiempo encendido del LED en BLOCKED. */
#define BLK_ON 300UL
/** @brief Tiempo apagado del LED en BLOCKED. */
#define BLK_OFF 700UL
/** @brief Tiempo encendido del LED en ALARM. */
#define ALM_ON 100UL
/** @brief Tiempo apagado del LED en ALARM. */
#define ALM_OFF 500UL

/** @} */

// ============================================================================
// @name Umbrales de Sensores
// @brief Niveles de disparo analógicos para monitoreo ambiental
// @{
// ============================================================================

/** @brief Umbral bajo de temperatura (°C). */
#define TEMP_LOW 20.0f
/** @brief Umbral alto de temperatura (°C). */
#define TEMP_HIGH 50.0f
/** @brief Nivel mínimo de luz (unidades ADC). */
#define LIGHT_MIN 100
/** @brief Umbral de sonido fuerte (unidades ADC). */
#define SOUND_HIGH 800

/** @} */

// ============================================================================
// @name Constantes de Política de PIN
// @brief Configuración de la política de contraseñas
// @{
// ============================================================================

/** @brief Mínimo de dígitos del PIN. */
#define PIN_MIN_LEN 4
/** @brief Máximo de dígitos del PIN. */
#define PIN_MAX_LEN 6
/**
 * @brief Máximo de usos antes de rotación forzada.
 * @details Después de PIN_MAX_USES accesos exitosos, el usuario debe cambiar el PIN.
 */
#define PIN_MAX_USES 10
/** @brief Cantidad de PINs anteriores registrados (buffer circular). */
#define PIN_HIST_LEN 4

/** @} */

// ============================================================================
// @name Coeficientes Steinhart-Hart
// @brief Constantes de conversión de temperatura del termistor NTC
// @{
// ============================================================================

/** @brief Valor de la resistencia de referencia (10k Ohm). */
#define R1 10000.0f
/** @brief Coeficiente A de Steinhart-Hart. */
#define C1 0.001129148f
/** @brief Coeficiente B de Steinhart-Hart. */
#define C2 0.000234125f
/** @brief Coeficiente C de Steinhart-Hart. */
#define C3 0.0000000876741f

/** @} */

/** @brief Cantidad de muestras para el promedio móvil de sensores. */
#define AVG_SAMPLES 5

// ============================================================================
// @name Layout de EEPROM
// @brief Mapa de memoria para almacenamiento persistente de usuarios (4KB ATmega2560)
// @{
// ============================================================================

/**
 * @brief Layout de EEPROM:
 * @code{.txt}
 * 0x00: Byte mágico (0xA5)
 * 0x01: Cantidad de usuarios
 * 0x02..0xF1: Usuarios (10 x 24 bytes)
 * @endcode
 * Cada registro de usuario = 24 bytes:
 * @code{.txt}
 * [PIN 4B][rol 1B][usos 1B][activo 1B][histIdx 1B][historial 16B]
 * @endcode
 */

/** @brief Dirección del número mágico de EEPROM. */
#define EEP_MAGIC 0
/** @brief Dirección del contador de usuarios en EEPROM. */
#define EEP_USER_COUNT 1
/** @brief Dirección del primer registro de usuario en EEPROM. */
#define EEP_USERS_START 2
/** @brief Tamaño de cada registro de usuario (bytes). */
#define EEP_USER_SIZE 24
/** @brief Valor mágico para detectar EEPROM inicializada. */
#define EEP_MAGIC_VAL 0xA5
/** @brief Máximo de usuarios almacenables. */
#define MAX_USERS 10

/** @brief Offset del PIN dentro del registro de usuario (4 bytes). */
#define OFF_PIN 0
/** @brief Offset del rol (1 byte). */
#define OFF_ROLE 4
/** @brief Offset del contador de usos (1 byte). */
#define OFF_USES 5
/** @brief Offset del flag activo (1 byte). */
#define OFF_ACTIVE 6
/** @brief Offset del índice de historial (1 byte). */
#define OFF_HIST_IDX 7
/**
 * @brief Offset del buffer de historial (16 bytes).
 * @details 4 PINs anteriores x 4 bytes cada uno, circular.
 */
#define OFF_HIST 8

/** @} */

// ============================================================================
// @name Enumeraciones
// @brief Estados de la FSM, disparadores y definiciones de roles
// @{
// ============================================================================

/**
 * @brief Estados de la FSM para el sistema de control de acceso.
 * @details Máquina de estados finita de 6 estados.
 */
enum State : uint8_t {
  S_IDLE,              /**< Reposo. Espera entrada del teclado o menú. */
  S_OPEN,              /**< Acceso concedido. Servo desbloqueado (2s). */
  S_BLOCKED,           /**< 3 intentos fallidos. LED parpadea (5s). */
  S_INTRUSION_MONITOR, /**< Vigilando puerta + micrófono por intrusión. */
  S_ENV_MONITOR,       /**< Monitoreando temperatura + luz. */
  S_ALARM              /**< Alarma activa: zumbador + LED. */
};

/**
 * @brief Disparadores de transición de la FSM.
 * @details Se establecen por eventos en `updateState()`, son consumidos por `StateMachineLib`.
 */
enum Trigger : uint8_t {
  TRIG_NONE,
  TRIG_AUTH_OK,
  TRIG_TIMER_DONE,
  TRIG_LOCKOUT,
  TRIG_ENV_ALARM,
  TRIG_INTRUSION
};

/**
 * @brief Niveles de acceso por rol de usuario.
 * @details Según PRD sección 6.1: Security, Operator, Coordinator, Manager.
 */
enum Role : uint8_t {
  ROLE_SECURITY = 1,
  ROLE_OPERATOR = 2,
  ROLE_COORDINATOR = 3,
  ROLE_MANAGER = 4
};

/** @brief Nombres de rol legibles (indexados por valor de Role). */
const char* ROLE_NAMES[5] = {
  "", "Security", "Operator", "Coordinator", "Manager"
};

/** @} */

// ============================================================================
// @name RFID
// @brief Configuración del lector MFRC522 y tarjetas autorizadas
// @{
// ============================================================================

MFRC522 rfid(PIN_RFID_SS, PIN_RFID_RST);

const uint8_t RFID_CARDS[][4] = {
  { 0x3B, 0xCA, 0xCC, 0x05 },  // tarjeta 1 → ROLE_MANAGER
  { 0x81, 0xAA, 0xE0, 0x26 },  // tarjeta 2 → ROLE_OPERATOR
};
const uint8_t RFID_ROLES[] = { ROLE_MANAGER, ROLE_OPERATOR };
#define RFID_CARD_COUNT 2

/** @} */

// ============================================================================
// @name Objetos Globales
// @brief Instancias de FSM, periféricos y sensores
// @{
// ============================================================================

/**
 * @brief Instancia de la Máquina de Estados Finita.
 * @details 6 estados, 10 transiciones usando StateMachineLib.
 */
StateMachine fsm(6, 10);

/** @brief Estado actual de la FSM, cacheado para display y lógica. */
State currentState = S_IDLE;

/** @brief Dimensiones de la matriz del teclado. */
#define KP_ROWS 4
#define KP_COLS 4

/**
 * @brief Mapa de caracteres del teclado (4x4).
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
  { '1', '2', '3', 'A' }, { '4', '5', '6', 'B' }, { '7', '8', '9', 'C' }, { '*', '0', '#', 'D' }
};

/** @brief Instancia de la librería Keypad. */
Keypad keypad = Keypad(makeKeymap(keyMap),
                       (byte*)ROW_PINS, (byte*)COL_PINS, KP_ROWS, KP_COLS);

/** @brief Servo motor para la cerradura de puerta. */
Servo doorServo;
/** @brief Posición del servo: cerradura bloqueada (grados). */
#define SRV_LOCKED 0
/** @brief Posición del servo: cerradura desbloqueada (grados). */
#define SRV_UNLOCKED 90

/** @brief Display LCD 16x2 (modo paralelo 4 bits). */
LiquidCrystal lcd(LCD_RS, LCD_EN, LCD_D4, LCD_D5, LCD_D6, LCD_D7);

/** @brief Promedio móvil de temperatura (5 muestras). */
RunningAverage tempAvg(AVG_SAMPLES);
/** @brief Promedio móvil de luz (5 muestras). */
RunningAverage lightAvg(AVG_SAMPLES);

/** @} */

// ============================================================================
// @name Variables de Estado Globales
// @brief Estado en tiempo de ejecución: buffers, contadores, flags
// @{
// ============================================================================

/** @brief Buffer actual de ingreso de PIN. */
char pinBuf[PIN_MAX_LEN + 1] = { 0 };
/** @brief Cantidad de dígitos ingresados actualmente. */
uint8_t pinLen = 0;
/** @brief Timestamp de la última tecla presionada (para detección de timeout). */
unsigned long pinStartTime = 0;

/** @brief Intentos de autenticación fallidos consecutivos. */
uint8_t failCount = 0;

/** @brief Rol del último usuario autenticado (0 = ninguno). */
uint8_t lastAuthRole = 0;
/** @brief Índice del último usuario autenticado (0xFF = ninguno). */
uint8_t lastAuthUserIdx = 0xFF;

/**
 * @brief Contador de eventos de alarma para detección triple.
 * @details Se incrementa en eventos de apertura de puerta (interrupción) y sonido fuerte (sondeo).
 * Se limpia al entrar al estado.
 */
uint8_t alarmCount = 0;
/**
 * @brief Timestamp del primer evento de alarma en la ventana T_TRIPLE.
 */
unsigned long firstAlarmTime = 0;

/** @brief Timestamp de cuando se ingresó al estado actual de la FSM. */
unsigned long stateEntryTime = 0;

/**
 * @brief Disparador de transición de FSM pendiente.
 * @details Establecido por eventos en `updateState()`, consumido por `fsm.Update()`.
 * Crítico: reiniciar a TRIG_NONE en cada entrada de estado.
 */
Trigger trig = TRIG_NONE;

/**
 * @brief Flag de cambio de estado de puerta, establecido por la ISR.
 * @details Volátil porque se modifica en contexto de interrupción.
 * - INTRUSION_MONITOR usa `hallDoorOpen` (verificación de nivel), limpia este flag.
 * - ALARM consume este flag (disparado por flanco) solo para eventos de apertura.
 * - Al inicio de `updateState()` se lee este flag pero NO se limpia.
 */
volatile bool doorChanged = false;
/**
 * @brief Estado actual de la puerta.
 * @details `true` si la puerta está abierta (pin HIGH, sin imán).
 * Se actualiza desde `doorChanged` al inicio de `updateState()`.
 */
bool hallDoorOpen = false;
/** @brief Valor ADC del micrófono (sondeado cada 200ms). */
int micVal = 0;
/** @brief Temperatura actual en Celsius (calculada del promedio NTC). */
float temperature = 0.0f;
/** @brief Nivel de luz actual (ADC, promediado). */
int lightLevel = 0;

/** @brief Último timestamp de toggle del LED de parpadeo. */
unsigned long lastBlink = 0;
/** @brief Estado actual del LED (encendido/apagado) para patrones de parpadeo. */
bool ledOn = false;
/** @brief Duración de encendido del parpadeo para el estado actual. */
unsigned long blinkOnMs = 0;
/** @brief Duración de apagado del parpadeo para el estado actual. */
unsigned long blinkOffMs = 0;
/** @brief Indica si el patrón de parpadeo está activo. */
bool blinkActive = false;

/** @brief Indica si el menú de cambio de PIN está activo (dentro del estado IDLE). */
bool menuActive = false;
/** @brief Paso del menú: 0=PIN antiguo, 1=PIN nuevo, 2=confirmar. */
uint8_t menuStep = 0;
/** @brief Buffer de ingreso del menú. */
char menuBuf[PIN_MAX_LEN + 1] = { 0 };
/** @brief Cantidad de dígitos en el buffer del menú. */
uint8_t menuBufLen = 0;
/** @brief Índice del usuario que está cambiando el PIN (0xFF = ninguno). */
uint8_t menuUserIdx = 0xFF;

/** @brief Flag de cambio de PIN forzado, se activa cuando usos >= PIN_MAX_USES. */
bool pinChangeRequired = false;
/** @brief Índice del usuario que requiere cambio de PIN (0xFF = ninguno). */
uint8_t pinChangeUserIdx = 0xFF;

/** @brief Último timestamp de actualización del LCD. */
unsigned long lastLcdUpdate = 0;
/** @brief Intervalo de actualización del LCD (ms). */
#define LCD_INTERVAL 250UL

unsigned long buzzerOffTime = 0;
bool buzzerTimedOff = false;
unsigned long menuConfirmTime = 0;
bool menuConfirmPending = false;
unsigned long lastPageSwitch = 0;
bool displayPage = false;

/** @} */

// ============================================================================
// @name Rutinas de Servicio de Interrupción
// @brief Manejadores de interrupción hardware
// @{
// ============================================================================

/**
 * @brief ISR del reed switch de puerta.
 * @details
 * Se dispara en cualquier CHANGE de D21 (INT2). ISR mínima: solo establece un flag
 * volátil. El bucle principal lee y procesa el flag en `updateState()`.
 *
 * @par Seguridad ISR
 * - No usar `digitalRead()` dentro de la ISR (demasiado lento para contexto de interrupción).
 * - No usar `Serial.print()` dentro de la ISR (bloqueante).
 * - No usar `millis()` dentro de la ISR (no determinístico).
 */
void doorISR() {
  doorChanged = true;
}

/** @} */

// ============================================================================
// @name Declaraciones Adelantadas
// @brief Prototipos de funciones llamadas desde los manejadores de la FSM y el bucle principal
// @{
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

/** @} */

// ============================================================================
// @name Funciones de EEPROM
// @brief Lectura/escritura de memoria persistente para perfiles de usuario
// @{
// ============================================================================

/**
 * @brief Inicializa la EEPROM con el número mágico si no está configurada.
 * @details
 * Escribe EEP_MAGIC_VAL en la dirección 0 si no está presente. Establece el contador
 * de usuarios en 0. Idempotente — seguro de llamar en cada arranque.
 */
void initEEPROM() {
  if (EEPROM.read(EEP_MAGIC) != EEP_MAGIC_VAL) {
    EEPROM.write(EEP_MAGIC, EEP_MAGIC_VAL);
    EEPROM.write(EEP_USER_COUNT, 0);
    addUser("1234", ROLE_MANAGER);
    addUser("5678", ROLE_OPERATOR);
    addUser("9999", ROLE_SECURITY);
    Serial.println(F("[EEPROM] Test users loaded: 1234=Mgr, 5678=Op, 9999=Sec"));
  } else {
    Serial.println(F("[EEPROM] Existing users preserved"));
  }
}

/**
 * @brief Agrega un nuevo usuario a la EEPROM.
 * @param [in] pin  PIN de 4 dígitos.
 * @param [in] role Rol del usuario (1-4).
 */
void addUser(const char* pin, uint8_t role) {
  uint8_t n = userCount();
  if (n >= MAX_USERS) {
    Serial.println(F("[EEPROM] Max users reached"));
    return;
  }
  char hist[4][4];
  memset(hist, 0, sizeof(hist));
  char p[5];
  for (uint8_t i = 0; i < 4; i++) p[i] = pin[i];
  p[4] = '\0';
  saveUser(n, p, role, 0, true, 0, (const char(*)[4])hist);
  EEPROM.update(EEP_USER_COUNT, n + 1);
  Serial.print(F("[EEPROM] Added user "));
  Serial.print(n);
  Serial.print(F(" role "));
  Serial.println(ROLE_NAMES[role]);
}

/**
 * @brief Obtiene la cantidad de usuarios registrados.
 * @return Cantidad de usuarios (0–10).
 */
uint8_t userCount() {
  return EEPROM.read(EEP_USER_COUNT);
}

/**
 * @brief Establece la cantidad de usuarios registrados.
 * @param [in] c  Nueva cantidad de usuarios.
 */
void setUserCount(uint8_t c) {
  EEPROM.write(EEP_USER_COUNT, c);
}

/**
 * @brief Carga un registro de usuario desde la EEPROM.
 * @details
 * Lee el registro de 24 bytes en el índice dado.
 * Valida el PIN cargado: si el primer byte no es un dígito, se marca como inactivo.
 *
 * @param [in]  idx       Índice de usuario (0..MAX_USERS-1).
 * @param [out] pin       Buffer de destino del PIN (5 bytes, null-terminado).
 * @param [out] role      Rol del usuario (1-4).
 * @param [out] uses      Contador de usos de acceso.
 * @param [out] active    Indica si la cuenta de usuario está habilitada.
 * @param [out] histIdx   Índice del buffer circular de historial.
 * @param [out] history   Buffer de historial 4x4 (4 PINs anteriores).
 */
void loadUser(uint8_t idx, char pin[5], uint8_t& role, uint8_t& uses,
              bool& active, uint8_t& histIdx, char history[4][4]) {
  uint16_t addr = EEP_USERS_START + idx * EEP_USER_SIZE;
  for (uint8_t i = 0; i < 4; i++) pin[i] = EEPROM.read(addr + OFF_PIN + i);
  pin[4] = '\0';
  role = EEPROM.read(addr + OFF_ROLE);
  uses = EEPROM.read(addr + OFF_USES);
  active = EEPROM.read(addr + OFF_ACTIVE) != 0;
  histIdx = EEPROM.read(addr + OFF_HIST_IDX);
  for (uint8_t h = 0; h < PIN_HIST_LEN; h++)
    for (uint8_t i = 0; i < 4; i++)
      history[h][i] = EEPROM.read(addr + OFF_HIST + h * 4 + i);
  if (pin[0] < '0' || pin[0] > '9') active = false;
}

/**
 * @brief Guarda un registro de usuario en la EEPROM.
 * @details
 * Usa `EEPROM.update()` para todas las escrituras, minimizando el desgaste de la EEPROM.
 * `update()` solo escribe si el valor ha cambiado.
 *
 * @param [in] idx       Índice de usuario (0..MAX_USERS-1).
 * @param [in] pin       Buffer del PIN (5 bytes, null-terminado).
 * @param [in] role      Rol del usuario (1-4).
 * @param [in] uses      Contador de usos de acceso.
 * @param [in] active    Indica si la cuenta de usuario está habilitada.
 * @param [in] histIdx   Índice del buffer circular de historial.
 * @param [in] history   Buffer de historial 4x4 (4 PINs anteriores).
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
// @name Validación de Acceso
// @brief Búsqueda de PIN, verificación de unicidad y rotación
// @{
// ============================================================================

/**
 * @brief Busca un usuario por su PIN.
 * @details Itera sobre todos los usuarios registrados y compara el PIN.
 * Omite usuarios inactivos.
 *
 * @param [in] pin  PIN de 4 dígitos.
 * @return Índice de usuario (0..MAX_USERS-1), o 0xFF si no se encuentra.
 */
uint8_t findUserByPin(const char* pin) {
  uint8_t n = userCount();
  for (uint8_t i = 0; i < n; i++) {
    char sp[5];
    uint8_t r, u, hi;
    bool a;
    char hist[4][4];
    loadUser(i, sp, r, u, a, hi, hist);
    if (!a) continue;
    bool match = true;
    for (uint8_t j = 0; j < 4; j++) {
      if (pin[j] != sp[j]) {
        match = false;
        break;
      }
    }
    if (match) return i;
  }
  return 0xFF;
}

/**
 * @brief Verifica si un nuevo PIN es único (no es el actual ni está en el historial).
 * @details
 * Compara contra el PIN actual del usuario y las 4 entradas del historial.
 * Previene la reutilización de PIN según la política de seguridad.
 *
 * @param [in] userIdx  Índice de usuario.
 * @param [in] newPin   Nuevo PIN propuesto.
 * @return `true` si el PIN es único, `false` si ya fue usado antes.
 */
bool pinIsUnique(uint8_t userIdx, const char* newPin) {
  char sp[5];
  uint8_t r, u, hi;
  bool a;
  char hist[4][4];
  loadUser(userIdx, sp, r, u, a, hi, hist);

  bool same = true;
  for (uint8_t j = 0; j < 4; j++) {
    if (newPin[j] != sp[j]) {
      same = false;
      break;
    }
  }
  if (same) return false;

  for (uint8_t h = 0; h < PIN_HIST_LEN; h++) {
    if (hist[h][0] < '0' || hist[h][0] > '9') continue;
    same = true;
    for (uint8_t j = 0; j < 4; j++) {
      if (newPin[j] != hist[h][j]) {
        same = false;
        break;
      }
    }
    if (same) return false;
  }
  return true;
}

/**
 * @brief Rota el PIN: guarda el PIN actual en el historial y almacena el nuevo PIN.
 * @details
 * Implementa el buffer circular de historial: guarda el PIN actual en
 * `hist[histIdx]`, avanza el índice, y guarda el nuevo PIN con `usos = 0`.
 *
 * @param [in] userIdx  Índice de usuario.
 * @param [in] newPin   Nuevo PIN a asignar.
 */
void rotatePin(uint8_t userIdx, const char* newPin) {
  char sp[5];
  uint8_t r, u, hi;
  bool a;
  char hist[4][4];
  loadUser(userIdx, sp, r, u, a, hi, hist);

  for (uint8_t i = 0; i < 4; i++) hist[hi][i] = sp[i];
  hi = (hi + 1) % PIN_HIST_LEN;

  saveUser(userIdx, newPin, r, 0, true, hi, (const char(*)[4])hist);
}

/**
 * @brief Valida un PIN y concede acceso si es válido.
 * @details
 * Pasos:
 * 1. Buscar usuario por PIN.
 * 2. Validar rango de rol (1-4).
 * 3. Incrementar contador de usos.
 * 4. Si usos >= PIN_MAX_USES, establecer el flag `pinChangeRequired`.
 *
 * @param [in] pin  PIN de 4 dígitos.
 * @return `true` si el acceso es concedido, `false` en caso contrario.
 */
bool validateAccess(const char* pin) {
  uint8_t idx = findUserByPin(pin);
  if (idx == 0xFF) {
    Serial.println(F("[AUTH] PIN not found"));
    return false;
  }

  char sp[5];
  uint8_t role, uses, hi;
  bool a;
  char hist[4][4];
  loadUser(idx, sp, role, uses, a, hi, hist);

  if (role < 1 || role > 4) {
    Serial.println(F("[AUTH] Invalid role"));
    return false;
  }

  lastAuthRole = role;
  lastAuthUserIdx = idx;
  uses++;
  if (uses >= PIN_MAX_USES) {
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
// @name Funciones Auxiliares de Actuadores
// @brief Wrappers de control hardware de bajo nivel
// @{
// ============================================================================

/** @brief Establece el color del LED RGB. @param r Rojo, @param g Verde, @param b Azul. */
void setRGB(bool r, bool g, bool b) {
  digitalWrite(PIN_LED_R, r ? HIGH : LOW);
  digitalWrite(PIN_LED_G, g ? HIGH : LOW);
  digitalWrite(PIN_LED_B, b ? HIGH : LOW);
}
/** @brief Apaga el LED RGB y detiene el parpadeo. */
void ledOff() {
  setRGB(false, false, false);
  blinkActive = false;
}
/** @brief Rojo fijo. */
void ledRed() {
  setRGB(true, false, false);
  blinkActive = false;
}
/** @brief Verde fijo. */
void ledGreen() {
  setRGB(false, true, false);
  blinkActive = false;
}
/** @brief Azul fijo. */
void ledBlue() {
  setRGB(false, false, true);
  blinkActive = false;
}
/**
 * @brief Establece el estado del zumbador usando tone().
 * @param on `true` = tono de 1kHz, `false` = apagado.
 */
void setBuzzer(bool on) {
  if (on) tone(PIN_BUZZER, 1000);
  else noTone(PIN_BUZZER);
}
/** @brief Desbloquea la puerta (servo a 90°). */
void unlockDoor() {
  doorServo.write(SRV_UNLOCKED);
}
/** @brief Bloquea la puerta (servo a 0°). */
void lockDoor() {
  doorServo.write(SRV_LOCKED);
}

/**
 * @brief Actualiza el patrón de parpadeo del LED RGB.
 * @details Se llama desde updateState(). Parpadea solo el canal rojo.
 * En BLOCKED el buzzer se sincroniza con el LED.
 */
void updateBlinkPattern() {
  if (!blinkActive) return;
  unsigned long now = millis();
  unsigned long interval = ledOn ? blinkOffMs : blinkOnMs;
  if (now - lastBlink >= interval) {
    lastBlink = now;
    ledOn = !ledOn;
    digitalWrite(PIN_LED_R, ledOn ? HIGH : LOW);
    if (currentState == S_BLOCKED) {
      if (ledOn) tone(PIN_BUZZER, 1000);
      else noTone(PIN_BUZZER);
    }
  }
}

/** @} */

// ============================================================================
// @name Display LCD
// @brief Salida al LCD por estado (display de caracteres 16x2)
// @{
// ============================================================================

/**
 * @brief Actualiza el LCD con la información del estado actual.
 * @details
 * Se llama cada LCD_INTERVAL ms. Limpia el display y muestra según el estado.
 * No hay case para S_ALARM — el LCD no se actualiza en alarma para evitar
 * latencia en el parpadeo del LED (la alarma dura 2s).
 */
void updateDisplay() {
  lcd.clear();
  lcd.setCursor(0, 0);
  switch (currentState) {
    case S_IDLE:
      {
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
    case S_OPEN:
      {
        lcd.print(F("ACCESS GRANTED"));
        unsigned long elapsed = millis() - stateEntryTime;
        unsigned long rem = (elapsed < T_UNLOCK) ? (T_UNLOCK - elapsed) / 1000 : 0;
        lcd.setCursor(0, 1);
        if (lastAuthRole > 0 && lastAuthRole < 5) {
          lcd.print(ROLE_NAMES[lastAuthRole]);
          lcd.print(' ');
        }
        lcd.print(rem);
        lcd.print('s');
        break;
      }
    case S_BLOCKED:
      lcd.print(F("BLOCKED"));
      lcd.setCursor(0, 1);
      lcd.print(F("Wait 5s..."));
      break;
    case S_ENV_MONITOR:
      {
        lcd.print(F("MONITOR ENV"));
        lcd.setCursor(0, 1);
        lcd.print(F("T:"));
        lcd.print((int)temperature);
        lcd.print(F("C L:"));
        lcd.print(lightLevel);
        lcd.print(F("  "));
        break;
      }
    case S_INTRUSION_MONITOR:
      {
        if (millis() - lastPageSwitch >= 1000) {
          lastPageSwitch = millis();
          displayPage = !displayPage;
        }

        if (!displayPage) {
          lcd.print(F("T:"));
          lcd.print((int)temperature);
          lcd.print(F("C "));
          lcd.print(hallDoorOpen ? F("DOOR:OPEN ") : F("DOOR:CLSD "));
          lcd.setCursor(0, 1);
          lcd.print(F("L:"));
          lcd.print(lightLevel);
          lcd.print(F(" MIC:"));
          lcd.print(micVal);
        } else {
          lcd.print(F("MONITOR SEC"));
          lcd.setCursor(0, 1);
          lcd.print(hallDoorOpen ? F("** OPEN  **") : F("Door: CLOSED"));
        }
        break;
      }
  }
}

/**
 * @brief Verifica si hay una tarjeta RFID presente y si está autorizada.
 * @return Rol (1-4) si es válida, 0 si no.
 */
uint8_t checkRFID() {
  if (!rfid.PICC_IsNewCardPresent()) return 0;
  if (!rfid.PICC_ReadCardSerial()) return 0;

  uint8_t* uid = rfid.uid.uidByte;
  Serial.print(F("[RFID] UID: "));
  for (uint8_t i = 0; i < 4; i++) {
    Serial.print(uid[i], HEX);
    Serial.print(' ');
  }
  Serial.println();

  for (uint8_t c = 0; c < RFID_CARD_COUNT; c++) {
    bool match = true;
    for (uint8_t i = 0; i < 4; i++) {
      if (uid[i] != RFID_CARDS[c][i]) {
        match = false;
        break;
      }
    }
    if (match) {
      rfid.PICC_HaltA();
      Serial.print(F("[RFID] Granted — role: "));
      Serial.println(ROLE_NAMES[RFID_ROLES[c]]);
      return RFID_ROLES[c];
    }
  }

  rfid.PICC_HaltA();
  Serial.println(F("[RFID] Card not recognized"));
  return 0;
}

/** @} */

// ============================================================================
// @name Manejadores de Estado de la FSM
// @brief Callbacks de entrada y salida de StateMachineLib
// @{
// ============================================================================

/**
 * @brief Manejador de entrada al estado S_IDLE.
 * @details
 * Reinicia contadores, limpia buffers, apaga actuadores, fuerza actualización del LCD.
 * Deja el sistema en un estado limpio de reposo, listo para ingreso de PIN.
 */
void onEnterIdle() {
  currentState = S_IDLE;
  failCount = 0;
  pinLen = 0;
  memset(pinBuf, 0, sizeof(pinBuf));
  trig = TRIG_NONE;
  menuActive = false;
  menuStep = 0;
  menuBufLen = 0;
  menuUserIdx = 0xFF;
  ledOff();
  setBuzzer(false);
  lockDoor();
  lastLcdUpdate = 0;
  Serial.println(F("[STATE] IDLE — System ready"));
}

/**
 * @brief Manejador de salida del estado S_IDLE.
 * @details Limpia el buffer de PIN y desactiva el modo menú.
 */
void onLeaveIdle() {
  pinLen = 0;
  memset(pinBuf, 0, sizeof(pinBuf));
  menuActive = false;
}

/**
 * @brief Manejador de entrada al estado S_OPEN.
 * @details Desbloquea el servo de la cerradura e inicia la cuenta regresiva de 2 segundos.
 */
void onEnterOpen() {
  currentState = S_OPEN;
  trig = TRIG_NONE;
  stateEntryTime = millis();
  ledGreen();
  setBuzzer(false);
  unlockDoor();
  Serial.println(F("[STATE] OPEN — Door unlocked (2s)"));
}

/**
 * @brief Manejador de salida del estado S_OPEN.
 * @details Bloquea el servo de la puerta y registra el evento.
 */
void onLeaveOpen() {
  lockDoor();
  Serial.println(F("[STATE] OPEN — Door locked"));
}

/**
 * @brief Manejador de entrada al estado S_BLOCKED.
 * @details
 * Activa el patrón de parpadeo lento del LED (300ms ON / 700ms OFF)
 * durante la duración del bloqueo de 5 segundos. El LED arranca encendido
 * para feedback inmediato. El zumbador se sincroniza con el LED
 * (manejado en updateBlinkPattern()).
 */
void onEnterBlocked() {
  currentState = S_BLOCKED;
  trig = TRIG_NONE;
  stateEntryTime = millis();

  digitalWrite(PIN_LED_G, LOW);
  digitalWrite(PIN_LED_B, LOW);
  digitalWrite(PIN_LED_R, HIGH);
  blinkActive = true;
  blinkOnMs = BLK_ON;
  blinkOffMs = BLK_OFF;
  lastBlink = millis();
  ledOn = true;
  Serial.println(F("[STATE] BLOCKED — 3 failed attempts, 5s block"));
}

/**
 * @brief Manejador de salida del estado S_BLOCKED.
 * @details Detiene el patrón de parpadeo y apaga el zumbador.
 */
void onLeaveBlocked() {
  ledOff();
  setBuzzer(false);
}

/**
 * @brief Manejador de entrada al estado S_INTRUSION_MONITOR.
 * @details Inicia la ventana de vigilancia para detección de intrusión
 * por puerta (reed switch) o sonido fuerte (micrófono KY-037).
 * Resetea contadores de alarma y flags de puerta.
 */
void onEnterIntrusionMonitor() {
  currentState = S_INTRUSION_MONITOR;
  trig = TRIG_NONE;
  stateEntryTime = millis();
  ledBlue();
  setBuzzer(false);
  alarmCount = 0;
  hallDoorOpen = false;
  doorChanged = false;
  lastPageSwitch = millis();
  displayPage = false;
  Serial.println(F("[STATE] INTRUSION_MONITOR — Watching..."));
}

/** @brief Manejador de salida de INTRUSION_MONITOR (sin acción requerida). */
void onLeaveIntrusionMonitor() {}

/**
 * @brief Manejador de entrada al estado S_ENV_MONITOR.
 * @details Inicia la ventana de monitoreo ambiental. Lee temperatura
 * (NTC + Steinhart-Hart) y nivel de luz (LDR). La salida se dispara
 * al superar umbrales (TEMP_LOW, TEMP_HIGH, LIGHT_MIN) o por tiempo.
 */
void onEnterEnvMonitor() {
  currentState = S_ENV_MONITOR;
  trig = TRIG_NONE;
  stateEntryTime = millis();
  ledBlue();
  setBuzzer(false);
  alarmCount = 0;
  Serial.println(F("[STATE] ENV_MONITOR — Temp + light"));
}

/** @brief Manejador de salida de ENV_MONITOR (sin acción requerida). */
void onLeaveEnvMonitor() {}

/**
 * @brief Manejador de entrada al estado S_ALARM.
 * @details
 * Activa el zumbador (continuo) y el parpadeo rápido del LED
 * (100ms ON / 500ms OFF). El LED arranca encendido para feedback inmediato.
 * Incrementa el contador de alarma y registra el timestamp de la
 * primera alarma para detección triple (3 eventos en 12s = bloqueo extendido).
 */
void onEnterAlarm() {
  currentState = S_ALARM;
  trig = TRIG_NONE;
  stateEntryTime = millis();
  setBuzzer(true);
  digitalWrite(PIN_LED_G, LOW);
  digitalWrite(PIN_LED_B, LOW);
  digitalWrite(PIN_LED_R, HIGH);
  blinkActive = true;
  blinkOnMs = ALM_ON;
  blinkOffMs = ALM_OFF;
  lastBlink = millis();
  ledOn = true;
  alarmCount++;
  if (alarmCount == 1) firstAlarmTime = millis();
  Serial.print(F("[STATE] ALARM — Event #"));
  Serial.println(alarmCount);
}

/**
 * @brief Manejador de salida del estado S_ALARM.
 * @details Desactiva el zumbador y el LED, limpia el patrón de parpadeo.
 */
void onLeaveAlarm() {
  setBuzzer(false);
  ledOff();
}

/** @} */

// ============================================================================
// @name Procesamiento de Entrada del Teclado
// @brief Manejadores de ingreso de PIN y navegación del menú
// @{
// ============================================================================

/** @brief Cierra el menú de cambio de PIN y reinicia buffers al estado inicial. */
void closeMenu() {
  menuActive = false;
  menuStep = 0;
  menuBufLen = 0;
  menuUserIdx = 0xFF;
  memset(menuBuf, 0, sizeof(menuBuf));
  pinLen = 0;
  memset(pinBuf, 0, sizeof(pinBuf));
}

/**
 * @brief Maneja la entrada del teclado durante el menú de cambio de PIN.
 * @details
 * Flujo del menú:
 * - Paso 0: Ingresar el PIN actual para validación.
 * - Paso 1: Ingresar el PIN nuevo (mín. 4, máx. 6 dígitos, verificar unicidad).
 * - Tecla `*` cancela en cualquier paso.
 * - Tecla `#` confirma cada paso.
 *
 * @param [in] key  Carácter del teclado ('0'-'9', '#', '*').
 */
void handleMenuKey(char key) {
  if (key == '*') {
    closeMenu();
    Serial.println(F("[MENU] Cancelled"));
    return;
  }

  if (key == '#') {
    if (menuStep == 0) {
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
      menuStep = 1;
      menuBufLen = 0;
      memset(menuBuf, 0, sizeof(menuBuf));
      Serial.println(F("[MENU] Enter new PIN (4-6 digits):"));
    } else if (menuStep == 1) {
      menuBuf[menuBufLen] = '\0';
      if (menuBufLen < PIN_MIN_LEN) {
        Serial.println(F("[MENU] Too short (min 4)"));
        menuBufLen = 0;
        return;
      }
      if (!pinIsUnique(menuUserIdx, menuBuf)) {
        Serial.println(F("[MENU] PIN was used before. Choose another."));
        menuBufLen = 0;
        return;
      }
      rotatePin(menuUserIdx, menuBuf);
      if (pinChangeRequired && pinChangeUserIdx == menuUserIdx) {
        pinChangeRequired = false;
        pinChangeUserIdx = 0xFF;
      }
      Serial.println(F("[MENU] PIN changed successfully!"));
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print(F("PIN changed!"));
      lcd.setCursor(0, 1);
      lcd.print(F("OK"));
      menuConfirmTime = millis();
      menuConfirmPending = true;
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
 * @brief Maneja la entrada del teclado para autenticación por PIN.
 * @details
 * Acumula dígitos hasta que `#` confirma. Al confirmar:
 * - Si el PIN está vacío (pinLen == 0): abre el menú de cambio de PIN.
 * - Si la longitud del PIN >= 4: valida mediante `validateAccess()`.
 * - En caso exitoso: establece `trig = TRIG_AUTH_OK`.
 * - En caso fallido: incrementa `failCount`. Al llegar a >= 3: establece `TRIG_LOCKOUT`.
 * - `*` limpia el buffer de ingreso en cualquier momento.
 *
 * @param [in] key  Carácter del teclado ('0'-'9', '#', '*').
 */
void handlePinEntry(char key) {
  if (key >= '0' && key <= '9') {
    if (pinLen < PIN_MAX_LEN) {
      pinBuf[pinLen++] = key;
      pinStartTime = millis();
    }
  } else if (key == '#') {
    if (pinLen == 0) {
      menuActive = true;
      menuStep = 0;
      menuBufLen = 0;
      memset(menuBuf, 0, sizeof(menuBuf));
      Serial.println(F("[MENU] Enter current PIN:"));
    } else if (pinLen >= PIN_MIN_LEN) {
      pinBuf[pinLen] = '\0';

      if (pinChangeRequired) {
        uint8_t idx = findUserByPin(pinBuf);
        if (idx != 0xFF && idx == pinChangeUserIdx) {
          Serial.println(F("[AUTH] PIN expired. Change via menu (# at IDLE)."));
          pinLen = 0;
          memset(pinBuf, 0, sizeof(pinBuf));
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
    pinLen = 0;
    memset(pinBuf, 0, sizeof(pinBuf));
    Serial.println(F("[INPUT] Cancelled"));
  }
}

/**
 * @brief Despachador principal de entrada del teclado.
 * @details
 * Solo procesa entrada en el estado S_IDLE. Lee una tecla de la librería Keypad
 * y la enruta según el contexto:
 * - Menú activo: `handleMenuKey()`
 * - Tecla `A`: transición directa a ENV_MONITOR (debug)
 * - Tecla `B`: transición directa a INTRUSION_MONITOR (debug)
 * - Otra tecla: `handlePinEntry()` para autenticación o cambio de PIN
 * También verifica el timeout de ingreso de PIN (10s).
 * En IDLE también verifica presencia de tarjeta RFID.
 */
void processInput() {
  if (menuConfirmPending) {
    if (millis() - menuConfirmTime >= 1500) {
      menuConfirmPending = false;
      closeMenu();
    }
    return;
  }

  char key = keypad.getKey();

  if (currentState == S_IDLE) {
    uint8_t rfidRole = checkRFID();
    if (rfidRole > 0) {
      lastAuthRole = rfidRole;
      lastAuthUserIdx = 0xFF;
      failCount = 0;
      trig = TRIG_AUTH_OK;
      return;
    }
    if (key != NO_KEY) {
      if (menuActive) {
        handleMenuKey(key);
      } else if (key == 'A') {
        Serial.println(F("[INPUT] Entering ENV_MONITOR"));
        trig = TRIG_ENV_ALARM;
      } else if (key == 'B') {
        Serial.println(F("[INPUT] Entering INTRUSION_MONITOR"));
        trig = TRIG_INTRUSION;
      } else {
        handlePinEntry(key);
      }
    }

    if (pinLen > 0 && (millis() - pinStartTime) >= T_PIN_TIMEOUT) {
      Serial.println(F("[INPUT] PIN timeout"));
      pinLen = 0;
      memset(pinBuf, 0, sizeof(pinBuf));
    }
  }
}

/** @} */

// ============================================================================
// @name Actualización de Estado
// @brief Lógica principal por iteración: procesamiento de interrupciones, umbrales de sensores, temporización
// @{
// ============================================================================

/**
 * @brief Lógica principal de actualización de estado por iteración del bucle.
 * @details
 * Se llama en cada iteración de `loop()`. Ejecuta en orden:
 *
 * 1. **Procesamiento de flags de interrupción**: Lee `doorChanged` (establecido por la ISR)
 *    y actualiza `hallDoorOpen` con el estado actual del pin. NO limpia
 *    `doorChanged` — los manejadores específicos de estado lo consumen.
 *
 * 2. **Temporización de estado**: Para cada estado temporizado verifica si el
 *    tiempo transcurrido supera la duración. ENV_MONITOR excepcionalmente usa
 *    `lightLevel < LIGHT_MIN` (luz baja) como condición de salida.
 *
 * 3. **Cálculo de sensores**: Temperatura (Steinhart-Hart con inversión del
 *    divisor de voltaje NTC, clamps de seguridad) y nivel de luz.
 *
 * 4. **Monitoreo específico de estado**:
 *    - ENV_MONITOR: verifica umbrales de temperatura y luz.
 *    - INTRUSION_MONITOR: verifica estado de puerta y nivel de micrófono.
 *    - ALARM: cuenta nuevos eventos y detecta condición de alarma triple.
 *
 * 5. **Debug serial**: Cada 500ms imprime T, L, MIC y DOOR.
 *
 * 6. **Parpadeo LED**: Actualiza el LED rojo según el patrón activo.
 *
 * 7. **Actualización de LCD**: Cada LCD_INTERVAL.
 *
 * @note
 * Los eventos de puerta en ALARM usan detección por flanco (doorChanged),
 * mientras que INTRUSION_MONITOR usa detección por nivel (hallDoorOpen).
 */
void updateState() {
  unsigned long now = millis();

  if (buzzerTimedOff && millis() >= buzzerOffTime) {
    buzzerTimedOff = false;
    setBuzzer(false);
  }

  // La ISR de puerta dispara en CHANGE; actualizar hallDoorOpen con el estado actual.
  // NOTA: leemos doorChanged pero NO lo limpiamos aquí — los manejadores
  // específicos de estado (ej. ALARM) lo consumen para conteo por flanco.
  if (doorChanged) {
    hallDoorOpen = digitalRead(PIN_HALL_DOOR) == HIGH;
  }

  // --- Temporización de estado (basada en millis) ---
  unsigned long elapsed = now - stateEntryTime;
  if (trig == TRIG_NONE) {
    switch (currentState) {
      case S_OPEN:
        if (elapsed >= T_UNLOCK) trig = TRIG_TIMER_DONE;
        break;
      case S_BLOCKED:
        if (elapsed >= T_LOCKOUT) trig = TRIG_TIMER_DONE;
        break;
      case S_ENV_MONITOR:
        if (lightLevel > 0 && lightLevel < LIGHT_MIN) trig = TRIG_TIMER_DONE;
        break;
      case S_INTRUSION_MONITOR:
        if (elapsed >= T_INTRUSION) trig = TRIG_TIMER_DONE;
        break;
      default: break;
    }
  }

  // --- Lectura de sensores ---
  if (tempAvg.getCount() > 0) {
    float avgRaw = tempAvg.getAverage();
    if (avgRaw <= 0) avgRaw = 1;
    if (avgRaw >= 1023) avgRaw = 1022;
    float R2 = R1 * (1023.0f / avgRaw - 1.0f);
    if (R2 <= 0) R2 = 1;
    float logR2 = log(R2);
    temperature = (1.0f / (C1 + C2 * logR2 + C3 * logR2 * logR2 * logR2)) - 273.15f;
    if (temperature < -40 || temperature > 150) temperature = 25.0f;
  }
  if (lightAvg.getCount() > 0) {
    lightLevel = (int)lightAvg.getAverage();
  }

  static unsigned long lastDebug = 0;
  if (now - lastDebug >= 500) {
    lastDebug = now;
    Serial.print(F("T="));
    Serial.print(temperature, 1);
    Serial.print(F("C L="));
    Serial.print(lightLevel);
    Serial.print(F(" MIC="));
    Serial.print(micVal);
    Serial.print(F(" DOOR="));
    Serial.println(hallDoorOpen ? "OPEN" : "CLOSED");
  }

  // --- Monitoreo específico de estado ---
  switch (currentState) {
    case S_ENV_MONITOR:
      if (trig == TRIG_NONE && ((temperature > 0 && temperature < TEMP_LOW) || temperature > TEMP_HIGH || (lightLevel > 900))) {
        trig = TRIG_ENV_ALARM;
        Serial.print(F("[ENV] Threshold: T="));
        Serial.print(temperature);
        Serial.print(F(" L="));
        Serial.println(lightLevel);
      }
      break;

    case S_INTRUSION_MONITOR:
      if (trig == TRIG_NONE && (hallDoorOpen || micVal > SOUND_HIGH)) {
        trig = TRIG_INTRUSION;
        if (hallDoorOpen) {
          doorChanged = false;
          Serial.println(F("[INTRUSION] Door opened!"));
        }
        if (micVal > SOUND_HIGH) Serial.println(F("[INTRUSION] Sound detected!"));
      }
      break;

    case S_ALARM:
      if (alarmCount >= 3 && (now - firstAlarmTime) < T_TRIPLE) {
        alarmCount = 0;
        stateEntryTime = now;
        trig = TRIG_NONE;
      } else if (elapsed >= T_ALARM) {
        onLeaveAlarm();
        currentState = S_INTRUSION_MONITOR;
        onEnterIntrusionMonitor();
        trig = TRIG_NONE;
        fsm.SetState(S_INTRUSION_MONITOR, false, false);
      }
      break;

    default: break;
  }

  // --- Patrón de parpadeo LED (canal rojo del RGB) ---
  updateBlinkPattern();

  // --- Actualización de LCD ---
  if (now - lastLcdUpdate >= LCD_INTERVAL) {
    lastLcdUpdate = now;
    updateDisplay();
  }
}

/** @} */

// ============================================================================
// @name Configuración de la FSM
// @brief Cableado de transiciones de StateMachineLib
// @{
// ============================================================================

/**
 * @brief Configura la máquina de estados finita.
 * @details
 * Registra todos los callbacks de entrada/salida de estado y establece las
 * transiciones entre los 6 estados. Las transiciones usan funciones lambda
 * que verifican la variable global `trig`.
 *
 * Tabla de transiciones:
 * @code{.txt}
 * S_IDLE --[TRIG_AUTH_OK]--> S_OPEN
 * S_IDLE --[TRIG_LOCKOUT]--> S_BLOCKED
 * S_IDLE --[TRIG_ENV_ALARM]--> S_ENV_MONITOR     (tecla A debug)
 * S_IDLE --[TRIG_INTRUSION]--> S_INTRUSION_MONITOR (tecla B debug)
 * S_OPEN --[TRIG_TIMER_DONE]--> S_IDLE
 * S_BLOCKED --[TRIG_TIMER_DONE]--> S_IDLE
 * S_ENV_MONITOR --[TRIG_ENV_ALARM]--> S_ALARM     (umbral temp/light)
 * S_ENV_MONITOR --[TRIG_TIMER_DONE]--> S_IDLE      (luz baja)
 * S_INTRUSION_MONITOR --[TRIG_INTRUSION]--> S_ALARM
 * S_INTRUSION_MONITOR --[TRIG_TIMER_DONE]--> S_IDLE
 * S_ALARM --[TRIG_TIMER_DONE]--> S_INTRUSION_MONITOR
 * @endcode
 */
void setupFSM() {
  fsm.SetOnEntering(S_IDLE, onEnterIdle);
  fsm.SetOnEntering(S_OPEN, onEnterOpen);
  fsm.SetOnEntering(S_BLOCKED, onEnterBlocked);
  fsm.SetOnEntering(S_INTRUSION_MONITOR, onEnterIntrusionMonitor);
  fsm.SetOnEntering(S_ENV_MONITOR, onEnterEnvMonitor);
  fsm.SetOnEntering(S_ALARM, onEnterAlarm);

  fsm.SetOnLeaving(S_IDLE, onLeaveIdle);
  fsm.SetOnLeaving(S_OPEN, onLeaveOpen);
  fsm.SetOnLeaving(S_BLOCKED, onLeaveBlocked);
  fsm.SetOnLeaving(S_INTRUSION_MONITOR, onLeaveIntrusionMonitor);
  fsm.SetOnLeaving(S_ENV_MONITOR, onLeaveEnvMonitor);
  fsm.SetOnLeaving(S_ALARM, onLeaveAlarm);

  fsm.AddTransition(S_IDLE, S_OPEN, []() {
    return trig == TRIG_AUTH_OK;
  });
  fsm.AddTransition(S_IDLE, S_BLOCKED, []() {
    return trig == TRIG_LOCKOUT;
  });
  fsm.AddTransition(S_IDLE, S_ENV_MONITOR, []() {
    return trig == TRIG_ENV_ALARM;
  });
  fsm.AddTransition(S_IDLE, S_INTRUSION_MONITOR, []() {
    return trig == TRIG_INTRUSION;
  });

  auto timedDone = []() {
    return trig == TRIG_TIMER_DONE;
  };

  fsm.AddTransition(S_OPEN, S_IDLE, timedDone);
  fsm.AddTransition(S_BLOCKED, S_IDLE, timedDone);
  fsm.AddTransition(S_ENV_MONITOR, S_IDLE, timedDone);
  fsm.AddTransition(S_INTRUSION_MONITOR, S_IDLE, timedDone);

  fsm.AddTransition(S_ENV_MONITOR, S_ALARM, []() {
    return trig == TRIG_ENV_ALARM;
  });
  fsm.AddTransition(S_INTRUSION_MONITOR, S_ALARM, []() {
    return trig == TRIG_INTRUSION;
  });
  fsm.AddTransition(S_ALARM, S_INTRUSION_MONITOR, timedDone);
}

/** @} */

// ============================================================================
// @name Configuración de AsyncTask
// @brief Tarea periódica de promediado de sensores
// @{
// ============================================================================

/** @brief Intervalo de lectura de sensores (ms). */
#define SENSOR_INTERVAL 200UL

/**
 * @brief Tarea periódica de promediado de sensores.
 * @details
 * Lee los sensores analógicos y agrega los valores a los objetos RunningAverage.
 * Se ejecuta cada `SENSOR_INTERVAL` ms, con reinicio automático.
 * El sensor de puerta (reed switch) NO se lee aquí — usa interrupción hardware
 * en D21 (INT2) para detección instantánea.
 */
AsyncTask sensorTask(SENSOR_INTERVAL, true, []() {
  int ntc = analogRead(PIN_NTC);
  int ldr = analogRead(PIN_LDR);
  micVal = analogRead(PIN_MIC);

  tempAvg.add(ntc);
  lightAvg.add(ldr);
});

/** @} */

// ============================================================================
// @name Inicialización del Sistema
// @brief Función setup() de Arduino
// @{
// ============================================================================

/**
 * @brief setup() de Arduino — inicialización del sistema.
 * @details
 * Inicializa en orden:
 * 1. Monitor serie (9600 baud).
 * 2. Pines de salida (LED, zumbador) a estado OFF seguro.
 * 3. Reed switch de puerta en D21 con INPUT_PULLUP + attachInterrupt.
 * 4. Servo motor, en posición bloqueada.
 * 5. Display LCD (16x2, paralelo 4 bits).
 * 6. Inicialización de EEPROM.
 * 7. Tarea AsyncTask de promediado de sensores.
 * 8. RFID (SPI + MFRC522).
 * 9. Configuración de FSM y estado inicial (S_IDLE).
 */
void setup() {
  Serial.begin(9600);
  randomSeed(analogRead(PIN_MIC));

  pinMode(PIN_LED_R, OUTPUT);
  pinMode(PIN_LED_G, OUTPUT);
  pinMode(PIN_LED_B, OUTPUT);
  ledOff();

  pinMode(PIN_HALL_DOOR, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_HALL_DOOR), doorISR, CHANGE);

  doorServo.attach(PIN_SERVO);
  doorServo.write(SRV_LOCKED);

  lcd.begin(16, 2);
  lcd.print(F("Access Control"));
  lcd.setCursor(0, 1);
  lcd.print(F("Starting..."));

  initEEPROM();

  sensorTask.Start();

  SPI.begin();
  rfid.PCD_Init();
  Serial.println(F("[RFID] Reader ready"));

  setupFSM();
  fsm.SetState(S_IDLE, false, true);

  Serial.println(F("=== ACCESS CONTROL & SECURITY ==="));
  Serial.println(F("Keys: [0-9]=digit [#]=ok [*]=cancel"));
  Serial.println(F("From IDLE: # with no digits = change PIN"));
}

/** @} */

// ============================================================================
// @name Bucle Principal
// @brief loop() de Arduino — punto de entrada de ejecución
// @{
// ============================================================================

/**
 * @brief Bucle principal de Arduino.
 * @details
 * Ejecuta en cada frame:
 * 1. `updateState()`: Procesar interrupciones, temporizadores, sensores, parpadeo, LCD.
 * 2. `processInput()`: Leer y manejar la entrada del teclado y RFID.
 * 3. `sensorTask.Update()`: Ejecutar tareas periódicas de promediado de sensores.
 * 4. `fsm.Update()`: Evaluar transiciones de la FSM.
 * 5. `trig = TRIG_NONE`: Limpiar disparador para la siguiente iteración.
 *
 * @note
 * Toda la temporización usa `millis()` para operación no bloqueante.
 * No hay llamadas a `delay()` fuera del debounce del teclado
 * (manejado internamente por la librería Keypad).
 */
void loop() {
  updateState();
  processInput();
  sensorTask.Update();
  fsm.Update();
  trig = TRIG_NONE;
}

/** @} */
