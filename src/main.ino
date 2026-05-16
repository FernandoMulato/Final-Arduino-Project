/*
 * SISTEMA DE CONTROL DE ACCESO Y SEGURIDAD
 * Arquitectura Computacional — Arduino Mega (ATmega2560)
 *
 * FSM de 10 estados: INICIO, BOTON, CLAVE_CORRECTA, CONFIG,
 * TIEMPO_2_SEC, MONITOR_AMBIENTAL, SISTEMA_BLOQUEADO, BLOQUEO,
 * ALARMA, MONITOR_INTRUSOS
 *
 * Librerías: StateMachineLib, Keypad, EEPROM
 * LCD 16x2 I2C opcional: descomentar #define USE_LCD
 *
 * Diagrama de transiciones:
 *   INICIO --(tecla)--> BOTON --(# vacio)--> CONFIG
 *     ^                    |--(PIN ok)--> CLAVE_CORRECTA --(2s)--> TIEMPO_2_SEC
 *     |                    |--(3 fallos)--> SISTEMA_BLOQUEADO --> BLOQUEO --(4s)--> INICIO
 *     |                    +--(* o timeout)-------------------------------------> INICIO
 *     |    TIEMPO_2_SEC --(2s)--> MONITOR_AMBIENTAL --(umbral)--> ALARMA --(5s)--> MONITOR_INTRUSOS
 *     |    TIEMPO_2_SEC --(tecla)--> INICIO            --(3s)----> INICIO   --(2s)--> INICIO
 *     +-----------------------------------------------------------------------------+
 */

#include <Arduino.h>
#include <StateMachineLib.h>
#include <EEPROM.h>
#include <Keypad.h>

// #define USE_LCD  // Descomentar para LCD 16x2 I2C (Mega: D20=SDA, D21=SCL)
#ifdef USE_LCD
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 16, 2);
#endif

// ========== PINES ==========
// Teclado 4x4 (arrays no-const para Keypad library)
byte pinesFilas[4] = {2, 3, 4, 5};
byte pinesCols[4]  = {6, 7, 8, 9};
// Actuadores
constexpr uint8_t PIN_LED_ROJO   = 10;
constexpr uint8_t PIN_BUZZER     = 11;
constexpr uint8_t PIN_RELE       = 12;
constexpr uint8_t PIN_LED_BUILTIN = 13;
// Sensores analógicos
constexpr uint8_t PIN_MICROFONO = A0;
constexpr uint8_t PIN_TERMISTOR = A1;
constexpr uint8_t PIN_LDR       = A2;
constexpr uint8_t PIN_HALL      = A3;

// ========== CONSTANTES ==========
constexpr unsigned long T_REBOTE     = 50;
constexpr unsigned long T_INPUT      = 10000;
constexpr unsigned long T_DESBLOQUEO = 2000;
constexpr unsigned long T_CONTEO     = 2000;
constexpr unsigned long T_BLOQUEO    = 4000;
constexpr unsigned long T_ALARMA     = 5000;
constexpr unsigned long T_INTRUSOS   = 2000;
constexpr unsigned long T_AMBIENTAL  = 3000;
constexpr unsigned long T_TRIPLE     = 12000;
constexpr float UMBRAL_TEMP_BAJA = 20.0f;
constexpr float UMBRAL_TEMP_ALTA = 50.0f;
constexpr int   UMBRAL_LUZ       = 100;
constexpr int   UMBRAL_SONIDO    = 800;
constexpr int   UMBRAL_HALL      = 512;
constexpr int   MAX_DIGITOS      = 4;
constexpr float R1 = 10000.0f, C1 = 0.001129148f, C2 = 0.000234125f, C3 = 0.0000000876741f;

// ========== ESTRUCTURAS ==========
struct Usuario {
  char    pin[5];
  uint8_t rol;       // 0=seguridad, 1=operario, 2=coordinador, 3=gerente
  uint8_t usos;      // Contador de usos (máx 4 antes de rotación)
  bool    activo;
};
struct Horario {
  uint8_t indiceUsuario;
  uint8_t horaInicio, minInicio, horaFin, minFin, dias;
  bool    activo;
};

// ========== EEPROM LAYOUT ==========
constexpr uint8_t DIR_MAGICO        = 0;
constexpr uint8_t DIR_CANT_USUARIOS = 1;
constexpr uint8_t DIR_USUARIOS      = 2;     // 10 × 8 = 80
constexpr uint8_t DIR_CANT_HORARIOS = 82;
constexpr uint8_t DIR_HORARIOS      = 83;    // 8 × 8 = 64
constexpr uint8_t DIR_PIN_MAESTRO   = 200;
constexpr uint8_t MAGICO            = 0xA5;
constexpr uint8_t MAX_USUARIOS      = 10;
constexpr uint8_t MAX_HORARIOS      = 8;

// ========== ENUMERACIONES ==========
enum Estado : uint8_t {
  E_INICIO, E_BOTON, E_CLAVE_CORRECTA, E_CONFIG, E_TIEMPO_2_SEC,
  E_MONITOR_AMBIENTAL, E_SISTEMA_BLOQUEADO, E_BLOQUEO, E_ALARMA, E_MONITOR_INTRUSOS
};
enum Trigger : uint8_t {
  TRIG_NONE, TRIG_CONFIG, TRIG_PIN_OK, TRIG_LOCKOUT, TRIG_CANCEL,
  TRIG_SALIR_CONFIG, TRIG_ALARMA, TRIG_INTRUSION
};

// ========== VARIABLES GLOBALES ==========
StateMachine fsm(10, 18);
Estado estadoActual = E_INICIO;

// Teclado 4x4
const byte FILAS = 4, COLUMNAS = 4;
char mapaTeclas[FILAS][COLUMNAS] = {
  {'1','2','3','A'}, {'4','5','6','B'}, {'7','8','9','C'}, {'*','0','#','D'}
};
Keypad teclado = Keypad(makeKeymap(mapaTeclas), pinesFilas, pinesCols, FILAS, COLUMNAS);

// Buffer PIN
char bufferPIN[MAX_DIGITOS + 1] = {0};
uint8_t digitosIngresados = 0;
unsigned long tiempoInicioInput = 0;

// Disparador y temporizador
Trigger triggerTransicion = TRIG_NONE;
unsigned long tiempoEstado = 0;
unsigned long tiempoUltimoBlink = 0;
bool estadoLED = false;

// Seguridad
uint8_t intentosFallidos = 0;
uint8_t contadorBloqueos = 0;
uint8_t contadorDisparosAlarma = 0;
unsigned long tiempoInicioAlarma = 0;

// Config
uint8_t nivelConfig = 0, opcionConfig = 0, pasoConfig = 0;
char bufferConfig[20] = {0};

// PIN maestro
char pinMaestro[5] = "1234";

// Sensores
int valorTermistor, valorLuz, valorHall, valorMicrofono;
float logR2, R2, temperatura;

// Bandera de tecla presionada (para transiciones FSM)
bool teclaPresionada = false;

// ========== FUNCIONES SENSORES ==========
bool leerNTC() {
  valorTermistor = analogRead(PIN_TERMISTOR);
  if (valorTermistor == 0) valorTermistor = 1;
  R2 = R1 * (1023.0f / (float)valorTermistor - 1.0f);
  logR2 = log(R2);
  temperatura = (1.0f / (C1 + C2 * logR2 + C3 * logR2 * logR2 * logR2)) - 273.15f;
  return (temperatura < UMBRAL_TEMP_BAJA || temperatura > UMBRAL_TEMP_ALTA);
}
bool leerLDR() { valorLuz = analogRead(PIN_LDR); return (valorLuz < UMBRAL_LUZ); }
bool leerHall() { valorHall = analogRead(PIN_HALL); return (valorHall > UMBRAL_HALL); }
bool leerMicrofono() { valorMicrofono = analogRead(PIN_MICROFONO); return (valorMicrofono > UMBRAL_SONIDO); }

// ========== ACTUADORES ==========
void encenderRele() { digitalWrite(PIN_RELE, HIGH); digitalWrite(PIN_LED_BUILTIN, HIGH); }
void apagarRele() { digitalWrite(PIN_RELE, LOW); digitalWrite(PIN_LED_BUILTIN, LOW); }
void activarLEDAlarma(bool e) { digitalWrite(PIN_LED_ROJO, e ? HIGH : LOW); estadoLED = e; }
void activarBuzzer(bool e) { digitalWrite(PIN_BUZZER, e ? HIGH : LOW); }

// ========== EEPROM ==========
void initEEPROM() {
  if (EEPROM.read(DIR_MAGICO) != MAGICO) {
    EEPROM.write(DIR_MAGICO, MAGICO);
    EEPROM.write(DIR_CANT_USUARIOS, 0);
    EEPROM.write(DIR_CANT_HORARIOS, 0);
    for (uint8_t i = 0; i < 4; i++) EEPROM.write(DIR_PIN_MAESTRO + i, "1234"[i]);
  }
}
void leerPinMaestroEEPROM() {
  for (uint8_t i = 0; i < 4; i++) pinMaestro[i] = EEPROM.read(DIR_PIN_MAESTRO + i);
  pinMaestro[4] = '\0';
}
void escribirPinMaestroEEPROM(const char* p) {
  for (uint8_t i = 0; i < 4; i++) EEPROM.write(DIR_PIN_MAESTRO + i, p[i]);
}
Usuario leerUsuario(uint8_t idx) {
  Usuario u; uint8_t addr = DIR_USUARIOS + idx * 8;
  for (uint8_t i = 0; i < 4; i++) u.pin[i] = EEPROM.read(addr + i);
  u.pin[4] = '\0'; u.rol = EEPROM.read(addr + 4); u.usos = EEPROM.read(addr + 5);
  u.activo = EEPROM.read(addr + 6) != 0;
  if (u.pin[0] < '0' || u.pin[0] > '9') u.activo = false;
  return u;
}
void escribirUsuario(uint8_t idx, const Usuario& u) {
  uint8_t addr = DIR_USUARIOS + idx * 8;
  for (uint8_t i = 0; i < 4; i++) EEPROM.write(addr + i, u.pin[i]);
  EEPROM.write(addr + 4, u.rol); EEPROM.write(addr + 5, u.usos);
  EEPROM.write(addr + 6, u.activo ? 1 : 0);
}
void escribirHorario(uint8_t idx, const Horario& h) {
  uint8_t addr = DIR_HORARIOS + idx * 8;
  EEPROM.write(addr, h.indiceUsuario); EEPROM.write(addr+1, h.horaInicio);
  EEPROM.write(addr+2, h.minInicio); EEPROM.write(addr+3, h.horaFin);
  EEPROM.write(addr+4, h.minFin); EEPROM.write(addr+5, h.dias);
  EEPROM.write(addr+6, h.activo ? 1 : 0);
}

// ========== VALIDACIÓN PIN ==========
bool validarPIN(const char* ingresado) {
  // Comparar contra PIN maestro
  bool ok = true;
  for (uint8_t i = 0; i < 4; i++) { if (ingresado[i] != pinMaestro[i]) { ok = false; break; } }
  if (ok) return true;

  // Comparar contra usuarios
  uint8_t cant = EEPROM.read(DIR_CANT_USUARIOS);
  for (uint8_t i = 0; i < cant; i++) {
    Usuario u = leerUsuario(i);
    if (!u.activo) continue;
    ok = true;
    for (uint8_t j = 0; j < 4; j++) { if (ingresado[j] != u.pin[j]) { ok = false; break; } }
    if (ok) {
      u.usos++;
      if (u.usos >= 4) {
        u.usos = 0;
        int nuevo = random(0, 10000);
        snprintf(u.pin, 5, "%04d", nuevo);
        Serial.print(F("PIN rotado usuario ")); Serial.print(i);
        Serial.print(F(": ")); Serial.println(u.pin);
      }
      escribirUsuario(i, u);
      return true;
    }
  }
  return false;
}

// ========== DISPLAY ==========
void mostrarInfoEstado() {
#ifdef USE_LCD
  lcd.clear(); lcd.setCursor(0, 0);
#endif
  switch (estadoActual) {
    case E_INICIO:
      Serial.println(F("[INICIO] Sistema en reposo. Presione tecla para PIN."));
#ifdef USE_LCD
      lcd.print(F("SISTEMA REPOSO")); lcd.setCursor(0,1); lcd.print(F("Tecla para PIN"));
#endif
      break;
    case E_BOTON:
      Serial.print(F("[PIN] "));
      for (uint8_t i = 0; i < MAX_DIGITOS; i++) Serial.print(i < digitosIngresados ? '*' : '-');
      Serial.println(F("  #=ok *=cancel"));
#ifdef USE_LCD
      lcd.print(F("PIN:"));
      for (uint8_t i = 0; i < MAX_DIGITOS; i++) lcd.print(i < digitosIngresados ? '*' : '-');
#endif
      break;
    case E_CLAVE_CORRECTA:
      Serial.println(F("[ACCESO] Cerradura abierta 2s"));
#ifdef USE_LCD
      lcd.print(F("ACCESO CONCEDIDO"));
#endif
      break;
    case E_CONFIG:
      Serial.println(F("[CONFIG] 1=Usuario 2=Horario 3=Rol *=Salir"));
      break;
    case E_TIEMPO_2_SEC: {
      unsigned long r = (millis() < tiempoEstado) ? (tiempoEstado - millis()) / 1000 + 1 : 0;
      Serial.print(F("[TIEMPO] ")); Serial.print(r); Serial.println(F(" seg"));
      break;
    }
    case E_MONITOR_AMBIENTAL:
      Serial.print(F("[AMBIENTE] Temp:")); Serial.print(temperatura, 1);
      Serial.print(F("C Luz:")); Serial.println(valorLuz);
      break;
    case E_SISTEMA_BLOQUEADO:
      Serial.println(F("[BLOQUEO] 3 intentos fallidos"));
#ifdef USE_LCD
      lcd.print(F("SISTEMA BLOQUEADO"));
#endif
      break;
    case E_BLOQUEO:
      Serial.println(F("[ESPERA] 4 segundos..."));
      break;
    case E_ALARMA:
      Serial.println(F("[ALARMA] !!! Intrusion detectada !!!"));
#ifdef USE_LCD
      lcd.print(F("!!! ALARMA !!!"));
#endif
      break;
    case E_MONITOR_INTRUSOS:
      Serial.print(F("[INTRUSOS] Hall:")); Serial.print(valorHall);
      Serial.print(F(" Mic:")); Serial.println(valorMicrofono);
      break;
  }
}

// ========== DECLARACIONES ANTICIPADAS ==========
void alEntrarInicio();
void alEntrarBoton();
void alEntrarClaveCorrecta();
void alEntrarConfig();
void alEntrarTiempo2Seg();
void alEntrarMonitorAmbiental();
void alEntrarSistemaBloqueado();
void alEntrarBloqueo();
void alEntrarAlarma();
void alEntrarMonitorIntrusos();
void alSalirInicio();
void alSalirBoton();
void alSalirClaveCorrecta();
void alSalirConfig();
void alSalirTiempo2Seg();
void alSalirMonitorAmbiental();
void alSalirSistemaBloqueado();
void alSalirBloqueo();
void alSalirAlarma();
void alSalirMonitorIntrusos();
void menuConfig(char tecla);

// ========== CONFIGURACIÓN FSM ==========
void configurarFSM() {
  fsm.SetOnEntering(E_INICIO, alEntrarInicio);
  fsm.SetOnEntering(E_BOTON, alEntrarBoton);
  fsm.SetOnEntering(E_CLAVE_CORRECTA, alEntrarClaveCorrecta);
  fsm.SetOnEntering(E_CONFIG, alEntrarConfig);
  fsm.SetOnEntering(E_TIEMPO_2_SEC, alEntrarTiempo2Seg);
  fsm.SetOnEntering(E_MONITOR_AMBIENTAL, alEntrarMonitorAmbiental);
  fsm.SetOnEntering(E_SISTEMA_BLOQUEADO, alEntrarSistemaBloqueado);
  fsm.SetOnEntering(E_BLOQUEO, alEntrarBloqueo);
  fsm.SetOnEntering(E_ALARMA, alEntrarAlarma);
  fsm.SetOnEntering(E_MONITOR_INTRUSOS, alEntrarMonitorIntrusos);
  fsm.SetOnLeaving(E_INICIO, alSalirInicio);
  fsm.SetOnLeaving(E_BOTON, alSalirBoton);
  fsm.SetOnLeaving(E_CLAVE_CORRECTA, alSalirClaveCorrecta);
  fsm.SetOnLeaving(E_CONFIG, alSalirConfig);
  fsm.SetOnLeaving(E_TIEMPO_2_SEC, alSalirTiempo2Seg);
  fsm.SetOnLeaving(E_MONITOR_AMBIENTAL, alSalirMonitorAmbiental);
  fsm.SetOnLeaving(E_SISTEMA_BLOQUEADO, alSalirSistemaBloqueado);
  fsm.SetOnLeaving(E_BLOQUEO, alSalirBloqueo);
  fsm.SetOnLeaving(E_ALARMA, alSalirAlarma);
  fsm.SetOnLeaving(E_MONITOR_INTRUSOS, alSalirMonitorIntrusos);

  // INICIO → BOTON: cualquier tecla
  fsm.AddTransition(E_INICIO, E_BOTON, []() { return teclaPresionada; });
  // BOTON → CONFIG: # sin dígitos
  fsm.AddTransition(E_BOTON, E_CONFIG, []() { return triggerTransicion == TRIG_CONFIG; });
  // BOTON → CLAVE_CORRECTA: PIN válido
  fsm.AddTransition(E_BOTON, E_CLAVE_CORRECTA, []() { return triggerTransicion == TRIG_PIN_OK; });
  // BOTON → SISTEMA_BLOQUEADO: 3 fallos
  fsm.AddTransition(E_BOTON, E_SISTEMA_BLOQUEADO, []() { return triggerTransicion == TRIG_LOCKOUT; });
  // BOTON → INICIO: cancelación o timeout
  fsm.AddTransition(E_BOTON, E_INICIO, []() {
    return triggerTransicion == TRIG_CANCEL || (millis() - tiempoInicioInput) >= T_INPUT;
  });
  // CONFIG → INICIO: salir
  fsm.AddTransition(E_CONFIG, E_INICIO, []() { return triggerTransicion == TRIG_SALIR_CONFIG; });
  // CLAVE_CORRECTA → TIEMPO_2_SEC: 2s
  fsm.AddTransition(E_CLAVE_CORRECTA, E_TIEMPO_2_SEC, []() { return millis() >= tiempoEstado; });
  // TIEMPO_2_SEC → MONITOR_AMBIENTAL: 2s
  fsm.AddTransition(E_TIEMPO_2_SEC, E_MONITOR_AMBIENTAL, []() { return millis() >= tiempoEstado; });
  // TIEMPO_2_SEC → INICIO: cualquier tecla
  fsm.AddTransition(E_TIEMPO_2_SEC, E_INICIO, []() { return teclaPresionada; });
  // MONITOR_AMBIENTAL → ALARMA: umbral
  fsm.AddTransition(E_MONITOR_AMBIENTAL, E_ALARMA, []() { return triggerTransicion == TRIG_ALARMA; });
  // MONITOR_AMBIENTAL → INICIO: 3s sin novedad
  fsm.AddTransition(E_MONITOR_AMBIENTAL, E_INICIO, []() { return millis() >= tiempoEstado; });
  // SISTEMA_BLOQUEADO → BLOQUEO: inmediato
  fsm.AddTransition(E_SISTEMA_BLOQUEADO, E_BLOQUEO, []() { return true; });
  // BLOQUEO → INICIO: 4s
  fsm.AddTransition(E_BLOQUEO, E_INICIO, []() { return millis() >= tiempoEstado; });
  // ALARMA → MONITOR_INTRUSOS: 5s
  fsm.AddTransition(E_ALARMA, E_MONITOR_INTRUSOS, []() { return millis() >= tiempoEstado; });
  // MONITOR_INTRUSOS → ALARMA: intrusión
  fsm.AddTransition(E_MONITOR_INTRUSOS, E_ALARMA, []() { return triggerTransicion == TRIG_INTRUSION; });
  // MONITOR_INTRUSOS → INICIO: 2s
  fsm.AddTransition(E_MONITOR_INTRUSOS, E_INICIO, []() { return millis() >= tiempoEstado; });
}

// ========== HANDLERS DE ESTADOS ==========
void alEntrarInicio() {
  estadoActual = E_INICIO; intentosFallidos = 0; digitosIngresados = 0;
  triggerTransicion = TRIG_NONE; teclaPresionada = false;
  apagarRele(); activarBuzzer(false); activarLEDAlarma(false);
  mostrarInfoEstado();
}
void alSalirInicio() {}
void alEntrarBoton() {
  estadoActual = E_BOTON; digitosIngresados = 0;
  tiempoInicioInput = millis(); triggerTransicion = TRIG_NONE; teclaPresionada = false;
  mostrarInfoEstado();
}
void alSalirBoton() { for (uint8_t i = 0; i < MAX_DIGITOS; i++) bufferPIN[i] = 0; digitosIngresados = 0; }
void alEntrarClaveCorrecta() {
  estadoActual = E_CLAVE_CORRECTA; triggerTransicion = TRIG_NONE;
  tiempoEstado = millis() + T_DESBLOQUEO; encenderRele(); mostrarInfoEstado();
}
void alSalirClaveCorrecta() { apagarRele(); }
void alEntrarConfig() {
  estadoActual = E_CONFIG; triggerTransicion = TRIG_NONE;
  nivelConfig = 0; opcionConfig = 0; pasoConfig = 0;
  Serial.println(F("[CONFIG] 1=Usuario 2=Horario 3=Rol *=Salir"));
}
void alSalirConfig() { Serial.println(F("Configuracion guardada.")); }
void alEntrarTiempo2Seg() {
  estadoActual = E_TIEMPO_2_SEC; triggerTransicion = TRIG_NONE;
  teclaPresionada = false; tiempoEstado = millis() + T_CONTEO; mostrarInfoEstado();
}
void alSalirTiempo2Seg() {}
void alEntrarMonitorAmbiental() {
  estadoActual = E_MONITOR_AMBIENTAL; triggerTransicion = TRIG_NONE;
  tiempoEstado = millis() + T_AMBIENTAL; leerNTC(); leerLDR(); mostrarInfoEstado();
}
void alSalirMonitorAmbiental() {}
void alEntrarSistemaBloqueado() {
  estadoActual = E_SISTEMA_BLOQUEADO; contadorBloqueos++; mostrarInfoEstado();
}
void alSalirSistemaBloqueado() {}
void alEntrarBloqueo() {
  estadoActual = E_BLOQUEO; triggerTransicion = TRIG_NONE;
  tiempoEstado = millis() + T_BLOQUEO; tiempoUltimoBlink = millis(); mostrarInfoEstado();
}
void alSalirBloqueo() { activarLEDAlarma(false); }
void alEntrarAlarma() {
  estadoActual = E_ALARMA; triggerTransicion = TRIG_NONE;
  tiempoEstado = millis() + T_ALARMA; tiempoInicioAlarma = millis();
  contadorDisparosAlarma = 0; tiempoUltimoBlink = millis(); activarBuzzer(true); mostrarInfoEstado();
}
void alSalirAlarma() { activarBuzzer(false); activarLEDAlarma(false); }
void alEntrarMonitorIntrusos() {
  estadoActual = E_MONITOR_INTRUSOS; triggerTransicion = TRIG_NONE;
  tiempoEstado = millis() + T_INTRUSOS; leerHall(); leerMicrofono(); mostrarInfoEstado();
}
void alSalirMonitorIntrusos() {}

// ========== PROCESAMIENTO DE ENTRADA ==========
void procesarEntrada() {
  char tecla = teclado.getKey();  // getKey() internamente hace scanKeys()
  if (tecla == NO_KEY) { teclaPresionada = false; return; }
  teclaPresionada = true;

  if (estadoActual == E_BOTON) {
    if (tecla >= '0' && tecla <= '9' && digitosIngresados < MAX_DIGITOS) {
      bufferPIN[digitosIngresados++] = tecla;
      mostrarInfoEstado();
      if (digitosIngresados == MAX_DIGITOS) {
        if (validarPIN(bufferPIN)) { triggerTransicion = TRIG_PIN_OK; }
        else {
          intentosFallidos++;
          if (intentosFallidos >= 3) { triggerTransicion = TRIG_LOCKOUT; }
          else { Serial.println(F("PIN incorrecto.")); digitosIngresados = 0; tiempoInicioInput = millis(); }
        }
      }
    } else if (tecla == '#') {
      if (digitosIngresados == 0) { triggerTransicion = TRIG_CONFIG; }
      else {
        if (validarPIN(bufferPIN)) { triggerTransicion = TRIG_PIN_OK; }
        else {
          intentosFallidos++;
          if (intentosFallidos >= 3) { triggerTransicion = TRIG_LOCKOUT; }
          else { Serial.println(F("PIN incorrecto.")); digitosIngresados = 0; tiempoInicioInput = millis(); }
        }
      }
    } else if (tecla == '*') { triggerTransicion = TRIG_CANCEL; }
  } else if (estadoActual == E_CONFIG) {
    if (tecla == '*') { triggerTransicion = TRIG_SALIR_CONFIG; }
    else { menuConfig(tecla); }
  } else if (estadoActual == E_TIEMPO_2_SEC) {
    // teclaPresionada se usa en la transición a INICIO
  }
}

// ========== MENÚ CONFIGURACIÓN ==========
void menuConfig(char tecla) {
  if (nivelConfig == 0) {
    if (tecla >= '1' && tecla <= '3') {
      opcionConfig = tecla - '0'; nivelConfig = 1; pasoConfig = 0;
      memset(bufferConfig, 0, sizeof(bufferConfig));
      if (opcionConfig == 1) Serial.println(F("Nro usuario (0-9):"));
      else if (opcionConfig == 2) Serial.println(F("Nro usuario:"));
      else Serial.println(F("Nro usuario (0-9):"));
    }
  } else {
    if (opcionConfig == 1) {  // Agregar/Editar usuario
      if (pasoConfig == 0 && tecla >= '0' && tecla <= '9') {
        bufferConfig[0] = tecla; pasoConfig = 1;
        Serial.println(F("PIN 4 digitos:"));
      } else if (pasoConfig == 1 && tecla >= '0' && tecla <= '9') {
        uint8_t len = strlen(bufferConfig);
        if (len < 4) { bufferConfig[len] = tecla; bufferConfig[len+1] = '\0'; Serial.print('*'); }
        if (strlen(bufferConfig) == 4) {
          Serial.println(); uint8_t idx = bufferConfig[0] - '0';
          Usuario u; u.pin[0] = bufferConfig[1]; u.pin[1] = bufferConfig[2];
          u.pin[2] = bufferConfig[3]; u.pin[3] = bufferConfig[4]; u.pin[4] = '\0';
          u.rol = 1; u.usos = 0; u.activo = true;
          escribirUsuario(idx, u);
          if (idx >= EEPROM.read(DIR_CANT_USUARIOS)) EEPROM.write(DIR_CANT_USUARIOS, idx + 1);
          Serial.print(F("Usuario ")); Serial.print(idx); Serial.print(F(" PIN: ")); Serial.println(u.pin);
          nivelConfig = 0; alEntrarConfig();
        }
      }
    } else if (opcionConfig == 2) {  // Configurar horario
      if (pasoConfig == 0 && tecla >= '0' && tecla <= '9') { bufferConfig[0] = tecla; pasoConfig = 1; Serial.println(F("Hora inicio (0-23):")); }
      else if (pasoConfig == 1 && tecla >= '0' && tecla <= '9') { bufferConfig[1] = tecla; pasoConfig = 2; Serial.println(F("Min inicio (0-59):")); }
      else if (pasoConfig == 2 && tecla >= '0' && tecla <= '9') { bufferConfig[2] = tecla; pasoConfig = 3; Serial.println(F("Hora fin (0-23):")); }
      else if (pasoConfig == 3 && tecla >= '0' && tecla <= '9') { bufferConfig[3] = tecla; pasoConfig = 4; Serial.println(F("Min fin (0-59):")); }
      else if (pasoConfig == 4 && tecla >= '0' && tecla <= '9') {
        bufferConfig[4] = tecla; pasoConfig = 5;
        uint8_t cant = EEPROM.read(DIR_CANT_HORARIOS);
        if (cant < MAX_HORARIOS) {
          Horario h; h.indiceUsuario = bufferConfig[0] - '0';
          h.horaInicio = bufferConfig[1] - '0'; h.minInicio = bufferConfig[2] - '0';
          h.horaFin = bufferConfig[3] - '0'; h.minFin = bufferConfig[4] - '0';
          h.dias = 0x7F; h.activo = true;
          escribirHorario(cant, h); EEPROM.write(DIR_CANT_HORARIOS, cant + 1);
          Serial.println(F("Horario guardado."));
        } else Serial.println(F("Max horarios alcanzado."));
        nivelConfig = 0; alEntrarConfig();
      }
    } else if (opcionConfig == 3) {  // Asignar rol
      if (pasoConfig == 0 && tecla >= '0' && tecla <= '9') {
        bufferConfig[0] = tecla; pasoConfig = 1;
        Serial.println(F("Rol: 0=Seg 1=Op 2=Coord 3=Ger"));
      } else if (pasoConfig == 1 && tecla >= '0' && tecla <= '3') {
        uint8_t idx = bufferConfig[0] - '0'; uint8_t rol = tecla - '0';
        Usuario u = leerUsuario(idx);
        if (u.activo) {
          u.rol = rol; escribirUsuario(idx, u);
          const char* roles[] = {"Seguridad","Operario","Coordinador","Gerente"};
          Serial.print(F("Rol: ")); Serial.println(roles[rol]);
        } else Serial.println(F("Usuario no existe."));
        nivelConfig = 0; alEntrarConfig();
      }
    }
  }
}

// ========== ACTUALIZACIÓN DE ESTADO ==========
void actualizarEstado() {
  switch (estadoActual) {
    case E_MONITOR_AMBIENTAL:
      if (leerNTC() || leerLDR()) triggerTransicion = TRIG_ALARMA;
      break;
    case E_MONITOR_INTRUSOS:
      if (leerHall() || leerMicrofono()) triggerTransicion = TRIG_INTRUSION;
      break;
    case E_ALARMA:
      if (millis() - tiempoUltimoBlink >= (estadoLED ? 700UL : 300UL)) {
        tiempoUltimoBlink = millis(); estadoLED = !estadoLED; activarLEDAlarma(estadoLED);
      }
      if (leerHall() || leerMicrofono()) {
        contadorDisparosAlarma++;
        Serial.print(F("Disparo #")); Serial.println(contadorDisparosAlarma);
        if (contadorDisparosAlarma >= 3 && (millis() - tiempoInicioAlarma) < T_TRIPLE) {
          tiempoEstado = millis() + T_ALARMA; contadorDisparosAlarma = 0;
          tiempoInicioAlarma = millis(); Serial.println(F("TRIPLE: Alarma rearmada"));
        }
      }
      break;
    case E_BLOQUEO:
      if (millis() - tiempoUltimoBlink >= (estadoLED ? 500UL : 100UL)) {
        tiempoUltimoBlink = millis(); estadoLED = !estadoLED; activarLEDAlarma(estadoLED);
      }
      break;
    default: break;
  }
}

// ========== SETUP ==========
void setup() {
  Serial.begin(9600);
  randomSeed(analogRead(A0));
  pinMode(PIN_LED_ROJO, OUTPUT); pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_RELE, OUTPUT); pinMode(PIN_LED_BUILTIN, OUTPUT);
  digitalWrite(PIN_LED_ROJO, LOW); digitalWrite(PIN_BUZZER, LOW);
  digitalWrite(PIN_RELE, LOW); digitalWrite(PIN_LED_BUILTIN, LOW);
#ifdef USE_LCD
  lcd.init(); lcd.backlight(); lcd.clear();
  lcd.print(F("Sistema Acceso")); lcd.setCursor(0,1); lcd.print(F("Iniciando..."));
#endif
  initEEPROM(); leerPinMaestroEEPROM();
  Serial.println(F("=== CONTROL ACCESO Y SEGURIDAD ==="));
  Serial.print(F("PIN maestro: ")); Serial.println(pinMaestro);
  Serial.println(F("Teclas: [0-9]=digito [#]=ok [*]=cancel"));
  configurarFSM();
  fsm.SetState(E_INICIO, false, true);
}

// ========== LOOP ==========
void loop() {
  procesarEntrada();
  actualizarEstado();
  fsm.Update();
}
