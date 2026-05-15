#include "StateMachineLib.h"
#include "AsyncTaskLib.h"

// ============================================================
//  CONFIGURACIÓN DE PINES
// ============================================================

// Sonido
constexpr uint8_t PIN_SONIDO_DIGITAL = 7;
constexpr uint8_t PIN_SONIDO_ANALOGO = A0;
constexpr uint8_t PIN_LED = 13;

// Temperatura (termistor NTC)
constexpr uint8_t PIN_TERMISTOR = A1;
constexpr float R1 = 10000.0f;
constexpr float C1 = 0.001129148f;
constexpr float C2 = 0.000234125f;
constexpr float C3 = 0.0000000876741f;

// Luz (fotorresistencia)
constexpr uint8_t PIN_LDR = A2;

// Hall (sensor magnético)
constexpr uint8_t PIN_HALL = A3;

// ============================================================
//  VARIABLES GLOBALES DE SENSORES
// ============================================================

int valorSonidoDigital;
int valorSonidoAnalogico;
int valorTermistor;
float logR2, R2, temperatura;
int valorLuz;
int valorHall;

// ============================================================
//  MÁQUINA DE ESTADOS
// ============================================================

enum Estado : uint8_t {
  REPOSO = 0,
  TEMPERATURA = 1,
  LUZ = 2,
  SONIDO = 3,
};

enum Entrada : uint8_t {
  REINICIAR = 0,
  AVANZAR = 1,
  RETROCEDER = 2,
  DESCONOCIDA = 3,
};

StateMachine maquinaEstados(4, 9);
Entrada entrada;

// ============================================================
//  TAREAS ASÍNCRONAS
// ============================================================

void ejecutarTareaHall();
void ejecutarTareaTemp();
void ejecutarTareaLuz();
void ejecutarTareaSonido();

AsyncTask tareaHall(1500, true, ejecutarTareaHall);
AsyncTask tareaTemp(1500, true, ejecutarTareaTemp);
AsyncTask tareaLuz(1500, true, ejecutarTareaLuz);
AsyncTask tareaSonido(1500, true, ejecutarTareaSonido);

// ============================================================
//  ACCIONES DE ENTRADA Y SALIDA DE CADA ESTADO
// ============================================================

// ---- REPOSO
void alEntrarReposo() {
  Serial.println(F("→ Estado: REPOSO"));
  tareaHall.Start();
}

void alSalirReposo() {
  tareaHall.Stop();
}

// ---- TEMPERATURA
void alEntrarTemperatura() {
  Serial.println(F("→ Estado: TEMPERATURA"));
  tareaTemp.Start();
}

void alSalirTemperatura() {
  tareaTemp.Stop();
}

// ---- LUZ
void alEntrarLuz() {
  Serial.println(F("→ Estado: LUZ"));
  tareaLuz.Start();
}

void alSalirLuz() {
  tareaLuz.Stop();
}

// ---- SONIDO
void alEntrarSonido() {
  Serial.println(F("→ Estado: SONIDO"));
  tareaSonido.Start();
}

void alSalirSonido() {
  tareaSonido.Stop();
}

// ============================================================
//  CONFIGURACIÓN DE LA MÁQUINA DE ESTADOS
// ============================================================

void configurarMaquinaEstados() {
  // Transiciones
  maquinaEstados.AddTransition(REPOSO, TEMPERATURA,
    []() { return entrada == AVANZAR; });
  maquinaEstados.AddTransition(TEMPERATURA, LUZ,
    []() { return entrada == AVANZAR; });
  maquinaEstados.AddTransition(LUZ, SONIDO,
    []() { return entrada == AVANZAR; });
  maquinaEstados.AddTransition(SONIDO, LUZ,
    []() { return entrada == RETROCEDER; });
  maquinaEstados.AddTransition(LUZ, TEMPERATURA,
    []() { return entrada == RETROCEDER; });
  maquinaEstados.AddTransition(TEMPERATURA, REPOSO,
    []() { return entrada == RETROCEDER; });

  // Reset desde cualquier estado
  maquinaEstados.AddTransition(TEMPERATURA, REPOSO,
    []() { return entrada == REINICIAR; });
  maquinaEstados.AddTransition(LUZ, REPOSO,
    []() { return entrada == REINICIAR; });
  maquinaEstados.AddTransition(SONIDO, REPOSO,
    []() { return entrada == REINICIAR; });

  // Acciones al entrar/salir de cada estado
  maquinaEstados.SetOnEntering(REPOSO, alEntrarReposo);
  maquinaEstados.SetOnLeaving(REPOSO, alSalirReposo);
  maquinaEstados.SetOnEntering(TEMPERATURA, alEntrarTemperatura);
  maquinaEstados.SetOnLeaving(TEMPERATURA, alSalirTemperatura);
  maquinaEstados.SetOnEntering(LUZ, alEntrarLuz);
  maquinaEstados.SetOnLeaving(LUZ, alSalirLuz);
  maquinaEstados.SetOnEntering(SONIDO, alEntrarSonido);
  maquinaEstados.SetOnLeaving(SONIDO, alSalirSonido);
}

// ============================================================
//  SETUP
// ============================================================

void setup() {
  Serial.begin(9600);

  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_SONIDO_DIGITAL, INPUT);

  Serial.println(F("========================================"));
  Serial.println(F("  SISTEMA DE MONITOREO DE SENSORES"));
  Serial.println(F("========================================"));
  Serial.println(F(""));
  Serial.println(F("Comandos:"));
  Serial.println(F("  D → Avanzar al siguiente sensor"));
  Serial.println(F("  A → Retroceder al sensor anterior"));
  Serial.println(F("  R → Volver al estado inicial (REPOSO)"));
  Serial.println(F(""));

  configurarMaquinaEstados();
  maquinaEstados.SetState(REPOSO, false, true);
}

// ============================================================
//  LOOP PRINCIPAL
// ============================================================

void loop() {
  entrada = static_cast<Entrada>(leerEntrada());

  tareaHall.Update();
  tareaTemp.Update();
  tareaLuz.Update();
  tareaSonido.Update();

  maquinaEstados.Update();
}

// ============================================================
//  LECTURA DE ENTRADA SERIAL
// ============================================================

Entrada leerEntrada() {
  if (!Serial.available()) {
    return DESCONOCIDA;
  }

  char caracter = Serial.read();

  switch (caracter) {
    case 'R':
    case 'r':
      return REINICIAR;
    case 'A':
    case 'a':
      return RETROCEDER;
    case 'D':
    case 'd':
      return AVANZAR;
    default:
      return DESCONOCIDA;
  }
}

// ============================================================
//  TAREAS DE SENSORES
// ============================================================

void ejecutarTareaHall() {
  valorHall = analogRead(PIN_HALL);

  Serial.print(F("── HALL "));
  Serial.print(F("[A"));  // DEBUG
  Serial.print(PIN_HALL);
  Serial.print(F("] "));
  Serial.print(F("→ "));
  Serial.println(valorHall);
}

void ejecutarTareaTemp() {
  valorTermistor = analogRead(PIN_TERMISTOR);

  // Fórmula Steinhart-Hart para termistor NTC
  R2 = R1 * (1023.0f / (float)valorTermistor - 1.0f);
  logR2 = log(R2);
  temperatura = (1.0f / (C1 + C2 * logR2 + C3 * logR2 * logR2 * logR2));
  temperatura = temperatura - 273.15f;

  Serial.print(F("── TEMPERATURA → "));
  Serial.print(temperatura, 1);
  Serial.println(F(" °C"));
}

void ejecutarTareaLuz() {
  valorLuz = analogRead(PIN_LDR);

  Serial.print(F("── LUZ "));
  Serial.print(F("[A")); 
  Serial.print(PIN_LDR);
  Serial.print(F("] → "));
  Serial.println(valorLuz);
}

void ejecutarTareaSonido() {
  valorSonidoDigital = digitalRead(PIN_SONIDO_DIGITAL);
  valorSonidoAnalogico = analogRead(PIN_SONIDO_ANALOGO);

  if (valorSonidoDigital == HIGH) {
    digitalWrite(PIN_LED, HIGH);
  } else {
    digitalWrite(PIN_LED, LOW);
  }

  Serial.print(F("── SONIDO "));
  Serial.print(F("[D"));
  Serial.print(PIN_SONIDO_DIGITAL);
  Serial.print(F(" → "));
  Serial.print(valorSonidoDigital == HIGH ? F("DETECTADO") : F("SILENCIO"));
  Serial.print(F(" | A"));
  Serial.print(PIN_SONIDO_ANALOGO);
  Serial.print(F(" → "));
  Serial.print(valorSonidoAnalogico);
  Serial.println(F("]"));
}
