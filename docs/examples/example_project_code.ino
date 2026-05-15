#include "StateMachineLib.h"
#include "AsyncTaskLib.h"

// ---------------- SENSORES ----------------

// SONIDO
#define digitalPin 7
#define analogPin A0
#define ledPin 13

int digitalVal;
int analogVal;

// TEMPERATURA
#define ThermistorPin A1
#define R1 10000
#define c1 0.001129148
#define c2 0.000234125
#define c3 0.0000000876741

int Vo;
float logR2, R2, T;

// LUZ
#define sensorPinP A2
int value = 0;

// HALL
#define sensorPinH A3
int val = 0;

// ---------------- TASKS ----------------

void sonidoTask(void);
void temperaturaTask(void);
void luzTask(void);
void hallTask(void);

AsyncTask TaskHall(1500, true, hallTask);
AsyncTask TaskTemp(1500, true, temperaturaTask);
AsyncTask TaskLuz(1500, true, luzTask);
AsyncTask TaskSonido(1500, true, sonidoTask);

// ---------------- ESTADOS ----------------

enum State
{
  PosicionA = 0,
  PosicionB = 1,
  PosicionC = 2,
  PosicionD = 3
};

enum Input
{
  Reset = 0,
  Forward = 1,
  Backward = 2,
  Unknown = 3,
};

StateMachine stateMachine(4, 9);

Input input;

// ---------------- FUNCIONES DE ENTRADA/SALIDA ----------------

// ---- A
void enterA() {
  Serial.println("Entrando A");
  TaskHall.Start();
}

void exitA() {
  Serial.println("Leaving A");
  TaskHall.Stop();
}

// ---- B
void enterB() {
  Serial.println("Entrando B");
  TaskTemp.Start();
}

void exitB() {
  Serial.println("Leaving B");
  TaskTemp.Stop();
}

// ---- C
void enterC() {
  Serial.println("Entrando C");
  TaskLuz.Start();
}

void exitC() {
  Serial.println("Leaving C");
  TaskLuz.Stop();
}

// ---- D
void enterD() {
  Serial.println("Entrando D");
  TaskSonido.Start();
}

void exitD() {
  Serial.println("Leaving D");
  TaskSonido.Stop();
}

// ---------------- STATE MACHINE ----------------

void setupStateMachine()
{
  // TRANSICIONES

  // A -> B
  stateMachine.AddTransition(PosicionA, PosicionB,
    []() { return input == Forward; });

  // B -> C
  stateMachine.AddTransition(PosicionB, PosicionC,
    []() { return input == Forward; });

  // C -> D
  stateMachine.AddTransition(PosicionC, PosicionD,
    []() { return input == Forward; });

  // D -> C
  stateMachine.AddTransition(PosicionD, PosicionC,
    []() { return input == Backward; });

  // C -> B
  stateMachine.AddTransition(PosicionC, PosicionB,
    []() { return input == Backward; });

  // B -> A
  stateMachine.AddTransition(PosicionB, PosicionA,
    []() { return input == Backward; });

  // RESET
  stateMachine.AddTransition(PosicionB, PosicionA,
    []() { return input == Reset; });

  stateMachine.AddTransition(PosicionC, PosicionA,
    []() { return input == Reset; });

  stateMachine.AddTransition(PosicionD, PosicionA,
    []() { return input == Reset; });

  // ACCIONES

  stateMachine.SetOnEntering(PosicionA, enterA);
  stateMachine.SetOnLeaving(PosicionA, exitA);

  stateMachine.SetOnEntering(PosicionB, enterB);
  stateMachine.SetOnLeaving(PosicionB, exitB);

  stateMachine.SetOnEntering(PosicionC, enterC);
  stateMachine.SetOnLeaving(PosicionC, exitC);

  stateMachine.SetOnEntering(PosicionD, enterD);
  stateMachine.SetOnLeaving(PosicionD, exitD);
}

// ---------------- SETUP ----------------

void setup()
{
  Serial.begin(9600);

  pinMode(ledPin, OUTPUT);
  pinMode(digitalPin, INPUT);

  Serial.println("Starting State Machine...");

  setupStateMachine();

  stateMachine.SetState(PosicionA, false, true);
}

// ---------------- LOOP ----------------

void loop()
{
  input = static_cast<Input>(readInput());

  // Actualizar tareas
  TaskHall.Update();
  TaskTemp.Update();
  TaskLuz.Update();
  TaskSonido.Update();

  // Actualizar máquina
  stateMachine.Update();
}

// ---------------- LECTURA SERIAL ----------------

int readInput()
{
  Input currentInput = Unknown;

  if (Serial.available())
  {
    char incomingChar = Serial.read();

    switch (incomingChar)
    {
      case 'R':
        currentInput = Reset;
        break;

      case 'A':
        currentInput = Backward;
        break;

      case 'D':
        currentInput = Forward;
        break;
    }
  }

  return currentInput;
}

// ---------------- TASKS ----------------

void hallTask(void)
{
  val = analogRead(sensorPinH);

  Serial.print("HALL: ");
  Serial.println(val);

  Serial.println();
}

void temperaturaTask(void)
{
  Vo = analogRead(ThermistorPin);

  R2 = R1 * (1023.0 / (float)Vo - 1.0);

  logR2 = log(R2);

  T = (1.0 / (c1 + c2 * logR2 + c3 * logR2 * logR2 * logR2));

  T = T - 273.15;

  Serial.print("Temperatura: ");
  Serial.print(T);
  Serial.println(" C");

  Serial.println();
}

void luzTask(void)
{
  value = analogRead(sensorPinP);

  Serial.print("LUZ: ");
  Serial.println(value);

  Serial.println();
}

void sonidoTask(void)
{
  digitalVal = digitalRead(digitalPin);

  if (digitalVal == HIGH)
  {
    digitalWrite(ledPin, HIGH);
  }
  else
  {
    digitalWrite(ledPin, LOW);
  }

  Serial.print("SONIDO: ");
  Serial.println(digitalVal);

  Serial.println();
}
