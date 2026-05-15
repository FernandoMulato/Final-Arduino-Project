#define digitalPin 7
#define analogPin A0
#define ledPin 13

int digitalVal;
int analogVal;

void setup() {
  pinMode(digitalPin, INPUT);
  pinMode(analogPin, INPUT);
  pinMode(ledPin, OUTPUT);

  Serial.begin(9600);
}

void loop() {

  digitalVal = digitalRead(digitalPin);

  if (digitalVal == HIGH) {
    digitalWrite(ledPin, HIGH);
  } else {
    digitalWrite(ledPin, LOW);
  }

  analogVal = analogRead(analogPin);

  Serial.println(analogVal);
}