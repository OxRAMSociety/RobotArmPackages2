const int analog_pin = A0;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(9600);
}

void loop() {
  int value = analogRead(analog_pin);
  Serial.println(value);

  if (value >= 1000) {
    digitalWrite(LED_BUILTIN, HIGH); 
  } else {
    digitalWrite(LED_BUILTIN, LOW);
  }
  delay(2);
}
