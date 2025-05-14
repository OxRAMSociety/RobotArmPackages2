#include <ArduinoJson.h>
// include AccelStepper, docs here: https://www.airspayce.com/mikem/arduino/AccelStepper/index.html
// also here: https://hackaday.io/project/183279/details/ 
#include <AccelStepper.h>
#include "data_structures.h"

#define DIR_PIN 2
#define STEP_PIN 3
#define MOTOR_INTERFACE_TYPE 1

#define SERIAL_QUEUE_LENGTH 300

char serial_buf[SERIAL_QUEUE_LENGTH];
SerialQueue serial_queue = SerialQueue(serial_buf, SERIAL_QUEUE_LENGTH);

// accelstepper stuff
AccelStepper stepper = AccelStepper(MOTOR_INTERFACE_TYPE, STEP_PIN, DIR_PIN);

// json stuff

int num_messages_received = 0;

void showParsedData(JsonDocument json) {
  bool turn_on_led = json["turn_on_led"];
  num_messages_received += 1;
  Serial.print("Receiving message #");
  Serial.print(num_messages_received);
  Serial.print("; New LED state: ");
  Serial.println(turn_on_led);

  if (turn_on_led) {
    digitalWrite(LED_BUILTIN, HIGH);
    stepper.setSpeed(500);
  } else {
    digitalWrite(LED_BUILTIN, LOW);
    stepper.setSpeed(0);
  }
}

void setup() {

  // accelstepper setup
  stepper.setMaxSpeed(1000);
  stepper.setAcceleration(500);

  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(9600);

  while (!Serial) continue;
}

void loop() {
  JsonDocument json;
  const SerialReadResult res = serial_queue.try_get_json(json);
  if (!res.is_ok()) {
      Serial.print("Read error: ");
      Serial.println(res.c_str());
  } else if (res.is_data_available()){
      showParsedData(json);
  }

  // accelstepper stuff
  stepper.runSpeed();
}