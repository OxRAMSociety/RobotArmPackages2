#include <ArduinoJson.h>
// include AccelStepper, docs here: https://www.airspayce.com/mikem/arduino/AccelStepper/index.html
// also here: https://hackaday.io/project/183279/details/ 
#include <AccelStepper.h>
#include "data_structures.h"

#define DIR_PIN 2 // CW+
#define STEP_PIN 3 // CLK+
#define MOTOR_INTERFACE_TYPE 1

#define SERIAL_QUEUE_LENGTH 300

char serial_buf[SERIAL_QUEUE_LENGTH];
SerialQueue serial_queue = SerialQueue(serial_buf, SERIAL_QUEUE_LENGTH);

AccelStepper stepper = AccelStepper(MOTOR_INTERFACE_TYPE, STEP_PIN, DIR_PIN);

unsigned long last_loop_time;

void useParsedData(JsonDocument json) {
  int position = json["motor_1"]["position"];

  if(position != stepper.targetPosition()) {
    // Resets speed, so don't call it in a loop
    stepper.moveTo(position);
  }
}

void setup() {

  // accelstepper setup
  stepper.setMaxSpeed(1500);
  stepper.setAcceleration(1000);

  Serial.begin(9600);

  while (!Serial) continue;
  last_loop_time = millis();
}

void loop() {


  unsigned long delta = millis() - last_loop_time;
  if (delta > 3) {
    Serial.println("{\"overrun_delta\":" + String(delta));
  }
  last_loop_time = millis();


  JsonDocument json;
  const SerialReadResult res = serial_queue.try_get_json(json);
  if (!res.is_ok()) {
      Serial.print("Read error: ");
      Serial.println(res.c_str());
  } else if (res.is_data_available()){
      useParsedData(json);
  }

  // accelstepper stuff
  stepper.run();
}