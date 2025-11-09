#include <ArduinoJson.h>
// include AccelStepper, docs here: https://www.airspayce.com/mikem/arduino/AccelStepper/index.html
// also here: https://hackaday.io/project/183279/details/ 
#include <AccelStepper.h>
#include "data_structures.h"

#define MOTOR_INTERFACE_TYPE 1

#define SERIAL_QUEUE_LENGTH 300

#define NUM_MOTORS 7
char dir_pins[NUM_MOTORS] = {30, 32, 34, 36, 38, 40, 42};
char step_pins[NUM_MOTORS] = {31, 33, 35, 37, 39, 41, 43};
char enable_pins[NUM_MOTORS] = {2, 3, 4, 5, 6, 7, 8};

char serial_buf[SERIAL_QUEUE_LENGTH];
SerialQueue serial_queue = SerialQueue(serial_buf, SERIAL_QUEUE_LENGTH);

AccelStepper steppers[NUM_MOTORS];

unsigned long last_loop_time;

bool is_running = true;

void setup() {
  for (int i = 0; i < NUM_MOTORS; i++) {
    char step_pin = step_pins[i];
    char dir_pin = dir_pins[i];
    char enable_pin = enable_pins[i];
    steppers[i] = AccelStepper(MOTOR_INTERFACE_TYPE, step_pin, dir_pin);

    // NOTE: doesn't affect MultiStepper
    steppers[i].setMaxSpeed(3000);
    steppers[i].setAcceleration(1000);

    // Disable all motors when not in use
    pinMode(enable_pin, OUTPUT); 
    digitalWrite(enable_pin, HIGH);
  }

  Serial.begin(9600);

  while (!Serial) continue;
  last_loop_time = millis();

  is_running = true;
}

void useParsedData(JsonDocument json) {
  String keyswitch_key = "killswitch";
  bool killswitch = json[keyswitch_key];
  if (killswitch) {
    is_running = false;
    return;
  }
  
  for (int i = 0; i < NUM_MOTORS; i++) {
    char enable_pin = enable_pins[i];
    
    String motor_key = String("motor_") + i;
    auto motor_json = json[motor_key].as<JsonObject>();
    if (motor_json["position"].is<int>()) {
      long position = motor_json["position"];

      auto &stepper = steppers[i];
      // Activate
      digitalWrite(enable_pin, LOW);
      if(position != stepper.targetPosition()) {
        // Resets speed, so don't call it in a loop
        stepper.moveTo(position);
      }
    }
  }
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

  if (!is_running){
    return;
  }
  // accelstepper stuff
  for (int i = 0; i < NUM_MOTORS; i++) {
    steppers[i].run();
  }
}
