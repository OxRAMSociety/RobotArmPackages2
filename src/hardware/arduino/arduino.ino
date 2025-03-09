#include <ArduinoJson.h>
// include AccelStepper, docs here: https://www.airspayce.com/mikem/arduino/AccelStepper/index.html
// also here: https://hackaday.io/project/183279/details/ 
#include <AccelStepper.h>



// accelstepper stuff
AccelStepper myStepper(AccelStepper::DRIVER, 3, 2);

// json stuff

int num_messages_received = 0;

class DataInputError {
  bool stream_timeout;
  DeserializationError deserialization_error;

public:
  DataInputError(bool s, DeserializationError d)
    : stream_timeout(s), deserialization_error(d) {}
  bool is_ok() {
    return this->stream_timeout == false && this->deserialization_error == DeserializationError::Ok;
  }
};

DataInputError read_data(JsonDocument &out) {
  const String in = Serial.readStringUntil('\n');
  if (in == NULL) return DataInputError(true, NULL);
  return DataInputError(false, deserializeJson(out, in));
}

void showParsedData(JsonDocument json) {
  bool turn_on_led = json["turn_on_led"];
  num_messages_received += 1;
  Serial.print("Receiving message #");
  Serial.print(num_messages_received);
  Serial.print("; New LED state: ");
  Serial.println(turn_on_led);

  if (turn_on_led) {
    digitalWrite(LED_BUILTIN, HIGH);
    myStepper.setSpeed(500);
  } else {
    digitalWrite(LED_BUILTIN, LOW);
    myStepper.setSpeed(0);
  }
}









void setup() {

  // accelstepper setup
  myStepper.setMaxSpeed(1000);
  myStepper.setSpeed(500);

  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(9600);

  while (!Serial) continue;
}

void loop() {
  JsonDocument json;
  const DataInputError err = read_data(json);
  if (err.is_ok()) {
      showParsedData(json);
  } else {
    Serial.println("Read error");
  }

  // accelstepper stuff
  myStepper.runSpeed();
}