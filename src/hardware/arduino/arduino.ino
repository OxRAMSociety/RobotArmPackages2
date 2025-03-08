#include <ArduinoJson.h>

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
  Serial.print("New LED state: ");
  Serial.println(turn_on_led);
  num_messages_received += 1;
  Serial.print("Num messages received: ");
  Serial.println(num_messages_received);

  if (turn_on_led) {
    digitalWrite(LED_BUILTIN, HIGH);
  } else {
    digitalWrite(LED_BUILTIN, LOW);
  }
}

void setup() {
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
}