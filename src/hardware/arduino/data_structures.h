#pragma once
#include "HardwareSerial.h"
#include <ArduinoJson.h>

enum SerialQueueState {
  OkReading,
  OkDone,
  Overflow,
  AssertionError
};

class SerialReadResult {
  SerialQueueState _state;
  DeserializationError _deserialization_error;

public:
  SerialReadResult(SerialQueueState s, DeserializationError d)
    : _state(s), _deserialization_error(d) {}
  bool is_ok() {
    return (this->_state == SerialQueueState::OkReading || this->_state == SerialQueueState::OkDone) && 
    this->_deserialization_error == DeserializationError::Ok;
  }

  bool is_data_available() {
    return is_ok() && _state == SerialQueueState::OkDone;
  }

  const char* c_str() const {
    if (is_ok()) return "No error";
    if (this->_deserialization_error != DeserializationError::Ok) {
      return _deserialization_error.c_str();
    } else {
      switch (_state) {
        case SerialQueueState::Overflow:
          return "Read buffer overflow";
        case SerialQueueState::AssertionError:
          return "Assertion error";
        default:
          return "Unknown error";
      }
    }
  }
};

class SerialQueue {
  private:
  unsigned int _capacity;
  unsigned int _len;
  char* _buf;
  SerialQueueState _state;

  bool _push(char c) {
    _buf[_len] = c;

    if (_len < _capacity) {
      _len += 1;
      return true;
    } else {
      _state = SerialQueueState::Overflow;
      return false;
    }
  }

  /**
  * Read all incoming data until the first newline.
  */
  SerialQueueState _readAvailable() {
    // Can't continue if error
    if(is_error()) return _state;

    // If just finished reading a string, then reset the buffer and keep listening
    if(_state == SerialQueueState::OkDone) {
      _state = SerialQueueState::OkReading;
      _len = 0;
    }

    while (Serial.available() > 0) {
      int new_char = Serial.read();
      // If end of stream
      if (new_char == -1) {
        _state = SerialQueueState::AssertionError;
        return _state;
      }
      // if end of line, return the line
      else if (new_char == '\n') {
        // Terminate the string
        if(!_push(NULL)) {
          _state = SerialQueueState::Overflow;
          return _state;
        } else {
          _state = SerialQueueState::OkDone;
          return _state;
        }
      } else {
        if(!_push(new_char)) {
          _state = SerialQueueState::Overflow;
          return _state;
        }
      }
    }
    return SerialQueueState::OkReading;
  }

  public:
  SerialQueue(char* buf, unsigned int capacity) {
    _capacity = capacity;
    _len = 0;
    _buf = buf;
    _state = SerialQueueState::OkReading;
  }

  bool is_error() {
    switch(_state) {
      case SerialQueueState::OkDone:
      case SerialQueueState::OkReading:
        return false;
      default:
        return true;
    }
  }

  SerialReadResult try_get_json(JsonDocument &out) {
    _readAvailable();
    if (is_error()) return SerialReadResult(_state, DeserializationError::Ok);
    else if(_state == SerialQueueState::OkDone)
      return SerialReadResult(_state, deserializeJson(out, _buf));
    else return SerialReadResult(_state, DeserializationError::Ok);
  }
};
