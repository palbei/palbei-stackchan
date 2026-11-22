#pragma once

#ifndef _EM_ASR_SPEECH_RECOGNITION_H_
#define _EM_ASR_SPEECH_RECOGNITION_H_

#include <Arduino.h>
#include <stdint.h>

namespace em {

#define ASR_SERIAL_BAUD_RATE (9600)
#define ASR_READ_GENERAL_COMMAND (0x81)
#define ASR_VERIFY_COMMAND (0x02)

class AsrSpeechRecognition {
 public:
  explicit AsrSpeechRecognition(HardwareSerial& mySerial);

  void read_protocol(uint8_t* buffer, const uint8_t cmd, const uint8_t id);
  void verify_operation(const uint8_t cmd);

 private:
  void send_protocol(const uint8_t cmd, const uint8_t id);

  HardwareSerial& mySerial_;
};
}
#endif
