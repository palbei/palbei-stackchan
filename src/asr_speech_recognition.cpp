#include "asr_speech_recognition.h"

namespace em {

AsrSpeechRecognition::AsrSpeechRecognition(HardwareSerial& mySerial)
    : mySerial_(mySerial) {
}

void AsrSpeechRecognition::read_protocol(uint8_t* buffer, const uint8_t cmd,
                                       const uint8_t id) {
  (void)cmd;
  (void)id;
  if (mySerial_.available() >= 8) {
    while (mySerial_.available() > 0) {
      uint8_t data = mySerial_.peek();
      if (data == 0xA5) {
        mySerial_.readBytes(buffer, 8);
        if (buffer[0] == 0xA5 && buffer[1] == 0xFA && buffer[7] == 0xFB) {
          return;
        }
      } else {
        mySerial_.read();
      }
    }
  }
}

void AsrSpeechRecognition::send_protocol(const uint8_t cmd, const uint8_t id) {
  uint8_t buffer[8] = {0xA5, 0xFA, 0x00, cmd, id, 0x00, 0x00, 0xFB};
  buffer[6] = buffer[2] + buffer[3] + buffer[4] + buffer[5];
  mySerial_.write(buffer, 8);
}

void AsrSpeechRecognition::verify_operation(const uint8_t cmd) {
  send_protocol(ASR_VERIFY_COMMAND, cmd);
}

}
