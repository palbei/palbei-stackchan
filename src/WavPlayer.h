#pragma once

#include <M5Unified.h>
#include <math.h>

// MIT-compatible WAV playback via M5.Speaker (replaces ESP8266Audio).
class WavPlayer {
 public:
  explicit WavPlayer(m5::Speaker_Class* speaker, uint8_t channel = 0)
      : _speaker(speaker), _channel(channel) {}

  bool begin(const uint8_t* wav_data, size_t length) {
    if (!_speaker || !wav_data || length < 44) {
      return false;
    }
    stop();
    _playing = _speaker->playWav(wav_data, length, 1, _channel, true);
    _started_ms = millis();
    return _playing;
  }

  bool isRunning() const {
    if (!_speaker) {
      return false;
    }
    return _speaker->isPlaying(_channel) != 0;
  }

  bool loop() {
    _playing = isRunning();
    return _playing;
  }

  void stop() {
    if (_speaker) {
      _speaker->stop(_channel);
    }
    _playing = false;
    _lip[0] = 0;
    _lip[1] = 0;
  }

  // Rough mouth level for Avatar lip-sync while speaker is active.
  const int16_t* getBuffer() {
    if (isRunning()) {
      float t = (millis() - _started_ms) / 35.0f;
      int16_t level = (int16_t)(4000.0f + 3500.0f * fabsf(sinf(t)));
      _lip[0] = level;
      _lip[1] = level;
    } else {
      _lip[0] = 0;
      _lip[1] = 0;
    }
    return _lip;
  }

 private:
  m5::Speaker_Class* _speaker = nullptr;
  uint8_t _channel = 0;
  bool _playing = false;
  uint32_t _started_ms = 0;
  int16_t _lip[2] = {0, 0};
};
