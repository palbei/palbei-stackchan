#pragma once

#include <Arduino.h>
#include <ESP32Servo.h>
#include <math.h>

// MIT-compatible easing servo helper (API subset compatible with former ServoEasing usage).
enum EasyServoEasingType {
  EASE_LINEAR = 0,
  EASE_QUADRATIC_IN_OUT = 1,
};

#ifndef DEFAULT_MICROSECONDS_FOR_0_DEGREE
#define DEFAULT_MICROSECONDS_FOR_0_DEGREE 544
#endif
#ifndef DEFAULT_MICROSECONDS_FOR_180_DEGREE
#define DEFAULT_MICROSECONDS_FOR_180_DEGREE 2400
#endif

class EasyServo;

static constexpr int kMaxEasyServos = 8;
static EasyServo* g_easy_servos[kMaxEasyServos] = {};
static int g_easy_servo_count = 0;
static float g_easy_servo_default_speed = 30.0f;
static bool g_easy_servo_task_started = false;

class EasyServo {
 public:
  EasyServo() {
    if (g_easy_servo_count < kMaxEasyServos) {
      g_easy_servos[g_easy_servo_count++] = this;
    }
  }

  int attach(int pin, int initial_angle, int min_us, int max_us);

  void setEasingType(EasyServoEasingType type) { _easing = type; }

  void setSpeed(float deg_per_sec) {
    if (deg_per_sec > 0) {
      _speed = deg_per_sec;
    }
  }

  float getSpeed() const { return _speed; }

  void setEaseTo(int angle) {
    _target = constrain(angle, 0, 180);
    _start = _current;
    _move_start_ms = millis();
    float dist = fabsf((float)_target - (float)_start);
    float spd = _speed > 0 ? _speed : g_easy_servo_default_speed;
    _duration_ms = (uint32_t)((dist / spd) * 1000.0f);
    if (_duration_ms < 1) {
      _duration_ms = 1;
    }
    _moving = dist >= 0.5f;
    if (!_moving) {
      _current = (float)_target;
      if (_servo.attached()) {
        _servo.write(_target);
      }
    }
  }

  bool isMoving() {
    update();
    return _moving;
  }

  int read() {
    update();
    return (int)(_current + 0.5f);
  }

  void stop() {
    update();
    _target = (int)(_current + 0.5f);
    _moving = false;
  }

  void update() {
    if (!_moving) {
      return;
    }
    uint32_t elapsed = millis() - _move_start_ms;
    float t = _duration_ms > 0 ? (float)elapsed / (float)_duration_ms : 1.0f;
    if (t >= 1.0f) {
      t = 1.0f;
      _moving = false;
    }
    float e = ease(t);
    _current = _start + ((float)_target - _start) * e;
    if (_servo.attached()) {
      _servo.write((int)(_current + 0.5f));
    }
  }

 private:
  float ease(float t) const {
    t = constrain(t, 0.0f, 1.0f);
    if (_easing == EASE_QUADRATIC_IN_OUT) {
      return t < 0.5f ? 2.0f * t * t : 1.0f - powf(-2.0f * t + 2.0f, 2.0f) / 2.0f;
    }
    return t;
  }

  Servo _servo;
  EasyServoEasingType _easing = EASE_QUADRATIC_IN_OUT;
  float _speed = 30.0f;
  float _current = 90.0f;
  float _start = 90.0f;
  int _target = 90;
  uint32_t _move_start_ms = 0;
  uint32_t _duration_ms = 1;
  bool _moving = false;
};

inline void updateAllEasyServos() {
  for (int i = 0; i < g_easy_servo_count; ++i) {
    if (g_easy_servos[i]) {
      g_easy_servos[i]->update();
    }
  }
}

inline void ensureEasyServoUpdater() {
  if (g_easy_servo_task_started) {
    return;
  }
  g_easy_servo_task_started = true;
  xTaskCreate(
      [](void*) {
        for (;;) {
          updateAllEasyServos();
          vTaskDelay(pdMS_TO_TICKS(10));
        }
      },
      "easyServo",
      2048,
      nullptr,
      1,
      nullptr);
}

inline int EasyServo::attach(int pin, int initial_angle, int min_us, int max_us) {
  ensureEasyServoUpdater();
  _current = (float)constrain(initial_angle, 0, 180);
  _target = (int)_current;
  _moving = false;
  int ok = _servo.attach(pin, min_us, max_us);
  if (ok >= 0) {
    _servo.write((int)_current);
  }
  return ok;
}

inline void setSpeedForAllServos(float deg_per_sec) {
  g_easy_servo_default_speed = deg_per_sec;
  for (int i = 0; i < g_easy_servo_count; ++i) {
    if (g_easy_servos[i]) {
      g_easy_servos[i]->setSpeed(deg_per_sec);
    }
  }
}

inline void synchronizeAllServosStartAndWaitForAllServosToStop() {
  bool any = true;
  while (any) {
    any = false;
    updateAllEasyServos();
    for (int i = 0; i < g_easy_servo_count; ++i) {
      if (g_easy_servos[i] && g_easy_servos[i]->isMoving()) {
        any = true;
        break;
      }
    }
    delay(5);
    yield();
  }
}
