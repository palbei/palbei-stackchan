#include <Arduino.h>
#include <M5Unified.h>
#include <nvs.h>
#include <Avatar.h>
#include "asr_speech_recognition.h"
#include "RadarControl.h"
#include "TOF050FSensor.h"

#define ASR_RX_PIN 17
#define ASR_TX_PIN 18

HardwareSerial asrSerial(2);
em::AsrSpeechRecognition asr(asrSerial);

TOF050FSensor tofSensor;

#define SERVO_TASK_STACK_SIZE 8192
#define SERVO_TASK_PRIORITY   1

#define EAR_SERVO_TASK_STACK_SIZE 4096
#define EAR_SERVO_TASK_PRIORITY   2

#define CAMERA_TASK_STACK_SIZE 52536
#define CAMERA_TASK_PRIORITY   3

bool camera_needs_reset = false;
bool camera_initialized = false;

uint8_t* frame_buffer_copy = nullptr;
size_t frame_buffer_size = 0;
bool buffer_allocated = false;

#include "WavPlayer.h"
#include <lgfx/v1/lgfx_fonts.hpp>

#include "EasyServo.h"
#include "AudioWhisper.h"

#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <ESP32WebServer.h>
#include <WiFi.h>

#include "SenseVoiceClient.h"

#if defined( ENABLE_FACE_DETECT )
#include <esp_camera.h>
#include <fb_gfx.h>
#include <vector>
#include "human_face_detect_msr01.hpp"
#include "human_face_detect_mnp01.hpp"

#include "gesture/CameraGestureRecognizer.h"
#include "gesture/ProximitySensor.h"

CameraGestureRecognizer cameraGesture;

ProximitySensor proximitySensor;

bool enableGestureDetection = true;

bool gestureModuleEnabled = false;
bool asrEnabled = true;
/** 多轮语音聊天中；退出后不再自动 SST_ChatGPT */
bool chatSessionActive = false;
unsigned long touchStartTime = 0;
bool touchHolding = false;
const unsigned long LONG_PRESS_THRESHOLD = 300;

bool enableProximitySensor = true;
const uint16_t PROXIMITY_THRESHOLD = 500;
const uint16_t PROXIMITY_CLOSE_THRESHOLD = 800;
unsigned long lastProximityCheckTime = 0;
const unsigned long PROXIMITY_CHECK_INTERVAL = 200;
bool objectDetected = false;
unsigned long objectDetectedTime = 0;
unsigned long sensorDisableTime = 0;

#define TWO_STAGE 1

#define FACE_COLOR_WHITE  0x00FFFFFF
#define FACE_COLOR_BLACK  0x00000000
#define FACE_COLOR_RED    0x000000FF
#define FACE_COLOR_GREEN  0x0000FF00
#define FACE_COLOR_BLUE   0x00FF0000
#define FACE_COLOR_YELLOW (FACE_COLOR_RED | FACE_COLOR_GREEN)
#define FACE_COLOR_CYAN   (FACE_COLOR_BLUE | FACE_COLOR_GREEN)
#define FACE_COLOR_PURPLE (FACE_COLOR_BLUE | FACE_COLOR_RED)

static camera_config_t camera_config = {
    .pin_pwdn     = -1,
    .pin_reset    = -1,
    .pin_xclk     = -1,
    .pin_sscb_sda = 12,
    .pin_sscb_scl = 11,

    .pin_d7 = 47,
    .pin_d6 = 48,
    .pin_d5 = 16,
    .pin_d4 = 15,
    .pin_d3 = 42,
    .pin_d2 = 41,
    .pin_d1 = 40,
    .pin_d0 = 39,

    .pin_vsync = 46,
    .pin_href  = 38,
    .pin_pclk  = 45,

    .xclk_freq_hz = 16100000,
    .ledc_timer   = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,

    .pixel_format = PIXFORMAT_RGB565,

    .frame_size   = FRAMESIZE_QVGA,
    .jpeg_quality = 0,

    .fb_count     = 1,
    .fb_location  = CAMERA_FB_IN_PSRAM,
    .grab_mode    = CAMERA_GRAB_WHEN_EMPTY,
};

bool isSubWindowON = true;
bool isSilentMode = false;
bool mainsleep = false;

#endif

#include "ServoControl.h"

#include "driver/mcpwm.h"

#include "esp_task_wdt.h"

#define CAM_WATCHDOG_TIMEOUT 10

#define WIFI_SSID "SET YOUR WIFI SSID"
#define WIFI_PASS "SET YOUR WIFI PASS"

#define USE_SERVO
#ifdef USE_SERVO
#if defined(ARDUINO_M5STACK_Core2)
#define PORT_A

#if defined (PORT_A)
  #define SERVO_PIN_X 33
  #define SERVO_PIN_Y 32
#elif defined (PORT_C)
  #define SERVO_PIN_X 14
  #define SERVO_PIN_Y 13
#endif
#elif defined( ARDUINO_M5STACK_FIRE )
  #define SERVO_PIN_X 21
  #define SERVO_PIN_Y 22
#elif defined( ARDUINO_M5Stack_Core_ESP32 )
  #define SERVO_PIN_X 21
  #define SERVO_PIN_Y 22
#elif defined( ARDUINO_M5STACK_CORES3 )
  #define SERVO_PIN_XX 10
  #define SERVO_PIN_YY 9
  #define SERVO_PIN_X 7
  #define SERVO_PIN_Y 6

  #define LEFT_WHEEL_PIN 5
  #define RIGHT_WHEEL_PIN 8

#define MCPWM_UNIT        MCPWM_UNIT_0
#define MCPWM_TIMER       MCPWM_TIMER_0
#define MCPWM_IO_LEFT     LEFT_WHEEL_PIN
#define MCPWM_IO_RIGHT    RIGHT_WHEEL_PIN

#define MCPWM_FREQ         50

#define EAR_MCPWM_UNIT    MCPWM_UNIT_1
#define EAR_MCPWM_TIMER   MCPWM_TIMER_0
#define EAR_MCPWM_IO_XX   SERVO_PIN_XX
#define EAR_MCPWM_IO_YY   SERVO_PIN_YY
#define EAR_MCPWM_FREQ    50

#define EAR_MIN_DUTY      5.0
#define EAR_MAX_DUTY      10.0
#define EAR_CENTER_DUTY   7.5

#define EAR_DEAD_ZONE     0.1
#endif
#endif

static constexpr uint8_t m5spk_virtual_channel = 0;

using namespace m5avatar;
Avatar avatar;

extern "C" const Expression expressions_table[] = {
    Expression::Neutral,
    Expression::Happy,
    Expression::Sleepy,
    Expression::Doubt,
    Expression::Sad,
    Expression::Angry
};

extern "C" {
    void executeServoMotion(int x, int y, int duration);
    void resetServoPosition();
}

ESP32WebServer server(80);

bool is_speaking = false;

bool hasActivity = false;

unsigned long lastPowerCheckTime = 0;
const unsigned long POWER_CHECK_INTERVAL = 8000;
bool lastChargingState = false;
bool lastVbusState = false;
int lastBatteryLevel = -1;
float lastVbusVoltage = 0.0f;
bool lowBatteryWarning = false;
const int LOW_BATTERY_THRESHOLD = 15;

String speech_text = "";
String speech_text_buffer = "";
String deviceUniqueID;

String getDeviceUniqueID() {
  uint64_t mac = ESP.getEfuseMac();
  uint32_t high = (uint32_t)(mac >> 32);
  uint32_t low = (uint32_t)mac;
  return String(high, HEX) + String(low, HEX);
}

struct ServoPosition {
    int x;
    int y;
} lastServoPos = {0, 0};

int original_servo_speed = 30;

#define IMU_READ_INTERVAL 150
#define GYRO_THRESHOLD 25.0f
float lastGyroX = 0.0f;
float lastGyroY = 0.0f;
float lastGyroZ = 0.0f;
unsigned long lastImuReadTime = 0;
bool wheelControlEnabled = false;
float headCenterGyroZ = 0.0f;
bool headCenterCalibrated = false;

#define NOD_THRESHOLD 8.5f
#define NOD_COOLDOWN 1500
float headCenterGyroX = 0.0f;
bool isDetectingNod = false;
bool nodDetected = false;
unsigned long lastNodTime = 0;
int nodDirection = 0;
int nodPhase = 0;
unsigned long nodStartTime = 0;

unsigned long nodFeedbackStartTime = 0;
bool nodFeedbackActive = false;
#define NOD_FEEDBACK_DURATION 1500

bool pauseHeadShakeControl = false;
unsigned long wheelControlResumeTime = 0;

bool isAdjustingHeadPosition = false;

WavPlayer wavPlayer(&M5.Speaker, m5spk_virtual_channel);

const int expressions_options[] = {0, 1, 3, 4};
const int num_options = sizeof(expressions_options) / sizeof(expressions_options[0]);
bool servo_home = false;
bool servo_home_flag = false;
static void endChatSession(const char* tip) {
  chatSessionActive = false;
  is_speaking = false;
  avatar.setExpression(Expression::Happy);
  avatar.setSpeechText(tip && tip[0] ? tip : "下次再聊");
  delay(800);
  avatar.setSpeechText("");
  avatar.setExpression(Expression::Neutral);
}

String SpeechToText(){

  String ret = "";

     AudioWhisper* audio = new AudioWhisper();
    audio->Record();

    if (audio->ListenAborted() || audio->GetSize() <= 44) {
      delete audio;
      return "__EXIT_CHAT__";
    }

    avatar.setSpeechText("思考中...");

        int randomIndex = random(num_options);
        int selectedExpression = expressions_options[randomIndex];
        avatar.setExpression(expressions_table[selectedExpression]);

    SenseVoiceClient* senseClient = new SenseVoiceClient();
    ret = senseClient->Transcribe(audio);
    delete senseClient;
    delete audio;

  return ret;
}

bool is_searching_face =false;
bool face_detected = false;
bool is_tracking_target_with_wheels = false;
bool gestureControlPaused = false;
bool face_detected_flag = true;
bool sayhello = true;
unsigned long last_face_time = 0;
unsigned long face_stable_time = 0;
unsigned long greeting_stable_time = 0;
const int FACE_STABLE_THRESHOLD = 2000;
int search_pattern = 0;

const int FACE_TOO_CLOSE_THRESHOLD = 180;
unsigned long last_random_move = 0;
const int RANDOM_MOVE_INTERVAL = 5000;
bool face_too_close = false;
int gestureModuleX1 = -30;
int gestureModuleX2 = 31;
int gestureModuleY1 = -15;
int gestureModuleY2 = 16;

bool ear_servo_enabled = true;
unsigned long last_ear_move_time = 0;
const unsigned long EAR_MOVE_INTERVAL = 6000;
int current_ear_xx_angle = 90;
int current_ear_yy_angle = 90;
float ear_speed_factor = 1.0;

bool ear_servo_paused_for_wheels = false;
unsigned long ear_servo_pause_start_time = 0;
const unsigned long EAR_SERVO_PAUSE_DELAY = 1000;

int target_ear_xx_angle = 90;
int target_ear_yy_angle = 90;
bool is_ear_moving = false;
unsigned long ear_move_start_time = 0;
unsigned long current_ear_move_duration = 3000;
const unsigned long DEFAULT_EAR_MOVE_DURATION = 3000;

#define STOP_DUTY      7.5
#define FORWARD_DUTY   6.5
#define BACKWARD_DUTY  8.5

void setupWheels() {

  mcpwm_gpio_init(MCPWM_UNIT, MCPWM0A, MCPWM_IO_LEFT);
  mcpwm_gpio_init(MCPWM_UNIT, MCPWM0B, MCPWM_IO_RIGHT);

  mcpwm_config_t pwm_config;
  pwm_config.frequency = MCPWM_FREQ;
  pwm_config.cmpr_a = 0;
  pwm_config.cmpr_b = 0;
  pwm_config.counter_mode = MCPWM_UP_COUNTER;
  pwm_config.duty_mode = MCPWM_DUTY_MODE_0;

  mcpwm_init(MCPWM_UNIT, MCPWM_TIMER, &pwm_config);

  mcpwm_set_duty(MCPWM_UNIT, MCPWM_TIMER, MCPWM_OPR_A, STOP_DUTY);
  mcpwm_set_duty(MCPWM_UNIT, MCPWM_TIMER, MCPWM_OPR_B, STOP_DUTY);
  mcpwm_set_duty_type(MCPWM_UNIT, MCPWM_TIMER, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);
  mcpwm_set_duty_type(MCPWM_UNIT, MCPWM_TIMER, MCPWM_OPR_B, MCPWM_DUTY_MODE_0);
}

void setupEarServos() {
  pinMode(10, OUTPUT);

  mcpwm_gpio_init(EAR_MCPWM_UNIT, MCPWM0A, EAR_MCPWM_IO_XX);
  mcpwm_gpio_init(EAR_MCPWM_UNIT, MCPWM0B, EAR_MCPWM_IO_YY);

  mcpwm_config_t ear_pwm_config;
  ear_pwm_config.frequency = EAR_MCPWM_FREQ;
  ear_pwm_config.cmpr_a = 0;
  ear_pwm_config.cmpr_b = 0;
  ear_pwm_config.counter_mode = MCPWM_UP_COUNTER;
  ear_pwm_config.duty_mode = MCPWM_DUTY_MODE_0;

  mcpwm_init(EAR_MCPWM_UNIT, EAR_MCPWM_TIMER, &ear_pwm_config);

  mcpwm_set_duty(EAR_MCPWM_UNIT, EAR_MCPWM_TIMER, MCPWM_OPR_A, EAR_CENTER_DUTY);
  mcpwm_set_duty(EAR_MCPWM_UNIT, EAR_MCPWM_TIMER, MCPWM_OPR_B, EAR_CENTER_DUTY);
  mcpwm_set_duty_type(EAR_MCPWM_UNIT, EAR_MCPWM_TIMER, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);
  mcpwm_set_duty_type(EAR_MCPWM_UNIT, EAR_MCPWM_TIMER, MCPWM_OPR_B, MCPWM_DUTY_MODE_0);

  current_ear_xx_angle = 90;
  current_ear_yy_angle = 90;
}

void moveForward() {
  mcpwm_set_duty(MCPWM_UNIT, MCPWM_TIMER, MCPWM_OPR_A, FORWARD_DUTY);
  mcpwm_set_duty_type(MCPWM_UNIT, MCPWM_TIMER, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);

  mcpwm_set_duty(MCPWM_UNIT, MCPWM_TIMER, MCPWM_OPR_B, BACKWARD_DUTY);
  mcpwm_set_duty_type(MCPWM_UNIT, MCPWM_TIMER, MCPWM_OPR_B, MCPWM_DUTY_MODE_0);
}

void moveBackward() {
  mcpwm_set_duty(MCPWM_UNIT, MCPWM_TIMER, MCPWM_OPR_A, BACKWARD_DUTY);
  mcpwm_set_duty_type(MCPWM_UNIT, MCPWM_TIMER, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);

  mcpwm_set_duty(MCPWM_UNIT, MCPWM_TIMER, MCPWM_OPR_B, FORWARD_DUTY);
  mcpwm_set_duty_type(MCPWM_UNIT, MCPWM_TIMER, MCPWM_OPR_B, MCPWM_DUTY_MODE_0);
}

void turnLeft() {
  mcpwm_set_duty(MCPWM_UNIT, MCPWM_TIMER, MCPWM_OPR_A, BACKWARD_DUTY);
  mcpwm_set_duty_type(MCPWM_UNIT, MCPWM_TIMER, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);

  mcpwm_set_duty(MCPWM_UNIT, MCPWM_TIMER, MCPWM_OPR_B, BACKWARD_DUTY);
  mcpwm_set_duty_type(MCPWM_UNIT, MCPWM_TIMER, MCPWM_OPR_B, MCPWM_DUTY_MODE_0);
}

void turnRight() {
  mcpwm_set_duty(MCPWM_UNIT, MCPWM_TIMER, MCPWM_OPR_A, FORWARD_DUTY);
  mcpwm_set_duty_type(MCPWM_UNIT, MCPWM_TIMER, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);

  mcpwm_set_duty(MCPWM_UNIT, MCPWM_TIMER, MCPWM_OPR_B, FORWARD_DUTY);
  mcpwm_set_duty_type(MCPWM_UNIT, MCPWM_TIMER, MCPWM_OPR_B, MCPWM_DUTY_MODE_0);
}

void stopWheels() {
  mcpwm_set_duty(MCPWM_UNIT, MCPWM_TIMER, MCPWM_OPR_A, STOP_DUTY);
  mcpwm_set_duty_type(MCPWM_UNIT, MCPWM_TIMER, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);

  mcpwm_set_duty(MCPWM_UNIT, MCPWM_TIMER, MCPWM_OPR_B, STOP_DUTY);
  mcpwm_set_duty_type(MCPWM_UNIT, MCPWM_TIMER, MCPWM_OPR_B, MCPWM_DUTY_MODE_0);
}

void setEarServoPosition(int servo_xx_angle, int servo_yy_angle) {

  float duty_xx = EAR_MIN_DUTY + (servo_xx_angle / 180.0) * (EAR_MAX_DUTY - EAR_MIN_DUTY);
  float duty_yy = EAR_MIN_DUTY + (servo_yy_angle / 180.0) * (EAR_MAX_DUTY - EAR_MIN_DUTY);

  duty_xx = constrain(duty_xx, EAR_MIN_DUTY, EAR_MAX_DUTY);
  duty_yy = constrain(duty_yy, EAR_MIN_DUTY, EAR_MAX_DUTY);

  static float last_duty_xx = EAR_CENTER_DUTY;
  static float last_duty_yy = EAR_CENTER_DUTY;

  if (abs(duty_xx - last_duty_xx) > EAR_DEAD_ZONE) {
    mcpwm_set_duty(EAR_MCPWM_UNIT, EAR_MCPWM_TIMER, MCPWM_OPR_A, duty_xx);
    last_duty_xx = duty_xx;
  }

  if (abs(duty_yy - last_duty_yy) > EAR_DEAD_ZONE) {
    mcpwm_set_duty(EAR_MCPWM_UNIT, EAR_MCPWM_TIMER, MCPWM_OPR_B, duty_yy);
    last_duty_yy = duty_yy;
  }
}

void setEarServoPositionSmooth(int servo_xx_angle, int servo_yy_angle, unsigned long duration_ms) {

  target_ear_xx_angle = constrain(servo_xx_angle, 20, 161);
  target_ear_yy_angle = constrain(servo_yy_angle, 20, 161);

  is_ear_moving = true;
  ear_move_start_time = millis();

  current_ear_move_duration = duration_ms;
}

void updateEarServoPosition() {
  if (!is_ear_moving) return;

  unsigned long current_time = millis();
  unsigned long elapsed_time = current_time - ear_move_start_time;

  if (elapsed_time >= current_ear_move_duration) {

    setEarServoPosition(target_ear_xx_angle, target_ear_yy_angle);
    current_ear_xx_angle = target_ear_xx_angle;
    current_ear_yy_angle = target_ear_yy_angle;
    is_ear_moving = false;
  } else {

    float progress = (float)elapsed_time / current_ear_move_duration;

    float eased_progress;
    if (progress < 0.5) {
      eased_progress = 4 * progress * progress * progress;
    } else {
      float f = ((2 * progress) - 2);
      eased_progress = 0.5 * f * f * f + 1;
    }

    int current_xx = current_ear_xx_angle + (target_ear_xx_angle - current_ear_xx_angle) * eased_progress;
    int current_yy = current_ear_yy_angle + (target_ear_yy_angle - current_ear_yy_angle) * eased_progress;

    setEarServoPosition(current_xx, current_yy);
  }
}

void startInternalSpeaker() {
  if (!M5.Speaker.isRunning()) {
    Serial.println("正在启动内部扬声器...");
    Serial.printf("扬声器配置 - pin_data_out: %d, pin_bck: %d, pin_ws: %d\n",
                  M5.Speaker.config().pin_data_out,
                  M5.Speaker.config().pin_bck,
                  M5.Speaker.config().pin_ws);
    Serial.printf("扬声器配置 - i2s_port: %d, magnification: %d, sample_rate: %d\n",
                  M5.Speaker.config().i2s_port,
                  M5.Speaker.config().magnification,
                  M5.Speaker.config().sample_rate);

    bool result = M5.Speaker.begin();
    if (result) {
      M5.Speaker.setVolume(200);
      M5.Speaker.setChannelVolume(m5spk_virtual_channel, 200);
      Serial.println("内部扬声器已启动");
    } else {
      Serial.println("内部扬声器启动失败！");
    }
  } else {
    Serial.println("内部扬声器已经在运行");
  }
}

void stopInternalSpeaker() {
  if (M5.Speaker.isRunning()) {
    M5.Speaker.end();

    auto spk_cfg = M5.Speaker.config();
    if (spk_cfg.i2s_port >= 0) {
      esp_err_t err = i2s_driver_uninstall((i2s_port_t)spk_cfg.i2s_port);
      if (err == ESP_OK) {
        Serial.println("I2S 驱动已卸载");
      } else {
        Serial.printf("I2S 驱动卸载失败: %d\n", err);
      }
    }

    Serial.println("内部扬声器已停止");
  } else {
    Serial.println("内部扬声器未运行");
  }
}

void initVoiceWakeupI2S() {
  esp_err_t err;

  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = 16000,
    .bits_per_sample = i2s_bits_per_sample_t(16),
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_STAND_I2S),
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 4,
    .dma_buf_len = 1024,
    .use_apll = true,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0
  };

  i2s_pin_config_t pin_config = {
    .mck_io_num = 0,
    .bck_io_num = 34,
    .ws_io_num = 33,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num = 14
  };

  err = i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
  if (err != ESP_OK) {
    Serial.printf("麦克风 I2S 驱动安装失败: %d\n", err);
    return;
  }

  err = i2s_set_pin(I2S_NUM_0, &pin_config);
  if (err != ESP_OK) {
    Serial.printf("麦克风 I2S 引脚设置失败: %d\n", err);
    i2s_driver_uninstall(I2S_NUM_0);
    return;
  }

  err = i2s_zero_dma_buffer(I2S_NUM_0);
  if (err != ESP_OK) {
    Serial.printf("麦克风 DMA 缓冲区清零失败: %d\n", err);
    i2s_driver_uninstall(I2S_NUM_0);
    return;
  }

  Serial.println("麦克风 I2S 初始化成功");

}

void startInternalMic() {
  if (!M5.Mic.isRunning()) {
    Serial.println("正在启动内部麦克风...");
    Serial.printf("麦克风配置 - pin_data_in: %d, pin_ws: %d, pin_bck: %d\n",
                  M5.Mic.config().pin_data_in,
                  M5.Mic.config().pin_ws,
                  M5.Mic.config().pin_bck);
    Serial.printf("麦克风配置 - i2s_port: %d, sample_rate: %d, stereo: %s\n",
                  M5.Mic.config().i2s_port,
                  M5.Mic.config().sample_rate,
                  M5.Mic.config().stereo ? "是" : "否");

    bool result = M5.Mic.begin();
    if (result) {
      Serial.println("内部麦克风已启动");
      initVoiceWakeupI2S();
    } else {
      Serial.println("内部麦克风启动失败！");
    }
  } else {
    Serial.println("内部麦克风已经在运行");
  }
}

void stopInternalI2S() {
  esp_err_t err = i2s_driver_uninstall(I2S_NUM_0);
  if (err == ESP_OK) {
    Serial.println("麦克风 I2S 已卸载");
  } else {
    Serial.printf("麦克风 I2S 卸载失败: %d\n", err);
  }
}

void stopInternalMic() {
  if (M5.Mic.isRunning()) {
    M5.Mic.end();
    Serial.println("内部麦克风已停止");
    stopInternalI2S();
  } else {
    Serial.println("内部麦克风未运行");
  }
}

void SST_ChatGPT() {
  asrEnabled = false;
  chatSessionActive = true;

  wheelControlEnabled = false;
  face_detected = false;
  if(gestureModuleEnabled){
  is_searching_face = false;
  }
  face_detected_flag = false;

  stopWheels();

  stopInternalMic();

  startInternalMic();
  delay(100);

  String ret;
  avatar.setExpression(Expression::Neutral);
  ret = SpeechToText();
  speech_text = ret;

  startInternalMic();

  if (ret == "__EXIT_CHAT__") {
    endChatSession("下次再聊");
  }

  if(!is_speaking){
  }
  avatar.setSpeechText("");

  face_detected = true;
  if(gestureModuleEnabled){
  is_searching_face = true;
  }
  face_detected_flag = true;
  wheelControlEnabled = true;
  headCenterCalibrated = false;
  last_face_time = millis();
  asrEnabled = true;
}

bool isWheelsRunning = false;
unsigned long wheelStartTime = 0;
unsigned long wheelDuration = 0;
int currentWheelAction = 0;
int ignoreUpGestureCount = 0;
int ignoreRightGestureCount = 0;
int ignoreLeftGestureCount = 0;
bool ignoreRightGestureonetime = false;

void startMoveForward(unsigned long duration) {
  moveForward();
  isWheelsRunning = true;
  wheelStartTime = millis();
  wheelDuration = duration;
  currentWheelAction = 1;

}

void startMoveBackward(unsigned long duration) {
  moveBackward();
  isWheelsRunning = true;
  wheelStartTime = millis();
  wheelDuration = duration;
  currentWheelAction = 2;

}

void startTurnLeft(unsigned long duration) {
  turnLeft();
  isWheelsRunning = true;
  wheelStartTime = millis();
  wheelDuration = duration;
  currentWheelAction = 3;

}

void startTurnRight(unsigned long duration) {
  turnRight();
  isWheelsRunning = true;
  wheelStartTime = millis();
  wheelDuration = duration;
  currentWheelAction = 4;

}

void emergencyStop() {
  stopWheels();
  isWheelsRunning = false;
  currentWheelAction = 0;

}

void checkWheelStatus() {
  if (isWheelsRunning && (millis() - wheelStartTime >= wheelDuration)) {

    int completedAction = currentWheelAction;

    stopWheels();
    isWheelsRunning = false;
    currentWheelAction = 0;

    switch (completedAction) {
      case 1:

        break;
      case 2:

        break;
      case 3:

        break;
      case 4:

        break;
    }
  }
}

#ifdef USE_SERVO
#define START_DEGREE_VALUE_X 90

#define START_DEGREE_VALUE_Y 90
EasyServo servo_x;
EasyServo servo_y;

#define START_DEGREE_VALUE_XX 90

#define START_DEGREE_VALUE_YY 90
EasyServo servo_xx;
EasyServo servo_yy;

#endif

void earServoTask(void *args) {

  setupEarServos();

  setEarServoPosition(START_DEGREE_VALUE_XX, START_DEGREE_VALUE_YY);
  current_ear_xx_angle = START_DEGREE_VALUE_XX;
  current_ear_yy_angle = START_DEGREE_VALUE_YY;

    randomSeed(millis());

  for (;;) {
    if (ear_servo_enabled) {
      unsigned long current_time = millis();

      if (isWheelsRunning) {
        if (!ear_servo_paused_for_wheels) {

          ear_servo_paused_for_wheels = true;
          ear_servo_pause_start_time = current_time;

        }

        vTaskDelay(pdMS_TO_TICKS(100));
        continue;
      } else {

        if (ear_servo_paused_for_wheels) {

          if (current_time - ear_servo_pause_start_time > EAR_SERVO_PAUSE_DELAY) {
            ear_servo_paused_for_wheels = false;

            last_ear_move_time = 0;
          } else {

            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
          }
        }
      }

      if (current_time - last_ear_move_time > EAR_MOVE_INTERVAL && !is_ear_moving) {

        int target_xx = random(20, 161);
        int target_yy = 180-target_xx;

        unsigned long move_duration = random(3300, 3501);

        setEarServoPositionSmooth(target_xx, target_yy, move_duration);

        last_ear_move_time = current_time;
      }

      updateEarServoPosition();

      vTaskDelay(pdMS_TO_TICKS(20));
    } else {

      vTaskDelay(pdMS_TO_TICKS(500));
    }
  }
}

void lipSync(void *args)
{
  float gazeX, gazeY;
  int level = 0;
  DriveContext *ctx = (DriveContext *)args;
  Avatar *avatar = ctx->getAvatar();
  for (;;)
  {
    level = abs(*wavPlayer.getBuffer());
    if(level < 100) level = 0;
    if(level > 12000) {
      level = 12000;
    }
    float open = (float)level/12000.0;
    avatar->setMouthOpenRatio(open);
    avatar->getGaze(&gazeY, &gazeX);
    avatar->setRotation(gazeX * 5);
    delay(20);
  }
}

bool is_shaking_head = false;
int shake_step = 0;

const int CENTER_X = 160;
const int CENTER_Y = 120;
int g_face_centroid_x = CENTER_X;
int g_face_centroid_y = CENTER_Y;
int g_angle_x = 0;
int g_angle_y = 0;
void coordinateWheelServoTracking(bool is_wheel_tracking, int diff_x, int diff_y, int current_x, int current_y, int tracking_speed) {

    float servo_speed_factor = 1.0f;
    float servo_movement_factor = 1.0f;

    if (is_wheel_tracking) {

        servo_speed_factor = 0.8f;
        servo_movement_factor = 0.6f;

        if (abs(diff_x) > 40) {

            servo_movement_factor = 0.8f;
        } else if (abs(diff_y) > 15) {

            servo_movement_factor = 0.7f;
        }

    }

    if (servo_x.getSpeed() != tracking_speed * servo_speed_factor) {
        servo_x.setSpeed(tracking_speed * servo_speed_factor);
        servo_y.setSpeed(tracking_speed * servo_speed_factor);
    }

    float x_factor = (abs(diff_x) > 100) ? 0.1f : (abs(diff_x) > 50 ? 0.08f : 0.05f);
    float y_factor = (abs(diff_y) > 80) ? 0.1f : (abs(diff_y) > 40 ? 0.08f : 0.05f);

    x_factor *= servo_movement_factor;
    y_factor *= servo_movement_factor;

    if (abs(diff_x) > 25) {
        int target_x = constrain(current_x + (x_factor * diff_x), 30, 150);
        servo_x.setEaseTo(target_x);
    }
    if (abs(diff_y) > 8) {
        int target_y = constrain(current_y + (y_factor * diff_y), 65, 115);
        servo_y.setEaseTo(target_y);
    }
}

void shakeHead() {
    if (!is_shaking_head) {
        is_shaking_head = true;
        shake_step = 0;
        lastServoPos.x = servo_x.read();

        setSpeedForAllServos(80);
    }
}

bool isFaceDetected;
void servo(void *args)
{

    static unsigned long last_face_movement_time = 0;
    static int last_face_x = 0;
    static int last_face_y = 0;
    static bool is_random_moving = false;
    static ServoPosition lastStablePos = {0, 0};
    static int last_diff_x = 0;
    static int last_diff_y = 0;
    static float filtered_diff_x = 0.0f;
    static float filtered_diff_y = 0.0f;

    int FACE_STATIC_THRESHOLD = 1500;
    const int RANDOM_MOVE_RANGE_X = 15;
    const int RANDOM_MOVE_RANGE_Y = 10;

    static unsigned long last_random_move = 0;
    unsigned long current_time = 0;
    int target_x = 0, target_y = 0;
    int diff_x = 0, diff_y = 0;
    float distance_factor = 0.03;
    bool last_face_detected_state = false;

    DriveContext *ctx = (DriveContext *)args;
    if (!ctx) {

        vTaskDelete(NULL);
        return;
    }

    const int SERVO_SPEED = 480;
    const int TRACKING_SPEED = 120;
    const int MID_TRACKING_SPEED = 200;
    const int FAST_TRACKING_SPEED = 300;
    const int SEARCH_SPEED = 30;
    const float DIFF_SMOOTHING = 0.65f;

    last_face_detected_state = face_detected;
    last_random_move = millis();

    for (;;)
    {
#ifdef USE_SERVO

        current_time = millis();

        if (face_detected && !last_face_detected_state) {

            servo_x.stop();
            servo_y.stop();
            setSpeedForAllServos(TRACKING_SPEED);

        }

        else if (!face_detected && last_face_detected_state) {
            setSpeedForAllServos(SEARCH_SPEED);

        }

        last_face_detected_state = face_detected;

        if (is_shaking_head) {

            setSpeedForAllServos(TRACKING_SPEED * 0.5);
            switch(shake_step) {
                case 0:
                    servo_y.setEaseTo(START_DEGREE_VALUE_Y + 15);
                    shake_step++;
                    break;
                case 1:
                    servo_y.setEaseTo(START_DEGREE_VALUE_Y + 25);
                    shake_step++;
                    delay(150);
                    break;
                case 2:
                    servo_y.setEaseTo(START_DEGREE_VALUE_Y - 5);
                    shake_step++;
                    delay(150);
                    break;
                case 3:
                    servo_y.setEaseTo(START_DEGREE_VALUE_Y + 15);
                    shake_step++;
                    delay(200);
                    break;
                case 4:
                    servo_y.setEaseTo(START_DEGREE_VALUE_Y - 3);
                    shake_step++;
                    delay(150);
                    break;
                case 5:
                    if (!servo_y.isMoving()) {
                        servo_y.setEaseTo(START_DEGREE_VALUE_Y);
                        delay(150);
                        shake_step = 0;
                        is_shaking_head = false;

                        if (face_detected) {
                            setSpeedForAllServos(TRACKING_SPEED);
                        } else {
                            setSpeedForAllServos(SEARCH_SPEED);
                        }
                    }
                    break;
            }
        }
        else if (face_detected) {
            int current_x = servo_x.read();
            int current_y = servo_y.read();

            diff_x = g_face_centroid_x - CENTER_X;
            diff_y = g_face_centroid_y - CENTER_Y;

            filtered_diff_x = DIFF_SMOOTHING * filtered_diff_x + (1.0f - DIFF_SMOOTHING) * diff_x;
            filtered_diff_y = DIFF_SMOOTHING * filtered_diff_y + (1.0f - DIFF_SMOOTHING) * diff_y;

            int smooth_diff_x = (int)filtered_diff_x;
            int smooth_diff_y = (int)filtered_diff_y;
            int smooth_abs_diff_x = abs(smooth_diff_x);
            int smooth_abs_diff_y = abs(smooth_diff_y);
            int abs_diff_x = abs(diff_x);
            int abs_diff_y = abs(diff_y);

            int velocity_x = smooth_diff_x - last_diff_x;
            int velocity_y = smooth_diff_y - last_diff_y;
            last_diff_x = smooth_diff_x;
            last_diff_y = smooth_diff_y;
            int predictive_diff_x = smooth_diff_x + velocity_x * 2;
            int predictive_diff_y = smooth_diff_y + velocity_y * 2;

            int desired_speed = TRACKING_SPEED;
            if (smooth_abs_diff_x > 110 || smooth_abs_diff_y > 80) {
                desired_speed = FAST_TRACKING_SPEED;
            } else if (smooth_abs_diff_x > 70 || smooth_abs_diff_y > 50) {
                desired_speed = MID_TRACKING_SPEED;
            }
            if (servo_x.getSpeed() != desired_speed) {
                setSpeedForAllServos(desired_speed);
            }

            static int last_face_x = g_face_centroid_x;
            static int last_face_y = g_face_centroid_y;

            if (abs(last_face_x - g_face_centroid_x) > 3 || abs(last_face_y - g_face_centroid_y) > 3) {
                last_face_movement_time = current_time;
                is_random_moving = false;

                last_face_x = g_face_centroid_x;
                last_face_y = g_face_centroid_y;
            }

            if (face_too_close) {
                g_angle_y = constrain(g_angle_y - 5, -15, 5);
                avatar.setExpression(Expression::Doubt);
                startMoveBackward(3000);
            }
            else {

                if (!is_random_moving && (current_time - last_face_movement_time > FACE_STATIC_THRESHOLD) && smooth_abs_diff_x < 25 && smooth_abs_diff_y < 20) {

                    lastStablePos.x = current_x;
                    lastStablePos.y = current_y;

                    target_x = lastStablePos.x + random(-RANDOM_MOVE_RANGE_X, RANDOM_MOVE_RANGE_X + 1);
                    target_y = lastStablePos.y + random(-RANDOM_MOVE_RANGE_Y, RANDOM_MOVE_RANGE_Y + 1);

                    target_x = constrain(target_x, 30, 150);
                    target_y = constrain(target_y, 65, 115);

                    setSpeedForAllServos(SEARCH_SPEED);

                    servo_y.setEaseTo(target_y);

                    is_random_moving = true;

                }

                else if (is_random_moving && !servo_x.isMoving() && !servo_y.isMoving()) {
                    setSpeedForAllServos(SEARCH_SPEED);

                    servo_y.setEaseTo(lastStablePos.y);
                    FACE_STATIC_THRESHOLD = random(3000, 5000);
                    is_random_moving = false;

                }

                else if (!is_random_moving) {

                    if (avatar.getExpression() == Expression::Doubt && !nodFeedbackActive) {
                        avatar.setExpression(Expression::Neutral);
                        avatar.setSpeechText("");
                    }

                    if (!nodFeedbackActive) {

                        float x_factor = (smooth_abs_diff_x > 120) ? 0.18f : (smooth_abs_diff_x > 70 ? 0.12f : 0.07f);
                        float y_factor = (smooth_abs_diff_y > 90) ? 0.15f : (smooth_abs_diff_y > 50 ? 0.1f : 0.06f);
                        int effective_diff_x = (smooth_abs_diff_x > 40) ? constrain(predictive_diff_x, -200, 200) : smooth_diff_x;
                        int effective_diff_y = (smooth_abs_diff_y > 30) ? constrain(predictive_diff_y, -160, 160) : smooth_diff_y;

                        static unsigned long last_wheel_tracking_time = 0;
                        static bool is_wheel_tracking = false;
                        const unsigned long WHEEL_TRACKING_INTERVAL = 8000;
                        const unsigned long WHEEL_TRACKING_DURATION = 1000;

                        static unsigned long last_face_check_time = 0;
                        static int last_face_check_x = 0;
                        static int last_face_check_y = 0;
                        const unsigned long FACE_MOVEMENT_CHECK_INTERVAL = 2000;
                        const int FACE_MOVEMENT_THRESHOLD = 8;

                        coordinateWheelServoTracking(is_wheel_tracking, smooth_diff_x, smooth_diff_y, current_x, current_y, TRACKING_SPEED);
                    }
                }
            }

            if (!face_too_close && abs(diff_x) < 40 && abs(diff_y) < 40) {
                if (greeting_stable_time == 0) {
                    greeting_stable_time = current_time;
                }
                else if (current_time - greeting_stable_time > FACE_STABLE_THRESHOLD && sayhello) {

                    sayhello = false;
                    greeting_stable_time = 0;
                }
            }
            else {
                greeting_stable_time = 0;
            }
        }
        else if (is_searching_face && !servo_home && !mainsleep) {

            if (servo_x.getSpeed() != SEARCH_SPEED) {
                setSpeedForAllServos(SEARCH_SPEED);
            }

                            if (current_time - last_random_move > RANDOM_MOVE_INTERVAL) {
                    last_random_move = current_time;

                    target_x = START_DEGREE_VALUE_X + random(gestureModuleX1, gestureModuleX2);
                    target_y = START_DEGREE_VALUE_Y + random(gestureModuleY1, gestureModuleY2);

                    servo_x.setEaseTo(target_x);
                    servo_y.setEaseTo(target_y);
                }
        }

        synchronizeAllServosStartAndWaitForAllServosToStop();
#endif

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

bool detectNodding(float gyroX) {

    unsigned long currentTime = millis();

    if (currentTime - lastNodTime < NOD_COOLDOWN) {
        return false;
    }

    float deltaFromCenterX = gyroX - headCenterGyroX;

    if (!isDetectingNod || (abs(deltaFromCenterX) < 0.8f && currentTime - lastImuReadTime > 400)) {

        if (isDetectingNod) {
            headCenterGyroX = headCenterGyroX * 0.6f + gyroX * 0.4f;
        } else {
            headCenterGyroX = gyroX;
            isDetectingNod = true;
        }
    }

    if (abs(deltaFromCenterX) > NOD_THRESHOLD) {

        if (nodPhase == 0) {

            nodPhase = 1;
            nodStartTime = currentTime;
            nodDirection = (deltaFromCenterX > 0) ? 1 : -1;
        }

        else if (nodPhase == 1 && ((nodDirection > 0 && deltaFromCenterX < -NOD_THRESHOLD * 0.5) ||
                                   (nodDirection < 0 && deltaFromCenterX > NOD_THRESHOLD * 0.5))) {

            nodPhase = 2;
        }
    }

    if (nodPhase == 2) {
        unsigned long nodDuration = currentTime - nodStartTime;

        if (nodDuration > 250 && nodDuration < 2000) {

            Serial.printf("检测到有效点头动作! 方向:%s 持续时间:%lu ms\n",
                           (nodDirection > 0) ? "向下点头" : "向上仰头", nodDuration);

            int currentServoX = servo_x.read();
            int diffFromCenter = currentServoX - 90;

            unsigned long turnDuration = map(abs(diffFromCenter), 0, 30, 50, 250);
            turnDuration = constrain(turnDuration, 250, 500);

            avatar.setExpression(Expression::Happy);

            if (diffFromCenter > 12) {
                Serial.printf("检测到点头动作，头部偏右 %d 度，轮子右转 %lu ms\n", diffFromCenter, turnDuration);
                startTurnRight(turnDuration);

                pauseHeadShakeControl = true;
                wheelControlResumeTime = currentTime + turnDuration + 500;
                Serial.println("暂停摇头控制轮子");
            } else if (diffFromCenter < -12) {
                Serial.printf("检测到点头动作，头部偏左 %d 度，轮子左转 %lu ms\n", -diffFromCenter, turnDuration);
                startTurnLeft(turnDuration);

                pauseHeadShakeControl = true;
                wheelControlResumeTime = currentTime + turnDuration + 500;
                Serial.println("暂停摇头控制轮子");
            } else {
                Serial.println("检测到点头动作，头部正中，轮子前进 1500 ms");
                startMoveForward(500);

                pauseHeadShakeControl = true;
                wheelControlResumeTime = currentTime + 500 + 500;
                Serial.println("暂停摇头控制轮子");
            }

            nodFeedbackActive = true;
            nodFeedbackStartTime = currentTime;

            nodPhase = 0;
            lastNodTime = currentTime;
            nodDetected = true;

            return true;
        } else {

            Serial.printf("点头动作时间异常 (%lu ms)，忽略\n", nodDuration);
            nodPhase = 0;
        }
    }

    if (nodPhase > 0 && currentTime - nodStartTime > 2000) {
        Serial.println("点头检测超时，重置状态");
        nodPhase = 0;
    }

    return false;
}

void Servo_setup() {
#ifdef USE_SERVO

  Serial.println("开始舵机初始化...");

    int current_x = START_DEGREE_VALUE_X;
    int current_y = START_DEGREE_VALUE_Y;

  if(servo_home_flag) {
     current_x = servo_x.read();
     current_y = servo_y.read();

  } else {
     servo_home_flag = true;
  }

  Serial.println("初始化头部舵机...");
        int result = servo_x.attach(SERVO_PIN_X, current_x, DEFAULT_MICROSECONDS_FOR_0_DEGREE, DEFAULT_MICROSECONDS_FOR_180_DEGREE);
        Serial.printf("servo_x初始化结果: %d\n", result);

  delay(50);

  result = servo_y.attach(SERVO_PIN_Y, current_y, DEFAULT_MICROSECONDS_FOR_0_DEGREE, DEFAULT_MICROSECONDS_FOR_180_DEGREE);
  Serial.printf("servo_y初始化结果: %d\n", result);
  delay(50);

  Serial.println("\n初始化轮子...");
  setupWheels();

  Serial.println("初始化耳朵舵机...");
  setupEarServos();

  servo_x.setEasingType(EASE_QUADRATIC_IN_OUT);
  servo_y.setEasingType(EASE_QUADRATIC_IN_OUT);

  setSpeedForAllServos(30);

  synchronizeAllServosStartAndWaitForAllServosToStop();
#endif
}

void demoWheelControls() {

  Serial.println("演示非阻塞控制...");

  startMoveForward(500);

  Serial.println("演示中：轮子前进时耳朵舵机应该暂停");

  while(isWheelsRunning) {
    checkWheelStatus();
    delay(100);
  }

  Serial.println("演示完成：轮子停止后耳朵舵机应该恢复");

  Serial.println("轮子控制演示完成");
  delay(500);
  avatar.setSpeechText("");
}

void controlHeadByServoPosition() {

    static bool servoCenterCalibrated = false;
    static int servoCenterX = 90;
    static unsigned long lastServoActionTime = 0;
    static int consecutiveServoActions = 0;
    static bool pauseServoControl = false;
    static unsigned long servoControlResumeTime = 0;

    const int SERVO_CENTER_POSITION = 90;
    const int SERVO_THRESHOLD = 8;
    const int MAX_SERVO_CONSECUTIVE_ACTIONS = 2;
    const unsigned long MIN_SERVO_ACTION_INTERVAL = 300;
    const unsigned long SERVO_PAUSE_DURATION = 1000;

    if (pauseServoControl) {
        unsigned long currentTime = millis();
        if (currentTime >= servoControlResumeTime) {
            pauseServoControl = false;
            consecutiveServoActions = 0;
            Serial.println("舵机控制暂停结束，恢复控制");
        } else {

            if (!servoCenterCalibrated) {
                int currentServoX = servo_x.read();
                servoCenterX = currentServoX;
                servoCenterCalibrated = true;
                Serial.printf("舵机中心位置校准: X=%d\n", servoCenterX);
            }
            return;
        }
    }

    if (!face_detected && (radar_state.is_tracking && radar_state.distance <= 150)) {

      if (!servoCenterCalibrated) {
          servoCenterX = SERVO_CENTER_POSITION;
          servoCenterCalibrated = true;
          Serial.printf("舵机中心位置校准: X=%d (固定中心位置)\n", servoCenterX);
      }

      int currentServoX = servo_x.read();

      int deltaFromCenter = currentServoX - servoCenterX;

      unsigned long currentTime = millis();
      if (currentTime - lastServoActionTime < MIN_SERVO_ACTION_INTERVAL) {
          return;
      }

      int effectiveThreshold = SERVO_THRESHOLD;
      if (consecutiveServoActions > 0) {
          effectiveThreshold = SERVO_THRESHOLD + (consecutiveServoActions * 2);
      }

      if (abs(deltaFromCenter) > effectiveThreshold && !is_shaking_head) {

          unsigned long turnDuration = abs(deltaFromCenter) * 25;
          turnDuration = constrain(turnDuration, 100, 1600);

          if (consecutiveServoActions > 0) {
              turnDuration = turnDuration / (consecutiveServoActions + 1);
          }

          if (deltaFromCenter > 0) {

              Serial.printf("头部向右偏(X=%d，偏差=%d)，轮子右转 %lums (连续次数:%d)\n",
                           currentServoX, deltaFromCenter, turnDuration, consecutiveServoActions);
              startTurnRight(turnDuration);

              if (radar_state.distance > 50 && gestureModuleEnabled) {

                  unsigned long forwardDuration = map(radar_state.distance, 50, 150, 500, 800);
                  forwardDuration = constrain(forwardDuration, 500, 800);

                  if (turnDuration < 400) {

                      Serial.printf("距离较远(%.1fcm)，转向时间短，转向后立即前进 %lums 靠近目标\n",
                                   radar_state.distance, forwardDuration);
                      startMoveForward(forwardDuration);
                  } else {

                      Serial.printf("距离较远(%.1fcm)，转向时间长(%lums)，暂不前进，避免覆盖转向\n",
                                   radar_state.distance, turnDuration);
                  }
              }

              if (radar_state.distance < 40) {

              }
          } else {

              Serial.printf("头部向左偏(X=%d，偏差=%d)，轮子左转 %lums (连续次数:%d)\n",
                           currentServoX, deltaFromCenter, turnDuration, consecutiveServoActions);
              startTurnLeft(turnDuration);

              if (radar_state.distance > 50 && gestureModuleEnabled) {

                  unsigned long forwardDuration = map(radar_state.distance, 50, 150, 500, 800);
                  forwardDuration = constrain(forwardDuration, 500, 800);

                  if (turnDuration < 400) {

                      Serial.printf("距离较远(%.1fcm)，转向时间短，转向后立即前进 %lums 靠近目标\n",
                                   radar_state.distance, forwardDuration);
                      startMoveForward(forwardDuration);
                  } else {

                      Serial.printf("距离较远(%.1fcm)，转向时间长(%lums)，暂不前进，避免覆盖转向\n",
                                   radar_state.distance, turnDuration);
                  }
              }

              if (radar_state.distance < 40) {

              }
          }

          lastServoActionTime = currentTime;
          consecutiveServoActions++;

          if (consecutiveServoActions >= MAX_SERVO_CONSECUTIVE_ACTIONS) {
              pauseServoControl = true;
              servoControlResumeTime = currentTime + SERVO_PAUSE_DURATION;
              consecutiveServoActions = 0;
              Serial.println("达到最大连续次数，暂停舵机控制2秒");
          }

      } else if (abs(deltaFromCenter) < SERVO_THRESHOLD / 2) {

          consecutiveServoActions = 0;
      }

  } else {

      servoCenterCalibrated = false;
      consecutiveServoActions = 0;
      if (wheelControlEnabled) {
          stopWheels();
      }
  }

}

void executeServoMotion(int x, int y, int duration) {

    lastServoPos.x = x;
    lastServoPos.y = y;

    setSpeedForAllServos(duration > 0 ? duration : 30);

    servo_x.setEaseTo(START_DEGREE_VALUE_X + x);
    servo_y.setEaseTo(START_DEGREE_VALUE_Y + y);

    synchronizeAllServosStartAndWaitForAllServosToStop();
}

void resetServoPosition() {

    servo_x.setEaseTo(START_DEGREE_VALUE_X);
    servo_y.setEaseTo(START_DEGREE_VALUE_Y);
    setSpeedForAllServos(30);

    synchronizeAllServosStartAndWaitForAllServosToStop();

    lastServoPos.x = 0;
    lastServoPos.y = 0;
}

struct box_t
{
  int x;
  int y;
  int w;
  int h;
  int touch_id = -1;

  void setupBox(int x, int y, int w, int h) {
    this->x = x;
    this->y = y;
    this->w = w;
    this->h = h;
  }
  bool contain(int x, int y)
  {
    return this->x <= x && x < (this->x + this->w)
        && this->y <= y && y < (this->y + this->h);
  }
};
static box_t box_servo;
static box_t box_stt;
static box_t box_BtnA;
static box_t box_BtnC;

#define MAX_NETWORKS 20
struct WifiNetwork {
    String ssid;
    int32_t rssi;
};
WifiNetwork networks[MAX_NETWORKS];
int wifi_scan_count = 0;
bool scanComplete = false;

void Wifi_setup() {

  char saved_ssid[32] = "";
  char saved_pass[64] = "";
  uint32_t nvs_handle;
  if (nvs_open("wifi", NVS_READONLY, &nvs_handle) == ESP_OK) {
    size_t len = sizeof(saved_ssid);
    nvs_get_str(nvs_handle, "ssid", saved_ssid, &len);
    len = sizeof(saved_pass);
    nvs_get_str(nvs_handle, "pass", saved_pass, &len);
    nvs_close(nvs_handle);
  }

  if (strlen(saved_ssid) > 0) {
    WiFi.begin(saved_ssid, saved_pass);
    M5.Display.setFont(&fonts::efontCN_12);

    for(int i=0; i<10; i++){
      if (WiFi.status() == WL_CONNECTED) break;

      delay(500);
    }
  }

  if (WiFi.status() != WL_CONNECTED) {
    M5.Display.setFont(&fonts::efontCN_12);
    M5.Display.println(u8"\n连接失败,进入配置模式");

    WiFi.mode(WIFI_AP_STA);
    String ap_ssid = "Palbei-" + String((uint32_t)ESP.getEfuseMac(), HEX);
    WiFi.softAP(ap_ssid.c_str());

    delay(1000);

    Serial.println("开始扫描WiFi网络...");
    WiFi.disconnect(true);
    WiFi.scanDelete();

    int16_t scanResult = WiFi.scanNetworks(
        false,
        true,
        false,
        100,
        0
    );

    if (scanResult >= 0) {
        wifi_scan_count = (scanResult < MAX_NETWORKS) ? scanResult : MAX_NETWORKS;
        Serial.printf("\n扫描完成,发现 %d 个网络\n", wifi_scan_count);

        for(int i = 0; i < wifi_scan_count; i++) {
            networks[i].ssid = WiFi.SSID(i);
            networks[i].rssi = WiFi.RSSI(i);
            Serial.printf("网络 %d: %s (%ddBm)\n", i + 1, networks[i].ssid.c_str(), networks[i].rssi);
        }
    } else {
        Serial.printf("\n扫描失败,错误代码: %d\n", scanResult);
        wifi_scan_count = 0;
    }

    WiFi.softAP(ap_ssid.c_str());

    M5.Display.fillScreen(TFT_BLACK);
    M5.Display.setCursor(0, 0);
    M5.Display.setFont(&fonts::efontCN_16);
    M5.Display.println(u8"配置WiFi");
    M5.Display.println(u8"");
    M5.Display.printf("连接网络名称:%s\n", ap_ssid.c_str());
    M5.Display.println(u8"浏览器中打开: 192.168.4.1");
    M5.Display.println(u8"60秒内未配置将自动跳过");

    server.on("/", [](){
      String html = R"(
<html>
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>WiFi配置</title>
  <style>
    body {font-family: Arial; padding: 20px;}
    select, input {width: 100%; padding: 8px; margin: 5px 0 20px 0;}
    button {
      background: #4CAF50;
      color: white;
      padding: 12px 24px;
      border: none;
      border-radius: 4px;
      font-size: 16px;
      margin: 5px;
    }
    .container {
      max-width: 400px;
      margin: 0 auto;
      padding: 20px;
      background: #f5f5f5;
      border-radius: 8px;
    }
    h2 {color: #333; text-align: center;}
    #networks {
      margin-bottom: 20px;
      border: 1px solid #ddd;
      border-radius: 4px;
    }
    .countdown {
      text-align: center;
      margin-top: 20px;
      color: #f44336;
      font-weight: bold;
    }
  </style>
</head>
<body>
  <div class="container">
    <h2>WiFi配置</h2>
    <form method="post" action="/config">
      <label for="networks">可用网络:</label>
      <select id="networks" name="ssid" required>
        <option value="">请选择WiFi网络...</option>)";

      if (wifi_scan_count > 0) {
        for (int i = 0; i < wifi_scan_count; ++i) {
          if (networks[i].ssid.length() > 0) {
            html += "<option value=\"" + networks[i].ssid + "\">" +
                    networks[i].ssid + "</option>";
          }
        }
      }

      html += R"(
      </select>

      <label for="pass">密码:</label>
      <input type="password" id="pass" name="pass"
             placeholder="请输入WiFi密码">

      <button type="submit">立即连接</button>
    </form>
    <div class="countdown">60秒后将自动退出配置模式</div>
  </div>
</body>
</html>)";

      server.send(200, "text/html; charset=utf-8", html);

      WiFi.scanDelete();
    });

    server.on("/config", HTTP_POST, [](){
      String ssid = server.arg("ssid");
      String pass = server.arg("pass");

      if(ssid.length() > 0){

        uint32_t nvs_handle;
        nvs_open("wifi", NVS_READWRITE, &nvs_handle);
        nvs_set_str(nvs_handle, "ssid", ssid.c_str());
        nvs_set_str(nvs_handle, "pass", pass.c_str());
        nvs_commit(nvs_handle);
        nvs_close(nvs_handle);

        server.send(200, "text/plain; charset=utf-8",
          "配置已保存，设备正在重启...\n"
          "你可以关闭此页面");
        delay(3000);
        ESP.restart();
      }else{
        server.send(400, "text/plain; charset=utf-8", "WiFi名称不能为空");
      }
    });

    server.begin();

    unsigned long startTime = millis();
    unsigned long timeout = 60000;
    unsigned long lastUpdateTime = 0;
    int lastShownSeconds = 60;

    while(millis() - startTime < timeout) {
      server.handleClient();

      unsigned long currentTime = millis();
      int remainingSeconds = (timeout - (currentTime - startTime)) / 1000;

      if (remainingSeconds != lastShownSeconds && currentTime - lastUpdateTime >= 1000) {
        lastShownSeconds = remainingSeconds;
        lastUpdateTime = currentTime;

        M5.Display.fillRect(0, 120, 320, 30, TFT_BLACK);
        M5.Display.setCursor(0, 120);
        M5.Display.printf("倒计时: %d 秒", remainingSeconds);

        Serial.printf("WiFi配置倒计时: %d 秒\n", remainingSeconds);
      }

      delay(10);
    }

    WiFi.softAPdisconnect(true);
    server.close();

    M5.Display.fillScreen(TFT_BLACK);
    M5.Display.setCursor(0, 0);
    M5.Display.println(u8"WiFi配置超时，继续启动...");
    delay(1000);
  }
  else {

  }
}

#if defined(ENABLE_FACE_DETECT)
esp_err_t camera_init(){

    M5.In_I2C.release();
    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK) {

        M5.Display.println("Camera Init Failed");
        return err;
    }

    camera_initialized = true;
    return ESP_OK;
}

TaskHandle_t cameraTaskHandle = NULL;

bool resetCamera() {
  Serial.println("正在重置相机...");
  avatar.setExpression(Expression::Doubt);

  if (cameraTaskHandle != NULL) {
    vTaskSuspend(cameraTaskHandle);
  }

  camera_needs_reset = false;

  if (frame_buffer_copy) {
    free(frame_buffer_copy);
    frame_buffer_copy = nullptr;
    buffer_allocated = false;
  }

  if (camera_initialized) {
    esp_camera_deinit();
    camera_initialized = false;
  }

  delay(500);

  esp_err_t err = camera_init();
  if (err != ESP_OK) {
    Serial.println("相机重置失败");
    M5.Display.println("相机重置失败");

    if (cameraTaskHandle != NULL) {
      vTaskResume(cameraTaskHandle);
    }

    return false;
  }

  Serial.println("相机重置成功");

  if (enableGestureDetection && gestureModuleEnabled) {
    if (cameraGesture.initialize()) {
      Serial.println("手势识别器重新初始化成功");
    } else {
      Serial.println("手势识别器重新初始化失败");
    }
  }

  if (cameraTaskHandle != NULL) {
    vTaskResume(cameraTaskHandle);
  }

  avatar.setExpression(Expression::Neutral);
  return true;
}

int forward_count = 0;
unsigned long last_target_time = 0;
bool camera_capture_and_face_detect(){
  bool isDetected = false;

  if (camera_needs_reset) {
    if (!resetCamera()) {
      return false;
    }
  }

  try {

    esp_task_wdt_init(CAM_WATCHDOG_TIMEOUT, true);
    esp_task_wdt_add(NULL);

    M5.In_I2C.release();
    camera_fb_t * fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("Camera Capture Failed");

      esp_task_wdt_reset();
      esp_task_wdt_delete(NULL);
      return false;
    }

    if(fb->len == 0 || fb->buf == nullptr) {
      esp_camera_fb_return(fb);

      esp_task_wdt_reset();
      esp_task_wdt_delete(NULL);
      return false;
    }

    if (!buffer_allocated || frame_buffer_size < fb->len) {
      if (frame_buffer_copy) {
        free(frame_buffer_copy);
      }
      frame_buffer_copy = (uint8_t*)heap_caps_malloc(fb->len, MALLOC_CAP_SPIRAM);
      if (!frame_buffer_copy) {
        Serial.println("无法分配帧缓冲区拷贝内存");
        esp_camera_fb_return(fb);
        esp_task_wdt_reset();
        esp_task_wdt_delete(NULL);
        return false;
      }
      frame_buffer_size = fb->len;
      buffer_allocated = true;
    }

    memcpy(frame_buffer_copy, fb->buf, fb->len);

    esp_camera_fb_return(fb);

    int face_id = 0;

    esp_task_wdt_reset();

    #if TWO_STAGE
      HumanFaceDetectMSR01 s1(0.1F, 0.5F, 10, 0.2F);
      HumanFaceDetectMNP01 s2(0.5F, 0.3F, 5);
      std::list<dl::detect::result_t> &candidates = s1.infer((uint16_t *)frame_buffer_copy, {(int)fb->height, (int)fb->width, 3});

      esp_task_wdt_reset();
      std::list<dl::detect::result_t> &results = s2.infer((uint16_t *)frame_buffer_copy, {(int)fb->height, (int)fb->width, 3}, candidates);

      esp_task_wdt_reset();
    #else
      HumanFaceDetectMSR01 s1(0.3F, 0.5F, 10, 0.2F);
      std::list<dl::detect::result_t> &results = s1.infer((uint16_t *)frame_buffer_copy, {(int)fb->height, (int)fb->width, 3});

      esp_task_wdt_reset();
    #endif

    esp_task_wdt_reset();

      if (enableGestureDetection && !face_detected && gestureModuleEnabled && !servo_home && !gestureControlPaused) {

        camera_fb_t temp_fb;
        temp_fb.buf = frame_buffer_copy;
        temp_fb.len = frame_buffer_size;
        temp_fb.width = fb->width;
        temp_fb.height = fb->height;
        temp_fb.format = fb->format;

        CameraGestureRecognizer::GestureType gesture = cameraGesture.detectGesture(&temp_fb);

      esp_task_wdt_reset();

      if (gesture != CameraGestureRecognizer::GESTURE_NONE) {
        const char* gestureName = cameraGesture.getGestureName(gesture);

        int leftCount = cameraGesture.getLeftGestureCount();

        if (gesture == CameraGestureRecognizer::GESTURE_RIGHT && (leftCount == 9 || leftCount == 10)) {
          Serial.printf("检测到手势: %s (原左手势被替换，计数：%d)\n", gestureName, leftCount);
        } else {
          Serial.printf("检测到手势: %s (左手势计数：%d)\n", gestureName, leftCount);
        }

        switch (gesture) {
          case CameraGestureRecognizer::GESTURE_LEFT:

            if (isWheelsRunning && currentWheelAction == 1 && ignoreUpGestureCount == 0) {
              Serial.println("前进过程中，忽略左转手势");
              avatar.setExpression(Expression::Doubt);

              avatar.setExpression(Expression::Happy);
            } else {

              if (ignoreLeftGestureCount > 0) {
                Serial.println("忽略左转手势");
                 ignoreLeftGestureCount--;
              }
              else {
                if (ignoreRightGestureCount > 0) {
                  avatar.setExpression(Expression::Doubt);
                  startTurnLeft(random(1000, 2001));

                }
                else {
                  avatar.setExpression(Expression::Doubt);
                  startTurnLeft(random(500, 2001));

                }
              }
            }
            break;

          case CameraGestureRecognizer::GESTURE_RIGHT:

            if (isWheelsRunning && currentWheelAction == 1 && ignoreUpGestureCount == 0) {
              Serial.println("前进过程中，忽略右转手势");
              avatar.setExpression(Expression::Doubt);

              avatar.setExpression(Expression::Happy);
            } else {

              if (ignoreRightGestureCount > 0) {
                Serial.println("忽略右转手势");
                 ignoreRightGestureCount--;
              }
              else {
                avatar.setExpression(Expression::Happy);
                if (ignoreLeftGestureCount > 0) {
                  startTurnRight(random(1000, 2001));

                }
                else {
                  startTurnRight(random(1500, 2001));

                }
              }
            }
            break;

          case CameraGestureRecognizer::GESTURE_UP:

            if (ignoreUpGestureCount > 0) {
              Serial.printf("忽略向上手势，还剩 %d 次\n", ignoreUpGestureCount - 1);
              ignoreUpGestureCount--;

              avatar.setExpression(Expression::Doubt);

              avatar.setExpression(Expression::Neutral);
            } else {

              if (proximitySensor.readProximity() > 80) {

                Serial.println("检测到物体非常近，忽略向上手势");
                avatar.setExpression(Expression::Doubt);

                avatar.setExpression(Expression::Neutral);
              }
               else {
                int currentServoPosx = servo_x.read();
                if (currentServoPosx >= 60 && currentServoPosx <= 120 && proximitySensor.readProximity() < 30){
                  Serial.println("头部居中且物体非常近，忽略向上手势");
                }else{
                 startMoveForward(random(4500, 19500));
                  Serial.println("开始前进，随机4.5-9.5秒");
                }
              }
            }
            break;

          case CameraGestureRecognizer::GESTURE_DOWN:
            avatar.setExpression(Expression::Doubt);
            startMoveBackward(50);
            break;

          default:
            break;
        }

        hasActivity = true;
      }

      esp_task_wdt_reset();
    }

    if (results.size() > 0) {
      isDetected = true;

      bool justDetectedFace = !face_detected;

      face_detected = true;
      if(gestureModuleEnabled){
      is_searching_face = false;
      }
      last_face_time = millis();
      hasActivity = true;

      if (justDetectedFace) {
        servo_x.stop();
        face_stable_time = 0;
        Serial.println("人脸检测状态变化: 无人脸 -> 有人脸");
        last_target_time = 0;
        avatar.setExpression(Expression::Happy);
        controlHeadByServoPosition();
      }

      esp_task_wdt_reset();

      float gyroX, gyroY, gyroZ;
      float accX, accY, accZ;

      if(M5.Imu.getGyroData(&gyroX, &gyroY, &gyroZ) &&
         M5.Imu.getAccelData(&accX, &accY, &accZ)) {

        unsigned long current_time = millis();
        if (face_stable_time == 0) {
          face_stable_time = current_time;
          if (!wheelControlEnabled) {
            Serial.println("检测到人脸，等待稳定...");
            servo_x.stop();
          }
        }
        else if ((current_time - face_stable_time > FACE_STABLE_THRESHOLD) && !wheelControlEnabled) {
          wheelControlEnabled = true;
          headCenterCalibrated = false;
          isDetectingNod = false;
        }

        esp_task_wdt_reset();

        if (wheelControlEnabled && (millis() - lastImuReadTime > IMU_READ_INTERVAL)) {
          lastImuReadTime = millis();

          if (detectNodding(gyroX)) {
            hasActivity = true;
          }

          float deltaGyroZ = gyroZ - lastGyroZ;

          lastGyroX = gyroX;
          lastGyroY = gyroY;
          lastGyroZ = gyroZ;

          if (!pauseHeadShakeControl && (abs(deltaGyroZ) > GYRO_THRESHOLD * 0.8 || abs(gyroZ - headCenterGyroZ) > GYRO_THRESHOLD)) {

            controlHeadByServoPosition();
          } else if (!pauseHeadShakeControl && (!headCenterCalibrated || random(20) == 0)) {

            controlHeadByServoPosition();
          }
        }
      }

      esp_task_wdt_reset();

      auto prediction = results.begin();
      if(prediction != results.end()) {
        int x = (int)prediction->box[0];
        int y = (int)prediction->box[1];
        int w = (int)prediction->box[2] - x + 1;
        int h = (int)prediction->box[3] - y + 1;

        g_face_centroid_x = x + w / 2;
        g_face_centroid_y = y + h / 2;

        face_too_close = (w > FACE_TOO_CLOSE_THRESHOLD);

        int currentServoPos = servo_x.read();
        if (currentServoPos >= 80 && currentServoPos <= 95 && !isWheelsRunning) {

            forward_count++;

          avatar.setExpression(Expression::Happy);
          if (forward_count > 9) {
            forward_count = 0;

          }
        }
      }
    } else {
      if (face_detected && millis() - last_face_time > 5000) {
        face_detected = false;

        face_stable_time = 0;
        face_too_close = false;

        Serial.println("人脸检测状态变化: 有人脸 -> 无人脸，恢复搜索模式");

        last_target_time = 0;
        if (wheelControlEnabled) {
          wheelControlEnabled = false;
          stopWheels();
        }

        last_random_move = 0;
      }
    }

    esp_task_wdt_reset();
    esp_task_wdt_delete(NULL);

  } catch (std::exception &e) {
    Serial.printf("Camera task exception: %s\n", e.what());

    esp_task_wdt_reset();
    esp_task_wdt_delete(NULL);
    return false;
  }

  return isDetected;
}
#endif

void cameraTask(void *parameter) {

  esp_task_wdt_init(CAM_WATCHDOG_TIMEOUT, true);
  esp_task_wdt_add(NULL);

  unsigned long task_delay = 120;

  while (true) {

    if (camera_needs_reset) {

      resetCamera();

      vTaskDelay(pdMS_TO_TICKS(1000));

      task_delay = 150;
    }

    if (face_detected_flag) {
      isFaceDetected = camera_capture_and_face_detect();

    }

    vTaskDelay(pdMS_TO_TICKS(task_delay));

    esp_task_wdt_reset();
  }
}

void sw_tone(){
      startInternalSpeaker();
      M5.Speaker.tone(300, 100);
      stopInternalSpeaker();
}

void sw_tone1(){
  stopInternalMic();
  delay(300);

  startInternalSpeaker();
  M5.Speaker.tone(800, 100);
  delay(300);
  stopInternalSpeaker();
  delay(300);
  startInternalMic();
}

void setup()
{

  heap_caps_malloc_extmem_enable(1000);

  auto cfg = M5.config();

  cfg.external_spk = true;
  cfg.internal_imu = true;
  cfg.internal_mic = true;
  cfg.internal_spk = true;

  M5.begin(cfg);
  Serial.begin(115200);

  Serial.printf("设备类型: %d\n", (int)M5.getBoard());
  Serial.printf("扬声器配置 - pin_data_out: %d, pin_bck: %d, pin_ws: %d\n",
                M5.Speaker.config().pin_data_out,
                M5.Speaker.config().pin_bck,
                M5.Speaker.config().pin_ws);
  Serial.printf("扬声器配置 - i2s_port: %d, magnification: %d, sample_rate: %d\n",
                M5.Speaker.config().i2s_port,
                M5.Speaker.config().magnification,
                M5.Speaker.config().sample_rate);
  Serial.printf("扬声器是否启用: %s\n", M5.Speaker.config().pin_data_out >= 0 ? "是" : "否");

  Serial.println("正在初始化语音识别模块...");

  asrSerial.begin(ASR_SERIAL_BAUD_RATE, SERIAL_8N1, ASR_RX_PIN, ASR_TX_PIN);
  delay(500);

  Serial.println("正在验证模块通信...");
  asr.verify_operation(0x00);
  delay(1000);

  Serial.println("语音识别模块初始化完成");

  Serial.println("正在初始化TOF050F测距传感器...");
  if (tofSensor.begin()) {
    Serial.println("TOF050F传感器初始化成功");

    tofSensor.setContinuousOutput(500);
  } else {
    Serial.println("TOF050F传感器初始化失败");
  }

  Servo_setup();
  delay(1000);

  deviceUniqueID = getDeviceUniqueID();

  isDetectingNod = false;
  nodDetected = false;
  nodPhase = 0;
  lastNodTime = 0;
  headCenterGyroX = 0.0f;
  nodFeedbackActive = false;
  pauseHeadShakeControl = false;

  ear_servo_paused_for_wheels = false;
  ear_servo_pause_start_time = 0;

  auto spk_cfg = M5.Speaker.config();
  spk_cfg.sample_rate = 192000;
  spk_cfg.task_pinned_core = APP_CPU_NUM;
  spk_cfg.magnification = 48;
  M5.Speaker.config(spk_cfg);

  sw_tone();

  M5.Lcd.setFont(&fonts::efontCN_12);
  Serial.println("正在连接WiFi");
  WiFi.disconnect();
  WiFi.softAPdisconnect(true);
  WiFi.mode(WIFI_STA);

  M5.Lcd.setFont(&fonts::efontCN_12);

  Wifi_setup();
  M5.Lcd.setFont(&fonts::efontCN_12);

#if defined(ENABLE_FACE_DETECT)
  avatar.init(16);
#else
  avatar.init();
#endif

  avatar.addTask(servo, "servo", SERVO_TASK_STACK_SIZE, SERVO_TASK_PRIORITY);
  avatar.addTask(earServoTask, "earServo", EAR_SERVO_TASK_STACK_SIZE, EAR_SERVO_TASK_PRIORITY);
  avatar.addTask(lipSync, "lipSync");
  avatar.setSpeechFont(&fonts::efontCN_14);

  box_servo.setupBox(80, 120, 80, 80);
  #if defined(ENABLE_FACE_DETECT)

  box_stt.setupBox(0, 0, M5.Display.width(), M5.Display.height());
#else
   box_stt.setupBox(0, 0, M5.Display.width(), M5.Display.height());
#endif

  box_BtnA.setupBox(0, 100, 40, 60);
  box_BtnC.setupBox(280, 100, 40, 60);

#if defined(ENABLE_FACE_DETECT)
  camera_init();

  if (camera_initialized) {
    Serial.println("相机初始化成功");
  } else {
    Serial.println("相机初始化失败，将在需要时尝试重新初始化");
  }

  enableGestureDetection = true;
  if(cameraGesture.initialize()) {
    Serial.println("摄像头手势识别初始化成功");
    cameraGesture.setDebug(true);

    avatar.setExpression(Expression::Doubt);

    avatar.setExpression(Expression::Neutral);
  } else {
    Serial.println("摄像头手势识别初始化失败，将继续尝试使用手势识别功能");

  }

  Serial.println("初始化接近传感器...");

  M5.In_I2C.release();
  delay(100);

  if (proximitySensor.initialize()) {
    Serial.println("接近传感器初始化成功");
    proximitySensor.setDebug(true);
    enableProximitySensor = true;
  } else {
    Serial.println("接近传感器初始化失败，但系统将继续运行");
    Serial.println("这可能是因为硬件问题或传感器未正确配置");

    enableProximitySensor = true;
  }

#endif

    uint8_t vbusVoltageH1 = 0;
    uint8_t vbusVoltageL1 = 0;
    uint8_t battVoltageH1 = 0;
    uint8_t battVoltageL1 = 0;

    float vbusVoltage1 = 0.0f;
    float battVoltage1 = 0.0f;
    bool highVbusDetected1 = false;

    if (M5.Power.Axp2101.readRegister(0x78, &battVoltageH1, 1) &&
        M5.Power.Axp2101.readRegister(0x79, &battVoltageL1, 1)) {

      int battRaw1 = (battVoltageH1 << 8) | battVoltageL1;
      battVoltage1 = battRaw1 * 1.1f / 1000.0f;

      Serial.printf("Battery Voltage: %.2fV (Raw: 0x%04X)\n", battVoltage1, battRaw1);
    }

    uint8_t vbusStatus1 = 0;
    bool vbusConnected1 = false;
    if (M5.Power.Axp2101.readRegister(0x01, &vbusStatus1, 1)) {

      bool vbusBit5 = ((vbusStatus1 & 0x20) != 0);
      bool vbusBit3 = ((vbusStatus1 & 0x08) != 0);
      bool vbusBit6 = ((vbusStatus1 & 0x40) != 0);

      vbusConnected1 = vbusBit5 || vbusBit3 || vbusBit6;

      Serial.printf("VBUS Status Register(0x01): 0x%02X (bit5:%d, bit3:%d, bit6:%d, Connected:%s)\n",
                   vbusStatus1, vbusBit5, vbusBit3, vbusBit6, vbusConnected1 ? "Yes" : "No");
    }

    if (M5.Power.Axp2101.readRegister(0x5A, &vbusVoltageH1, 1) &&
        M5.Power.Axp2101.readRegister(0x5B, &vbusVoltageL1, 1)) {

      int vbusRaw1 = (vbusVoltageH1 << 8) | vbusVoltageL1;

      vbusVoltage1 = vbusRaw1 * 0.354f / 1000.0f;

      Serial.printf("VBUS Voltage: %.2fV, Battery Voltage: %.2fV (Raw: 0x%04X)\n", vbusVoltage1, battVoltage1, vbusRaw1);

      bool voltageCondition = false;
      if (battVoltage1 > 4.0f) {

        voltageCondition = (vbusVoltage1 > 4.5f);
      } else {

        voltageCondition = (vbusVoltage1 > 4.0f) && (vbusVoltage1 > (battVoltage1 + 0.5f));
      }

      highVbusDetected1 = vbusConnected1 && voltageCondition;

      Serial.printf("VBUS Connected: %s, High VBUS Detected: %s\n",
                   vbusConnected1 ? "Yes" : "No",
                   highVbusDetected1 ? "Yes" : "No");

    }
    if(highVbusDetected1){
      gestureModuleEnabled = false;
      gestureControlPaused = false;
      Serial.println("Type-C供电");
    }

   radar_setup();

   demoWheelControls();

   Serial.printf("Total PSRAM: %d bytes\n", ESP.getPsramSize());
   Serial.printf("Free PSRAM: %d bytes\n", ESP.getFreePsram());

   xTaskCreatePinnedToCore(
     cameraTask,
     "CameraTask",
     CAMERA_TASK_STACK_SIZE,
     NULL,
     CAMERA_TASK_PRIORITY,
     &cameraTaskHandle,
     1
   );

}

int random_time = -1;
static int lastms1 = 0;

unsigned long lastActivityTime = 0;
const unsigned long IDLEstack1_TIMEOUT = 30000;
const unsigned long IDLEstack2_TIMEOUT = 50000;
const unsigned long IDLEstack3_TIMEOUT = 80000;
const unsigned long IDLEstack4_TIMEOUT = 100000;
const unsigned long IDLEstack5_TIMEOUT = 150000;
const unsigned long IDLEstack6_TIMEOUT = 180000;
const unsigned long IDLEstack7_TIMEOUT = 220000;
const unsigned long IDLEstack8_TIMEOUT = 250000;

const unsigned long IDLE_TIMEOUT = 600000;
const unsigned long POWER_OFF_TIMEOUT = 1200000;
bool power_off_warning = false;
unsigned long power_off_warning_start = 0;
bool shouldShowCharging = true;
bool isTypeC = false;
int currentBatteryLevel = 0;
 bool currentVbusConnected = false;
bool is_moving = false;
bool likai_place = false;
bool ditou_place = false;

const unsigned long NO_TARGET_THRESHOLD = 45000;
unsigned long last_radar_target_time = 0;
const unsigned long RADAR_TARGET_GRACE = 2500;
float last_tracked_distance = 200.0f;

      bool waiting_for_stable = false;
      unsigned long stable_wait_start = 0;
      const unsigned long STABLE_WAIT_TIMEOUT = 60000;

void update_search_behavior() {
    unsigned long current_time = millis();

    if (radar_state.is_tracking && radar_state.distance <= 150) {
        last_target_time = current_time;
        return;
    }

    if (current_time - last_target_time > NO_TARGET_THRESHOLD && !face_detected) {

       if (isWheelsRunning && !is_searching_face) {
        Serial.println("轮子正在运行，先停止轮子");
        emergencyStop();
        delay(100);
    }
        is_searching_face = true;

    }
}

void loop()
{
    M5.update();

    tofSensor.update();

    if (tofSensor.hasNewData()) {
        int16_t distance = tofSensor.getLastDistance();
        if (tofSensor.isValid()) {

            if (distance < 20) {
              likai_place = false;
            }

            if (distance >= 20 && !likai_place) {
              likai_place = true;

               ditou_place = true;
            emergencyStop();

        is_searching_face = false;
        face_detected = false;
        face_detected_flag = false;

        int saved_x_position = servo_x.read();

        servo_y.setEaseTo(START_DEGREE_VALUE_Y + 35);
        synchronizeAllServosStartAndWaitForAllServosToStop();

        delay(2000);
        servo_x.setEaseTo(saved_x_position-random(45, 65));
        synchronizeAllServosStartAndWaitForAllServosToStop();
        delay(2000);

        servo_x.setEaseTo(saved_x_position+random(35, 45));
        synchronizeAllServosStartAndWaitForAllServosToStop();
        delay(2000);
        servo_x.setEaseTo(90);
        synchronizeAllServosStartAndWaitForAllServosToStop();
        delay(2000);

        avatar.setExpression(Expression::Doubt);

        startMoveBackward(3000);
        delay(3000);

        avatar.setExpression(Expression::Neutral);

        hasActivity = true;
        ditou_place = false;
        face_detected_flag = true;

            }
        } else {

        }
    }

      static unsigned long last_radar_check = 0;
    const unsigned long RADAR_CHECK_INTERVAL = 50;

    if (millis() - last_radar_check >= RADAR_CHECK_INTERVAL) {
      if (read_radar_data()) {
        RadarDetection detection = get_radar_detection();

        update_radar_tracking(detection);

        if (!face_detected && !mainsleep) {

            unsigned long tracking_now = millis();
            bool target_recent = is_tracking_target_with_wheels && (tracking_now - last_radar_target_time <= RADAR_TARGET_GRACE);
            static unsigned long last_wheel_tracking_time = 0;

            if (radar_state.is_tracking && radar_state.distance <= 150) {
              is_searching_face = false;

              if(gestureModuleEnabled){

                if (!is_tracking_target_with_wheels) {
                  is_tracking_target_with_wheels = true;
                  gestureControlPaused = true;
                  Serial.println("自由走动模式：检测到目标，开始轮子跟踪");
                  emergencyStop();
                }
              }

              last_radar_target_time = tracking_now;
              last_tracked_distance = radar_state.distance;
              target_recent = true;

                setSpeedForAllServos(80);
                int suggested_angle = calculate_head_angle(detection);

                int current_angle = servo_x.read();
                float angle_diff = abs(current_angle - suggested_angle);

                if (angle_diff > 1.0f) {
                    servo_x.setEaseTo(suggested_angle);

                    if (gestureModuleEnabled && is_tracking_target_with_wheels) {
                      const int ALIGN_THRESHOLD = 3;

                      int deltaFromCenter = current_angle - 90;

                      if (tracking_now - last_wheel_tracking_time >= 200) {
                        last_wheel_tracking_time = tracking_now;

                        if (abs(deltaFromCenter) > ALIGN_THRESHOLD) {
                          unsigned long turnDuration = abs(deltaFromCenter) * 20;
                          turnDuration = constrain(turnDuration, 100, 800);

                          if (deltaFromCenter > 0) {
                            startTurnRight(turnDuration);
                            Serial.printf("轮子跟踪：头部偏右%d°，右转%lums\n", deltaFromCenter, turnDuration);
                          } else {
                            startTurnLeft(turnDuration);
                            Serial.printf("轮子跟踪：头部偏左%d°，左转%lums\n", -deltaFromCenter, turnDuration);
                          }
                        }

                        else if (!isWheelsRunning) {
                          if (last_tracked_distance > 55) {

                            unsigned long forwardDuration = map((int)last_tracked_distance, 55, 150, 300, 600);
                            forwardDuration = constrain(forwardDuration, 300, 600);
                            startMoveForward(forwardDuration);
                            Serial.printf("轮子跟踪：距离%.1fcm，前进%lums靠近目标\n",
                                         last_tracked_distance, forwardDuration);
                          } else if (last_tracked_distance < 45) {

                            unsigned long backwardDuration = map((int)last_tracked_distance, 20, 45, 200, 400);
                            backwardDuration = constrain(backwardDuration, 200, 400);
                            startMoveBackward(backwardDuration);
                            Serial.printf("轮子跟踪：距离%.1fcm，后退%lums远离目标\n",
                                         last_tracked_distance, backwardDuration);
                          } else {

                            if (isWheelsRunning) {
                              emergencyStop();
                              Serial.printf("轮子跟踪：到达目标距离(%.1fcm)，停止跟踪，恢复自由走动\n",
                                           last_tracked_distance);
                              is_tracking_target_with_wheels = false;
                              gestureControlPaused = false;

                            }
                          }
                        }
                      }
                    } else {

                      controlHeadByServoPosition();
                    }

                    static unsigned long last_tracking_debug = 0;
                    if (millis() - last_tracking_debug >= 300) {
                         Serial.printf("雷达跟踪: 目标角度=%d°, 当前角度=%d°, 差异=%.1f°, 距离=%.1fcm\n",
                                      suggested_angle, current_angle, angle_diff, last_tracked_distance);
                        last_tracking_debug = millis();
                    }
                } else {

                    servo_x.stop();
                }

                if (radar_state.distance >= RADAR_MIN_TRACKING_DISTANCE &&
                    radar_state.distance <= 150 && !ditou_place) {

                    int y_angle = 90 - (int)(radar_state.distance / 2);
                    y_angle = constrain(y_angle, 60, 90);
                    servo_y.setEaseTo(y_angle);
                }

            } else if (target_recent) {

                if (gestureModuleEnabled && is_tracking_target_with_wheels) {
                  int current_angle = servo_x.read();
                  int deltaFromCenter = current_angle - 90;
                  const int ALIGN_THRESHOLD = 3;

                  if (tracking_now - last_wheel_tracking_time >= 200) {
                    last_wheel_tracking_time = tracking_now;

                    if (abs(deltaFromCenter) > ALIGN_THRESHOLD) {
                      unsigned long turnDuration = abs(deltaFromCenter) * 20;
                      turnDuration = constrain(turnDuration, 100, 800);

                      if (deltaFromCenter > 0) {
                        startTurnRight(turnDuration);
                        Serial.printf("轮子跟踪(静止目标)：头部偏右%d°，右转%lums\n", deltaFromCenter, turnDuration);
                      } else {
                        startTurnLeft(turnDuration);
                        Serial.printf("轮子跟踪(静止目标)：头部偏左%d°，左转%lums\n", -deltaFromCenter, turnDuration);
                      }
                    } else if (!isWheelsRunning) {
                      if (last_tracked_distance > 55) {
                        unsigned long forwardDuration = map((int)last_tracked_distance, 55, 150, 300, 600);
                        forwardDuration = constrain(forwardDuration, 300, 600);
                        startMoveForward(forwardDuration);
                        Serial.printf("轮子跟踪(静止目标)：距离%.1fcm，前进%lums靠近目标\n",
                                     last_tracked_distance, forwardDuration);
                      } else if (last_tracked_distance < 45) {
                        unsigned long backwardDuration = map((int)last_tracked_distance, 20, 45, 200, 400);
                        backwardDuration = constrain(backwardDuration, 200, 400);
                        startMoveBackward(backwardDuration);
                        Serial.printf("轮子跟踪(静止目标)：距离%.1fcm，后退%lums远离目标\n",
                                     last_tracked_distance, backwardDuration);
                      } else {
                        Serial.printf("轮子跟踪(静止目标)：到达目标距离(%.1fcm)，保持位置\n", last_tracked_distance);
                      }
                    }
                  }
                }
            } else {

                if (gestureModuleEnabled && is_tracking_target_with_wheels) {

                  emergencyStop();
                  is_tracking_target_with_wheels = false;
                  gestureControlPaused = false;

                  Serial.println("轮子跟踪：目标丢失，停止跟踪，恢复自由走动");
                }

                update_search_behavior();
            }
        }
    }
    last_radar_check = millis();
}

      if (waiting_for_stable) {

          SST_ChatGPT();
           while(asrSerial.available()) {
    asrSerial.read();
  }
          waiting_for_stable = false;

      }

  if (asrEnabled && asrSerial.available() >= 8) {
    uint8_t buffer[8] = {0};
    asr.read_protocol(buffer, ASR_READ_GENERAL_COMMAND, 0x00);

    if (buffer[0] == 0xA5 && buffer[1] == 0xFA&& buffer[3] == 0x81 && buffer[5] == 0x00 && buffer[7] == 0xFB) {
      uint8_t current_command = buffer[4];

      String recognizedWord = "未知命令";
      bool validCommand = true;

      switch(current_command) {
        case 1:
          recognizedWord = "北鼻";
          avatar.setExpression(Expression::Happy);
          avatar.setSpeechText("我在呢！");
          break;
        case 2:
          emergencyStop();
          recognizedWord = "北北";
          avatar.setExpression(Expression::Happy);
          avatar.setSpeechText("你好啊！");
          servo_y.setEaseTo(65);
          delay(500);
          servo_y.setEaseTo(55);
          delay(800);
          servo_y.setEaseTo(60);
          gestureModuleEnabled = false;
          gestureControlPaused = false;
          waiting_for_stable = true;
          stable_wait_start = millis();
          break;
        case 3:
          recognizedWord = "鼻鼻";
          avatar.setExpression(Expression::Happy);
          avatar.setSpeechText("哈哈！");
          break;
        default:
          validCommand = false;
          break;
      }

      if (validCommand) {

        Serial.print("识别到语音命令：");
        Serial.println(recognizedWord);

        hasActivity = true;
        validCommand = false;
        delay(1000);
        avatar.setExpression(Expression::Neutral);
        avatar.setSpeechText("");
      }
    }
  }

  if (nodFeedbackActive) {

    if (millis() - nodFeedbackStartTime > NOD_FEEDBACK_DURATION) {

      avatar.setSpeechText("");
      avatar.setExpression(Expression::Neutral);
      nodFeedbackActive = false;
    }
  }

  if (pauseHeadShakeControl && millis() >= wheelControlResumeTime) {
    pauseHeadShakeControl = false;
    Serial.println("恢复摇头控制轮子");
  }

  static unsigned long lastImuTestTime = 0;
  if (millis() - lastImuTestTime > 500) {
    lastImuTestTime = millis();

  }

  checkWheelStatus();

  if (wheelControlEnabled && !isWheelsRunning && (millis() - lastImuReadTime > IMU_READ_INTERVAL)) {
    float gyroX, gyroY, gyroZ;
    if (M5.Imu.getGyroData(&gyroX, &gyroY, &gyroZ)) {
      lastImuReadTime = millis();

      if (face_detected && detectNodding(gyroX)) {

        hasActivity = true;
      }

      float deltaGyroZ = gyroZ - lastGyroZ;

      lastGyroX = gyroX;
      lastGyroY = gyroY;
      lastGyroZ = gyroZ;

      if (!pauseHeadShakeControl) {

        if (abs(deltaGyroZ) > GYRO_THRESHOLD * 1.2) {

          controlHeadByServoPosition();
        }

        else if (headCenterCalibrated && abs(gyroZ - headCenterGyroZ) > GYRO_THRESHOLD * 0.6) {

          controlHeadByServoPosition();
        }

        else if (face_detected && random(30) == 0) {

          controlHeadByServoPosition();
        }
      }
    }
  }

  unsigned long currentTime = millis();

  static unsigned long lastTouchTime = 0;
  const unsigned long touchDelay = 1000;
  static int touchCount = 0;

  auto count = M5.Touch.getCount();
  if (count) {
    auto t = M5.Touch.getDetail();

    if (t.wasPressed()) {
      touchStartTime = millis();
      touchHolding = true;
      hasActivity = true;

    }

    if (touchHolding && t.isPressed()) {
      if (millis() - touchStartTime > LONG_PRESS_THRESHOLD) {

        gestureModuleEnabled = !gestureModuleEnabled;

        if(!gestureModuleEnabled){

          stopWheels();
          is_searching_face = false;
          is_tracking_target_with_wheels = false;
          gestureControlPaused = false;
          gestureModuleX1 = -30;
          gestureModuleX2 = 31;
          gestureModuleY1 = -15;
          gestureModuleY2 = 16;

          avatar.setExpression(Expression::Doubt);
          avatar.setSpeechText("停止走动");

          sw_tone1();

          hasActivity = true;

          touchHolding = false;

          delay(1000);
          avatar.setSpeechText("");
          avatar.setExpression(Expression::Neutral);
        } else {

          enableProximitySensor = true;
          gestureControlPaused = false;

          avatar.setExpression(Expression::Happy);
          avatar.setSpeechText("自由走动");
          is_searching_face = true;
          gestureModuleX1 = -60;
          gestureModuleX2 = 61;
          gestureModuleY1 = -25;
          gestureModuleY2 = 26;

          sw_tone1();

          hasActivity = true;

          touchHolding = false;

          delay(1000);
          avatar.setSpeechText("");
          avatar.setExpression(Expression::Neutral);
        }
      }
    }
  } else {

    touchHolding = false;
  }

  if (touchCount > 0 && (millis() - lastTouchTime > 20000)) {
    touchCount = 0;
    avatar.setExpression(Expression::Neutral);
    avatar.setSpeechText("");
  }

  if (enableProximitySensor) {
    static unsigned long lastProximityCheckTime = 0;
    static bool objectWasDetected = false;
    static bool objectWasVeryClose = false;
    static int failedReadCount = 0;

    if (millis() - lastProximityCheckTime > 100) {
      lastProximityCheckTime = millis();

      uint16_t distance = 0;

      try {
        distance = proximitySensor.readProximity();

        failedReadCount = 0;
      } catch (...) {

        failedReadCount++;
        Serial.printf("近距离传感器读取异常，失败计数: %d\n", failedReadCount);

        if (failedReadCount > 10) {
          Serial.println("近距离传感器失败次数过多，暂停10秒");
          enableProximitySensor = false;

          sensorDisableTime = millis();
        }

        distance = 100;
      }

      if (distance >80) {
        if (!objectWasVeryClose) {
          Serial.println("物体非常近，后退！");
          is_moving = true;
          avatar.setExpression(Expression::Doubt);
          avatar.setSpeechText("");

          emergencyStop();

          startMoveBackward(2000);

          objectWasVeryClose = true;
          hasActivity = true;

          ignoreUpGestureCount = 6;
          if(ignoreRightGestureonetime && !face_detected){
            ignoreRightGestureonetime = false;
            ignoreLeftGestureCount = 8;
            ignoreRightGestureCount = 0;
            startTurnRight(random(500, 1000));
          }else if (random(2) == 1 && !face_detected) {
            ignoreLeftGestureCount = 8;
            ignoreRightGestureCount = 0;
            startTurnRight(random(500, 1000));
          }else{
            ignoreRightGestureCount = 6;
            ignoreLeftGestureCount = 0;
            ignoreRightGestureonetime = true;
          }

        }
      }

      else if (distance < 30) {
        if (!objectWasDetected || objectWasVeryClose) {
          Serial.println("检测到物体接近");
          avatar.setExpression(Expression::Doubt);

          objectWasDetected = true;
          objectWasVeryClose = false;
          hasActivity = true;
        }
      }

      else {

        if (objectWasDetected) {
          Serial.println("物体已离开");
          avatar.setExpression(Expression::Neutral);

          objectWasDetected = false;
          objectWasVeryClose = false;
          hasActivity = true;
        }
      }
    }
  } else {

    if (millis() - sensorDisableTime > 10000) {
      enableProximitySensor = true;
      Serial.println("尝试重新启用近距离传感器");

      if (proximitySensor.initialize()) {
        Serial.println("近距离传感器重新初始化成功");
      }
    }
  }

  if (wavPlayer.isRunning() || is_speaking) {
    hasActivity = true;
  }

  if (currentTime - lastPowerCheckTime > POWER_CHECK_INTERVAL) {
    lastPowerCheckTime = currentTime;

    uint8_t powerStatus = 0;
    uint8_t vbusStatus = 0;
    uint8_t battVoltageH = 0;
    uint8_t battVoltageL = 0;
    uint8_t regValue = 0;
    uint8_t vbusVoltageH = 0;
    uint8_t vbusVoltageL = 0;
    bool currentCharging = false;

    bool statusChanged = false;

    float vbusVoltage = 0.0f;
    float battVoltage = 0.0f;

    if (M5.Power.Axp2101.readRegister(0x78, &battVoltageH, 1) &&
        M5.Power.Axp2101.readRegister(0x79, &battVoltageL, 1)) {

      int battRaw = (battVoltageH << 8) | battVoltageL;
      battVoltage = battRaw * 1.1f / 1000.0f;

    }

    bool vbusStatusBit = false;
    if (M5.Power.Axp2101.readRegister(0x01, &vbusStatus, 1)) {

      vbusStatusBit = ((vbusStatus & 0x20) != 0);

    }

    bool highVbusDetected = false;
    if (M5.Power.Axp2101.readRegister(0x5A, &vbusVoltageH, 1) &&
        M5.Power.Axp2101.readRegister(0x5B, &vbusVoltageL, 1)) {

      int vbusRaw = (vbusVoltageH << 8) | vbusVoltageL;
      vbusVoltage = vbusRaw * 1.7f / 1000.0f;

      highVbusDetected = (vbusVoltage > 4.0f) && (vbusVoltage > (battVoltage + 0.5f));
    }

    currentVbusConnected = vbusStatusBit && (vbusVoltage > (battVoltage + 0.3f));

    bool chargingFromReg00 = false;
    if (M5.Power.Axp2101.readRegister(0x00, &powerStatus, 1)) {
      chargingFromReg00 = (powerStatus & 0x04) != 0;

    }

    bool chargingFromReg02 = false;
    uint8_t chargeStatus = 0;
    if (M5.Power.Axp2101.readRegister(0x02, &chargeStatus, 1)) {
      chargingFromReg02 = (chargeStatus & 0x04) != 0;

    }

    currentCharging = currentVbusConnected && (chargingFromReg00 || chargingFromReg02);

    if (M5.Power.Axp2101.readRegister(0xA4, &regValue, 1)) {
      currentBatteryLevel = regValue;

    }

    bool isTypeC = highVbusDetected && vbusStatusBit;

    if (currentCharging != lastChargingState ||
        currentVbusConnected != lastVbusState ||
        abs(currentBatteryLevel - lastBatteryLevel) > 5 ||
        abs(vbusVoltage - lastVbusVoltage) > 0.5f ||
        isTypeC) {

      statusChanged = true;

      lastChargingState = currentCharging;
      lastVbusState = currentVbusConnected;
      lastBatteryLevel = currentBatteryLevel;
      lastVbusVoltage = vbusVoltage;

      if (currentBatteryLevel > 0 && currentBatteryLevel <= LOW_BATTERY_THRESHOLD && !currentVbusConnected) {
        if (!lowBatteryWarning) {
          lowBatteryWarning = true;
          String warningText = "电量低，仅剩" + String(currentBatteryLevel) + "%，请及时充电";
          Serial.println(warningText);

            avatar.setExpression(Expression::Sad);
            avatar.setSpeechText(warningText.c_str());
            delay(2000);
            avatar.setSpeechText("");
            avatar.setExpression(Expression::Neutral);

        }
      } else {
        lowBatteryWarning = false;
      }

      String statusText = "";
      bool shouldShowStatus = false;

      if (isTypeC) {
        shouldShowStatus = true;
        shouldShowCharging = true;

        if (currentVbusConnected && !currentCharging && currentBatteryLevel >= 95) {
          statusText = "电池已充足，电量" + String(currentBatteryLevel) + "%";
        }

        else if (currentVbusConnected) {
          statusText = "充电中勿关机 " + String(currentBatteryLevel) + "%";
        }
      }else{
        shouldShowStatus = true;
        if(currentBatteryLevel > 0 && currentBatteryLevel <= LOW_BATTERY_THRESHOLD){
          statusText = "低电量请充电 " + String(currentBatteryLevel) + "%";
        }
      }

      if (shouldShowCharging && !currentVbusConnected) {
        shouldShowStatus = true;
        shouldShowCharging = false;
        avatar.setExpression(Expression::Doubt);
        avatar.setSpeechText("");
      }

      if (shouldShowStatus && statusText != "") {

        avatar.setSpeechText(statusText.c_str());

        avatar.setExpression(Expression::Neutral);
      }
    }
  }

  if (hasActivity) {
    lastActivityTime = currentTime;

      servo_home = false;
      mainsleep = false;
      hasActivity = false;
      avatar.setExpression(Expression::Neutral);
      avatar.setSpeechText("");

    if (power_off_warning) {
      power_off_warning = false;
      power_off_warning_start = 0;

    }
  }

  if(currentTime - lastActivityTime > IDLEstack1_TIMEOUT){
    servo_home = true;
  }
  if(currentTime - lastActivityTime > IDLEstack2_TIMEOUT){
    servo_home = false;

  }
  if(currentTime - lastActivityTime > IDLEstack3_TIMEOUT){
    servo_home = true;
  }
  if(currentTime - lastActivityTime > IDLEstack4_TIMEOUT){
    servo_home = false;
  }
  if(currentTime - lastActivityTime > IDLEstack5_TIMEOUT){
    servo_home = true;
  }
  if(currentTime - lastActivityTime > IDLEstack6_TIMEOUT){
    servo_home = false;
  }
  if(currentTime - lastActivityTime > IDLEstack7_TIMEOUT){
    servo_home = true;
  }
    if(currentTime - lastActivityTime > IDLEstack8_TIMEOUT){
    servo_home = false;
  }

  if (!power_off_warning && !servo_home && (currentTime - lastActivityTime > IDLE_TIMEOUT)) {
    servo_home = true;
    mainsleep = true;
    avatar.setExpression(Expression::Sleepy);
    avatar.setSpeechText("睡觉..");
    servo_x.setEaseTo(START_DEGREE_VALUE_X);
    servo_y.setEaseTo(START_DEGREE_VALUE_Y);
    synchronizeAllServosStartAndWaitForAllServosToStop();

  }

if (!currentVbusConnected || (currentVbusConnected && currentBatteryLevel >= 95)) {

  if (!power_off_warning && (currentTime - lastActivityTime > POWER_OFF_TIMEOUT - 30000)) {
    power_off_warning = true;
    power_off_warning_start = currentTime;
    avatar.setExpression(Expression::Sad);
  }

  if (power_off_warning) {
    int remaining_seconds = 30 - ((currentTime - power_off_warning_start) / 1000);
    if (remaining_seconds > 0) {
      char countdown_text[32];
      snprintf(countdown_text, sizeof(countdown_text), "%d秒后将自动关机...", remaining_seconds);
      avatar.setSpeechText(countdown_text);
    }
  }

  if (currentTime - lastActivityTime > POWER_OFF_TIMEOUT) {
    avatar.setSpeechText("关机");
    delay(1000);
    M5.Power.powerOff();
  }

}

  if (is_speaking) {
      wavPlayer.stop();
      Serial.println("again start");
      avatar.setExpression(Expression::Neutral);
      speech_text_buffer = "";
      is_speaking = false;
      if (chatSessionActive) {
        SST_ChatGPT();
      }
   }

}

int get_servo_x_angle() {
    return servo_x.read();
}
