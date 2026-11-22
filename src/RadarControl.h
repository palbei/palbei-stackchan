#ifndef RADAR_CONTROL_H
#define RADAR_CONTROL_H

#include <Arduino.h>

#define RADAR_RX_PIN 43
#define RADAR_TX_PIN 44
#define RADAR_BAUDRATE 256000

#define RADAR_MIN_DISTANCE 20
#define RADAR_MAX_DISTANCE 300

enum Direction {
    DIR_NONE,
    DIR_LEFT,
    DIR_RIGHT
};

#define HEAD_MOVE_STEP 5
#define HEAD_MAX_ANGLE 80
#define HEAD_MIN_ANGLE -80

struct TargetInfo {
    float distance;
    float angle;
    float speed;
    int signalStrength;
    int16_t x;
    int16_t y;
    Direction direction;
};

struct RadarData {
    uint8_t header[2];
    uint8_t data_length;
    uint8_t data_type;
    uint8_t data[32];
    uint8_t checksum;
};

struct RadarDetection {
    bool isValid;
    int targetCount;
    TargetInfo targets[5];
    Direction primaryDirection;
    int suggestedHeadAngle;
};

#define RADAR_MIN_TRACKING_DISTANCE 30
#define RADAR_MAX_TRACKING_DISTANCE 400
#define RADAR_ANGLE_SMOOTH_FACTOR 0.15f
#define RADAR_MIN_ANGLE_CHANGE 1.5f

#define RADAR_POSITION_OFFSET_X 0
#define RADAR_POSITION_OFFSET_Y 0
#define RADAR_POSITION_OFFSET_ANGLE 0
#define RADAR_STABLE_TIME_THRESHOLD 200

struct RadarState {
    bool is_tracking;
    unsigned long last_update;
    unsigned long stable_start;
    float current_angle;
    float target_angle;
    float distance;
    bool is_stable;
};

extern RadarState radar_state;
extern HardwareSerial radarSerial;

void radar_setup();
bool read_radar_data();
void update_radar_tracking(const RadarDetection& detection);
RadarDetection get_radar_detection();
int calculate_head_angle(const RadarDetection& detection);
int get_servo_x_angle();

#endif
