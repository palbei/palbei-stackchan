#include "RadarControl.h"

HardwareSerial radarSerial(1);
RadarData radar_data;
RadarDetection last_detection = {false, 0, {}, DIR_NONE, 0};

static unsigned long last_print_time = 0;
static const unsigned long PRINT_INTERVAL = 1000;

static unsigned long last_output_time = 0;
static const unsigned long OUTPUT_INTERVAL = 100;
static const float CHANGE_THRESHOLD = 2.0f;

RadarState radar_state = {
    .is_tracking = false,
    .last_update = 0,
    .stable_start = 0,
    .current_angle = 90.0f,
    .target_angle = 90.0f,
    .distance = 0,
    .is_stable = false
};

static float last_angle_diff = 0.0f;
static int consistent_movement_count = 0;
static const int CONSISTENT_THRESHOLD = 3;

static float last_head_angle = 0.0f;

static float current_head_angle = 0.0f;
static bool radar_enabled = true;
static bool radar_initialized = false;

static float get_current_head_angle() {
    return current_head_angle;
}

static bool is_radar_enabled() {
    return radar_enabled && radar_initialized;
}

static void compensate_radar_position(float& x, float& y, float& angle) {

    float compensated_x = x - RADAR_POSITION_OFFSET_X;
    float compensated_y = y - RADAR_POSITION_OFFSET_Y;

    if (compensated_x == 0) {
        angle = 0;
    } else {
        float raw_angle = atan2(compensated_x, abs(compensated_y)) * 180.0f / PI;

        angle = raw_angle + RADAR_POSITION_OFFSET_ANGLE;
    }
}

bool hasSignificantChange(const TargetInfo& new_target, const TargetInfo& old_target) {
    return abs(new_target.distance - old_target.distance) > CHANGE_THRESHOLD ||
           abs(new_target.angle - old_target.angle) > 5.0f ||
           abs(new_target.speed - old_target.speed) > 0.1f;
}

void flush_serial() {
    while (radarSerial.available() > 0) {
        radarSerial.read();
    }
}

void radar_setup() {
    radarSerial.begin(RADAR_BAUDRATE, SERIAL_8N1, RADAR_RX_PIN, RADAR_TX_PIN);
    Serial.println("RD-03D雷达模块初始化完成");
    delay(100);
    flush_serial();
    radar_initialized = true;
    radar_enabled = true;
}

struct KalmanFilter {
    float x;
    float p;
    float q;
    float r;
    float k;

    KalmanFilter() {
        x = 90.0f;
        p = 1.0f;
        q = 0.05f;
        r = 0.02f;
    }

    float update(float measurement) {

        p = p + q;

        k = p / (p + r);
        x = x + k * (measurement - x);
        p = (1 - k) * p;

        return x;
    }
};

static const int HISTORY_SIZE = 2;
struct MovingAverage {
    float values[HISTORY_SIZE];
    int index;
    bool filled;

    MovingAverage() : index(0), filled(false) {
        for(int i = 0; i < HISTORY_SIZE; i++) {
            values[i] = 90.0f;
        }
    }

    void add(float value) {
        values[index] = value;
        index = (index + 1) % HISTORY_SIZE;
        if(index == 0) filled = true;
    }

    float average() {
        float sum = 0;
        int count = filled ? HISTORY_SIZE : index;
        if(count == 0) return 90.0f;

        for(int i = 0; i < count; i++) {
            sum += values[i];
        }
        return sum / count;
    }

    bool isStable(float threshold = 2.0f) {
        if(!filled && index < 2) return false;

        int count = filled ? HISTORY_SIZE : index;
        float avg = average();

        for(int i = 0; i < count; i++) {
            if(abs(values[i] - avg) > threshold)
                return false;
        }
        return true;
    }
};

static KalmanFilter angle_filter;
static MovingAverage angle_history;

struct DirectionStability {
    static const int BUFFER_SIZE = 10;
    float angles[BUFFER_SIZE];
    int index;
    bool filled;

    DirectionStability() : index(0), filled(false) {
        for(int i = 0; i < BUFFER_SIZE; i++) {
            angles[i] = 90.0f;
        }
    }

    void add(float angle) {
        angles[index] = angle;
        index = (index + 1) % BUFFER_SIZE;
        if(index == 0) filled = true;
    }

    bool isDirectionStable() {
        if(!filled && index < 3) return true;

        int count = filled ? BUFFER_SIZE : index;
        if(count < 3) return true;

        float last_diff1 = angles[(index - 1 + BUFFER_SIZE) % BUFFER_SIZE] -
                          angles[(index - 2 + BUFFER_SIZE) % BUFFER_SIZE];
        float last_diff2 = angles[(index - 2 + BUFFER_SIZE) % BUFFER_SIZE] -
                          angles[(index - 3 + BUFFER_SIZE) % BUFFER_SIZE];

        return (last_diff1 * last_diff2 >= 0);
    }

    float getAverageAngle() {
        int count = filled ? BUFFER_SIZE : index;
        if(count == 0) return 90.0f;

        float sum = 0;
        for(int i = 0; i < count; i++) {
            sum += angles[i];
        }
        return sum / count;
    }
};

static DirectionStability direction_stability;

void update_radar_tracking(const RadarDetection& detection) {

    if (!is_radar_enabled()) {
        return;
    }

    unsigned long current_time = millis();
    static float last_angle = 90.0f;

    if (!detection.isValid || detection.targetCount == 0) {
        if (radar_state.is_tracking && (current_time - radar_state.last_update > 1000)) {
            radar_state.is_tracking = false;
            radar_state.is_stable = false;

        }
        return;
    }

    float target_distance = detection.targets[0].distance;
    float target_angle = detection.targets[0].angle;

    if (target_distance < RADAR_MIN_TRACKING_DISTANCE || target_distance > RADAR_MAX_TRACKING_DISTANCE) {
        if (radar_state.is_tracking) {
            radar_state.is_tracking = false;
            radar_state.is_stable = false;

        }
        return;
    }

    if (!radar_state.is_tracking) {
        radar_state.is_tracking = true;
        radar_state.stable_start = current_time;
        radar_state.current_angle = 90.0f;
        radar_state.target_angle = 90.0f;
        radar_state.distance = target_distance;
        last_angle = 90.0f;

        angle_filter = KalmanFilter();
        angle_history = MovingAverage();
    }

    float servo_target_angle = 90.0f + target_angle;
    servo_target_angle = constrain(servo_target_angle, 10.0f, 170.0f);

    float filtered_angle = angle_filter.update(servo_target_angle);

    angle_history.add(filtered_angle);

    float final_target_angle = angle_history.average();

    float angle_diff = final_target_angle - radar_state.current_angle;

    radar_state.target_angle = final_target_angle;

    static const float DEADZONE = 1.0f;
    static bool in_deadzone = true;

    if (abs(angle_diff) <= DEADZONE) {
        in_deadzone = true;
        radar_state.is_stable = true;

        radar_state.current_angle = final_target_angle;
        return;
    } else if (in_deadzone && abs(angle_diff) <= DEADZONE * 1.5f) {

        return;
    }

    in_deadzone = false;

    bool is_movement_stable = angle_history.isStable(1.0f);
    float move_factor;
    float max_step;

    static float last_filtered_angle = 90.0f;
    float angle_speed = (filtered_angle - last_filtered_angle) / 0.05f;
    last_filtered_angle = filtered_angle;

    if (is_movement_stable) {
        move_factor = 0.8f;
        max_step = 15.0f;
    } else if (abs(angle_diff) > 15.0f) {
        move_factor = 1.0f;
        max_step = 20.0f;
    } else {
        move_factor = 0.9f;
        max_step = 18.0f;
    }

    float angle_step = angle_diff * move_factor;

    if (abs(angle_speed) > 120.0f) {
        max_step *= 0.9f;
    }

    if (abs(angle_diff) < 3.0f) {
        float slow_factor = (abs(angle_diff) / 3.0f);
        max_step *= (slow_factor * 0.8f + 0.2f);
    }

    if (abs(angle_step) > max_step) {
        angle_step = (angle_step > 0) ? max_step : -max_step;
    }

    if (abs(angle_diff) > 0.5f || !is_movement_stable) {
        float predicted_angle = radar_state.current_angle + angle_step;
        float predicted_diff = final_target_angle - predicted_angle;

        if (angle_diff * predicted_diff < 0) {
            angle_step *= 0.9f;
        }

        static float last_step = 0;
        float acceleration = angle_step - last_step;
        if (abs(acceleration) > 8.0f) {
            angle_step = last_step + (acceleration > 0 ? 8.0f : -8.0f);
        }
        last_step = angle_step;

        radar_state.current_angle += angle_step;
        radar_state.current_angle = constrain(radar_state.current_angle, 10.0f, 170.0f);
        radar_state.stable_start = current_time;
        radar_state.is_stable = false;
    } else if (!radar_state.is_stable) {
        radar_state.is_stable = true;
    }

    float distance_diff = target_distance - radar_state.distance;
    float distance_factor = (abs(distance_diff) > 20.0f) ? 0.2f : 0.1f;
    radar_state.distance += distance_diff * distance_factor;

    static unsigned long last_debug_time = 0;
    if (millis() - last_debug_time >= 200) {

        int actual_servo_angle = get_servo_x_angle();

        last_debug_time = millis();
    }

    radar_state.last_update = current_time;
    last_angle = target_angle;
}

int16_t calculate_reference_angle(int16_t x, int16_t y) {
    if (x == 0 && y == 0) {
        return 0;
    }

    double angle_radians = atan2((double)y, (double)x);

    double angle_degrees = angle_radians * 180.0 / 3.14;

    int16_t angle;

    if (x < -30000) {

        angle = -((float)y / 1200.0f * 80.0f);
    } else if (x > 30000) {
        angle = ((float)y / 1200.0f * 80.0f);
    }else {
        if (x < 0) {
            angle = -((float)abs(x) / 1000.0f * 80.0f);
        } else {
            angle = ((float)x / 1000.0f * 80.0f);
        }
    }

    if (angle < -80) angle = -80;
    if (angle > 80) angle = 80;

    return angle;
}

void parse_target_info(uint8_t* data, int offset, TargetInfo& target) {

    uint16_t x_raw = data[offset] + (data[offset + 1] << 8);
    int16_t x;

    if (x_raw & 0x8000) {
        x = -(x_raw & 0x7FFF);
    } else {
        x = x_raw;
    }
    target.x = x;

    uint16_t y_raw = data[offset + 2] + (data[offset + 3] << 8);
    int16_t y = (y_raw & 0x7FFF) - ((y_raw & 0x8000) ? 0 : 32768);
    target.y = y;

    target.distance = abs(y) / 10.0f;

    float compensated_x = (float)x;
    float compensated_y = (float)y;
    float raw_angle = 0.0f;

    compensate_radar_position(compensated_x, compensated_y, raw_angle);

    float angle_factor = 0.95f;
    if (abs(raw_angle) > 50.0f) {

        angle_factor = 0.95f + (abs(raw_angle) - 50.0f) * 0.005f;
    }

    target.angle = raw_angle * angle_factor;

    if (target.angle > 80.0f) target.angle = 80.0f;
    if (target.angle < -80.0f) target.angle = -80.0f;

    if (target.angle < -0.5f) {
        target.direction = DIR_LEFT;
    } else if (target.angle > 0.5f) {
        target.direction = DIR_RIGHT;
    } else {
        target.direction = DIR_NONE;
    }

}

bool read_radar_data() {

    if (!is_radar_enabled()) {
        return false;
    }

    static unsigned long lastReadTime = 0;
    const unsigned long READ_INTERVAL = 50;
    static uint8_t buffer[64];
    static int bufferIndex = 0;

    if (millis() - lastReadTime < READ_INTERVAL) {
        return false;
    }

    while (radarSerial.available() && bufferIndex < sizeof(buffer)) {
        buffer[bufferIndex] = radarSerial.read();
        bufferIndex++;
    }

    for (int i = 0; i < bufferIndex - 29; i++) {
        if (buffer[i] == 0xAA && buffer[i+1] == 0xFF &&
            buffer[i+2] == 0x03 && buffer[i+3] == 0x00 &&
            buffer[i+28] == 0x55 && buffer[i+29] == 0xCC) {

            RadarDetection detection;
            detection.isValid = true;
            detection.targetCount = 1;
            detection.primaryDirection = DIR_NONE;

            parse_target_info(buffer, i + 4, detection.targets[0]);

            if (detection.targets[0].distance > 0) {
                float real_distance = detection.targets[0].distance;
                float angle = detection.targets[0].angle;

                if (angle < -5) {
                    detection.primaryDirection = DIR_LEFT;
                } else if (angle > 5) {
                    detection.primaryDirection = DIR_RIGHT;
                }

                detection.suggestedHeadAngle = calculate_head_angle(detection);

                bool has_change = hasSignificantChange(detection.targets[0], last_detection.targets[0]);
                bool time_to_output = millis() - last_output_time >= OUTPUT_INTERVAL;

                if (has_change || time_to_output) {

                    const char* status = has_change ? "变化" : "定期";

                    const char* direction = "正前方";
                    int angle = (int)detection.targets[0].angle;
                    if (angle < -5) {
                        direction = "左边";
                    } else if (angle > 5) {
                        direction = "右边";
                    }

                    last_output_time = millis();
                }

                last_detection = detection;
            }

            if (i + 30 < bufferIndex) {
                memmove(buffer, &buffer[i + 30], bufferIndex - (i + 30));
                bufferIndex -= (i + 30);
            } else {
                bufferIndex = 0;
            }

            lastReadTime = millis();
            return true;
        }
    }

    if (bufferIndex > sizeof(buffer) / 2) {
        memmove(buffer, &buffer[bufferIndex/2], bufferIndex - bufferIndex/2);
        bufferIndex = bufferIndex - bufferIndex/2;
    }

    return false;
}

RadarDetection get_radar_detection() {
    return last_detection;
}

int calculate_head_angle(const RadarDetection& detection) {

    if (!is_radar_enabled()) {
        return (int)get_current_head_angle();
    }

    if (radar_state.is_tracking) {
        return (int)radar_state.current_angle;
    }

    return (int)get_current_head_angle();
}
