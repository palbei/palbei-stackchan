#ifndef CAMERA_GESTURE_RECOGNIZER_H
#define CAMERA_GESTURE_RECOGNIZER_H

#include <Arduino.h>
#include <esp_camera.h>

class CameraGestureRecognizer {
public:

    enum GestureType {
        GESTURE_NONE = 0,
        GESTURE_LEFT,
        GESTURE_RIGHT,
        GESTURE_UP,
        GESTURE_DOWN,
        GESTURE_FORWARD,
        GESTURE_BACKWARD,
        GESTURE_CLOCKWISE,
        GESTURE_COUNTERCLOCKWISE,
        GESTURE_WAVE,
    };

    CameraGestureRecognizer();
    ~CameraGestureRecognizer();

    bool initialize();

    GestureType detectGesture(camera_fb_t* fb);

    GestureType getLastGesture() const { return m_lastGesture; }

    const char* getGestureName(GestureType gesture) const;

    void setDebug(bool debug) { m_debug = debug; }

    void resetLeftGestureCount() { m_leftGestureCount = 0; }

    int getLeftGestureCount() const { return m_leftGestureCount; }

private:

    bool detectMotion(camera_fb_t* current_fb);

    GestureType analyzeMotion();

    uint8_t* m_previousFrame;
    uint8_t* m_tempFrame;
    uint16_t m_width;
    uint16_t m_height;
    uint8_t  m_blockSize;

    int m_motionX;
    int m_motionY;
    int m_motionArea;

    int* m_blockDiffX;
    int* m_blockDiffY;
    int* m_blockDiffCount;

    GestureType m_lastGesture;
    unsigned long m_lastGestureTime;
    bool m_debug;

    int m_leftGestureCount;

    const int MOTION_THRESHOLD = 20;
    const int GESTURE_COOLDOWN_MS = 1000;
};

#endif
