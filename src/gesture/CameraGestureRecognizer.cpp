#include "CameraGestureRecognizer.h"

CameraGestureRecognizer::CameraGestureRecognizer() :
    m_previousFrame(nullptr),
    m_width(0),
    m_height(0),
    m_blockSize(16),
    m_motionX(0),
    m_motionY(0),
    m_motionArea(0),
    m_blockDiffX(nullptr),
    m_blockDiffY(nullptr),
    m_blockDiffCount(nullptr),
    m_lastGesture(GESTURE_NONE),
    m_lastGestureTime(0),
    m_debug(false),
    m_leftGestureCount(0),
    m_tempFrame(nullptr)
{
}

CameraGestureRecognizer::~CameraGestureRecognizer() {
    if (m_previousFrame) {
        free(m_previousFrame);
        m_previousFrame = nullptr;
    }

    if (m_blockDiffX) {
        free(m_blockDiffX);
        m_blockDiffX = nullptr;
    }

    if (m_blockDiffY) {
        free(m_blockDiffY);
        m_blockDiffY = nullptr;
    }

    if (m_blockDiffCount) {
        free(m_blockDiffCount);
        m_blockDiffCount = nullptr;
    }

    if (m_tempFrame) {
        free(m_tempFrame);
        m_tempFrame = nullptr;
    }
}

bool CameraGestureRecognizer::initialize() {

    sensor_t *sensor = esp_camera_sensor_get();
    if (!sensor) {
        Serial.println("无法获取相机传感器配置");
        return false;
    }

    framesize_t framesize = sensor->status.framesize;
    switch (framesize) {
        case FRAMESIZE_QVGA:
            m_width = 320;
            m_height = 240;
            break;
        case FRAMESIZE_VGA:
            m_width = 640;
            m_height = 480;
            break;
        case FRAMESIZE_SVGA:
            m_width = 800;
            m_height = 600;
            break;
        case FRAMESIZE_HD:
            m_width = 1280;
            m_height = 720;
            break;
        default:

            m_width = 320;
            m_height = 240;
            break;
    }

    int numBlocksX = m_width / m_blockSize;
    int numBlocksY = m_height / m_blockSize;
    int totalBlocks = numBlocksX * numBlocksY;

    m_previousFrame = (uint8_t*)malloc(m_width * m_height);
    m_blockDiffX = (int*)malloc(totalBlocks * sizeof(int));
    m_blockDiffY = (int*)malloc(totalBlocks * sizeof(int));
    m_blockDiffCount = (int*)malloc(totalBlocks * sizeof(int));
    m_tempFrame = (uint8_t*)malloc(m_width * m_height);

    if (!m_previousFrame || !m_blockDiffX || !m_blockDiffY || !m_blockDiffCount || !m_tempFrame) {
        Serial.println("内存分配失败");
        return false;
    }

    memset(m_previousFrame, 0, m_width * m_height);
    memset(m_blockDiffX, 0, totalBlocks * sizeof(int));
    memset(m_blockDiffY, 0, totalBlocks * sizeof(int));
    memset(m_blockDiffCount, 0, totalBlocks * sizeof(int));

    Serial.printf("手势识别器初始化成功：%dx%d, 块大小: %d\n", m_width, m_height, m_blockSize);
    return true;
}

CameraGestureRecognizer::GestureType CameraGestureRecognizer::detectGesture(camera_fb_t* fb) {
    if (!fb || !fb->buf) {
        return GESTURE_NONE;
    }

    if (!detectMotion(fb)) {
        return GESTURE_NONE;
    }

    GestureType gesture = analyzeMotion();

    unsigned long currentTime = millis();
    if (gesture != GESTURE_NONE) {
        if (currentTime - m_lastGestureTime > GESTURE_COOLDOWN_MS) {

            if (gesture == GESTURE_LEFT) {
                m_leftGestureCount++;

                if (random(3) == 1) {
                    if (m_debug) {
                        Serial.printf("检测到左手势，随机替换为右手势\n");
                    }
                    gesture = GESTURE_RIGHT;
                } else {
                    if (m_debug) {
                        Serial.printf("检测到左手势，保持为左手势\n");
                    }
                }

                if (m_leftGestureCount > 20) {
                    m_leftGestureCount = 0;
                    if (m_debug) {
                        Serial.println("左手势计数已重置");
                    }
                }

                if (m_debug) {
                    Serial.printf("左手势计数：%d\n", m_leftGestureCount);
                }
            }

            m_lastGesture = gesture;
            m_lastGestureTime = currentTime;
            if (m_debug) {
                Serial.printf("检测到手势: %s\n", getGestureName(gesture));
            }
        } else {

            gesture = GESTURE_NONE;
        }
    }

    return gesture;
}

bool CameraGestureRecognizer::detectMotion(camera_fb_t* current_fb) {
    if (!current_fb || !current_fb->buf || !m_previousFrame) {
        return false;
    }

    m_motionX = 0;
    m_motionY = 0;
    m_motionArea = 0;

    bool isJpeg = current_fb->format == PIXFORMAT_JPEG;
    bool isRgb565 = current_fb->format == PIXFORMAT_RGB565;

    if (!isRgb565 && !isJpeg) {
        if (m_debug) {
            Serial.printf("不支持的帧格式: %d\n", current_fb->format);
        }
        return false;
    }

    if (isJpeg) {
        if (m_debug) {
            Serial.println("JPEG格式暂不支持，请使用RGB565格式");
        }
        return false;
    }

    uint8_t* currentFrame = (uint8_t*)malloc(m_width * m_height);
    if (!currentFrame) {
        return false;
    }

    uint16_t* rgb565 = (uint16_t*)current_fb->buf;
    for (int y = 0; y < m_height; y++) {
        for (int x = 0; x < m_width; x++) {
            int pos = y * m_width + x;
            if (pos < current_fb->len / 2) {
                uint16_t pixel = rgb565[pos];

                uint8_t r = (pixel >> 11) & 0x1F;
                uint8_t g = (pixel >> 5) & 0x3F;
                uint8_t b = pixel & 0x1F;
                currentFrame[pos] = (r + g + b) / 3;
            }
        }
    }

    int numBlocksX = m_width / m_blockSize;
    int numBlocksY = m_height / m_blockSize;

    memset(m_blockDiffX, 0, numBlocksX * numBlocksY * sizeof(int));
    memset(m_blockDiffY, 0, numBlocksX * numBlocksY * sizeof(int));
    memset(m_blockDiffCount, 0, numBlocksX * numBlocksY * sizeof(int));

    int totalDiffPixels = 0;

    for (int by = 0; by < numBlocksY; by++) {
        for (int bx = 0; bx < numBlocksX; bx++) {
            int blockIdx = by * numBlocksX + bx;
            int diffCount = 0;
            int diffX = 0;
            int diffY = 0;

            for (int y = 0; y < m_blockSize; y++) {
                for (int x = 0; x < m_blockSize; x++) {
                    int px = bx * m_blockSize + x;
                    int py = by * m_blockSize + y;

                    if (px < m_width && py < m_height) {
                        int pos = py * m_width + px;
                        int diff = abs(currentFrame[pos] - m_previousFrame[pos]);

                        if (diff > MOTION_THRESHOLD) {
                            diffCount++;
                            diffX += x - m_blockSize/2;
                            diffY += y - m_blockSize/2;
                        }
                    }
                }
            }

            if (diffCount > (m_blockSize * m_blockSize / 10)) {
                m_blockDiffCount[blockIdx] = diffCount;
                m_blockDiffX[blockIdx] = diffX;
                m_blockDiffY[blockIdx] = diffY;
                totalDiffPixels += diffCount;

                m_motionX += diffX;
                m_motionY += diffY;
                m_motionArea += diffCount;
            }
        }
    }

    memcpy(m_previousFrame, currentFrame, m_width * m_height);
    free(currentFrame);

    return (totalDiffPixels > (m_width * m_height / 100));
}

CameraGestureRecognizer::GestureType CameraGestureRecognizer::analyzeMotion() {
    if (m_motionArea <= 0) {
        return GESTURE_NONE;
    }

    float avgX = (float)m_motionX / m_motionArea;
    float avgY = (float)m_motionY / m_motionArea;

    float magnitude = sqrt(avgX * avgX + avgY * avgY);

    if (magnitude < 0.5f) {
        return GESTURE_NONE;
    }

    avgX /= magnitude;
    avgY /= magnitude;

    if (fabs(avgX) > fabs(avgY) * 1.5f) {

        return (avgX > 0) ? GESTURE_RIGHT : GESTURE_LEFT;
    } else if (fabs(avgY) > fabs(avgX) * 1.5f) {

        return (avgY > 0) ? GESTURE_DOWN : GESTURE_UP;
    } else {

        return GESTURE_NONE;
    }

    return GESTURE_NONE;
}

const char* CameraGestureRecognizer::getGestureName(GestureType gesture) const {
    switch (gesture) {
        case GESTURE_NONE:            return "无";
        case GESTURE_LEFT:            return "左";
        case GESTURE_RIGHT:           return "右";
        case GESTURE_UP:              return "上";
        case GESTURE_DOWN:            return "下";
        case GESTURE_FORWARD:         return "前进";
        case GESTURE_BACKWARD:        return "后退";
        case GESTURE_CLOCKWISE:       return "顺时针";
        case GESTURE_COUNTERCLOCKWISE:return "逆时针";
        case GESTURE_WAVE:            return "挥手";
        default:                      return "未知";
    }
}
