#include "SenseVoiceClient.h"
#include "WavPlayer.h"
#include <WiFiClient.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <Avatar.h>
#include <ArduinoJson.h>
#include "ServoControl.h"
#include "CloudConfig.h"

namespace {
constexpr const char* API_SCHEME = STT_API_SCHEME;
constexpr const char* API_HOST = STT_API_HOST;
constexpr const char* API_PATH = STT_API_PATH;
}
using namespace m5avatar;
extern Avatar avatar;
extern WavPlayer wavPlayer;
extern bool is_speaking;
extern bool servo_home;
extern bool is_searching_face;
extern bool gestureModuleEnabled;

extern void executeServoMotion(int x, int y, int duration);
extern void resetServoPosition();
extern void startMoveForward(unsigned long duration);
extern void startMoveBackward(unsigned long duration);
extern void startTurnLeft(unsigned long duration);
extern void startTurnRight(unsigned long duration);
extern void stopWheels();
extern void shakeHead();
extern void startInternalSpeaker();
extern void stopInternalMic();
extern void stopInternalSpeaker();
extern void startInternalMic();

extern "C" const Expression expressions_table[];

extern String deviceUniqueID;

static QueueHandle_t audioQueue = NULL;
static TaskHandle_t audioReadTask = NULL;

struct AudioSegment {
    uint8_t* buffer;
    size_t size;
    bool isLast;
};

void audioReadTaskFunction(void* parameter) {
    HTTPClient* https = (HTTPClient*)parameter;
    const int WAV_HEADER_SIZE = 44;

    uint8_t wavHeader[WAV_HEADER_SIZE];
    if (https->getStream().readBytes(wavHeader, WAV_HEADER_SIZE) != WAV_HEADER_SIZE) {
        Serial.println("读取WAV头部失败");
        vTaskDelete(NULL);
        return;
    }

    if (wavHeader[0] != 'R' || wavHeader[1] != 'I' ||
        wavHeader[2] != 'F' || wavHeader[3] != 'F') {
        Serial.println("无效的WAV格式");
        vTaskDelete(NULL);
        return;
    }

    size_t audioDataSize = https->getSize() - WAV_HEADER_SIZE;
    const size_t READ_BUFFER_SIZE = 16384;
    uint8_t* readBuffer = (uint8_t*)malloc(READ_BUFFER_SIZE);

    if (!readBuffer) {
        Serial.println("分配读取缓冲区失败");
        vTaskDelete(NULL);
        return;
    }

    if (audioDataSize < 300000) {
        Serial.println("音频文件较小，直接处理...");
        uint8_t* audioBuffer = (uint8_t*)malloc(audioDataSize + WAV_HEADER_SIZE);
        if (!audioBuffer) {
            Serial.println("分配音频缓冲区失败");
            free(readBuffer);
            vTaskDelete(NULL);
            return;
        }

        memcpy(audioBuffer, wavHeader, WAV_HEADER_SIZE);

        size_t totalRead = 0;
        while (https->connected() && totalRead < audioDataSize) {
            size_t available = https->getStream().available();
            if (available == 0) {
                delay(5);
                continue;
            }

            size_t bytesToRead = min(min(available, READ_BUFFER_SIZE),
                                   audioDataSize - totalRead);
            size_t bytesRead = https->getStream().readBytes(readBuffer, bytesToRead);

            if (bytesRead > 0) {
                memcpy(audioBuffer + WAV_HEADER_SIZE + totalRead,
                       readBuffer, bytesRead);
                totalRead += bytesRead;
            }
        }

        uint32_t chunkSize = totalRead + WAV_HEADER_SIZE - 8;
        audioBuffer[4] = chunkSize & 0xFF;
        audioBuffer[5] = (chunkSize >> 8) & 0xFF;
        audioBuffer[6] = (chunkSize >> 16) & 0xFF;
        audioBuffer[7] = (chunkSize >> 24) & 0xFF;

        audioBuffer[40] = totalRead & 0xFF;
        audioBuffer[41] = (totalRead >> 8) & 0xFF;
        audioBuffer[42] = (totalRead >> 16) & 0xFF;
        audioBuffer[43] = (totalRead >> 24) & 0xFF;

        AudioSegment segment = {
            .buffer = audioBuffer,
            .size = WAV_HEADER_SIZE + totalRead,
            .isLast = true
        };

        if (xQueueSend(audioQueue, &segment, portMAX_DELAY) != pdPASS) {
            Serial.println("发送音频数据到队列失败");
            free(audioBuffer);
        }

        free(readBuffer);
        vTaskDelete(NULL);
        return;
    }

    const size_t SEGMENT_SIZE = audioDataSize / 10;

    for (int i = 0; i < 10; i++) {
        size_t currentSegmentSize = (i < 9) ? SEGMENT_SIZE :
                                   (audioDataSize - 9 * SEGMENT_SIZE);

        uint8_t* segmentBuffer = (uint8_t*)malloc(currentSegmentSize + WAV_HEADER_SIZE);
        if (!segmentBuffer) {
            Serial.printf("分配第%d段缓冲区失败\n", i + 1);
            continue;
        }

        memcpy(segmentBuffer, wavHeader, WAV_HEADER_SIZE);

        size_t segmentRead = 0;
        while (https->connected() && segmentRead < currentSegmentSize) {
            size_t available = https->getStream().available();
            if (available == 0) {
                delay(5);
                continue;
            }

            size_t bytesToRead = min(min(available, READ_BUFFER_SIZE),
                                   currentSegmentSize - segmentRead);
            size_t bytesRead = https->getStream().readBytes(readBuffer, bytesToRead);

            if (bytesRead > 0) {
                memcpy(segmentBuffer + WAV_HEADER_SIZE + segmentRead,
                       readBuffer, bytesRead);
                segmentRead += bytesRead;
            }
        }

        uint32_t chunkSize = segmentRead + WAV_HEADER_SIZE - 8;
        segmentBuffer[4] = chunkSize & 0xFF;
        segmentBuffer[5] = (chunkSize >> 8) & 0xFF;
        segmentBuffer[6] = (chunkSize >> 16) & 0xFF;
        segmentBuffer[7] = (chunkSize >> 24) & 0xFF;

        segmentBuffer[40] = segmentRead & 0xFF;
        segmentBuffer[41] = (segmentRead >> 8) & 0xFF;
        segmentBuffer[42] = (segmentRead >> 16) & 0xFF;
        segmentBuffer[43] = (segmentRead >> 24) & 0xFF;

        memcpy(segmentBuffer + 20, wavHeader + 20, 16);

        AudioSegment segment = {
            .buffer = segmentBuffer,
            .size = WAV_HEADER_SIZE + segmentRead,
            .isLast = (i == 9)
        };

        if (xQueueSend(audioQueue, &segment, portMAX_DELAY) != pdPASS) {
            Serial.printf("发送第%d段到队列失败\n", i + 1);
            free(segmentBuffer);
        }
    }

    free(readBuffer);
    vTaskDelete(NULL);
}

String SenseVoiceClient::Transcribe(AudioWhisper* audio) {

    if (audio == nullptr || audio->GetBuffer() == nullptr || audio->GetSize() <= 44) {
        Serial.println("无效的音频数据");
        stopInternalMic();
        return "";
    }

    const bool useTls = String(API_SCHEME).equalsIgnoreCase("https");
    std::unique_ptr<WiFiClient> client;
    if (useTls) {
        auto* secure = new WiFiClientSecure();
        if (!secure) {
            Serial.println("创建WiFiClientSecure失败");
            stopInternalMic();
            return "";
        }
        secure->setInsecure();
        secure->setTimeout(60000);
        client.reset(secure);
    } else {
        auto* plain = new WiFiClient();
        if (!plain) {
            Serial.println("创建WiFiClient失败");
            stopInternalMic();
            return "";
        }
        plain->setTimeout(60000);
        client.reset(plain);
    }

    HTTPClient https;
    String response = "";

    String url = String(API_SCHEME) + "://" + API_HOST + API_PATH;
    Serial.printf("[STT] %s\n", url.c_str());

    if (!https.begin(*client, url)) {
        Serial.println("开始HTTP请求失败");
        stopInternalMic();
        return "";
    }

    https.setTimeout(60000);

    String boundary = "------------------------";
    for (auto i = 0; i < 2; ++i) {
        boundary += String(random(0x7fffffff), HEX);
    }

    https.addHeader("Content-Type", "multipart/form-data; boundary=" + boundary);
    https.addHeader("Accept", "*/*");
    https.addHeader("Connection", "keep-alive");

    String head = "--" + boundary + "\r\n";
    head += "Content-Disposition: form-data; name=\"audio\"; filename=\"audio.wav\"\r\n";
    head += "Content-Type: application/octet-stream\r\n\r\n";

    String deviceIdPart = "\r\n--" + boundary + "\r\n";
    deviceIdPart += "Content-Disposition: form-data; name=\"device_id\"\r\n";
    deviceIdPart += "Content-Type: text/plain\r\n\r\n";
    deviceIdPart += deviceUniqueID + "\r\n";

    String tail = "--" + boundary + "--\r\n";

    size_t totalLength = head.length() + audio->GetSize() + deviceIdPart.length() + tail.length();

    Serial.println("发送POST请求...");
    uint8_t* postData = (uint8_t*)malloc(totalLength);
    if (!postData) {
        Serial.println("分配POST数据内存失败");
        return "";
    }

    size_t currentPos = 0;
    memcpy(postData, head.c_str(), head.length());
    currentPos += head.length();
    memcpy(postData + currentPos, audio->GetBuffer(), audio->GetSize());
    currentPos += audio->GetSize();
    memcpy(postData + currentPos, deviceIdPart.c_str(), deviceIdPart.length());
    currentPos += deviceIdPart.length();
    memcpy(postData + currentPos, tail.c_str(), tail.length());

    const int MAX_RETRIES = 3;
    const int RETRY_DELAY = 1000;
    int retryCount = 0;
    int httpCode;

    while (retryCount < MAX_RETRIES) {
        httpCode = https.POST(postData, totalLength);

        if (httpCode >= 0) {
            break;
        }

        Serial.printf("HTTP POST失败，错误码: %d，尝试重试 %d/%d\n",
                     httpCode, retryCount + 1, MAX_RETRIES);

        retryCount++;
        if (retryCount < MAX_RETRIES) {
            delay(RETRY_DELAY);
        }
    }

    free(postData);

    if (httpCode < 0) {
        Serial.printf("HTTP POST最终失败，错误码: %d\n", httpCode);
        https.end();
        stopInternalMic();
        return "请求失败，请稍后重试";
    }

    int len = https.getSize();
    Serial.printf("响应大小: %d bytes\n", len);

    if (len <= 0) {
        Serial.println("未获取到有效的响应大小");
        https.end();
        stopInternalMic();
        return "服务器返回数据无效，请稍后重试";
    }

    bool isAudio = (len > 10000);

if (!isAudio) {
    String textResponse = https.getString();
    Serial.printf("原始响应: %s\n", textResponse.c_str());
    https.end();
    avatar.setSpeechText("");

    String executionResults = "";

    int startPos = 0;
    int endPos = 0;

    while (startPos < textResponse.length()) {

        startPos = textResponse.indexOf('{', startPos);
        if (startPos == -1) break;

        endPos = textResponse.indexOf('}', startPos);
        if (endPos == -1) break;

        String actionJson = textResponse.substring(startPos, endPos + 1);
        startPos = endPos + 1;

        DynamicJsonDocument doc(2048);
        DeserializationError error = deserializeJson(doc, actionJson);

        if (!error && doc.containsKey("type")) {
            String actionType = doc["type"].as<String>();
            String result = "";

            if (actionType == "motion") {
                Serial.println("执行头部运动指令");
                JsonArray actions = doc["actions"];
                for (JsonVariant action : actions) {
                    int x = action["x"] | 85;
                    int y = action["y"] | 90;
                    int duration = action["duration"] | 1000;
                    Serial.printf("执行头部动作: x=%d, y=%d, duration=%d\n", x, y, duration);
                    executeServoMotion(x, y, duration);
                    delay(duration);
                }
                resetServoPosition();
                result = "头部运动完成";
            }
            else if (actionType == "wheel") {
                Serial.println("执行轮子动作指令");
                String command = doc["command"].as<String>();
                int duration = doc["duration"] | 1000;
                int circles = doc["circles"] | 1;

                Serial.printf("执行轮子动作: %s, duration=%d, circles=%d\n",
                            command.c_str(), duration, circles);

                if (command == "forward") {
                    is_searching_face = true;
                    startMoveForward(duration);
                    delay(duration);
                    is_searching_face = false;
                } else if (command == "backward") {
                    startMoveBackward(duration);
                    delay(duration);
                } else if (command == "left") {
                    startTurnLeft(duration);
                    delay(duration);
                } else if (command == "right") {
                    startTurnRight(duration);
                    delay(duration);
                } else if (command == "circle") {
                    String direction = "right";
                    if(random(2) == 1){
                        direction = "left";
                    }
                    for(int i = 0; i < circles; i++) {
                        if (direction == "left") {
                            startTurnLeft(18000);
                        } else {
                            startTurnRight(18000);
                        }
                        delay(18000);
                    }
                } else if (command == "stop") {
                    stopWheels();
                }
                result = "轮子动作完成";
            }
            else if (actionType == "head") {
                Serial.println("执行头部动作指令");
                String command = doc["command"].as<String>();

                if (command == "shake") {

                    executeServoMotion(0, 0, 100);
                    delay(100);

                    executeServoMotion(25, 0, 200);
                    delay(200);

                    executeServoMotion(-25, 0, 200);
                    delay(200);

                    executeServoMotion(25, 0, 200);
                    delay(200);

                    executeServoMotion(-25, 0, 200);
                    delay(200);

                    executeServoMotion(0, 0, 200);
                    delay(200);

                    result = "摇头动作完成";
                } else if (command == "nod") {
                     shakeHead();
                    result = "点头动作完成";
                }
            }
            else if (actionType == "expression") {
                Serial.println("执行表情动作指令");
                String expression = doc["expression"].as<String>();

                if (expression == "happy") {
                    avatar.setExpression(Expression::Happy);
                     delay(800);
                } else if (expression == "angry") {
                    avatar.setExpression(Expression::Angry);
                    delay(800);
                } else if (expression == "sad") {
                    avatar.setExpression(Expression::Sad);
                    delay(800);
                } else if (expression == "neutral") {
                    avatar.setExpression(Expression::Neutral);
                    delay(800);
                } else if (expression == "sleepy") {
                    avatar.setExpression(Expression::Sleepy);
                    delay(800);
                }
                result = "表情已更新";
            }
            else if (actionType == "other") {
                Serial.println("执行智能控制");
                String command = doc["command"].as<String>();

                if (command == "gestureModuleEnabled") {
                    gestureModuleEnabled = true;
                    startTurnLeft(200);
                    stopInternalMic();
                    return "";
                }
                result = "智能控制已开启";
                if (command == "shutdown") {
                    avatar.setSpeechText("关机");
                    delay(1000);
                    M5.Power.powerOff();
                    stopInternalMic();
                    return "";
                }
            }

            if (result.length() > 0) {
                if (executionResults.length() > 0) {
                    executionResults += "\n";
                }
                executionResults += result;
            }
        }

        if (error) {
            Serial.printf("JSON解析错误: %s\n", error.c_str());
            https.end();
            stopInternalMic();
            return "";
        }
    }
    if(textResponse != "520"){
        is_speaking = true;
    }
    return executionResults.length() > 0 ? executionResults : "无法执行动作指令";
}

    Serial.println("处理音频响应...");

    if (!https.connected()) {
        Serial.println("HTTP连接已断开，无法处理音频");
        https.end();
        stopInternalMic();
        return "服务连接已断开，请稍后重试";
    }

    if (audioQueue == NULL) {
        audioQueue = xQueueCreate(10, sizeof(AudioSegment));
        if (audioQueue == NULL) {
            Serial.println("创建音频队列失败");
            https.end();
            stopInternalMic();
            return "内存不足，无法创建音频队列";
        }
    }

    BaseType_t taskCreated = xTaskCreate(
        audioReadTaskFunction,
        "AudioRead",
        8192,
        &https,
        1,
        &audioReadTask
    );

    if (taskCreated != pdPASS) {
        Serial.println("创建音频读取任务失败");
        if (audioQueue != NULL) {
            vQueueDelete(audioQueue);
            audioQueue = NULL;
        }
        https.end();
        stopInternalMic();
        return "内存不足，无法创建音频处理任务";
    }

    int bufferedSegments = 0;
    const int START_PLAY_THRESHOLD = 4;
    AudioSegment segments[10];
    int segmentCount = 0;
    bool firstPlay = true;

    while (true) {
        AudioSegment segment;
        if (xQueueReceive(audioQueue, &segment, portMAX_DELAY) == pdPASS) {
            if (firstPlay) {

                segments[segmentCount] = segment;
                segmentCount++;
                bufferedSegments++;
                stopInternalMic();
                delay(100);
                startInternalSpeaker();

                Serial.printf("首次播放缓冲中: %d/%d\n", bufferedSegments, START_PLAY_THRESHOLD);

                if (bufferedSegments < START_PLAY_THRESHOLD && !segment.isLast) {
                    continue;
                }

                avatar.setSpeechText("");
                servo_home = true;
                Serial.println("开始播放首批缓存的音频段");
                for (int i = 0; i < segmentCount; i++) {
                    if (wavPlayer.begin(segments[i].buffer, segments[i].size)) {
                        while (wavPlayer.isRunning()) {
                            if (!wavPlayer.loop()) {
                                break;
                            }
                            yield();
                        }
                        wavPlayer.stop();
                    }
                    free(segments[i].buffer);

                    if (segments[i].isLast) {

                        vQueueDelete(audioQueue);
                        audioQueue = NULL;
                        https.end();
                        is_speaking = true;
                        stopInternalSpeaker();
                        delay(100);
                        return "";
                    }
                }

                firstPlay = false;
                segmentCount = 0;
                bufferedSegments = 0;
            } else {

                if (wavPlayer.begin(segment.buffer, segment.size)) {
                    while (wavPlayer.isRunning()) {
                        if (!wavPlayer.loop()) {
                            break;
                        }
                        yield();
                    }
                    wavPlayer.stop();
                }
                free(segment.buffer);
                stopInternalSpeaker();
                delay(100);

                if (segment.isLast) {

                    vQueueDelete(audioQueue);
                    audioQueue = NULL;
                    https.end();
                    is_speaking = true;
                    return "";
                }
            }
        }
    }

}
