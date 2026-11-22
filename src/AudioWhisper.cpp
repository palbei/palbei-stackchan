#include <M5Unified.h>
#include "AudioWhisper.h"
#include <Avatar.h>
#include <driver/i2s.h>
#include <vector>
#include <algorithm>

using namespace m5avatar;

extern Avatar avatar;

#define I2S_WS 33
#define I2S_SD 14
#define I2S_SCK 34
#define I2S_PORT I2S_NUM_0
#define I2S_MCLK1 0
#define I2S_SAMPLE_RATE   16000
#define I2S_SAMPLE_BITS   16
#define I2S_READ_LEN      (2 * 1024)
#define RECORD_TIME       20
#define I2S_CHANNEL_NUM   1
#define BYTES_PER_SAMPLE  (I2S_SAMPLE_BITS / 8)
#define FLASH_RECORD_SIZE (I2S_CHANNEL_NUM * I2S_SAMPLE_RATE * BYTES_PER_SAMPLE * RECORD_TIME)

#define VAD_FRAME_SIZE 480
#define SILENCE_THRESHOLD 0.002f
#define SILENCE_TIMEOUT 1000
#define MAX_RECORD_TIME 20000
#define MIN_SPEECH_FRAMES 1
#define ENERGY_SMOOTH_FACTOR 0.3f
#define CALIBRATION_FRAMES 20
#define PRE_BUFFER_SIZE 16000

float g_speech_threshold = 0.003f;
float g_min_valid_energy = 0.002f;
float g_noise_floor = 0.0f;
float g_smoothed_baseline = 0.0f;

constexpr int headerSize = 44;

AudioWhisper::AudioWhisper() : record_buffer(nullptr), actual_size(0), listen_aborted(false) {
    const auto size = FLASH_RECORD_SIZE + headerSize;
    record_buffer = static_cast<byte*>(heap_caps_malloc(size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    if (record_buffer == nullptr) {
        Serial.println("Failed to allocate memory for record buffer");
        return;
    }
    memset(record_buffer, 0, size);
}

AudioWhisper::~AudioWhisper() {
    if (record_buffer != nullptr) {
        heap_caps_free(record_buffer);
        record_buffer = nullptr;
    }
}

void AudioWhisper::i2s_init() {
    esp_err_t err;

    i2s_config_t i2s_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
        .sample_rate = I2S_SAMPLE_RATE,
        .bits_per_sample = i2s_bits_per_sample_t(I2S_SAMPLE_BITS),
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
        .mck_io_num = I2S_MCLK1,
        .bck_io_num = I2S_SCK,
        .ws_io_num = I2S_WS,
        .data_out_num = I2S_PIN_NO_CHANGE,
        .data_in_num = I2S_SD
    };

    err = i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
    if (err != ESP_OK) {
        Serial.printf("Failed to install I2S driver: %d\n", err);
        return;
    }

    err = i2s_set_pin(I2S_PORT, &pin_config);
    if (err != ESP_OK) {
        Serial.printf("Failed to set I2S pins: %d\n", err);
        i2s_driver_uninstall(I2S_PORT);
        return;
    }

    err = i2s_zero_dma_buffer(I2S_PORT);
    if (err != ESP_OK) {
        Serial.printf("Failed to zero DMA buffer: %d\n", err);
        i2s_driver_uninstall(I2S_PORT);
        return;
    }

    Serial.println("I2S initialized successfully");
}

int16_t* AudioWhisper::MakeHeader(byte* header, size_t actualDataSize) {
    header[0] = 'R';
    header[1] = 'I';
    header[2] = 'F';
    header[3] = 'F';
    unsigned int fileSizeMinus8 = actualDataSize + headerSize - 8;
    header[4] = (byte)(fileSizeMinus8 & 0xFF);
    header[5] = (byte)((fileSizeMinus8 >> 8) & 0xFF);
    header[6] = (byte)((fileSizeMinus8 >> 16) & 0xFF);
    header[7] = (byte)((fileSizeMinus8 >> 24) & 0xFF);
    header[8] = 'W';
    header[9] = 'A';
    header[10] = 'V';
    header[11] = 'E';
    header[12] = 'f';
    header[13] = 'm';
    header[14] = 't';
    header[15] = ' ';
    header[16] = 0x10;
    header[17] = 0x00;
    header[18] = 0x00;
    header[19] = 0x00;
    header[20] = 0x01;
    header[21] = 0x00;
    header[22] = 0x01;
    header[23] = 0x00;
    header[24] = 0x80;
    header[25] = 0x3E;
    header[26] = 0x00;
    header[27] = 0x00;
    header[28] = 0x00;
    header[29] = 0x7D;
    header[30] = 0x00;
    header[31] = 0x00;
    header[32] = 0x02;
    header[33] = 0x00;
    header[34] = 0x10;
    header[35] = 0x00;
    header[36] = 'd';
    header[37] = 'a';
    header[38] = 't';
    header[39] = 'a';
    header[40] = (byte)(actualDataSize & 0xFF);
    header[41] = (byte)((actualDataSize >> 8) & 0xFF);
    header[42] = (byte)((actualDataSize >> 16) & 0xFF);
    header[43] = (byte)((actualDataSize >> 24) & 0xFF);
    return (int16_t*)&header[headerSize];
}

bool AudioWhisper::isHumanVoice(float currentEnergy) {
    static float smoothedEnergy = 0;
    static int steadyCount = 0;
    static float lastEnergy = 0;
    static float maxEnergy = 0;
    static int frameCount = 0;
    static int silenceCount = 0;
    static float minEnergy = 1.0f;

    smoothedEnergy = ENERGY_SMOOTH_FACTOR * currentEnergy + (1 - ENERGY_SMOOTH_FACTOR) * lastEnergy;

    maxEnergy = max(smoothedEnergy, maxEnergy * 0.998f);
    minEnergy = min(smoothedEnergy, minEnergy * 1.002f);

    float energyChange = abs(smoothedEnergy - lastEnergy);
    lastEnergy = smoothedEnergy;

    frameCount++;

    if (frameCount > 500) {
        maxEnergy *= 0.97f;
        minEnergy = smoothedEnergy;
        frameCount = 0;
    }

    float dynamicThreshold = g_noise_floor * 1.05f + (maxEnergy - g_noise_floor) * 0.05f;
    float currentThreshold = min(0.1f, dynamicThreshold);

    bool isValidEnergy = smoothedEnergy > g_min_valid_energy;
    bool hasStableDynamics = energyChange > 0.00005f && energyChange < 0.4f;
    bool hasSignificantEnergy = smoothedEnergy > currentThreshold;

    bool isSpeech = isValidEnergy && hasStableDynamics && hasSignificantEnergy;

    if (isSpeech) {
        steadyCount++;
        silenceCount = 0;
    } else {
        silenceCount++;
        if (silenceCount > 15) {
            steadyCount = max(0, steadyCount - 1);
        }
    }

    return (steadyCount >= 2) && (smoothedEnergy > currentThreshold);
}

void AudioWhisper::Record() {
    if (record_buffer == nullptr) {
        Serial.println("Record buffer not initialized");
        return;
    }

    listen_aborted = false;
    actual_size = 0;

    size_t bytesRead = 0;
    uint8_t* temp_buffer = (uint8_t*)heap_caps_malloc(I2S_READ_LEN, MALLOC_CAP_8BIT);
    if (temp_buffer == nullptr) {
        Serial.println("Failed to allocate temp buffer");
        return;
    }

    size_t totalBytesRead = 0;
    unsigned long startTime = 0;
    unsigned long currentTime;
    unsigned long lastSpeechTime = 0;
    bool isRecording = false;
    int consecutiveSpeechFrames = 0;
    bool speechDetected = false;
    int silenceFrames = 0;

    std::vector<uint8_t> recordBuffer;
    recordBuffer.reserve(FLASH_RECORD_SIZE);

    std::vector<uint8_t> preBuffer;
    preBuffer.reserve(PRE_BUFFER_SIZE);

    float avgEnergy = 0;
    int energyCount = 0;

    while (true) {
        currentTime = millis();

        // 开口前点一下屏幕 → 退出聊天
        if (!isRecording) {
            M5.update();
            auto tc = M5.Touch.getCount();
            if (tc) {
                auto td = M5.Touch.getDetail();
                if (td.wasPressed()) {
                    Serial.println("点屏取消聆听，退出聊天");
                    listen_aborted = true;
                    speechDetected = false;
                    break;
                }
            }
        }

        if (i2s_read(I2S_PORT, temp_buffer, I2S_READ_LEN, &bytesRead, 100) == ESP_OK) {
            float energy = 0;
            int16_t* samples = (int16_t*)temp_buffer;
            int numSamples = bytesRead / 2;

            for (int i = 0; i < numSamples; i++) {
                float sample = abs(samples[i]) / 32768.0f;
                energy += sample * sample;
            }
            energy = sqrt(energy / numSamples);

            if (energyCount < CALIBRATION_FRAMES) {
                avgEnergy += energy;
                energyCount++;
                continue;
            } else if (energyCount == CALIBRATION_FRAMES) {
                avgEnergy /= CALIBRATION_FRAMES;
                if (g_smoothed_baseline == 0) {
                    g_smoothed_baseline = avgEnergy;
                }
                g_smoothed_baseline = 0.95f * g_smoothed_baseline + 0.05f * avgEnergy;
                g_noise_floor = g_smoothed_baseline;
                g_min_valid_energy = g_smoothed_baseline * 1.01f;
                g_speech_threshold = g_smoothed_baseline * 1.05f;
                energyCount++;
                Serial.printf("环境噪音基准: %.6f, 平滑基准: %.6f, 调整后的阈值: %.6f\n",
                             avgEnergy, g_smoothed_baseline, g_speech_threshold);
                Serial.println("等待说话...");
                avatar.setSpeechText("聆听中...");
            } else {
                g_smoothed_baseline = 0.95f * g_smoothed_baseline + 0.05f * energy;
                g_noise_floor = g_smoothed_baseline;
                g_min_valid_energy = g_smoothed_baseline * 1.01f;
                g_speech_threshold = g_smoothed_baseline * 1.05f;
            }

            bool isSpeech = isHumanVoice(energy);

            if (!isRecording) {
                if (preBuffer.size() >= PRE_BUFFER_SIZE) {
                    preBuffer.erase(preBuffer.begin(), preBuffer.begin() + bytesRead);
                }
                preBuffer.insert(preBuffer.end(), temp_buffer, temp_buffer + bytesRead);

                if (isSpeech) {
                    consecutiveSpeechFrames++;
                    if (consecutiveSpeechFrames >= MIN_SPEECH_FRAMES) {
                        isRecording = true;
                        speechDetected = true;
                        startTime = currentTime;
                        Serial.println("检测到说话，开始录音...");
                        avatar.setSpeechText("聆听中...");
                        recordBuffer.insert(recordBuffer.end(), preBuffer.begin(), preBuffer.end());
                        preBuffer.clear();
                    }
                } else {
                    consecutiveSpeechFrames = 0;
                }
            }

            if (isRecording) {
                if (isSpeech) {
                    lastSpeechTime = currentTime;
                    silenceFrames = 0;
                } else {
                    silenceFrames++;
                }

                recordBuffer.insert(recordBuffer.end(), temp_buffer, temp_buffer + bytesRead);
                totalBytesRead = recordBuffer.size();

                bool silenceTimeout = silenceFrames > (SILENCE_TIMEOUT / 30);
                if (silenceTimeout ||
                    currentTime - startTime > MAX_RECORD_TIME ||
                    totalBytesRead >= FLASH_RECORD_SIZE) {
                    break;
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }

    if (!speechDetected) {
        Serial.println("未检测到有效语音");
        totalBytesRead = 0;
        heap_caps_free(temp_buffer);

        return;
    }

    if (totalBytesRead > 0) {
        size_t copySize = (totalBytesRead < FLASH_RECORD_SIZE) ? totalBytesRead : FLASH_RECORD_SIZE;
        memcpy(record_buffer + headerSize, recordBuffer.data(), copySize);
        actual_size = copySize + headerSize;
        auto *wavData = MakeHeader(record_buffer, copySize);
    }

    recordBuffer.clear();
    preBuffer.clear();
    heap_caps_free(temp_buffer);

    Serial.printf("录音结束，总共读取: %d bytes\n", totalBytesRead);
    avatar.setSpeechText("");
}
