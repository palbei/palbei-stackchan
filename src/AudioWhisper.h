#ifndef _AUDIOWHISPER_H
#define _AUDIOWHISPER_H

#include <Arduino.h>
#include <esp32-hal.h>

class AudioWhisper {
private:
    byte* record_buffer;
    size_t actual_size;
    bool listen_aborted;

    void i2s_init();
    bool isHumanVoice(float currentEnergy);
    int16_t* MakeHeader(byte* header, size_t actualDataSize);

public:
    AudioWhisper();
    ~AudioWhisper();

    const byte* GetBuffer() const { return record_buffer; }
    inline size_t GetSize() const { return actual_size; }
    /** true: 点屏取消聆听，未采到有效语音 */
    inline bool ListenAborted() const { return listen_aborted; }
    void Record();
};

#endif
