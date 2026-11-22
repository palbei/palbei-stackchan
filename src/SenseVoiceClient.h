#ifndef _SENSEVOICECLIENT_H
#define _SENSEVOICECLIENT_H

#include <WiFiClientSecure.h>
#include "AudioWhisper.h"

class SenseVoiceClient {

public:

  String Transcribe(AudioWhisper* audio);
};

#endif
