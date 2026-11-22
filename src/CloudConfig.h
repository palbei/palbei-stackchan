#pragma once

// Fill these before build, or override via platformio.ini build_flags, e.g.:
//   -DSTT_API_SCHEME=\"https\"
//   -DSTT_API_HOST=\"api.example.com\"
//   -DSTT_API_PATH=\"/stt\"

#ifndef STT_API_SCHEME
#define STT_API_SCHEME "https"
#endif

#ifndef STT_API_HOST
#define STT_API_HOST "YOUR_STT_API_HOST"
#endif

#ifndef STT_API_PATH
#define STT_API_PATH "/YOUR_STT_PATH"
#endif
