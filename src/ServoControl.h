#ifndef SERVO_CONTROL_H
#define SERVO_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

void executeServoMotion(int x, int y, int duration);
void resetServoPosition();

#ifdef __cplusplus
}
#endif

#endif
