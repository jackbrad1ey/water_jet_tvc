#include "utils.h"

void get_roll_and_pitch(float acc[3], float *roll, float *pitch) {
    *roll = atan2(acc[1], acc[2]) * 57.3;
    *pitch = atan2(-acc[0], sqrt(acc[1]*acc[1] + acc[2]*acc[2])) * 57.3;
}