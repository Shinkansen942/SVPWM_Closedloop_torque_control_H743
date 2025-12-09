#ifndef __MUSIC_H__
#define __MUSIC_H__
#include "main.h"

#define MUSIC_LENGTH 335676
#define MUSIC_DIVIDER 10

const extern uint8_t mus[MUSIC_LENGTH];

float MUSIC_generate_voltage(float time, float frequency, float amplitude);

float MUSIC_generate_from_data(uint32_t* index, float amplitude);

#endif /* __MUSIC_H__ */