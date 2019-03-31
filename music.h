#ifndef __MUSIC_H
#define __MUSIC_H

#include <math.h>
#include <stdint.h>

#include "platform.h"

void music_play(char note, uint8_t octave, int8_t accidental, float duration);

#endif /* __MUSIC_H */
