/*
FT8 library for Arduino
Original code: https://github.com/kgoba/ft8_lib
From this commit: https://github.com/kgoba/ft8_lib/commit/cd0dc2eca3c14526d6d96a085be013dd63154e11
 */

#ifndef FT8_H_
#define FT8_H_

#include "Arduino.h"

void ftx_encode(char *message, uint8_t *tones, bool is_ft4);

#endif // FT8_H_
