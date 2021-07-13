#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <stdbool.h>

#include "pack.h"
#include "encode.h"
#include "constants.h"

int encoder(char *message, uint8_t *tones, int is_ft4)
{
    // First, pack the text data into binary message
    uint8_t packed[FTX_LDPC_K_BYTES];
    pack77(message, packed);

    // Second, encode the binary message as a sequence of FSK tones
    if (is_ft4)
    {
        ft4_encode(packed, tones);
    }
    else
    {
        ft8_encode(packed, tones);
    }

    return 0;
}
