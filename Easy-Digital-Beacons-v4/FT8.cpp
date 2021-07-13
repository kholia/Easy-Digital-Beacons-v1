#include "FT8.h"
#include "pack.h"
#include "encode.h"
#include "constants.h"

void ftx_encode(char *message, uint8_t *tones, bool isFT4)
{
    // First, pack the text data into binary message
    uint8_t packed[FTX_LDPC_K_BYTES];
    pack77(message, packed);

    // Second, encode the binary message as a sequence of FSK tones
    if (isFT4)
    {
        ft4_encode(packed, tones);
    }
    else
    {
        ft8_encode(packed, tones);
    }
}
