#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <stdbool.h>

#include "common/debug.h"
#include "pack.h"
#include "encode.h"
#include "constants.h"

#define LOG_LEVEL LOG_INFO

void usage() {
    printf("Generate a 15-second WAV file encoding a given message.\n");
    printf("Usage:\n");
    printf("\n");
    printf("gen_ft8 MESSAGE WAV_FILE [FREQUENCY]\n");
    printf("\n");
    printf("(Note that you might have to enclose your message in quote marks if it contains spaces)\n");
}

int encoder(char *message, uint8_t *tones, int is_ft4) {
    // First, pack the text data into binary message
    uint8_t packed[FT8_LDPC_K_BYTES];
    int rc = pack77(message, packed);
    if (rc < 0)
    {
        printf("Cannot parse message!\n");
        printf("RC = %d\n", rc);
        return -2;
    }

    printf("Packed data: ");
    for (int j = 0; j < 10; ++j)
    {
        printf("%02x ", packed[j]);
    }
    printf("\n");

    if (is_ft4)
    {
        // '[..] for FT4 only, in order to avoid transmitting a long string of zeros when sending CQ messages,
        // the assembled 77-bit message is bitwise exclusive-OR’ed with [a] pseudorandom sequence before computing the CRC and FEC parity bits'
        for (int i = 0; i < 10; ++i)
        {
            packed[i] ^= kFT4_XOR_sequence[i];
        }
    }

    int num_tones = (is_ft4) ? FT4_NN : FT8_NN;

    // Second, encode the binary message as a sequence of FSK tones
    // uint8_t tones[num_tones]; // Array of 79 tones (symbols)
    if (is_ft4)
    {
        genft4(packed, tones);
    }
    else
    {
        genft8(packed, tones);
    }

    printf("FSK tones: ");
    for (int j = 0; j < num_tones; ++j)
    {
        printf("%d", tones[j]);
    }
    printf("\n");

    return 0;
}
