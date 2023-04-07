/*
   wspr_enc.c

   This program implements a WSPR encoder function "wspr_enc". Its purpose is
   to encode the callsign, Maidenhead locator, and TX power into WSPR channel
   symbols. WSPR message types 1, 2, and 3 are supported. In addition, K1JT's
   nhash algorithm is also required. When building the target, files nhash.c
   and nhash.h must be added into the project.

   "wspr_enc" is written for AVRs. The native test was performed through Atmel
   Studio with toolchain 3.6.2. However, the code works on other platforms. It
   has been tested on Raspbian, Armbian, Ubuntu, macOS, and Windows recently.

   This encoder takes advantage of the following programs:
       Joseph Taylor, K1JT: nhash.c, nhash.h
           https://sourceforge.net/p/wsjt/wsjtx/ci/master/tree/lib/wsprd

       James Peroulas: wspr.cpp
           https://github.com/JamesP6000/WsprryPi

   References:
   G4JNT: The WSPR Coding Process
       http://www.g4jnt.com/wspr_coding_process.pdf

   VA7NRM: Inside WSPR, JT65 and JT9 Weak-signal HF Modes
       http://archive.nsarc.ca/hf/jt_modes.pdf

   Author: BD1ES

   21 FEB 2020:
       Released to the Gist:
           https://gist.github.com/bd1es/a782e2529b8289288fadd35e407f6440
   31 MAR 2020:
       Replaced the parity table with an algorithm introduced by Sean Anderson:
           http://graphics.stanford.edu/~seander/bithacks.html#ParityParallel
*/

#include <string.h>
#include <stdlib.h>
#include <ctype.h>

#if !defined(__AVR_ARCH__)
#include <stdio.h>
#endif

#include "nhash.h"
#include "wspr_enc.h"

// These are defined for porting between AVR GCC and PC.
#if defined(__GNUC__) && defined(__AVR_ARCH__)
#include    <avr/pgmspace.h>
#define     PGM_ROM_SPACE const PROGMEM
#else
#define     PGM_ROM_SPACE const
#define     pgm_read_byte *
#define     pgm_read_word *
#define     pgm_read_dword *
#endif

/* Character encoding table, based on "The WSPR Coding Process, G4JNT".
   The 37 allowed characters are allocated values from 0 to 36 such that
   '0' - '9' give 0 - 9, 'A' to 'Z' give 10 to 35 and [space] is given the
   value 36.
*/
static PGM_ROM_SPACE uint8_t wspr_cenc[] = { /* with ASCII ordered */
  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 36,  0,  0,  0,
  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  2,  3,  4,  5,
  6,  7,  8,  9,  0,  0,  0,  0,  0,  0,  0, 10, 11, 12, 13, 14, 15, 16,
  17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34,
  35,  0,  0,  0,  0,  0,  0, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20,
  21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35,  0,  0,  0,
  0,  0,
};

// This is ported from James Peroulas's wspr() with AVR GCC modification.
static void pack_call(const char *call, uint32_t *n1, uint32_t *ng,
                      uint8_t *nadd)
{
  const uint8_t *s = (uint8_t *)strchr(call, '/');
  const uint8_t *c = (uint8_t *)call;
  uint8_t clen = strlen(call);
  uint8_t n, i;
  uint32_t m = 0;

  if (s) {
    *nadd = 2;
    // stroke position
    i = s - c;
    // suffix len, prefix-call len
    n = clen - i - 1;
    // 1-digit suffix /A to /Z, /0 to /9
    if (n == 1) m = 60000 - 32768 + pgm_read_byte(&wspr_cenc[c[i + 1]]);
    // 2-digit suffix /10 to /99
    if (n == 2) m = 60000 + 26 + 10 * pgm_read_byte(&wspr_cenc[c[i + 1]]) +
                      pgm_read_byte(&wspr_cenc[c[i + 2]]);
    // prefix EA8/, right align
    if (n > 2) {
      m = i < 3 ? 36 : pgm_read_byte(&wspr_cenc[c[i - 3]]);
      m = 37 * m + (i < 2 ? 36 : pgm_read_byte(&wspr_cenc[c[i - 2]]));
      m = 37 * m + (i < 1 ? 36 : pgm_read_byte(&wspr_cenc[c[i - 1]]));
      if (m < 32768) {
        *nadd = 1;
      } else {
        m -= 32768;
      }
      c += i + 1;
      clen -= i + 1;
    } else {
      clen -= n + 1;
    }

    // in message type 2, ng contains prefix or suffix
    *ng = m;
  } else {
    *nadd = 0;
  }

  // n1 contains the normal call
  i = isdigit(c[2]) ? 2 : isdigit(c[1]) ? 1 : 0;
  n = clen - i - 1;
  m = i < 2 ? 36 : pgm_read_byte(&wspr_cenc[c[i - 2]]);
  m = 36 * m + (i < 1 ? 36 : pgm_read_byte(&wspr_cenc[c[i - 1]]));
  m = 10 * m + pgm_read_byte(&wspr_cenc[c[i]]);
  m = 27 * m + (n < 1 ? 26 : pgm_read_byte(&wspr_cenc[c[i + 1]]) - 10);
  m = 27 * m + (n < 2 ? 26 : pgm_read_byte(&wspr_cenc[c[i + 2]]) - 10);
  m = 27 * m + (n < 3 ? 26 : pgm_read_byte(&wspr_cenc[c[i + 3]]) - 10);
  *n1 = m;
}

// Pack entire message into WSPR codeblock.
static uint8_t pack_message(const char *call, const char *grid,
                            const char *dBm, uint8_t *packed)
{
  uint32_t n1, ng;
  uint8_t nadd, mtype;
  uint8_t gridlen = strlen(grid);

  if (gridlen != 6) {
    pack_call(call, &n1, &ng, &nadd);

    // in message type 1, ng contains 4-character grid locator
    if (nadd == 0) {
      const uint8_t *g = (uint8_t *)grid;
      ng = 180 * (179 - 10 * (pgm_read_byte(&wspr_cenc[g[0]]) - 10)
                  - pgm_read_byte(&wspr_cenc[g[2]]))
           + 10 * (pgm_read_byte(&wspr_cenc[g[1]]) - 10)
           + pgm_read_byte(&wspr_cenc[g[3]]);
      mtype = 1;
    } else {
      mtype = 2;
    }
  } else {
    char tmp[11];

    // in message type 3, ng contains nhashing of the call (in uppercase)
    uint8_t i, clen = strlen(call);
    for (i = 0; i < clen; i++) {
      tmp[i] = toupper(call[i]);
    }
    ng = nhash(tmp, clen, (uint32_t)146);

    // n1 contains left-rotated 6-character grid locator as a call
    for (i = 0; i < 5; i++) {
      tmp[i] = grid[i + 1];
    }
    tmp[5] = grid[0];
    tmp[6] = 0;
    pack_call(tmp, &n1, &ng, &nadd);

    mtype = 3;
  }

  // EIRP in dBm={0,3,7,10,13,17,20,23,27,30,33,37,40,43,47,50,53,57,60}
  static PGM_ROM_SPACE int8_t corr[] = {0, -1, 1, 0, -1, 2, 1, 0, -1, 1};
  int pwr = atoi(dBm);
  pwr = pwr > 60 ? 60 : pwr < 0 ? 0 : pwr + (int8_t)pgm_read_byte(&corr[pwr % 10]);

  // add power level to ng according to the corresponding message formats
  int8_t ntype = gridlen != 6 ? pwr + nadd : -(pwr + 1);
  ng = 128 * ng + ntype + 64;

  // pack n1, ng, zero-tail into 50 bits
  packed[0] = (uint8_t)(n1 >> 20);
  packed[1] = (uint8_t)(n1 >> 12),
  packed[2] = (uint8_t)(n1 >> 4),
  packed[3] = (uint8_t)(((n1 & 0x0f) << 4) | ((ng >> 18) & 0x0f)),
  packed[4] = (uint8_t)(ng >> 10),
  packed[5] = (uint8_t)(ng >> 2),
  packed[6] = (uint8_t)((ng & 0x03) << 6),
  packed[7] = packed[8] = packed[9] = packed[10] = 0;

  return mtype;
}

// Calculate 32-bit parity of 'v', with the returned value left shifted.
static unsigned char sparity(uint32_t v)
{
  v ^= v >> 16;
  v ^= v >> 8;
  v ^= v >> 4;
  v &= 0xf;

  return ((0x6996u * 2) >> v) & 2;
}

// Encode WSPR codeblock into channel symbols.
static void encode_symbols(const uint8_t *packed, uint8_t *symbols)
{
  /* interleave table */
  static PGM_ROM_SPACE uint8_t interleave[] = {
    0, 128,  64,  32, 160,  96,  16, 144,  80,  48, 112,   8, 136,  72,
    40, 104,  24, 152,  88,  56, 120,   4, 132,  68,  36, 100,  20, 148,
    84,  52, 116,  12, 140,  76,  44, 108,  28, 156,  92,  60, 124,   2,
    130,  66,  34,  98,  18, 146,  82,  50, 114,  10, 138,  74,  42, 106,
    26, 154,  90,  58, 122,   6, 134,  70,  38, 102,  22, 150,  86,  54,
    118,  14, 142,  78,  46, 110,  30, 158,  94,  62, 126,   1, 129,  65,
    33, 161,  97,  17, 145,  81,  49, 113,   9, 137,  73,  41, 105,  25,
    153,  89,  57, 121,   5, 133,  69,  37, 101,  21, 149,  85,  53, 117,
    13, 141,  77,  45, 109,  29, 157,  93,  61, 125,   3, 131,  67,  35,
    99,  19, 147,  83,  51, 115,  11, 139,  75,  43, 107,  27, 155,  91,
    59, 123,   7, 135,  71,  39, 103,  23, 151,  87,  55, 119,  15, 143,
    79,  47, 111,  31, 159,  95,  63, 127,
  };

  // channel syncs
  static PGM_ROM_SPACE uint8_t sync[] = {
    1, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 1, 1, 0, 0, 0, 1, 0, 0, 1, 0, 1,
    1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0,
    1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 0, 1, 1, 0, 1, 0, 0, 0, 0, 1, 1, 0, 1, 0,
    1, 0, 1, 0, 1, 0, 0, 1, 0, 0, 1, 0, 1, 1, 0, 0, 0, 1, 1, 0, 1, 0, 1, 0,
    0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 1, 1, 1, 0, 1, 1, 0, 0, 1, 1,
    0, 1, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 1, 1, 0, 0, 0, 0,
    0, 0, 0, 1, 1, 0, 1, 0, 1, 1, 0, 0, 0, 1, 1, 0, 0, 0,
  };

  /* The data is will be expanded to add FEC with a rate ½, constraint length
     32, convolutional encoder. (G4JNT)
  */
  uint8_t i, j, k, conv_ptr = 0;
  uint32_t reg = 0;

  for (i = 0; i < 11; i++) {
    for (j = 7; j < 255; j--) {
      if (packed[i] & (1 << j)) reg |= 1;

      k = pgm_read_byte(&interleave[conv_ptr++]);
      symbols[k] = sparity(reg & 0xF2D05351) | pgm_read_byte(&sync[k]);

      k = pgm_read_byte(&interleave[conv_ptr++]);
      symbols[k] = sparity(reg & 0xE4613C47) | pgm_read_byte(&sync[k]);

      if (conv_ptr == 162) {
        break;
      } else {
        reg <<= 1;
      }
    }
  }
}

// WSPR encoder.
uint8_t wspr_enc(const char *call, const char *grid, const char *dBm,
                 uint8_t *symbols)
{
  uint8_t packed[11];

  uint8_t msgtype = pack_message(call, grid, dBm, packed);

#if !defined(__AVR_ARCH__) && (WSPR_SOURCE_ENCODED_MESSAGE_DEBUG == 1)
  printf("Source-encoded message (50 bits, hex): ");
  int i;
  for (i = 0; i < 7; i++) {
    printf("%02X ", packed[i]);
  }
  printf("\n");
#endif

  encode_symbols(packed, symbols);

  return msgtype;
}
