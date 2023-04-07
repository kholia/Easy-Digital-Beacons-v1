/*
   This program tests a prototype of the WSPR encoder "wspr_enc" on PC. Check
   wspr_enc.h and wspr_enc.c for more details.

   The WSPR coding simulation tool "WSPRcode" is required to verify the symbols
   produced by the encoder. It can be built from the WSJT-X source:
        https://sourceforge.net/p/wsjt/wsjtx/ci/master/tree/lib/wsprcode/

   References:
   K1JT: WSPR 2.0 User’s Guide
        https://www.physics.princeton.edu/pulsar/K1JT/WSPR_2.0_User.pdf

   Author: BD1ES

   21 FEB 2020:
       Released to the Gist:
           https://gist.github.com/bd1es/a782e2529b8289288fadd35e407f6440
   31 MAR 2020:
       Removed parity table generation function.
   01 APR 2020:
       Added code auto-comparation using "WSPRcode" built from the source.
       Manual check is no longer available.
*/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include <getopt.h>

#include "wspr_enc.h"

// The following 2 functions generate lookup tables for wspr_enc.c
static void gen_char_enc_table()
{
  puts("/* Character encoding table, based on \"The WSPR Coding Process, G4JNT\".\n"
       " * The 37 allowed characters are allocated values from 0 to 36 such that\n"
       " * '0' - '9' give 0 - 9, 'A' to 'Z' give 10 to 35 and [space] is given the\n"
       " * value 36.\n"
       " */"
      );
  printf("static PGM_ROM_SPACE uint8_t wspr_cenc[] = { /* with ASCII ordered */");

  int i;
  for (i = 0; i < 128; i++) {
    int k;
    if (i == ' ') {
      k = 36;
    } else if (i >= '0' && i <= '9') {
      k = i - '0';
    } else if (i >= 'A' && i <= 'Z') {
      k = i - 'A' + 10;
    } else if (i >= 'a' && i <= 'z') {
      k = i - 'a' + 10;
    } else {
      k = 0;
    }

    if ((i % 18) == 0) {
      printf("\n    ");
    }
    printf("%2d, ", k);
  }
  puts("\n};");
}

static void gen_interleave_table()
{
  puts("/* interleave table. */");
  printf("static PGM_ROM_SPACE uint8_t interleave[] = {");

  int i, j, k;
  for (i = 0, k = 0; i < 256; i++) {
    uint8_t reversed_byte = 0;

    for (j = 0; j < 8; j++) {
      uint8_t temp = (i & (1 << j));
      if (temp) {
        reversed_byte |= (1 << ((8 - 1) - j));
      }
    }

    if (reversed_byte < 162) {
      if ((k++) % 14 == 0) {
        printf("\n    ");
      }
      printf("%3d, ", reversed_byte);

      if (k == 162) {
        break;
      }
    }
  }
  puts("\n};");
}

// ----------------------------------------------------------------------------

static void str_to_upper(char *dist, const char *str)
{
  while (*str) *dist++ = toupper(*str++);
  *dist = 0;
}

static void print_help(const char *pname)
{
  printf("WSPR encoder function \"wspr_enc\" test tool.\n");
  printf("Usage: %s [options] <callsign> <grid> <dBm>\n", pname);
  printf("Examples: %s BD1XYZ OM89 10         (message type 1)\n", pname);
  printf("          %s BD1XYZ/M xxxx 20       (message type 2)\n", pname);
  printf("          %s BD1XYZ OM89dw 30       (message type 3)\n", pname);
  printf("Options:\n");
  printf("    -h --help\n");
  printf("       Display this help screen.\n");
  printf("    --gen_tables\n");
  printf("       Generate constant lookup tables.\n");
  printf("Parameters:\n");
  printf("    callsign: BD1XYZ BD1XYZ/9 EA8/BD1XYZ ...\n");
  printf("        where, compound calls produce type 2 messages.\n");
  printf("    grid: 4- or 6-character grid locator\n");
  printf("        where, 6-character locators produce type 3 messages.\n");
  printf("    dBm: TX power in dBm, ranging 0 - 60.\n");
  printf("\n");
  printf("This program uses WSPRcode to generate the references. ");
  printf("Make sure it's installed in the system and can be invoked.\n");
}

struct strings_t {
  int size;
  char line[256][256];
};

/*
  static int run_pipe(struct strings_t *strs, const char *command)
  {
    FILE *fpipe;

    if(!(fpipe = (FILE*)popen(command, "r"))){
        perror("Operating the pipe: ");
        return -1;
    }

    strs->size = 0;
    while(fgets(strs->line[strs->size], 1024, fpipe)){
        strs->size++;
        if(strs->size == 256) break;
    }

    pclose(fpipe);
    return 0;
  } */

static int check_symbols(uint8_t *symbols, struct strings_t *strs)
{
  int i, symbol_pos = 0;
  for (i = 0; i < strs->size; i++) {
    if (strncmp(strs->line[i], "Channel symbols:", 16) == 0) {
      symbol_pos = i + 1;
      break;
    }
  }
  if (symbol_pos == 0) {
    printf("No channel symbols were found in messages produced by \"WSPRcode\".\n");
    return -1;
  }

  char line[1024];
  for (i = 0; i < 6; i++) {
    int j;
    char *line_p = line;
    for (j = 0; (j < 30) && ((i * 30 + j) < 162); j++) {
      sprintf(line_p, "%d ", symbols[i * 30 + j]);
      line_p += 2;
    }
    if (strncmp(strs->line[symbol_pos + i] + 6, line, line_p - line - 1) != 0) {
      printf("In line %d, one or more symbols are inconsistent:\n"
             "      %s\n%s", i + 1, line, strs->line[symbol_pos + i]);
      return -1;
    }
  }

  printf("%s", strs->line[strs->size - 1]);
  printf("Symbols check ok.");

  return 0;
}

/* run this program using the console pauser or add your own getch, system("pause") or input loop */

int original_main(int argc, char *argv[])
{
  static const char *PROG_NAME = "wspr_enc_test";
  char *call = NULL, *grid = NULL, *dBm = NULL;
  char call_u[11], grid_u[7];

  static const struct option long_options[] = {
    {"help",       no_argument, 0, 'h'},
    {"gen_tables", no_argument, 0, 1},
    {0, 0, 0, 0}
  };

  // get options
  while (1) {
    int option_index;
    int c = getopt_long(argc, argv, "h", long_options, &option_index);
    if (c == -1) {
      break;
    }
    switch (c) {
      case 'h':
        print_help(PROG_NAME);
        return 0;
      case 1:
        gen_char_enc_table();
        gen_interleave_table();
        return 0;
      default:
        return -1;
    }
  }

  // get parameters
  size_t nargs = 0;
  char *endp;
  while (optind < argc) {
    switch (nargs) {
      case 0:
        call = argv[optind++];
        if ((strlen(call) < 2) || (strlen(call) > 10)) {
          printf("The length of callsign %s is invalid.\n", call);
          return -1;
        }
        str_to_upper(call_u, call);
        // for Swaziland 3DA0YZ, 3D0YZ will be used for proper decoding
        if (strncmp(call_u, "3DA0", 4) == 0) {
          size_t i;
          for (i = 3; i < strlen(call); i++) {
            call[i - 1] = call[i];
          }
          call[i - 1] = 0;
        }
        str_to_upper(call_u, call);
        break;
      case 1:
        grid = argv[optind++];
        if ((strlen(grid) != 4) && (strlen(grid) != 6)) {
          printf("The length of grid locator %s is invalid.\n", grid);
          return -1;
        }
        str_to_upper(grid_u, grid);
        break;
      case 2:
        dBm = argv[optind++];
        strtol(dBm, &endp, 10);
        if ((dBm == endp) || (*endp != '\0')) {
          printf("The dBm value %s is invalid.\n", dBm);
          return -1;
        }
        break;
      default:
        puts("Too many parameters.\n");
        print_help(PROG_NAME);
        return -1;
    }
    nargs++;
  }

  if (nargs < 3) {
    puts("Too few parameters.\n");
    print_help(PROG_NAME);
    return -1;
  }

  printf("Message: %s %s %s\n\n", call, grid, dBm);

  // invoke encoder function to generate symbols
  uint8_t symbols[162];
  uint8_t msgtype = wspr_enc(call, grid, dBm, symbols);

  printf("Channel symbols: (message type %d)", msgtype);
  int i;
  for (i = 0; i < 162; i++) {
    if (i % 30 == 0) {
      printf("\n      %d", symbols[i]);
    } else {
      printf(" %d", symbols[i]);
    }
  }
  printf("\n");

  // prepare to invoke "WSPRcode"
  // WHERE, "WSPRcode" must be a later version, built from the source !!!
  char cmd[256], msg[256];
  if (msgtype == 1) {
    sprintf(cmd, "WSPRcode \"%s %s %s\"", call_u, grid_u, dBm);
    sprintf(msg, "Message: %s %s %s", call_u, grid_u, dBm);
  } else if (msgtype == 2) {
    sprintf(cmd, "WSPRcode \"%s %s\"", call_u, dBm);
    sprintf(msg, "Message: %s %s", call_u, dBm);
  } else if (msgtype == 3) {
    sprintf(cmd, "WSPRcode \"<%s> %s %s\"", call_u, grid_u, dBm);
    sprintf(msg, "Message: <%s> %s %s", call_u, grid_u, dBm);
  } else {
    printf("Unknown message type %d detected, check the code.\n", msgtype);
    return -1;
  }
  printf("\nRunning %s\n", cmd);

  // run the program in a pipe
  /* struct strings_t strs;
    run_pipe(&strs, cmd);
    if((strs.size!=31) || (strncmp(strs.line[0], msg, strlen(msg))!=0)){
      printf("Incorrect \"WSPRcode\" executed. Check installation!\n");
      return -1;
    }

    // return the results
    return check_symbols(symbols, &strs); */

  return 0;
}

// uint8_t symbols[162];

// Modified "main"
int real_main(char *call, char *grid, char *dBm, uint8_t *symbols)
{
  static const char *PROG_NAME = "wspr_enc_test";
  char call_u[11], grid_u[7];

  // Call
  if ((strlen(call) < 2) || (strlen(call) > 10)) {
    printf("The length of callsign %s is invalid.\n", call);
    return -1;
  }
  str_to_upper(call_u, call);
  // for Swaziland 3DA0YZ, 3D0YZ will be used for proper decoding
  if (strncmp(call_u, "3DA0", 4) == 0) {
    size_t i;
    for (i = 3; i < strlen(call); i++) {
      call[i - 1] = call[i];
    }
    call[i - 1] = 0;
  }
  str_to_upper(call_u, call);

  // Grid
  if ((strlen(grid) != 4) && (strlen(grid) != 6)) {
    printf("The length of grid locator %s is invalid.\n", grid);
    return -1;
  }
  str_to_upper(grid_u, grid);

  printf("WSPR Message: %s %s %s\n", call, grid, dBm);

  // invoke encoder function to generate symbols
  uint8_t msgtype = wspr_enc(call, grid, dBm, symbols);

  /* printf("Channel symbols: (message type %d)", msgtype);
    int i;
    for (i = 0; i < 162; i++) {
      if (i%30 == 0) {
          printf("\n      %d", symbols[i]);
      } else {
          printf(" %d", symbols[i]);
      }
    }
    printf("\n"); */

  return 0;
}
