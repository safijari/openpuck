/* OpenPuck modifications, 2026-09-17: remove duplicate state declaration
   that prevents compilation. See NOTICE.md. */

#include <stdlib.h>
#include <string.h>

#include "excrypt.h"

// SHA1 code based on https://github.com/mohaps/TinySHA1

const uint32_t intermediate[5] = {0x67452301, 0xEFCDAB89, 0x98BADCFE, 0x10325476, 0xC3D2E1F0};
void ExCryptSha(const uint8_t *input, uint32_t input_size, uint8_t *output)
{
  uint32_t w[80] = {0};
  memcpy(w, input, input_size);

  uint8_t *buffer = (uint8_t *)w;
  buffer[input_size] = 0x80;

  for (size_t i = 0; i < 15; i++)
  {
    w[i] = SWAP32(w[i]);
  }

  w[15] = input_size * 8;
  uint32_t a = intermediate[0];
  uint32_t b = intermediate[1];
  uint32_t c = intermediate[2];
  uint32_t d = intermediate[3];
  uint32_t e = intermediate[4];
  uint32_t temp, f, k;

  for (int i = 0; i < 80; ++i)
  {
    if (i >= 16) {
      w[i] = ROTL32((w[i - 3] ^ w[i - 8] ^ w[i - 14] ^ w[i - 16]), 1);
    }

    if (i < 20)
    {
      f = (b & c) | (~b & d);
      k = 0x5A827999;
    }
    else if (i < 40)
    {
      f = b ^ c ^ d;
      k = 0x6ED9EBA1;
    }
    else if (i < 60)
    {
      f = (b & c) | (b & d) | (c & d);
      k = 0x8F1BBCDC;
    }
    else
    {
      f = b ^ c ^ d;
      k = 0xCA62C1D6;
    }
    temp = ROTL32(a, 5) + f + e + k + w[i];
    e = d;
    d = c;
    c = ROTL32(b, 30);
    b = a;
    a = temp;
  }

  a += intermediate[0];
  b += intermediate[1];
  c += intermediate[2];
  d += intermediate[3];
  e += intermediate[4];
  uint32_t state[5] = {SWAP32(a), SWAP32(b), SWAP32(c), SWAP32(d), SWAP32(e)};

  memcpy(output, state, sizeof(state));
}
