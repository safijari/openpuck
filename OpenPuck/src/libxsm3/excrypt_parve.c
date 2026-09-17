/* OpenPuck modifications, 2026-09-17: byte-buffer chain-and-sum accesses
   and 32-bit result stores for alignment/endian safety. See NOTICE.md. */

#include <stdlib.h>
#include <string.h>

#include "excrypt.h"

void ExCryptParveEcb(const uint8_t* key, const uint8_t* sbox, const uint8_t* input, uint8_t* output)
{
  uint8_t block[9];

  memcpy(block, input, 8);
  block[8] = block[0];

  for (int i = 8; i > 0; i--)
  {
    for (int j = 0; j < 8; j++)
    {
      uint8_t x = key[j] + block[j] + i;
      uint8_t y = sbox[x] + block[j + 1];
      block[j + 1] = ROTL8(y, 1);
    }

    block[0] = block[8];
  }

  memcpy(output, block, 8);
}

void ExCryptParveCbcMac(const uint8_t* key, const uint8_t* sbox, const uint8_t* iv, const uint8_t* input, uint32_t input_size, uint8_t* output)
{
  uint64_t block;
  uint64_t temp;
  memcpy(&block, iv, 8);

  if (input_size >= 8)
  {
    for (uint32_t i = 0; i < input_size / 8; i++)
    {
      memcpy(&temp, input + (i * 8), sizeof(temp));
      block ^= temp;
      ExCryptParveEcb(key, sbox, (uint8_t*)&block, (uint8_t*)&block);
    }
  }

  memcpy(output, &block, 8);
}

static uint32_t load_be32(const uint8_t* input)
{
  uint32_t value;
  memcpy(&value, input, sizeof(value));
  return SWAP32(value);
}

void ExCryptChainAndSumMac(const uint8_t* cd, const uint8_t* ab, const uint8_t* input, uint32_t input_dwords, uint8_t* output)
{
  uint64_t out0 = 0;
  uint64_t out1 = 0;

  uint32_t ab0 = load_be32(ab) % 0x7FFFFFFF;
  uint32_t ab1 = load_be32(ab + 4) % 0x7FFFFFFF;
  uint32_t cd0 = load_be32(cd) % 0x7FFFFFFF;
  uint32_t cd1 = load_be32(cd + 4) % 0x7FFFFFFF;

  for (uint32_t i = 0; i < input_dwords / 2; i++)
  {
    out0 += (uint64_t)load_be32(input) * 0xE79A9C1;
    out0 = (out0 % 0x7FFFFFFF) * ab0;
    out0 += ab1;
    out0 = out0 % 0x7FFFFFFF;

    out1 += out0;

    out0 = (uint64_t)(load_be32(input + 4) + out0) * cd0;
    out0 = (out0 % 0x7FFFFFFF) + cd1;
    out0 = out0 % 0x7FFFFFFF;

    out1 += out0;

    input += 8;
  }
  uint32_t result0 = SWAP32((uint32_t)((out0 + ab1) % 0x7FFFFFFF));
  uint32_t result1 = SWAP32((uint32_t)((out1 + cd1) % 0x7FFFFFFF));
  memcpy(output, &result0, sizeof(result0));
  memcpy(output + 4, &result1, sizeof(result1));
}
