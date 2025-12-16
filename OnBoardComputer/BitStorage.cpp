#include "BitStorage.h"

BitStorage::BitStorage()
{
  clean();
}

void BitStorage::setBit(uint16_t bitIndex, bool val)
{
  if (bitIndex >= ARRAY_SIZE * 8) return;
  uint8_t byteIdx = bitIndex / 8;
  uint8_t currentBit = 7 - (bitIndex % 8);
  if (val) message[byteIdx] |= (1u << currentBit);
  else message[byteIdx] &= ~(1u << currentBit);
}

void BitStorage::add(uint32_t value, uint16_t pos, int8_t bitSize, bool isSigned)
{
  if (pos + bitSize > ARRAY_SIZE * 8)
  {
    debugln(F("Przekroczono zakres pamięci!"));
    return;
  }

  if (bitSize < 0 || bitSize > ARRAY_SIZE * 8)
  {
    debugln(F("Zla pozycja bita!"));
    return;
  }

  if (isSigned)
  {
    int32_t signedValue = (int32_t)value;
    bool signBit = (signedValue < 0);
    setBit(pos, signBit);
    pos++;
    bitSize--;
    value = abs(signedValue);
  }
  
  for (int8_t i=bitSize-1; i>=0; i--)
  {
    setBit(pos, ((value >> i) & 0x01));
    pos++;
  }
}

void BitStorage::addCheckSum()
{
  uint8_t sum = 0;
  for (uint8_t i = 0; i < ARRAY_SIZE - 1; i++)
    sum ^= message[i];
  message[ARRAY_SIZE - 1] = sum;
  debug(F("CheckSum: "));
  debugln(sum);
}

uint8_t* BitStorage::data()
{
  return message;
}

void BitStorage::clean()
{
  memset(message, 0, ARRAY_SIZE);
  message[0] = (uint8_t)((HEADER >> 8) & 0xFF);
  message[1] = (uint8_t)(HEADER & 0xFF);
}