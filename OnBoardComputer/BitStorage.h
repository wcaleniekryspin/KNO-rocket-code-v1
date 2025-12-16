#ifndef BIT_STORAGE_H
#define BIT_STORAGE_H

#include <Arduino.h>
#include "config.h"

class BitStorage
{
private:
  uint8_t message[ARRAY_SIZE];

  void setBit(uint16_t bitIndex, bool val);

public:
  BitStorage();

  void add(uint32_t value, uint16_t pos, int8_t bitSize, bool isSigned = false);
  void addCheckSum();
  uint8_t* data();
  void clean();
};

#endif  // BIT_STORAGE_H
