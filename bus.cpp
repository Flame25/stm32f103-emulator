#include "bus.h"
#include <algorithm>
#include <cstdint>

uint8_t bus::read8(uint32_t address) const {
  // TODO: no bounds check yet. Out-of-range reads silently return 0.
  if (address >= kMemorySize)
    return 0;
  return memory[address];
}

uint16_t bus::read16(uint32_t address) const {
  return static_cast<uint16_t>(read8(address) | (read8(address + 1) << 8));
}

uint32_t bus::read32(uint32_t address) const {
  return static_cast<uint32_t>(read8(address)) |
         (static_cast<uint32_t>(read8(address + 1)) << 8) |
         (static_cast<uint32_t>(read8(address + 2)) << 16) |
         (static_cast<uint32_t>(read8(address + 3)) << 24);
}

void bus::write8(uint32_t address, uint8_t value) {
  if (address >= kMemorySize)
    return;
  memory[address] = value;
}

void bus::write32(uint32_t address, uint32_t value) {
  write8(address, value & 0xFF);
  write8(address + 1, (value >> 8) & 0xFF);
  write8(address + 2, (value >> 16) & 0xFF);
  write8(address + 3, (value >> 24) & 0xFF);
}

void bus::load(uint32_t address, const uint8_t *src, std::size_t count) {
  for (std::size_t i = 0; i < count; ++i)
    write8(address + static_cast<uint32_t>(i), src[i]);
}
