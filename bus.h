#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <string>
#include <vector>

class bus {
public:
  bus() = default;
  ~bus() = default;

  // TODO: real STM32F103 memory map. Flash lives at 0x08000000 (512 KiB) and
  // SRAM at 0x20000000 (64 KiB). Until the decoder handles more than a few
  // instructions, a single flat window is enough for the CPU to fetch from.
  std::array<uint8_t, 512 * 1024> flash;
  std::array<uint8_t, 64 * 1024> sram;

  uint8_t read8(uint32_t address) const;
  uint8_t read8_flash(uint32_t address) const;

  uint16_t read16(uint32_t address) const;
  uint16_t read16_flash(uint32_t address) const;

  uint32_t read32(uint32_t address) const;

  void write8(uint32_t address, uint8_t value);
  void write8_flash(uint32_t address, uint8_t value);

  void write32(uint32_t address, uint32_t value);

  std::vector<std::string> m_out;

  // Copy a program image into memory. Out-of-range bytes are dropped.
  void load_flash(uint32_t address, const uint8_t *src, std::size_t count);
  void load_program(const std::string &path);
  void append_output(const std::string &s);

  const int kOutCols = 80;  // chars per line (8 px each)
  const int kOutLines = 15; // visible lines (10 px each)
};
