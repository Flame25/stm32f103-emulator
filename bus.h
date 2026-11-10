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

  // STM32F103C8: 64 KiB flash, 20 KiB SRAM.
  static constexpr uint32_t kFlashBase = 0x08000000;
  static constexpr uint32_t kFlashSize = 64 * 1024;
  static constexpr uint32_t kSramBase = 0x20000000;
  static constexpr uint32_t kSramSize = 20 * 1024;

  std::array<uint8_t, kFlashSize> flash{};
  std::array<uint8_t, kSramSize> sram{};

  uint8_t read8(uint32_t address) const;
  uint16_t read16(uint32_t address) const;
  uint32_t read32(uint32_t address) const;

  void write8(uint32_t address, uint8_t value);
  void write16(uint32_t address, uint16_t value);
  void write32(uint32_t address, uint32_t value);

  std::vector<std::string> m_out;

  void load_flash(uint32_t address, const uint8_t *src, std::size_t count);
  void load_program(const std::string &path);
  void append_output(const std::string &s);

  const int kOutCols = 80;  // chars per line (8 px each)
  const int kOutLines = 15; // visible lines (10 px each)
};
