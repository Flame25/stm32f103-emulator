#include "bus.h"

uint8_t bus::read8(uint32_t address) const {
  // TODO: no bounds check yet. Out-of-range reads silently return 0.
  if (address >= sizeof(sram))
    return 0;
  return sram[address];
}

uint8_t bus::read8_flash(uint32_t address) const {
  if (address >= sizeof(sram))
    return 0;
  return flash[address];
}

uint16_t bus::read16(uint32_t address) const {
  return static_cast<uint16_t>(read8(address) | (read8(address + 1) << 8));
}

uint16_t bus::read16_flash(uint32_t address) const {
  return static_cast<uint16_t>(read8_flash(address) |
                               (read8_flash(address + 1) << 8));
}

uint32_t bus::read32(uint32_t address) const {
  return static_cast<uint32_t>(read8(address)) |
         (static_cast<uint32_t>(read8(address + 1)) << 8) |
         (static_cast<uint32_t>(read8(address + 2)) << 16) |
         (static_cast<uint32_t>(read8(address + 3)) << 24);
}

void bus::write8(uint32_t address, uint8_t value) {
  if (address >= sizeof(sram))
    return;
  sram[address] = value;
}

void bus::write8_flash(uint32_t address, uint8_t value) {
  if (address >= sizeof(flash))
    return;
  flash[address] = value;
}

void bus::write32(uint32_t address, uint32_t value) {
  write8(address, value & 0xFF);
  write8(address + 1, (value >> 8) & 0xFF);
  write8(address + 2, (value >> 16) & 0xFF);
  write8(address + 3, (value >> 24) & 0xFF);
}

void bus::load_flash(uint32_t address, const uint8_t *src, std::size_t count) {
  for (std::size_t i = 0; i < count; ++i)
    write8_flash(address + static_cast<uint32_t>(i), src[i]);
}

// Call this from the CPU whenever the program prints something.
void bus::append_output(const std::string &s) {
  for (char c : s) {
    if (c == '\r')
      continue;
    if (c == '\n') {
      m_out.emplace_back();
      continue;
    }
    if (static_cast<int>(m_out.back().size()) >= kOutCols)
      m_out.emplace_back();
    m_out.back() += c;
  }
  while (m_out.size() > 200) // keep memory bounded
    m_out.erase(m_out.begin());
}
