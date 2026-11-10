#include "bus.h"

#include <stdexcept>

using std::vector;

// One decoder, selected by address. `address - base < size` is used everywhere
// because it is correct for any address (unsigned wrap turns an address below
// the region into a huge value, so the compare fails) and needs no overflow
// checks.
uint8_t bus::read8(uint32_t address) const {
  if (address - kFlashBase < kFlashSize) // 0x08000000.. flash
    return flash[address - kFlashBase];
  if (address < kFlashSize) // 0x00000000.. boot alias of flash
    return flash[address];
  if (address - kSramBase < kSramSize) // 0x20000000.. SRAM
    return sram[address - kSramBase];
  return 0; // TODO: BusFault
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
  // Flash writes need the unlock + erase sequence, so they are ignored here.
  if (address - kFlashBase < kFlashSize)
    return;
  if (address - kSramBase < kSramSize) {
    sram[address - kSramBase] = value;
    return;
  }
  // TODO: BusFault
}

void bus::write16(uint32_t address, uint16_t value) {
  write8(address, static_cast<uint8_t>(value & 0xFF));
  write8(address + 1, static_cast<uint8_t>((value >> 8) & 0xFF));
}

void bus::write32(uint32_t address, uint32_t value) {
  write8(address, value & 0xFF);
  write8(address + 1, (value >> 8) & 0xFF);
  write8(address + 2, (value >> 16) & 0xFF);
  write8(address + 3, (value >> 24) & 0xFF);
}

void bus::load_flash(uint32_t address, const uint8_t *src, std::size_t count) {
  for (std::size_t i = 0; i < count; ++i) {
    const uint32_t a = address + static_cast<uint32_t>(i);
    if (a - kFlashBase < kFlashSize)
      flash[a - kFlashBase] = src[i];
  }
}

void bus::load_program(const std::string &path) {
  std::ifstream file(path, std::ios::binary);

  if (!file) {
    throw std::runtime_error("Failed to open program");
  }

  vector<uint8_t> buffer((std::istreambuf_iterator<char>(file)),
                         std::istreambuf_iterator<char>());

  load_flash(kFlashBase, buffer.data(), buffer.size());
}

// Call this from the CPU whenever the program prints something.
void bus::append_output(const std::string &s) {
  if (m_out.empty())
    m_out.emplace_back();

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
