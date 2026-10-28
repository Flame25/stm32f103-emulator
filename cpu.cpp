#include "cpu.h"
#include <cstdint>
#include <cstdio>
#include <string>

cpu::cpu(bus &b) : m_bus(&b) {}

void cpu::reset() {
  for (auto &reg : r)
    reg = 0;
  pc = 0;
  xpsr = 0;
  it_state = 0;
  cycles = 0;
  clock_count = 0;
  ins = decoded{};
}

uint16_t cpu::read(uint32_t address) const { return m_bus->read16(address); }

uint32_t cpu::read32(uint32_t address) const { return m_bus->read32(address); }

bool cpu::GetFlag(apsr_flag f) const {
  return (xpsr & static_cast<uint32_t>(f)) != 0;
}

void cpu::SetFlag(apsr_flag f, bool v) {
  if (v)
    xpsr |= static_cast<uint32_t>(f);
  else
    xpsr &= ~static_cast<uint32_t>(f);
}

bool cpu::flag(char name) const {
  switch (name) {
  case 'N':
    return GetFlag(apsr_flag::N);
  case 'Z':
    return GetFlag(apsr_flag::Z);
  case 'C':
    return GetFlag(apsr_flag::C);
  case 'V':
    return GetFlag(apsr_flag::V);
  case 'Q':
    return GetFlag(apsr_flag::Q);
  default:
    return false;
  }
}

// Fetch one halfword at pc and advance pc. pc is the source of truth for now;
// r[15] is not kept in sync yet.
uint16_t cpu::fetch16() {
  uint16_t hw = read(pc);
  return hw;
}

void cpu::clock() {

  if (cycles == 0) {

    uint16_t hw1 = read(pc);

    bool adv_bit = false;

    // bits[15:11] == 11101 / 11110 / 11111 -> 32-bit Thumb instruction
    if ((hw1 >> 11) >= 0x1D) {
      ins = decode_thumb32(hw1, fetch16());
      adv_bit = true;
    } else {
      ins = decode_thumb16(hw1);
    }

    execute();

    if (adv_bit) {
      pc += 4;
    } else {
      pc += 2;
    }
  }

  clock_count++;
  --cycles;
}

cpu::decoded cpu::decode_thumb16(uint16_t hw) {
  decoded d;

  // 000xx: A5.2.1 Shift (immediate), add, subtract, move, and compare
  if ((hw >> 13) == 0b000) {
    return d; // TODO
  }

  switch (hw >> 10) {
  case 0x10:  // A5.2.2 Data processing
    return d; // TODO

  case 0x11:  // A5.2.3 Special data instructions and branch and exchange
    return d; // TODO

  // A7.7.5 LDR (literal): 01001 Rt imm8
  case 0x12:
  case 0x13:
    d.op = Instruction::LDR_LITERAL;
    d.rt = (hw >> 8) & 0x7;
    d.imm = static_cast<uint32_t>(hw & 0xFF) << 2;
    cycles = 2;
    return d;

  // A5.3.1 Load/store, register offset
  case 0x14:
  case 0x15:
  case 0x16:
  case 0x17:
    return d; // TODO

  // A5.3.2 Load/store, immediate
  case 0x18: // STR (immediate, word): 01100 imm5 Rn Rt
  case 0x19:
    return d; // TODO

  case 0x1A: // LDR (immediate, word): 01101 imm5 Rn Rt
  case 0x1B:
    d.op = Instruction::LDR_IMM;
    d.rt = hw & 0x7;
    d.rn = (hw >> 3) & 0x7;
    d.imm = static_cast<uint32_t>((hw >> 6) & 0x1F) << 2;
    cycles = 2;
    return d;

  case 0x1C: // STRB (immediate)
  case 0x1D:
  case 0x1E: // LDRB (immediate)
  case 0x1F:
    return d; // TODO

  case 0x20: // A5.3.4 Load/store halfword immediate
  case 0x21:
  case 0x22:
  case 0x23:
  case 0x24: // A5.3.5 Load/store SP-relative
  case 0x25:
  case 0x26:
  case 0x27:
    return d; // TODO

  case 0x28: // A7.7.1 ADR
  case 0x29:
  case 0x2A: // ADD (SP plus immediate)
  case 0x2B:
  case 0x2C: // A5.4 Miscellaneous 16-bit instructions
  case 0x2D:
  case 0x2E:
  case 0x2F:
  case 0x30: // A7.7.6 STM
  case 0x31:
  case 0x32: // A7.7.7 LDM
  case 0x33:
  case 0x34: // A7.7.8 B (conditional), A7.7.9 SVC, A7.7.10 UDF
  case 0x35:
  case 0x36:
  case 0x37:
  case 0x38: // A7.7.11 B (T2, unconditional)
  case 0x39:
  default:
    // Unknown, or 32-bit prefix already filtered by clock().
    cycles = 0;
    return d;
  }
}

cpu::decoded cpu::decode_thumb32(uint16_t hw1, uint16_t hw2) {
  // TODO: B5.5 32-bit Thumb instruction encoding.
  (void)hw1;
  (void)hw2;
  return decoded{};
}

void cpu::execute() {
  switch (ins.op) {
  case Instruction::LDR_LITERAL:
    LDR_LITERAL();
    break;
  case Instruction::LDR_IMM:
    LDR_IMM();
    break;
  case Instruction::UNKNOWN:
    break;
  }
}

// A7.7.5 LDR (literal), T1: Rt = MemU[Align(PC,4) + imm8*4]
uint8_t cpu::LDR_LITERAL() {
  // Architectural PC = instruction address + 4
  //
  // Align the address down to a 4-byte boundary by clearing the lowest 2
  // bits. ~0x3u = 0xFFFFFFFC (32-bit unsigned), so the bitwise AND clears
  // bits [1:0].
  //
  // Example: PC = 0x00000002
  //   PC + 4              = 0x00000006  (architectural PC for Thumb
  //   instructions)
  //   0x00000006  & ~0x3u = 0x00000004  (align down to a multiple of 4)
  uint32_t base = (pc + 4) & ~0x3u;
  uint32_t address = base + ins.imm;

  // R[15] is basically PC
  if (ins.rt == 15) {
    if ((base & 0x3) == 0) {
      pc = address;
    } else {
      // TODO: UNPREDICTABLE
      return 0;
    }

  } else {
    r[ins.rt] = read32(base + ins.imm);
  }
  // TODO: Checker bit 0 for fault
  return 0;
}

// A7.7.3 LDR (immediate), T1: Rt = MemU[Rn + imm5*4]
uint8_t cpu::LDR_IMM() {
  r[ins.rt] = read32(r[ins.rn] + ins.imm);
  // TODO: Rt == 15 is a branch (BXWritePC), not a register write.
  return 0;
}

namespace {

std::string hex(uint32_t value, int digits) {
  std::string s(static_cast<std::size_t>(digits), '0');
  for (int i = digits - 1; i >= 0; --i, value >>= 4)
    s[static_cast<std::size_t>(i)] = "0123456789ABCDEF"[value & 0xF];
  return s;
}

// Mnemonic plus operand text for a decoded Thumb-16 instruction. `addr` is the
// instruction address, used for PC-relative effective address display.
std::string format_thumb16(uint16_t hw, uint32_t addr) {
  // LDR (literal): 01001 Rt imm8
  if ((hw >> 11) == 0b01001) {
    uint32_t rt = (hw >> 8) & 0x7;
    uint32_t offset = static_cast<uint32_t>(hw & 0xFF) << 2;
    uint32_t target = ((addr + 4) & ~0x3u) + offset;
    return "LDR r" + std::to_string(rt) + ", [pc, #" + std::to_string(offset) +
           "] ; $" + hex(target, 8);
  }

  // LDR (immediate, word): 01101 imm5 Rn Rt
  if ((hw >> 11) == 0b01101) {
    uint32_t rt = hw & 0x7;
    uint32_t rn = (hw >> 3) & 0x7;
    uint32_t offset = static_cast<uint32_t>((hw >> 6) & 0x1F) << 2;
    return "LDR r" + std::to_string(rt) + ", [r" + std::to_string(rn) + ", #" +
           std::to_string(offset) + "]";
  }

  return "TODO " + hex(hw, 4);
}

} // namespace

std::string cpu::disassemble(uint32_t address, uint8_t &length) const {
  std::string s = "$" + hex(address, 8) + ": ";

  uint16_t hw1 = read(address);

  // bits[15:11] == 11101 / 11110 / 11111 -> 32-bit Thumb instruction
  if ((hw1 >> 11) >= 0x1D) {
    uint16_t hw2 = read(address + 2);
    length = 4;
    return s + "TODO " + hex(hw1, 4) + " " + hex(hw2, 4) + " {32-bit}";
  }

  length = 2;
  return s + format_thumb16(hw1, address);
}

std::map<uint32_t, std::string> cpu::disassemble(uint32_t start,
                                                 uint32_t stop) const {
  std::map<uint32_t, std::string> lines;

  uint32_t addr = start;
  while (addr <= stop) {
    uint8_t length = 0;
    lines[addr] = disassemble(addr, length);
    if (length == 0)
      break;
    addr += length;
  }

  return lines;
}
