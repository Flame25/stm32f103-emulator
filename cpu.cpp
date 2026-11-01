#include "cpu.h"
#include <cstdint>
#include <cstdio>
#include <string>

namespace {

// Sign-extend the low `bits` of `value` to 32 bits.
uint32_t sign_extend(uint32_t value, unsigned bits) {
  const uint32_t sign = 1u << (bits - 1);
  value &= (1u << bits) - 1u;
  return (value ^ sign) - sign;
}

} // namespace

cpu::cpu(bus &b) : m_bus(&b) {}

void cpu::reset() {
  for (auto &reg : r)
    reg = 0;
  pc = 0;
  xpsr = 0;
  it_state = 0;
  cycles = 0;
  clock_count = 0;
  pc_written = false;
  ins = decoded{};
}

uint16_t cpu::read(uint32_t address) const { return m_bus->read16(address); }

uint16_t cpu::read_flash(uint32_t address) const {
  return m_bus->read16_flash(address);
}

uint32_t cpu::read32(uint32_t address) const { return m_bus->read32(address); }

bool cpu::GetFlag(apsr_flag f) const {
  return (xpsr & static_cast<uint32_t>(f)) != 0;
}

// ConditionPassed() for the 4-bit cond field. Returns false for the reserved
// cond == 0b111x encodings.
bool cpu::condition_passed(uint8_t cond) {
  const bool n = GetFlag(N);
  const bool z = GetFlag(Z);
  const bool c = GetFlag(C);
  const bool v = GetFlag(V);

  switch (cond >> 1) {
  case 0b000: // EQ / NE
    return (cond & 1u) ? !z : z;
  case 0b001: // CS / CC
    return (cond & 1u) ? !c : c;
  case 0b010: // MI / PL
    return (cond & 1u) ? !n : n;
  case 0b011: // VS / VC
    return (cond & 1u) ? !v : v;
  case 0b100: // HI / LS
    return (cond & 1u) ? (!c || z) : (c && !z);
  case 0b101: // GE / LT
    return (cond & 1u) ? (n != v) : (n == v);
  case 0b110: // GT / LE
    return (cond & 1u) ? (z || (n != v)) : (!z && (n == v));
  default: // 0b111 -> AL/NV reserved in Thumb
    return false;
  }
}

void cpu::SetFlag(apsr_flag f, bool v) {
  if (v)
    xpsr |= static_cast<uint32_t>(f);
  else
    xpsr &= ~static_cast<uint32_t>(f);
}

void cpu::set_flag(char name, bool value) {
  switch (name) {
  case 'N':
    SetFlag(N, value);
    break;
  case 'Z':
    SetFlag(Z, value);
    break;
  case 'C':
    SetFlag(C, value);
    break;
  case 'V':
    SetFlag(V, value);
    break;
  case 'Q':
    SetFlag(Q, value);
    break;
  default:
    break;
  }
}

bool cpu::flag(char name) const {
  switch (name) {
  case 'N':
    return GetFlag(N);
  case 'Z':
    return GetFlag(Z);
  case 'C':
    return GetFlag(C);
  case 'V':
    return GetFlag(V);
  case 'Q':
    return GetFlag(Q);
  default:
    return false;
  }
}

uint16_t cpu::fetch16() { return read(pc); }

void cpu::clock() {

  if (cycles == 0) {

    uint16_t hw1 = read_flash(pc);

    bool wide = false;

    // bits[15:11] == 11101 / 11110 / 11111 -> 32-bit Thumb instruction
    if ((hw1 >> 11) >= 0x1D) {
      // Second halfword sits at pc + 2. Fetching it with fetch16() would read
      // pc again and hand hw1 to the decoder as hw2.
      ins = decode_thumb32(hw1, read_flash(pc + 2));
      wide = true;
    } else {
      ins = decode_thumb16(hw1);
    }

    execute();

    if (!pc_written) {
      pc += wide ? 4 : 2;
    }

    cycles = ins.cycles;
    pc_written = false;
  }

  clock_count++;
  if (cycles != 0) {
    --cycles;
  }
}

cpu::decoded cpu::decode_thumb16(uint16_t hw) {
  decoded d;

  // 000xx: A5.2.1 Shift (immediate), add, subtract, move, and compare
  // if ((hw >> 13) == 0b000) {
  //   return d; // TODO
  // }

  if ((hw >> 6) == 0) { // A.7.7.77 MOV (Register) T2
    d.op = Instruction::MOV_REG;
    d.rm = (hw >> 3) & 0x7;
    d.rd = hw & 0x7;
    d.l_t = 2;
    d.flags = true;
  }

  if ((hw >> 6) == 0b01000000101) { // A.7.7.2 ADC (Register) T1
    d.rm = (hw >> 3) & 0x7;
    d.rd = hw & 0x7;
    d.flags = !it_state;
  }

  if ((hw >> 8) == 0x46) { // A.7.7.77 MOV (Register) T1
    d.op = Instruction::MOV_REG;
    uint8_t D = (hw >> 7) & 0x1;
    d.rm = (hw >> 3) & 0xF;
    d.rd = (hw & 0x7) | (D << 3);
  }

  if ((hw >> 9) == 0b0001110) { // A.7.7.3 ADD (Immediate) T1
    d.op = Instruction::ADD_IMM;
    d.rn = (hw >> 3) & 0x7;
    d.rd = hw & 0x7;
    d.imm = static_cast<uint32_t>(hw >> 6) & 0x7;
    return d;
  }

  if ((hw >> 9) == 0b0001100) { // A.7.7.3 ADD (Immediate) T1
    d.op = Instruction::ADD_REG;
    d.rm = (hw >> 6) & 0x7;
    d.rn = (hw >> 3) & 0x7;
    d.rd = hw & 0x7;
    return d;
  }

  // Use 6 bits instead of 5 to include bit[10], which can be either 0 or 1.
  // Example: LDR (immediate) T1 has the fixed pattern 01101 in bit[15:11].
  // Including bit[10], the pattern becomes 01101x, matching switch cases
  // 0x1A and 0x1B when using `hw >> 10`.
  switch (hw >> 10) {
  case 0x07:

  case 0x08: // A7.7.76 MOV (immediate) T1
  case 0x09:
    d.op = Instruction::MOV_IMM;
    d.rd = (hw >> 8) & 0x7;
    d.imm = static_cast<uint32_t>(hw & 0xFF);
    d.flags = !it_state;
    d.cycles = 2;
    return d;
  case 0x0C:
  case 0x0D:
    return d;
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
    d.cycles = 2;
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
    d.op = Instruction::STR_IMM;
    d.rt = hw & 0x07;
    d.rn = (hw >> 3) & 0x07;
    d.imm = static_cast<uint32_t>((hw >> 6) & 0x1F) << 2;
    d.index = 1;
    d.add = 1;
    return d; // TODO

  case 0x1A: // LDR (immediate, word): 01101 imm5 Rn Rt
  case 0x1B:
    d.op = Instruction::LDR_IMM;
    d.rt = hw & 0x7;
    d.rn = (hw >> 3) & 0x07;
    d.imm = static_cast<uint32_t>((hw >> 6) & 0x1F) << 2;
    d.cycles = 2;
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
    d.op = Instruction::LDR_IMM;
    d.rt = (hw >> 8) & 0x07;
    d.imm = static_cast<uint32_t>(hw & 0xFF) << 2;
    return d;
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
  case 0x37: {
    d.op = Instruction::B_COND;
    d.cond = (uint8_t)(hw >> 8) & 0x0F;
    d.imm = ((hw & 0xFF) << 1);
    d.cycles = 1; // TODO: Check this again
    return d;
  }
  case 0x38: // A7.7.11 B (T2, unconditional)
  case 0x39:
  default:
    // Unknown, or 32-bit prefix already filtered by clock(). Costs one tick
    // so cycles never underflows.
    return d;
  }
}

cpu::decoded cpu::decode_thumb32(uint16_t hw1, uint16_t hw2) {
  decoded d;

  // Level 1 (A5.3): hw1[15:11] selects the top-level group.
  //   0x1D = 11101 -> load/store multiple, dual, exclusive, table branch
  //   0x1E = 11110 -> data processing (modified immediate) / branches + misc
  //   0x1F = 11111 -> data processing (plain imm) / load-store single
  switch (hw1 >> 11) {
  case 0x1E:
    break; // sole group implemented for now
  default:
    return d; // TODO
  }

  // Level 2 (A5.3.2 / A5.3.1): hw2[15:14] splits the 11110 group.
  // Verified against arm-none-eabi-as: BL with J1=0 has hw2[15:12]=0xD, so the
  // discriminator cannot be hw2[15:12].
  const uint32_t hi = hw2 >> 14;

  if (hi == 0b10 && ((hw2 >> 12) & 1u)) {
    // B (T4): 11110 S imm10 | 10 J1 1 J2 imm11
    // imm32 = SignExtend(S:I1:I2:imm10:imm11:'0', 32), I = NOT(J XOR S)
    const uint32_t s = (hw1 >> 10) & 1u;
    const uint32_t j1 = (hw2 >> 13) & 1u;
    const uint32_t j2 = (hw2 >> 11) & 1u;
    const uint32_t i1 = (~(j1 ^ s)) & 1u;
    const uint32_t i2 = (~(j2 ^ s)) & 1u;
    const uint32_t imm = (s << 24) | (i1 << 23) | (i2 << 22) |
                         ((hw1 & 0x3FFu) << 12) | ((hw2 & 0x7FFu) << 1);
    d.op = Instruction::B_W;
    d.imm = sign_extend(imm, 25);
    d.cycles = 3;
    return d;
  }

  if (hi == 0b10) {
    // B<c> (T3): 11110 S cond imm6 | 10 J1 0 J2 imm11
    // imm32 = SignExtend(S:J2:J1:imm6:imm11:'0', 32), J used directly.
    const uint32_t cond = (hw1 >> 6) & 0xF;
    if (cond >= 0b1110)
      return d; // 1110 = UNDEFINED, 1111 = UDF (permanently undefined)

    const uint32_t s = (hw1 >> 10) & 1u;
    const uint32_t j1 = (hw2 >> 13) & 1u;
    const uint32_t j2 = (hw2 >> 11) & 1u;
    const uint32_t imm = (s << 20) | (j2 << 19) | (j1 << 18) |
                         ((hw1 & 0x3Fu) << 12) | ((hw2 & 0x7FFu) << 1);
    d.op = Instruction::B_COND;
    d.cond = static_cast<uint8_t>(cond);
    d.imm = sign_extend(imm, 21);
    d.cycles = 3;
    return d;
  }

  if (hi == 0b11 && ((hw2 >> 12) & 1u)) {
    // BL (T1). BLX (immediate) is A/R-only: on M-profile bit12 == 0 here is
    // UNDEFINED, so it is deliberately not decoded.
    const uint32_t s = (hw1 >> 10) & 1u;
    const uint32_t j1 = (hw2 >> 13) & 1u;
    const uint32_t j2 = (hw2 >> 11) & 1u;
    const uint32_t i1 = (~(j1 ^ s)) & 1u;
    const uint32_t i2 = (~(j2 ^ s)) & 1u;
    const uint32_t imm = (s << 24) | (i1 << 23) | (i2 << 22) |
                         ((hw1 & 0x3FFu) << 12) | ((hw2 & 0x7FFu) << 1);
    d.op = Instruction::BL;
    d.imm = sign_extend(imm, 25);
    d.cycles = 3;
    return d;
  }

  return d; // TODO: remaining 11110 sub-groups
}

void cpu::execute() {
  switch (ins.op) {
  case Instruction::LDR_LITERAL:
    LDR_LITERAL();
    break;
  case Instruction::LDR_IMM:
    LDR_IMM();
    break;
  case Instruction::B_COND:
    B_COND();
    break;
  case Instruction::B_W:
    B_W();
    break;
  case Instruction::BL:
    BL();
    break;
  case Instruction::MOV_IMM:
    MOV_IMM();
    break;
  case Instruction::MOV_REG:
    MOV_REG();
    break;
  case Instruction::STR_IMM:
    STR_IMM();
    break;
  case Instruction::ADD_IMM:
    ADD_IMM();
    break;
  case Instruction::ADD_REG:
    ADD_REG();
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
    r[ins.rt] = m_bus->read32(address);
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

// A7.7.12 B<c> (T3): if ConditionPassed() then BranchWritePC(PC + imm32)
uint8_t cpu::B_COND() {
  if (condition_passed(ins.cond)) {
    pc = pc + 4 + ins.imm; // PC reads as insn_addr + 4 on M-profile
    pc_written = true;
  }
  return 0;
}

// A7.7.11 B (T4): BranchWritePC(PC + imm32)
uint8_t cpu::B_W() {
  pc = pc + 4 + ins.imm;
  pc_written = true;
  return 0;
}

// A7.7.18 BL (T1): LR = PC | 1 ; BranchWritePC(PC + imm32)
uint8_t cpu::BL() {
  r[14] = (pc + 4) | 1u; // return address, Thumb bit set
  pc = pc + 4 + ins.imm;
  pc_written = true;
  return 0;
}

uint8_t cpu::MOV_IMM() {
  uint32_t result = ins.imm;
  r[ins.rd] = result;

  // NON IT Block - MOV becoming MOVS
  if (!it_state) {
    SetFlag(N, (result >> 31) & 1u);
    SetFlag(Z, result == 0);
  } else {
    // MOV with Carry
  }

  return 0;
}

uint8_t cpu::MOV_REG() {
  uint32_t result = ins.rm;

  if (it_state && ins.l_t == 2) {
    // UNPREDICTABLE
    return 0;
  }

  if (ins.rd != 15) {
    r[ins.rd] = result;

    if (ins.flags) {
      SetFlag(N, (result >> 31) & 1u);
      SetFlag(Z, result == 0);
    }
  } else {

    // TODO: Last in ITBlock
  }
  return 0;
}

uint8_t cpu::ADD_IMM() {

  uint64_t unsigned_sum = static_cast<uint64_t>(r[ins.rn]) + ins.imm;

  bool sign_a = (r[ins.rn] >> 31) & 1u;
  bool sign_b = (ins.imm >> 31) & 1u;
  bool sign_result = (unsigned_sum >> 31) & 1u;

  bool carry = (unsigned_sum >> 32) & 1u;
  uint32_t result32 = static_cast<uint32_t>(unsigned_sum);

  if (!it_state) {
    SetFlag(N, (result32 >> 31) & 1u);
    SetFlag(C, carry);
    SetFlag(Z, result32 == 0);

    if (sign_a == sign_b && sign_result != sign_a) {
      SetFlag(V, 1);
    } else {
      SetFlag(V, 0);
    }
  }

  r[ins.rd] = result32;
  return 0;
}

uint8_t cpu::ADD_REG() {

  uint32_t operand1 = r[ins.rn];
  uint32_t operand2 = r[ins.rm];

  uint64_t unsigned_sum = static_cast<uint64_t>(operand1) + operand2;

  bool sign_a = (operand1 >> 31) & 1u;
  bool sign_b = (operand2 >> 31) & 1u;
  bool sign_result = (unsigned_sum >> 31) & 1u;

  bool carry = (unsigned_sum >> 32) & 1u;
  uint32_t result32 = static_cast<uint32_t>(unsigned_sum);

  if (!it_state) {
    SetFlag(N, (result32 >> 31) & 1u);
    SetFlag(C, carry);
    SetFlag(Z, result32 == 0);
    SetFlag(V, sign_a == sign_b && sign_result != sign_a);
  }

  r[ins.rd] = result32;
  return 0;
}

uint8_t cpu::STR_IMM() {
  uint32_t addr = r[ins.rn];
  uint32_t off_addr = addr;

  if (ins.add) {
    off_addr += ins.imm;
  } else {
    off_addr -= ins.imm;
  }

  if (ins.index) {
    addr = off_addr;
  }

  m_bus->write32(addr, r[ins.rt]);

  if (ins.wback) {
    r[ins.rn] = off_addr;
  }

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

  // STR (immediate, word): 01100 imm5 Rn Rt
  if ((hw >> 11) == 0b01100) {
    uint32_t rt = hw & 0x7;
    uint32_t rn = (hw >> 3) & 0x7;
    uint32_t imm5 = (hw >> 6) & 0x1F;
    uint32_t offset = imm5 << 2;

    return "STR r" + std::to_string(rt) + ", [r" + std::to_string(rn) + ", #" +
           std::to_string(offset) + "]";
  }

  if ((hw >> 11) == 0b00100) {
    uint32_t rd = (hw >> 8) & 0x7;
    uint32_t imm8 = hw & 0xFF;

    return "MOVS r" + std::to_string(rd) + ", #" + std::to_string(imm8);
  }

  if ((hw >> 12) == 0b1101) {
    uint32_t cond = (hw >> 8) & 0xF;

    if (cond == 0b1110)
      return "UDF";

    if (cond == 0b1111)
      return "SVC";

    int32_t offset = static_cast<int8_t>((hw & 0xFF) << 1);
    uint32_t target = addr + 4 + offset;

    static const char *conditions[] = {"EQ", "NE", "CS", "CC", "MI",
                                       "PL", "VS", "VC", "HI", "LS",
                                       "GE", "LT", "GT", "LE"};

    return "B" + std::string(conditions[cond]) + " $" + hex(target, 8);
  }

  if ((hw >> 9) == 0b0001110) {
    uint32_t rd = hw & 0x7;
    uint32_t rn = (hw >> 3) & 0x7;
    uint32_t imm3 = (hw >> 6) & 0x7;

    return "ADDS r" + std::to_string(rd) + ", r" + std::to_string(rn) + ", #" +
           std::to_string(imm3);
  }

  return "TODO " + hex(hw, 4);
}

} // namespace

std::string cpu::disassemble(uint32_t address, uint8_t &length) const {
  std::string s = "$" + hex(address, 8) + ": ";

  uint16_t hw1 = read_flash(address);

  // bits[15:11] == 11101 / 11110 / 11111 -> 32-bit Thumb instruction
  if ((hw1 >> 11) >= 0x1D) {
    uint16_t hw2 = read_flash(address + 2);
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
