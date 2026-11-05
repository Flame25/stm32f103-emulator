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
  decoded d; // make sure d.op defaults to UNDEF

  if ((hw >> 6) == 0) { // MOV (register) T2 (LSL #0)
    d.op = Instruction::MOV_REG;
    d.rm = (hw >> 3) & 0x7;
    d.rd = hw & 0x7;
    d.l_t = 2;
    d.flags = true;
    return d;
  }

  if ((hw >> 6) == 0b0100000101) { // ADC (register) T1
    d.op = Instruction::ADC_REG;
    d.rm = (hw >> 3) & 0x7;
    d.rd = hw & 0x7;
    d.flags = !it_state;
    return d;
  }

  if ((hw >> 8) == 0x46) { // MOV (register) T1
    d.op = Instruction::MOV_REG;
    d.rm = (hw >> 3) & 0xF;
    d.rd = (hw & 0x7) | (((hw >> 7) & 1) << 3);
    d.flags = false;
    return d;
  }

  if ((hw >> 8) == 0xBE) { // BKPT T1
    d.op = Instruction::BKPT;
    d.imm = static_cast<uint32_t>(hw & 0xFF);
    d.cycles = 3;
  }

  switch (hw >> 9) { // add/sub reg/imm3
  case 0b0001100:    // ADD (register) T1
  case 0b0001101:    // SUB (register) T1
    d.op = (hw >> 9) & 1 ? Instruction::SUB_REG : Instruction::ADD_REG;
    d.rm = (hw >> 6) & 0x7;
    d.rn = (hw >> 3) & 0x7;
    d.rd = hw & 0x7;
    d.flags = !it_state;
    return d;
  case 0b0001110: // ADD (immediate) T1
  case 0b0001111: // SUB (immediate) T1
    d.op = (hw >> 9) & 1 ? Instruction::SUB_IMM : Instruction::ADD_IMM;
    d.imm = (hw >> 6) & 0x7;
    d.rn = (hw >> 3) & 0x7;
    d.rd = hw & 0x7;
    d.flags = !it_state;
    return d;
  }

  switch (hw >> 10) {
  case 0x08: // MOV (immediate) T1
  case 0x09:
    d.op = Instruction::MOV_IMM;
    d.rd = (hw >> 8) & 0x7;
    d.imm = hw & 0xFF;
    d.flags = !it_state;
    d.cycles = 2;
    return d;

  case 0x0A: // CMP (immediate) T1
  case 0x0B:
    d.op = Instruction::CMP_IMM;
    d.rn = (hw >> 8) & 0x07;
    d.imm = static_cast<uint32_t>(hw & 0xFF);
    return d;

  case 0x0C: // ADD (immediate) T2
  case 0x0D:
  case 0x0E: // SUB (immediate) T2
  case 0x0F:
    d.op = (hw & 0x0800) ? Instruction::SUB_IMM : Instruction::ADD_IMM;
    d.rd = (hw >> 8) & 0x7;
    d.rn = d.rd;
    d.imm = hw & 0xFF;
    d.flags = !it_state;
    return d;

  case 0x10:
    return d; // A5.2.2 Data processing, TODO
  case 0x11:
    return d; // A5.2.3 Special data / BX, TODO

  case 0x12: // LDR (literal)
  case 0x13:
    d.op = Instruction::LDR_LITERAL;
    d.rt = (hw >> 8) & 0x7;
    d.imm = (hw & 0xFF) << 2;
    d.cycles = 2;
    return d;

  case 0x14:
  case 0x15:
  case 0x16:
  case 0x17:
    return d; // A5.3.1 load/store register offset, TODO

  case 0x18: // STR (immediate) T1
  case 0x19:
    d.op = Instruction::STR_IMM;
    d.rt = hw & 0x7;
    d.rn = (hw >> 3) & 0x7;
    d.imm = ((hw >> 6) & 0x1F) << 2;
    d.index = 1;
    d.add = 1;
    return d;

  case 0x1A: // LDR (immediate) T1
  case 0x1B:
    d.op = Instruction::LDR_IMM;
    d.rt = hw & 0x7;
    d.rn = (hw >> 3) & 0x7;
    d.imm = ((hw >> 6) & 0x1F) << 2;
    d.index = 1;
    d.add = 1;
    d.cycles = 2;
    return d;

  case 0x1C:
  case 0x1D: // STRB (immediate)
  case 0x1E:
  case 0x1F:  // LDRB (immediate)
    return d; // TODO

  case 0x20:
  case 0x21: // STRH (immediate)
  case 0x22:
  case 0x23:  // LDRH (immediate)
    return d; // TODO

  case 0x24: // STR (SP-relative) T2
  case 0x25:
    d.op = Instruction::STR_IMM;
    d.rt = (hw >> 8) & 0x7;
    d.rn = 13;
    d.imm = (hw & 0xFF) << 2;
    d.index = 1;
    d.add = 1;
    return d;

  case 0x26: // LDR (SP-relative) T2
  case 0x27:
    d.op = Instruction::LDR_IMM;
    d.rt = (hw >> 8) & 0x7;
    d.rn = 13;
    d.imm = (hw & 0xFF) << 2;
    d.index = 1;
    d.add = 1;
    d.cycles = 2;
    return d;

  case 0x28:
  case 0x29:
    return d; // ADR, TODO
  case 0x2A:
  case 0x2B:
    return d; // ADD (SP plus immediate), TODO
  case 0x2C:
  case 0x2D:
  case 0x2E:
  case 0x2F:
    return d; // A5.4 misc 16-bit (PUSH/POP/CPS/...), TODO
  case 0x30:
  case 0x31:
    return d; // STM, TODO
  case 0x32:
  case 0x33:
    return d; // LDM, TODO

  case 0x34:
  case 0x35:
  case 0x36:
  case 0x37: { // B<c>, SVC, UDF
    uint8_t cond = (hw >> 8) & 0xF;
    if (cond >= 0xE)
      return d; // UDF / SVC, TODO
    d.op = Instruction::B_COND;
    d.cond = cond;
    d.imm = static_cast<uint32_t>(
        static_cast<int32_t>(static_cast<int8_t>(hw & 0xFF)) * 2);
    d.cycles = 1; // TODO: check
    return d;
  }

  case 0x38: // B (T2, unconditional)
  case 0x39: {
    d.op = Instruction::B;
    d.imm = static_cast<uint32_t>(
        static_cast<int32_t>(static_cast<int16_t>((hw & 0x7FF) << 5) >> 4));
    d.cycles = 1;
    d.cond = 0x0;
  }

  default:
    return d; // TODO / unknown
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
  case Instruction::B:
    B();
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
  case Instruction::CMP_IMM:
    CMP_IMM();
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
  case Instruction::ADC_REG:
    ADC_REG();
    break;
  case Instruction::SUB_IMM:
    SUB_IMM();
    break;
  case Instruction::SUB_REG:
    break;
  case Instruction::BKPT:
    BKPT();
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
    pc =
        pc + 4 +
        static_cast<int32_t>(ins.imm); // PC reads as insn_addr + 4 on M-profile
    pc_written = true;
  }
  return 0;
}

// B (T2): BranchWritePC(PC + imm32)
uint8_t cpu::B() {
  pc = pc + 4 + static_cast<int32_t>(ins.imm);
  pc_written = true;
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

uint8_t cpu::CMP_IMM() {

  uint64_t unsigned_sum =
      static_cast<uint64_t>(r[ins.rn]) +
      static_cast<uint32_t>(~static_cast<uint32_t>(ins.imm)) + 1;

  bool sign_a = (r[ins.rn] >> 31) & 1u;
  bool sign_b = (ins.imm >> 31) & 1u;
  bool sign_result = (unsigned_sum >> 31) & 1u;

  bool carry = (unsigned_sum >> 32) & 1u;
  uint32_t result32 = static_cast<uint32_t>(unsigned_sum);

  SetFlag(N, (result32 >> 31) & 1u);
  SetFlag(Z, result32 == 0);
  SetFlag(C, carry);
  SetFlag(V, sign_a != sign_b && sign_result != sign_a);

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

  if (ins.flags) {
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

  if (ins.flags) {
    SetFlag(N, (result32 >> 31) & 1u);
    SetFlag(C, carry);
    SetFlag(Z, result32 == 0);
    SetFlag(V, sign_a == sign_b && sign_result != sign_a);
  }

  r[ins.rd] = result32;
  return 0;
}

uint8_t cpu::ADC_REG() {

  uint32_t operand1 = r[ins.rn];
  uint32_t operand2 = r[ins.rm];

  uint64_t unsigned_sum =
      static_cast<uint64_t>(operand1) + operand2 + apsr_flag::C;

  bool sign_a = (operand1 >> 31) & 1u;
  bool sign_b = (operand2 >> 31) & 1u;
  bool sign_result = (unsigned_sum >> 31) & 1u;

  bool carry = (unsigned_sum >> 32) & 1u;
  uint32_t result32 = static_cast<uint32_t>(unsigned_sum);

  if (ins.flags) {
    SetFlag(N, (result32 >> 31) & 1u);
    SetFlag(C, carry);
    SetFlag(Z, result32 == 0);
    SetFlag(V, sign_a == sign_b && sign_result != sign_a);
  }

  r[ins.rd] = result32;
  return 0;
}

uint8_t cpu::SUB_IMM() {

  uint64_t unsigned_sum =
      static_cast<uint64_t>(r[ins.rn]) +
      static_cast<uint32_t>(~static_cast<uint32_t>(ins.imm)) + 1;

  bool sign_a = (r[ins.rn] >> 31) & 1u;
  bool sign_b = (ins.imm >> 31) & 1u;
  bool sign_result = (unsigned_sum >> 31) & 1u;

  bool carry = (unsigned_sum >> 32) & 1u;
  uint32_t result32 = static_cast<uint32_t>(unsigned_sum);

  if (ins.flags) {
    SetFlag(N, (result32 >> 31) & 1u);
    SetFlag(Z, result32 == 0);
    SetFlag(C, carry);
    SetFlag(V, sign_a != sign_b && sign_result != sign_a);
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

uint8_t cpu::BKPT() {

  std::ostringstream oss;
  oss << "CHECK 0x" << std::hex << pc;

  std::string output = oss.str();
  m_bus->m_out.push_back(output);
  if (ins.imm == 0xAB && semihost_out) {

    switch (reg(0)) {
    case 0x03: // SYS_WRITEC: r1 = address of one char
      semihost_out(std::string(1, (char)m_bus->read8(reg(1))));
      break;
    case 0x04: { // SYS_WRITE0: r1 = address of a NUL-terminated string
      std::string s;
      for (uint32_t a = reg(1); m_bus->read8(a) != 0; ++a)
        s += (char)m_bus->read8(a);
      semihost_out(s);
      break;
    }
    }
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

} // namespace

static const char *reg_name(uint32_t r) {
  static const char *names[] = {"r0",  "r1", "r2", "r3", "r4",  "r5",
                                "r6",  "r7", "r8", "r9", "r10", "r11",
                                "r12", "sp", "lr", "pc"};
  return names[r & 0xF];
}

std::string format_thumb16(uint16_t hw, uint32_t addr) {
  const uint32_t rd3 = hw & 0x7;
  const uint32_t rn3 = (hw >> 3) & 0x7;
  const uint32_t rm3 = (hw >> 6) & 0x7;
  const uint32_t rd8 = (hw >> 8) & 0x7;
  const uint32_t imm8 = hw & 0xFF;

  // MOVS rd, rm (LSL #0) T2
  if ((hw >> 6) == 0) {
    return std::string("MOVS ") + reg_name(rd3) + ", " + reg_name(rn3);
  }

  // ADC (register) T1: 0100000101
  if ((hw >> 6) == 0b0100000101) {
    return std::string("ADCS ") + reg_name(rd3) + ", " + reg_name(rn3);
  }

  // MOV (register) T1: 01000110 D Rm Rd
  if ((hw >> 8) == 0x46) {
    uint32_t rd = rd3 | (((hw >> 7) & 1) << 3);
    uint32_t rm = (hw >> 3) & 0xF;
    return std::string("MOV ") + reg_name(rd) + ", " + reg_name(rm);
  }

  // ADD/SUB (register / imm3) T1: 000110x / 000111x
  switch (hw >> 9) {
  case 0b0001100:
    return std::string("ADDS ") + reg_name(rd3) + ", " + reg_name(rn3) + ", " +
           reg_name(rm3);
  case 0b0001101:
    return std::string("SUBS ") + reg_name(rd3) + ", " + reg_name(rn3) + ", " +
           reg_name(rm3);
  case 0b0001110:
    return std::string("ADDS ") + reg_name(rd3) + ", " + reg_name(rn3) + ", #" +
           std::to_string(rm3);
  case 0b0001111:
    return std::string("SUBS ") + reg_name(rd3) + ", " + reg_name(rn3) + ", #" +
           std::to_string(rm3);
  }

  switch (hw >> 11) {
  case 0b00100: // MOVS rd, #imm8
    return std::string("MOVS ") + reg_name(rd8) + ", #" + std::to_string(imm8);

  case 0b00101: // CMP rn, #imm8
    return std::string("CMP ") + reg_name(rd8) + ", #" + std::to_string(imm8);

  case 0b00110: // ADDS rdn, #imm8
    return std::string("ADDS ") + reg_name(rd8) + ", #" + std::to_string(imm8);

  case 0b00111: // SUBS rdn, #imm8
    return std::string("SUBS ") + reg_name(rd8) + ", #" + std::to_string(imm8);

  case 0b01001: { // LDR (literal)
    uint32_t offset = imm8 << 2;
    uint32_t target = ((addr + 4) & ~0x3u) + offset;
    return std::string("LDR ") + reg_name(rd8) + ", [pc, #" +
           std::to_string(offset) + "] ; $" + hex(target, 8);
  }

  case 0b01100:   // STR (immediate)
  case 0b01101: { // LDR (immediate)
    uint32_t offset = ((hw >> 6) & 0x1F) << 2;
    return std::string((hw >> 11) & 1 ? "LDR " : "STR ") + reg_name(rd3) +
           ", [" + reg_name(rn3) + ", #" + std::to_string(offset) + "]";
  }

  case 0b10010:   // STR (SP-relative)
  case 0b10011: { // LDR (SP-relative)
    uint32_t offset = imm8 << 2;
    return std::string((hw >> 11) & 1 ? "LDR " : "STR ") + reg_name(rd8) +
           ", [sp, #" + std::to_string(offset) + "]";
  }

  case 0b11100: { // B Unconditional T2
    uint32_t offset = static_cast<int32_t>(
        static_cast<int32_t>(static_cast<int16_t>((hw & 0x7FF) << 5) >> 4));
  }
  }

  // B<c>, SVC, UDF: 1101 cond imm8
  if ((hw >> 12) == 0b1101) {
    uint32_t cond = (hw >> 8) & 0xF;
    if (cond == 0xE)
      return "UDF #" + std::to_string(imm8);
    if (cond == 0xF)
      return "SVC #" + std::to_string(imm8);

    int32_t offset = static_cast<int32_t>(static_cast<int8_t>(imm8)) * 2;
    uint32_t target = addr + 4 + offset;

    static const char *conditions[] = {"EQ", "NE", "CS", "CC", "MI",
                                       "PL", "VS", "VC", "HI", "LS",
                                       "GE", "LT", "GT", "LE"};
    return std::string("B") + conditions[cond] + " $" + hex(target, 8);
  }

  if ((hw & 0xFF00) == 0xBE00) {
    return "BKPT #" + hex(imm8, 2);
  }

  return "TODO " + hex(hw, 4);
}

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
