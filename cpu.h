#pragma once

#include "bus.h"
#include <cstdint>
#include <functional>
#include <map>
#include <sstream>
#include <string>

#define S

// ARM Cortex M3 Core
class cpu {

public:
  enum apsr_flag {
    Q = (1 << 0),
    V = (1 << 1),
    Z = (1 << 2),
    C = (1 << 3),
    N = (1 << 4),
  };

  explicit cpu(bus &b);
  ~cpu() = default;

  void clock();
  void reset();

  // Inspection (UI / tests)
  uint32_t reg(int index) const { return r[index & 0xF]; }
  uint32_t reg_pc() const { return pc; }
  uint32_t reg_xpsr() const { return xpsr; }
  uint8_t reg_sp_cycles() const { return cycles; }
  uint32_t reg_clock_count() const { return clock_count; }

  inline bool complete() { return cycles == 0; }

  bool get_flag(apsr_flag f) const;
  void set_flag(apsr_flag f, bool v);

  void handle_semihosting();

  // Debug/UI hooks: place the core at an address, or force a flag.
  void set_pc(uint32_t address) { pc = address; }

  // Human readable form of the instruction at `address`. Advances `length`
  // by the size of the encoding (2 or 4 bytes).
  std::string disassemble(uint32_t address, uint8_t &length) const;

  // Map of address -> disassembly line, over [start, stop].
  std::map<uint32_t, std::string> disassemble(uint32_t start,
                                              uint32_t stop) const;

  uint16_t read(uint32_t address) const;

  uint32_t read32(uint32_t address) const;

private:
  // One entry per encoding we currently implement. decode fills it,
  // execute() dispatches on it.
  enum class Instruction : uint8_t {
    UNKNOWN,
    LDR_LITERAL,
    LDR_IMM,
    STR_IMM,
    B,
    B_COND, // A7.7.8  B<c>.W
    B_W,    // A7.7.11 B (T4)
    BL,     // A7.7.9  BL (immediate)
    CMP_IMM,
    CMP_REG,
    MOV_IMM,
    MOV_REG,
    MOVT,
    MOVW,
    PUSH,
    POP,
    BX_REG,
    ADD_IMM,
    ADD_REG,
    ADC_REG,
    AND_REG,
    EOR_REG,
    SUB_IMM,
    SUB_REG,
    BKPT,
  };

  // Operands pulled out of the raw halfword(s) by decode. `cycles` lives here
  // so decode stays a pure function and the disassembler can reuse it.
  struct decoded {
    Instruction op = Instruction::UNKNOWN;
    uint8_t rt = 0;
    uint8_t rn = 0;
    uint8_t rd = 0;
    uint8_t rm = 0;
    uint8_t cond = 0;
    uint8_t cycles = 1;
    uint32_t imm = 0;
    uint8_t l_t = 1; // T-(1,2,3,4...)
    bool add = 0;
    bool index = 0;
    bool wback = 0;
    bool flags = 0;
  };

  // Register following ARM Cortex M3 datasheet
  uint32_t r[16]{}; // r[13]=SP r[14]=LR r[15]=PC(addr of current insn)
  uint32_t pc = 0x00;
  uint32_t vtor = 0x00000000; // Vector Table Offset Register (SCB->VTOR)
  uint32_t xpsr{};            // N Z C V Q T (APSR) + IPSR[8:0] + EPSR
  uint8_t it_state = 0;

  uint32_t clock_count = 0;
  uint8_t cycles = 0;

  std::function<void(const std::string &)> semihost_out;

  // Set by an instruction that redirects pc; stops clock() from applying the
  // automatic advance.
  bool pc_written = false;

  // ConditionPassed() for B<c>.W. cond 111x is not a valid branch condition.
  bool condition_passed(uint8_t cond);

  decoded ins;

  bus *m_bus = nullptr;

  uint16_t fetch16();
  decoded decode_thumb16(uint16_t hw);
  decoded decode_thumb32(uint16_t hw1, uint16_t hw2);
  decoded decode_shift_add_sub_mov_cmp(uint16_t hw);
  decoded decode_ldr_literal(uint16_t hw);
  decoded decode_ldst_single(uint16_t hw);
  decoded decode_special_bx(uint16_t hw);

  void execute();

  // Instruction Set
  uint8_t STR_IMM();
  uint8_t LDR_LITERAL();
  uint8_t LDR_IMM();
  uint8_t B_COND();
  uint8_t B();
  uint8_t B_W();
  uint8_t BL();
  uint8_t CMP_IMM();
  uint8_t CMP_REG();
  uint8_t MOV_IMM();
  uint8_t MOV_REG();
  uint8_t MOVT();
  uint8_t MOVW();
  uint8_t PUSH();
  uint8_t POP();
  uint8_t BX_REG();
  uint8_t AND_REG();
  uint8_t ADD_IMM();
  uint8_t ADD_REG();
  uint8_t ADC_REG();
  uint8_t SUB_IMM();
  uint8_t SUB_REG();
  uint8_t BKPT();
};
