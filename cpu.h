#pragma once

#include "bus.h"
#include <cstdint>
#include <map>
#include <string>

// ARM Cortex M3 Core
class cpu {
public:
  explicit cpu(bus &b);
  ~cpu() = default;

  void clock();
  void reset();

  // ---- Inspection (UI / tests) -------------------------------------------
  uint32_t reg(int index) const { return r[index & 0xF]; }
  uint32_t reg_pc() const { return pc; }
  uint32_t reg_xpsr() const { return xpsr; }
  uint8_t reg_sp_cycles() const { return cycles; }
  uint32_t reg_clock_count() const { return clock_count; }

  inline bool complete() { return cycles == 0; }

  bool flag(char name) const;

  // Human readable form of the instruction at `address`. Advances `length`
  // by the size of the encoding (2 or 4 bytes).
  std::string disassemble(uint32_t address, uint8_t &length) const;

  // Map of address -> disassembly line, over [start, stop].
  std::map<uint32_t, std::string> disassemble(uint32_t start,
                                              uint32_t stop) const;

  uint16_t read(uint32_t address) const;
  uint32_t read32(uint32_t address) const;

private:
  enum class apsr_flag : uint32_t {
    Q = (1u << 0),
    V = (1u << 1),
    Z = (1u << 2),
    C = (1u << 3),
    N = (1u << 4),
  };

  // One entry per encoding we currently implement. decode fills it,
  // execute() dispatches on it.
  enum class Instruction : uint8_t {
    UNKNOWN,
    LDR_LITERAL,
    LDR_IMM,
  };

  // Operands pulled out of the raw halfword(s) by decode.
  struct decoded {
    Instruction op = Instruction::UNKNOWN;
    uint8_t rt = 0;
    uint8_t rn = 0;
    uint32_t imm = 0;
  };

  // Register following ARM Cortex M3 datasheet
  uint32_t r[16]{}; // r[13]=SP r[14]=LR r[15]=PC(addr of current insn)
  uint32_t pc = 0x00;
  uint32_t xpsr{}; // N Z C V Q T (APSR) + IPSR[8:0] + EPSR
  uint8_t it_state{};

  uint16_t clock_count = 0;
  uint8_t cycles = 0;

  decoded ins;

  bus *m_bus = nullptr;

  bool GetFlag(apsr_flag f) const;
  void SetFlag(apsr_flag f, bool v);

  uint16_t fetch16();
  decoded decode_thumb16(uint16_t hw);
  decoded decode_thumb32(uint16_t hw1, uint16_t hw2);
  void execute();

  // Instruction Set
  uint8_t LDR_LITERAL();
  uint8_t LDR_IMM();
};
