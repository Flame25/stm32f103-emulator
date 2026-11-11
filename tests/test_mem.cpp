#include "bus.h"
#include "cpu.h"

#include <cstdint>
#include <cstdio>

// Minimal assertion harness: no external dependency, returns non-zero on any
// failure so it works as a CTest test.

namespace {

int failures = 0;
int checks = 0;

void check(bool condition, const char *what) {
  ++checks;
  if (!condition)
    ++failures;
  std::printf("%s: %s\n", condition ? "PASS" : "FAIL", what);
}

// Test image: vector table + MOVS / PUSH / POP / BL / BX / CMP / BEQ.
//
//   0x08000000  SP    = 0x20005000
//   0x08000004  reset -> 0x08000009 (Thumb bit set)
//   0x08000008  MOVS r0, #5
//   0x0800000A  PUSH {r0}
//   0x0800000C  MOVS r0, #9
//   0x0800000E  POP  {r0}          r0 back to 5
//   0x08000010  BL   -> 0x0800001A
//   0x08000014  CMP  r0, #5        Z = 1
//   0x08000016  BEQ  0x0800001C
//   0x08000018  MOVS r2, #0xAA     must be skipped
//   0x0800001A  BX   lr           -> 0x08000014
//   0x0800001C  B    .             spin
const uint8_t kProgram[] = {
    0x00, 0x50, 0x00, 0x20, //
    0x09, 0x00, 0x00, 0x08, //
    0x05, 0x20,             //
    0x01, 0xB4,             //
    0x09, 0x20,             //
    0x01, 0xBC,             //
    0x00, 0xF0, 0x03, 0xF8, //
    0x05, 0x28,             //
    0x01, 0xD0,             //
    0xAA, 0x22,             //
    0x70, 0x47,             //
    0xFE, 0xE7,             //
};

void run_instruction(cpu &c) {
  c.clock();
  while (!c.complete())
    c.clock();
}

void test_memory_decode(bus &b) {
  // SRAM is addressed by absolute address, not by array index.
  b.write8(bus::kSramBase, 0x5A);
  check(b.read8(bus::kSramBase) == 0x5A, "SRAM absolute address decodes");
  check(b.read8(0x00) == 0, "SRAM write does not leak into address 0x0");

  b.write32(0x20001000, 0xDEADBEEF);
  check(b.read32(0x20001000) == 0xDEADBEEFu, "SRAM 32-bit store/load");

  // Flash is reachable both at 0x08000000 and through the 0x0 boot alias.
  b.flash[0] = 0x11;
  b.flash[1] = 0x22;
  b.flash[2] = 0x33;
  b.flash[3] = 0x44;
  check(b.read32(bus::kFlashBase) == 0x44332211u, "flash read via 0x08000000");
  check(b.read32(0x00) == 0x44332211u, "flash read via 0x0 boot alias");

  // Real flash needs unlock + erase, so plain writes must be dropped.
  b.write8(bus::kFlashBase, 0xFF);
  check(b.read8(bus::kFlashBase) == 0x11, "flash write is ignored");

  // Unmapped address reads as zero for now.
  check(b.read32(0x40000000) == 0, "unmapped address reads zero");
}

void test_reset(cpu &c) {
  c.reset();
  check(c.reg_pc() == 0x08000008u, "reset loads PC from vector table");
  check(c.reg(13) == 0x20005000u, "reset loads SP from vector table");
  check(c.reg(13) % 8 == 0, "initial SP is 8-byte aligned");
}

void test_execution(bus &b, cpu &c) {
  run_instruction(c);
  check(c.reg(0) == 5, "MOVS r0, #5");

  run_instruction(c);
  check(c.reg(13) == 0x20004FFCu, "PUSH {r0} decrements SP by 4");
  check(b.read32(0x20004FFC) == 5u, "PUSH stored r0 in SRAM");

  run_instruction(c);
  check(c.reg(0) == 9, "MOVS r0, #9");

  run_instruction(c);
  check(c.reg(0) == 5, "POP {r0} restores r0");
  check(c.reg(13) == 0x20005000u, "POP restores SP");

  run_instruction(c);
  check(c.reg(14) == 0x08000015u, "BL sets LR with the Thumb bit");
  check(c.reg_pc() == 0x0800001Au, "BL branches to target");

  run_instruction(c);
  check(c.reg_pc() == 0x08000014u, "BX lr returns to LR");

  run_instruction(c);
  check(c.get_flag(cpu::Z), "CMP r0, #5 sets Z");

  run_instruction(c);
  check(c.reg_pc() == 0x0800001Cu, "BEQ taken");
  check(c.reg(2) == 0, "instruction after BEQ was skipped");
}

// Regression tests for the two ALU flag bugs:
//   - ADC_REG added apsr_flag::C (== 8) instead of the carry value (0/1)
//   - CMP_REG computed the V flag from the stale `imm` field instead of Rm
void test_alu_flags() {
  {
    bus b;
    cpu c(b);
    const uint8_t program[] = {
        0x05, 0x20, // 0x08000000 MOVS r0, #5
        0x03, 0x21, // 0x08000002 MOVS r1, #3
        0x48, 0x41, // 0x08000004 ADC  r0, r1   r0 = 5 + 3 + C
        0xFE, 0xE7, // 0x08000006 B .
    };
    b.load_flash(bus::kFlashBase, program, sizeof(program));
    c.set_pc(bus::kFlashBase);
    c.set_flag(cpu::C, true);

    run_instruction(c); // MOVS r0, #5
    run_instruction(c); // MOVS r1, #3
    run_instruction(c); // ADC r0, r1

    check(c.reg(0) == 9u, "ADC adds carry of 1 (not the C bit value 8)");
  }

  {
    bus b;
    cpu c(b);
    const uint8_t program[] = {
        0x01, 0x48,             // 0x08000000 ldr r0, [pc, #4]  -> 0x08000008
        0x02, 0x49,             // 0x08000002 ldr r1, [pc, #8]  -> 0x0800000C
        0x88, 0x42,             // 0x08000004 cmp r0, r1
        0xFE, 0xE7,             // 0x08000006 B .
        0xFF, 0xFF, 0xFF, 0x7F, // 0x08000008 r0 = 0x7FFFFFFF
        0xFF, 0xFF, 0xFF, 0xFF, // 0x0800000C r1 = 0xFFFFFFFF
    };
    b.load_flash(bus::kFlashBase, program, sizeof(program));
    c.set_pc(bus::kFlashBase);

    run_instruction(c); // ldr r0
    run_instruction(c); // ldr r1
    run_instruction(c); // cmp r0, r1

    check(c.reg(0) == 0x7FFFFFFFu && c.reg(1) == 0xFFFFFFFFu,
          "literal loads placed r0/r1");
    // 0x7FFFFFFF - 0xFFFFFFFF = 0x80000000 -> signed overflow, V must be 1.
    check(c.get_flag(cpu::V),
          "CMP register sets V on signed overflow (uses Rm)");
    check(c.get_flag(cpu::N), "CMP register sets N from the result");
    check(!c.get_flag(cpu::Z), "CMP register clears Z for a non-zero result");
  }

  {
    bus b;
    cpu c(b);
    const uint8_t program[] = {
        0x09, 0x22, // MOVS R2, #9
        0x07, 0x21, // MOVS R1, #7
        0x53, 0x1A, // SUBS R3, R2, R1 -> r3 = -2, N = 1
    };

    b.load_flash(bus::kFlashBase, program, sizeof(program));
    c.set_pc(bus::kFlashBase);

    run_instruction(c);
    run_instruction(c);
    run_instruction(c);

    check(c.reg(3) == 0x2, "check register 3 for positive result");

    check(c.get_flag(cpu::C), "SUBS register set C no-borrow");
    check(!c.get_flag(cpu::N), "SUBS register set N to false");
    check(!c.get_flag(cpu::Z), "SUBS regsiter set Z to false");
  }

  bus b;
  cpu c(b);
  const uint8_t program[] = {
      0x05, 0x22, // MOVS R2, #9
      0x07, 0x21, // MOVS R1, #7
      0x53, 0x1A, // SUBS R3, R2, R1 -> r3 = -2, N = 1
  };

  c.set_flag(cpu::C, false);

  b.load_flash(bus::kFlashBase, program, sizeof(program));
  c.set_pc(bus::kFlashBase);

  run_instruction(c);
  run_instruction(c);
  run_instruction(c);

  check(c.reg(3) == 0xFFFFFFFE, "check register 3 for negative result");
  check(!c.get_flag(cpu::C), "SUBS register set C borrow");
  check(c.get_flag(cpu::N), "SUBS register set N to true");
  check(!c.get_flag(cpu::Z), "SUBS register set Z to false");
}

void test_movw_movt_decode() {
  bus b;
  cpu c(b);
  const uint8_t program[] = {
      0x41, 0xF2, 0x34, 0x23, // movw r3, #0x1234
      0xC8, 0xF2, 0x00, 0x03, // movt r3, #0x8000
  };
  b.load_flash(bus::kFlashBase, program, sizeof(program));

  uint8_t len = 0;
  check(c.disassemble(bus::kFlashBase, len) == "$08000000: MOVW r3, #0x1234",
        "MOVW disassembles");
  check(len == 4, "MOVW is a 32-bit encoding");

  check(c.disassemble(bus::kFlashBase + 4, len) ==
            "$08000004: MOVT r3, #0x8000",
        "MOVT disassembles");
  check(len == 4, "MOVT is a 32-bit encoding");
}

void test_movw_movt_exec() {
  bus b;
  cpu c(b);
  const uint8_t program[] = {
      0x41, 0xF2, 0x34, 0x23, // 0x00 movw r3, #0x1234
      0xC8, 0xF2, 0x00, 0x03, // 0x04 movt r3, #0x8000
      0xFE, 0xE7,             // 0x08 B .
  };
  b.load_flash(bus::kFlashBase, program, sizeof(program));

  c.set_flag(cpu::N, true); // poison flags so a stray write is visible
  c.set_flag(cpu::Z, true);
  c.set_pc(bus::kFlashBase);

  run_instruction(c);
  check(c.reg(3) == 0x00001234u, "MOVW zero-extends imm16 into Rd");

  run_instruction(c);
  check(c.reg(3) == 0x80001234u, "MOVT writes Rd[31:16], keeps Rd[15:0]");

  check(c.get_flag(cpu::N), "MOVW did not disturb N");
  check(c.get_flag(cpu::Z), "MOVT did not disturb Z");
}

} // namespace

int main() {
  {
    bus b;
    test_memory_decode(b);
  }

  bus b;
  cpu c(b);
  b.load_flash(bus::kFlashBase, kProgram, sizeof(kProgram));

  test_reset(c);
  test_execution(b, c);
  test_alu_flags();
  test_movw_movt_decode();
  test_movw_movt_exec();

  std::printf("\n%d checks, %d failure(s)\n", checks, failures);
  return failures == 0 ? 0 : 1;
}
