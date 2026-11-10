#include "bus.h"
#include "cpu.h"

#include <cstdint>
#include <cstdio>
#include <vector>

// Reset/CRT test: a real vector table + a hand-written Reset_Handler that does
// the work gcc's startup code does, using only the opcodes this core decodes.
//
// Image layout (all in flash at 0x08000000):
//
//   0x08000000  word 0        initial SP
//   0x08000004  word 1        Reset_Handler | 1
//   0x08000008  words 2..75   Default_Handler | 1  (exceptions 2..15 + IRQ0..59)
//   0x08000130  Reset_Handler
//               ldr r0, =_sdata       copy .data  flash -> SRAM
//               ldr r1, =_edata
//               ldr r2, =_sidata
//             copy: ldr r3, [r2]
//               str r3, [r0]
//               adds r2, #4
//               adds r0, #4
//               cmp r0, r1
//               bne copy
//               ldr r0, =_sbss        zero .bss
//               ldr r1, =_ebss
//               movs r3, #0
//             zero: str r3, [r0]
//               adds r0, #4
//               cmp r0, r1
//               bne zero
//               bl main
//               b .                   main returned
//   0x08000158  main: bx lr
//   0x0800015C  literal pool (_sdata, _edata, _sidata, _sbss, _ebss)
//   0x08000170  .data initial values (2 words)
//   0x08000178  Default_Handler: b .
//
namespace {

int failures = 0;
int checks = 0;

void check(bool condition, const char *what) {
  ++checks;
  if (!condition)
    ++failures;
  std::printf("%s: %s\n", condition ? "PASS" : "FAIL", what);
}

void run_instruction(cpu &c) {
  c.clock();
  while (!c.complete())
    c.clock();
}

void put32(std::vector<uint8_t> &image, uint32_t address, uint32_t value) {
  for (int i = 0; i < 4; ++i)
    image[address + static_cast<uint32_t>(i)] =
        static_cast<uint8_t>((value >> (8 * i)) & 0xFF);
}

// Code blob, copied to flash at 0x08000130.
const uint8_t kCode[] = {
    // Reset_Handler
    0x0A, 0x48,                   // 0x130  ldr r0, =_sdata
    0x0B, 0x49,                   // 0x132  ldr r1, =_edata
    0x0B, 0x4A,                   // 0x134  ldr r2, =_sidata
    0x13, 0x68,                   // 0x136  ldr r3, [r2]
    0x03, 0x60,                   // 0x138  str r3, [r0]
    0x04, 0x32,                   // 0x13A  adds r2, #4
    0x04, 0x30,                   // 0x13C  adds r0, #4
    0x88, 0x42,                   // 0x13E  cmp  r0, r1
    0xF9, 0xD1,                   // 0x140  bne  0x136
    0x09, 0x48,                   // 0x142  ldr r0, =_sbss
    0x09, 0x49,                   // 0x144  ldr r1, =_ebss
    0x00, 0x23,                   // 0x146  movs r3, #0
    0x03, 0x60,                   // 0x148  str r3, [r0]
    0x04, 0x30,                   // 0x14A  adds r0, #4
    0x88, 0x42,                   // 0x14C  cmp  r0, r1
    0xFB, 0xD1,                   // 0x14E  bne  0x148
    0x00, 0xF0, 0x02, 0xF8,       // 0x150  bl main
    0xFE, 0xE7,                   // 0x154  b .
    0x00, 0xBF,                   // 0x156  nop (align literal pool)
    0x70, 0x47,                   // 0x158  main: bx lr
    0x00, 0xBF,                   // 0x15A  nop
    0x00, 0x00, 0x00, 0x20,       // 0x15C  _sdata  = 0x20000000
    0x08, 0x00, 0x00, 0x20,       // 0x160  _edata  = 0x20000008
    0x70, 0x01, 0x00, 0x08,       // 0x164  _sidata = 0x08000170
    0x08, 0x00, 0x00, 0x20,       // 0x168  _sbss   = 0x20000008
    0x0C, 0x00, 0x00, 0x20,       // 0x16C  _ebss   = 0x2000000C
    0x11, 0x11, 0x11, 0x11,       // 0x170  .data word 0
    0x22, 0x22, 0x22, 0x22,       // 0x174  .data word 1
    0xFE, 0xE7,                   // 0x178  Default_Handler: b .
};

constexpr uint32_t kVectorCount = 76; // SP + exception 1..75 (16 + 60 IRQs)
constexpr uint32_t kCodeBase = 0x08000130;
constexpr uint32_t kResetHandler = 0x08000130;
constexpr uint32_t kDefaultHandler = 0x08000178;
constexpr uint32_t kMainReturned = 0x08000154; // `b .` after `bl main`

std::vector<uint8_t> build_image() {
  std::vector<uint8_t> image(
      kCodeBase + sizeof(kCode) - bus::kFlashBase, 0x00);

  // Word 0: initial SP. Full SRAM on the C8 part is 20 KiB, so top is
  // 0x20005000.
  put32(image, 0x00, 0x20005000);
  // Word 1: Reset_Handler, Thumb bit set.
  put32(image, 0x04, kResetHandler | 1u);
  // Words 2..75: every other exception/IRQ vector.
  for (uint32_t i = 2; i < kVectorCount; ++i)
    put32(image, i * 4, kDefaultHandler | 1u);

  for (std::size_t i = 0; i < sizeof(kCode); ++i)
    image[kCodeBase - bus::kFlashBase + i] = kCode[i];

  return image;
}

} // namespace

int main() {
  bus b;
  cpu c(b);

  // Poison SRAM so a skipped .data copy / .bss zero is detectable.
  for (uint32_t a = 0x20000000; a < 0x20000010; ++a)
    b.write8(a, 0xEE);

  const std::vector<uint8_t> image = build_image();
  b.load_flash(bus::kFlashBase, image.data(), image.size());

  c.reset();

  check(c.reg(13) == 0x20005000u, "reset SP from vector table word 0");
  check(c.reg_pc() == kResetHandler, "reset PC from vector table word 1");

  // Run until main has returned and we land on the spin after `bl main`.
  int steps = 0;
  while (c.reg_pc() != kMainReturned && steps < 1000) {
    run_instruction(c);
    ++steps;
  }

  check(c.reg_pc() == kMainReturned, "Reset_Handler ran to completion");
  check(steps < 1000, "no runaway execution");
  check(c.reg(14) == (kMainReturned | 1u), "bl main set LR with Thumb bit");

  check(b.read32(0x20000000) == 0x11111111u,
        ".data word 0 copied flash -> SRAM");
  check(b.read32(0x20000004) == 0x22222222u,
        ".data word 1 copied flash -> SRAM");
  check(b.read32(0x20000008) == 0u, ".bss word zeroed");
  check(b.read32(0x2000000C) == 0xEEEEEEEEu,
        ".bss end is exclusive, word past it untouched");

  // Flash copies must still hold the original initial values.
  check(b.read32(0x08000170) == 0x11111111u, ".data image still in flash");

  std::printf("\n%d checks, %d failure(s)\n", checks, failures);
  return failures == 0 ? 0 : 1;
}
