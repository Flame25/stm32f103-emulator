# stm32f103-emulator

A work-in-progress STM32F103 (ARM Cortex-M3) emulator in C++20, with a
javidx9-style visual front-end.

## Status
Instruction-level emulation of a subset of the ARMv7-M Thumb instruction set.
Not yet capable of booting real firmware.

## Build
    cmake -S . -B build
    cmake --build build -j
    ./build/stm32emu

## Test
    cd build && ctest --output-on-failure

## Memory map
F103C8: flash 0x08000000 (64 KiB), SRAM 0x20000000 (20 KiB), boot alias at 0x0.

## Implemented
- Address-decoded bus with flash + SRAM, 0x0 boot alias
- Reset from vector table (SP/PC), VTOR-shaped
- Thumb-16: MOV, MOVS, ADD, SUB, ADC, AND, CMP, LDR/STR word, LDR literal,
  PUSH, POP, B, B<c>, BL, BX, BKPT
- Thumb-32: B (T4), B<c> (T3), BL

## Not implemented
MOVW/MOVT, LDR.W/STR.W, LDM/STM, byte/halfword access, shifts, exceptions,
NVIC, peripherals.

## References
- ARMv7-M Architecture Reference Manual (ARM DDI 0403E)
- STM32F103 Reference Manual (RM0008)

## License
MIT. Bundles olcPixelGameEngine (OLC-3) by javidx9.
