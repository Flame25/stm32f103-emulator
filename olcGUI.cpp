#define OLC_PGE_APPLICATION
#include "olcPixelGameEngine.h"

#include "bus.h"
#include "cpu.h"

#include <cstdint>
#include <map>
#include <string>

// Visualization front-end ported from javidx9's olcNES "DrawCpu / DrawRam /
// DrawCode" panels, retargeted from the 6502 to this Cortex-M3 core.
class Demo_stm32 : public olc::PixelGameEngine {
public:
  Demo_stm32() { sAppName = "stm32f103 Emulator - CPU"; }

  bool OnUserCreate() override {
    loadTestProgram();
    m_asm = m_cpu.disassemble(0x0000, sizeof(bus::flash) - 1);
    m_cpu.reset();
    return true;
  }

  bool OnUserUpdate(float fElapsedTime) override {
    (void)fElapsedTime;
    Clear(olc::DARK_BLUE);

    if (GetKey(olc::Key::SPACE).bPressed) {
      do {
        m_cpu.clock();
      } while (m_cpu.reg_sp_cycles() != 0);
    }

    if (GetKey(olc::Key::R).bPressed)
      m_cpu.reset();

    DrawRam(2, 2, 0x0000, 8, 8);
    DrawCpu(300, 2);
    DrawCode(300, 100, 20);
    DrawOutput(2, 315, 674, 100);
    DrawString(2, 110, "SPACE = Step Instruction  R = RESET");

    return true;
  }

private:
  bus m_bus;
  cpu m_cpu{m_bus};
  std::map<uint32_t, std::string> m_asm;

  // 0x01, 0xD0,          // 0x0002: BEQ  +2

  void loadTestProgram() {
    const uint8_t program[] = {
        0x02, 0x48, // 0x0000: LDR  R0, [PC, #8]
        0x07, 0x21, // 0x0004: MOVS R1, #7
        0x08, 0x22, // MOVS R2, #3
        0x08, 0x23, // MOVS R3, #3
        0xC8, 0x1C, // 0x0006: ADDS R0, R1, #3
        0xBF, 0x21, // MOVS R1, #191
        0xBF, 0x22, // MOVS R2, #191
        0x88, 0x18, // ADDS R0, R1, R2
        0x58, 0x60, // STR R0, [R3, #4]
        0x0A, 0x20, // MOVS R0, #10
        0x88, 0x1E, // SUBS R0, R0, #3
        0xF6, 0xE7, // B (Uncond)
        0xAB, 0xBE, // BKPT #0xAB
    };
    m_bus.load_flash(0x0000, program, sizeof(program));
  }

  static std::string hex(uint32_t n, int d) {
    std::string s(static_cast<std::size_t>(d), '0');
    for (int i = d - 1; i >= 0; --i, n >>= 4)
      s[static_cast<std::size_t>(i)] = "0123456789ABCDEF"[n & 0xF];
    return s;
  }

  void DrawRam(int x, int y, uint32_t nAddr, int nRows, int nColumns) {
    int nRamY = y;
    for (int row = 0; row < nRows; ++row) {
      std::string sOffset = "$" + hex(nAddr, 8) + ":";
      for (int col = 0; col < nColumns; ++col) {
        sOffset += " " + hex(m_bus.read8(nAddr), 2);
        ++nAddr;
      }
      DrawString(x, nRamY, sOffset);
      nRamY += 10;
    }
  }

  void DrawOutput(int x, int y, int w, int h) {
    DrawRect(x, y, w, h, olc::WHITE);
    DrawString(x + 4, y + 4, "Semihosting Output", olc::YELLOW);
    DrawLine(x, y + 14, x + w, y + 14, olc::WHITE);

    const int visible_lines = 8;
    const int total = static_cast<int>(m_bus.m_out.size());

    // Show only the latest 5 lines
    const int first = std::max(0, total - visible_lines);

    int ly = y + 18;

    for (int i = first; i < total; ++i) {
      olc::Pixel color = (i == total - 1) ? olc::GREEN : olc::WHITE;

      DrawString(x + 4, ly, m_bus.m_out[i], color);
      ly += 10;
    }
  }

  void DrawCpu(int x, int y) {
    DrawString(x, y, "STATUS:", olc::WHITE);
    const char *names = "NZCVQ";
    for (int i = 0; i < 5; ++i) {
      DrawString(x + 64 + i * 16, y, std::string(1, names[i]),
                 m_cpu.flag(names[i]) ? olc::GREEN : olc::RED);
    }

    DrawString(x, y + 10, "PC: $" + hex(m_cpu.reg_pc(), 8));
    DrawString(x, y + 20, "CYCLES: $" + hex(m_cpu.reg_sp_cycles(), 8));
    DrawString(x, y + 30, "COUNT: $" + hex(m_cpu.reg_clock_count(), 8));
    for (int i = 0; i < 13; ++i) {
      DrawString(x, y + 40 + i * 10,
                 "R" + std::to_string(i) + ": $" + hex(m_cpu.reg(i), 8) +
                     "  [" + std::to_string(m_cpu.reg(i)) + "]");
    }
    DrawString(x, y + 170, "SP: $" + hex(m_cpu.reg(13), 8));
    DrawString(x, y + 180, "LR: $" + hex(m_cpu.reg(14), 8));
  }

  void DrawCode(int x, int y, int nLines) {
    auto it = m_asm.find(m_cpu.reg_pc());
    int nLineY = (nLines >> 1) * 10 + y;
    if (it == m_asm.end())
      return;

    DrawString(x, nLineY, (*it).second, olc::CYAN);
    while (nLineY < (nLines * 10) + y && ++it != m_asm.end()) {
      nLineY += 10;
      DrawString(x, nLineY, (*it).second);
    }

    it = m_asm.find(m_cpu.reg_pc());
    nLineY = (nLines >> 1) * 10 + y;
    while (nLineY > y && --it != m_asm.end()) {
      nLineY -= 10;
    }
  }
};

int main() {
  Demo_stm32 demo;
  if (demo.Construct(680, 420, 2, 2))
    demo.Start();
  return 0;
}
