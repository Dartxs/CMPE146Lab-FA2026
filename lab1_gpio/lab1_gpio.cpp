#include <cstdio>

#include "../hal/gpio.hpp"

// Peripheral Base Addresses:
// GPIOA = 0x400A0000, GPIOB = 0x400A2000
// IOMUX = 0x40428000

// GPIO Register Offsets:
// DOUTSET31_0 = 0x1290, DOUTCLR31_0 = 0x12A0, DOESET31_0 = 0x12D0,
// DOECLR31_0 = 0x12E0, DIN31_0 = 0x1380

// Needed IOMUX Register Offsets:
// SW1 = PA18 = PINCM40, offset 0xA0
// SW2 = PB21 = PINCM49, offset 0xC4
// BLUE = PB22 = PINCM50, offset 0xC8
// RED = PB26 = PINCM57, offset 0xE4
// GREEN = PB27 = PINCM58, offset 0xE8

// Needed IOMUX Bit Mapping:
// PF = bits 0-5, controls which peripheral to select (like GPIO)
// PC = bit 7, allows pin to drive peripheral
// PIPD = bit 16, enables pulldown
// PIPU = bit 17, enables pullup
// INENA = bit 18, allows pin to read input
// HIZ1 = bit 25, high impedance/open-drain

namespace {

struct gpio_register_map
{

  std::uint32_t reserved0[0x1290 / 4];

  std::uint32_t volatile DOUTSET31_0;  //  offset 0x1290

  std::uint32_t reserved1[3];

  std::uint32_t volatile DOUTCLR31_0;  //  offset 0x12A0

  std::uint32_t reserved2[0x2C / 4];

  std::uint32_t volatile DOESET31_0;  //  offset 0x12D0

  std::uint32_t reserved3[3];

  std::uint32_t volatile DOECLR31_0;  //  offset 0x12E0

  std::uint32_t reserved4[0x9C / 4];

  std::uint32_t volatile DIN31_0;  //  offset 0x1380
};

struct iomux_register_map
{

  std::uint32_t reserved0[0xA0 / 4];

  std::uint32_t volatile PINCM40;  //  offset 0xA0

  std::uint32_t reserved1[0x20 / 4];

  std::uint32_t volatile PINCM49;  //  offset 0xC4

  std::uint32_t volatile PINCM50;  //  offset 0xC8

  std::uint32_t reserved2[0x18 / 4];

  std::uint32_t volatile PINCM57;  //  offset 0xE4

  std::uint32_t volatile PINCM58;  //  offset 0xE8
};

constexpr std::uintptr_t gpio_a_base = 0x400A'0000;
constexpr std::uintptr_t gpio_b_base = 0x400A'2000;
constexpr std::uintptr_t iomux_base = 0x4042'8000;

}  // namespace

int main()
{
  std::printf("Hello, World\n");

  // TODO(lab1, step 2): Configure the RGB LED pins as outputs and the two
  // push-buttons (S1, S2) as inputs using the gpio:: driver you write in
  // gpio.cpp. See README.md for pin assignments and reference material.

  // TIP: the switches need pull resistor. See the user schematic of the launch
  // pad for which is needed for S1 and S2.

  while (true) {
    // TODO(lab1, step 2): implement the button -> LED color behavior
    // described in README.md
  }
}
