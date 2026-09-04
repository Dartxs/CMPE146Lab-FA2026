#include <cstdint>
#include <cstdio>

#include "../hal/gpio.hpp"

// Peripheral Base Addresses:
// GPIOA = 0x400A0000, GPIOB = 0x400A2000
// IOMUX = 0x40428000

// GPIO Register Offsets:
// PWREN = 0x800, RSTCTL = 0x804,
// DOUTSET31_0 = 0x1290, DOUTCLR31_0 = 0x12A0,
// DOESET31_0 = 0x12D0, DOECLR31_0 = 0x12E0, DIN31_0 = 0x1380

// Needed IOMUX Register Offsets:
// SW1 = PA18 = PINCM40, offset 0xA0
// SW2 = PB21 = PINCM49, offset 0xC4
// BLUE = PB22 = PINCM50, offset 0xC8
// RED = PB26 = PINCM57, offset 0xE4
// GREEN = PB27 = PINCM58, offset 0xE8

// Needed GPIO Bit Mapping:
// PWREN -> bit 0 = enable, key = 0x26
// RSTCTL -> bit 0 = reset assert, key = 0x1
// note: key enables writing to the enable bits and goes into
// the highest byte of the reg

// Needed IOMUX Bit Mapping:
// PF = bits 0-5, controls which peripheral to select (like GPIO)
// PC = bit 7, allows connection to from mux to peripheral
// PIPD = bit 16, enables pulldown
// PIPU = bit 17, enables pullup
// INENA = bit 18, allows the peripheral to be read by MCU
// HIZ1 = bit 25, high impedance/open-drain

namespace {

enum class gpio_port : std::uint8_t
{
  a,
  b
};

struct gpio_pin
{

  gpio_port port;
  std::uint8_t pin;
  std::uint8_t pincm;
};

struct gpio_register_map
{

  std::uint32_t reserved0[0x800 / 4];

  volatile std::uint32_t PWREN;  //  offset 0x800

  volatile std::uint32_t RSTCTL;  //  offset 0x804

  std::uint32_t reserved1[(0x1290 - 0x808) / 4];

  volatile std::uint32_t DOUTSET31_0;  //  offset 0x1290

  std::uint32_t reserved2[3];

  volatile std::uint32_t DOUTCLR31_0;  //  offset 0x12A0

  std::uint32_t reserved3[0x2C / 4];

  volatile std::uint32_t DOESET31_0;  //  offset 0x12D0

  std::uint32_t reserved4[3];

  volatile std::uint32_t DOECLR31_0;  //  offset 0x12E0

  std::uint32_t reserved5[0x9C / 4];

  volatile std::uint32_t DIN31_0;  //  offset 0x1380
};

struct iomux_register_map
{

  std::uint32_t reserved0[0xA0 / 4];

  volatile std::uint32_t PINCM40;  //  offset 0xA0

  std::uint32_t reserved1[0x20 / 4];

  volatile std::uint32_t PINCM49;  //  offset 0xC4

  volatile std::uint32_t PINCM50;  //  offset 0xC8

  std::uint32_t reserved2[0x18 / 4];

  volatile std::uint32_t PINCM57;  //  offset 0xE4

  volatile std::uint32_t PINCM58;  //  offset 0xE8
};

constexpr std::uintptr_t gpio_a_base = 0x400A'0000;
constexpr std::uintptr_t gpio_b_base = 0x400A'2000;
constexpr std::uintptr_t iomux_base = 0x4042'8000;

constexpr std::uint32_t pwren_mask = 0x2600'0001;
constexpr std::uint32_t rstctl_mask = 0xB100'0001;

constexpr std::uint32_t pf_gpio_mask = 0x0000'0001;
constexpr std::uint32_t pc_mask = 0x0000'0080;
constexpr std::uint32_t pipd_mask = 0x0001'0000;
constexpr std::uint32_t pipu_mask = 0x0002'0000;
constexpr std::uint32_t inena_mask = 0x0004'0000;
constexpr std::uint32_t hiz1_mask = 0x0200'0000;

constexpr gpio_pin sw1_pin{ gpio_port::a, 18, 40 };
constexpr gpio_pin sw2_pin{ gpio_port::b, 21, 49 };
constexpr gpio_pin blue_led_pin{ gpio_port::b, 22, 50 };
constexpr gpio_pin red_led_pin{ gpio_port::b, 26, 57 };
constexpr gpio_pin green_led_pin{ gpio_port::b, 27, 58 };

auto* gpio_a = reinterpret_cast<gpio_register_map*>(gpio_a_base);
auto* gpio_b = reinterpret_cast<gpio_register_map*>(gpio_b_base);
auto* iomux = reinterpret_cast<iomux_register_map*>(iomux_base);

gpio_register_map* gpio(gpio_port p_port)
{

  if (p_port == gpio_port::a) {
    return gpio_a;
  }

  return gpio_b;
}

volatile std::uint32_t* pincm(std::uint8_t p_pincm)
{

  switch (p_pincm) {
    case 40:
      return &iomux->PINCM40;
    case 49:
      return &iomux->PINCM49;
    case 50:
      return &iomux->PINCM50;
    case 57:
      return &iomux->PINCM57;
    case 58:
      return &iomux->PINCM58;
    default:
      return nullptr;
  }
}

bool valid_pin(gpio_port p_port, std::uint8_t p_pin)
{

  if (p_port == gpio_port::a) {
    return (p_pin <= 31);
  }

  return (p_pin <= 27);
}

}  // namespace

class msp0g_outpin : public lab1::output_pin
{

public:
  msp0g_outpin(const gpio_pin& p_gpio_pin)
    : m_gpio_pin(p_gpio_pin)
  {
  }

private:
  bool driver_configure(const settings& p_settings) override;

  void driver_level(bool p_high) override;

  bool driver_level() override;

  gpio_pin m_gpio_pin;
};

class msp0g_inpin : public lab1::input_pin
{

public:
  msp0g_inpin(const gpio_pin& p_gpio_pin)
    : m_gpio_pin(p_gpio_pin)
  {
  }

private:
  bool driver_configure(const settings& p_settings) override;

  bool driver_level() override;

  gpio_pin m_gpio_pin;
};

bool msp0g_outpin::driver_configure(const settings& p_settings)
{

  auto* pincm_reg = pincm(m_gpio_pin.pincm);

  if ((!valid_pin(m_gpio_pin.port, m_gpio_pin.pin)) || (pincm_reg == nullptr)) {

    return false;
  }

  *pincm_reg = pf_gpio_mask | pc_mask | inena_mask;

  if (p_settings.resistor == lab1::pin_resistor::pull_down) {

    *pincm_reg |= pipd_mask;

  } else if (p_settings.resistor == lab1::pin_resistor::pull_up) {

    *pincm_reg |= pipu_mask;

  } else {

    *pincm_reg = *pincm_reg & ~pipu_mask & ~pipd_mask;
  }

  if (p_settings.open_drain) {
    *pincm_reg |= hiz1_mask;
  } else {
    *pincm_reg &= ~hiz1_mask;
  }

  return true;
}

void msp0g_outpin::driver_level(bool p_high)
{

  auto* gpio_reg = gpio(m_gpio_pin.port);

  std::uint32_t pin_mask = (1U << m_gpio_pin.pin);

  if (p_high) {
    gpio_reg->DOUTSET31_0 = pin_mask;
    gpio_reg->DOESET31_0 = pin_mask;
  } else {
    gpio_reg->DOUTCLR31_0 = pin_mask;
    gpio_reg->DOECLR31_0 = pin_mask;
  }
}

bool msp0g_outpin::driver_level()
{

  auto* gpio_reg = gpio(m_gpio_pin.port);

  std::uint32_t pin_mask = (1U << m_gpio_pin.pin);

  return (gpio_reg->DIN31_0 & pin_mask);
}

bool msp0g_inpin::driver_configure(const settings& p_settings)
{

  auto* pincm_reg = pincm(m_gpio_pin.pincm);

  if ((!valid_pin(m_gpio_pin.port, m_gpio_pin.pin)) || (pincm_reg == nullptr)) {

    return false;
  }

  *pincm_reg = pf_gpio_mask | pc_mask | inena_mask;

  if (p_settings.resistor == lab1::pin_resistor::pull_down) {

    *pincm_reg |= pipd_mask;

  } else if (p_settings.resistor == lab1::pin_resistor::pull_up) {

    *pincm_reg |= pipu_mask;

  } else {

    *pincm_reg = *pincm_reg & ~pipu_mask & ~pipd_mask;
  }

  return true;
}

bool msp0g_inpin::driver_level()
{

  auto* gpio_reg = gpio(m_gpio_pin.port);

  std::uint32_t pin_mask = (1U << m_gpio_pin.pin);

  return (gpio_reg->DIN31_0 & pin_mask);
}

int main()
{

  // TODO(lab1, step 2): Configure the RGB LED pins as outputs and the two
  // push-buttons (S1, S2) as inputs using the gpio:: driver you write in
  // gpio.cpp. See README.md for pin assignments and reference material.

  // TIP: the switches need pull resistor. See the user schematic of the launch
  // pad for which is needed for S1 and S2.

  gpio_a->RSTCTL = rstctl_mask;
  gpio_b->RSTCTL = rstctl_mask;

  gpio_a->PWREN = pwren_mask;
  gpio_b->PWREN = pwren_mask;

  for (volatile int i = 0; i < 16; i++)
    ;  // required delay after setting PWREN

  msp0g_inpin sw1{ sw1_pin };
  msp0g_inpin sw2{ sw2_pin };

  msp0g_outpin blue_led{ blue_led_pin };
  msp0g_outpin red_led{ red_led_pin };
  msp0g_outpin green_led{ green_led_pin };

  lab1::input_pin::settings sw1_settings{ lab1::pin_resistor::none };
  lab1::input_pin::settings sw2_settings;  // default = pullup

  lab1::output_pin::settings default_settings;

  sw1.configure(sw1_settings);
  sw2.configure(sw2_settings);

  blue_led.configure(default_settings);
  red_led.configure(default_settings);
  green_led.configure(default_settings);

  while (true) {
    // TODO(lab1, step 2): implement the button -> LED color behavior
    // described in README.md

    if (sw1.level() && !sw2.level()) {  // sw2 active low

      blue_led.level(true);
      red_led.level(true);
      green_led.level(true);

      for (volatile int i = 0; i < 1000000; i++)
        ;

      blue_led.level(false);
      red_led.level(false);
      green_led.level(false);

      for (volatile int i = 0; i < 1000000; i++)
        ;

    } else if (sw1.level()) {

      blue_led.level(false);
      red_led.level(true);
      green_led.level(true);

      for (volatile int i = 0; i < 1000000; i++)
        ;

      blue_led.level(false);
      red_led.level(false);
      green_led.level(false);

      for (volatile int i = 0; i < 1000000; i++)
        ;

    } else if (!sw2.level()) {

      blue_led.level(true);
      red_led.level(true);
      green_led.level(false);

      for (volatile int i = 0; i < 1000000; i++)
        ;

      blue_led.level(false);
      red_led.level(false);
      green_led.level(false);

      for (volatile int i = 0; i < 1000000; i++)
        ;

    } else {

      blue_led.level(false);
      red_led.level(false);
      green_led.level(false);
    }
  }
}
