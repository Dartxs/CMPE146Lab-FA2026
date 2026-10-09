#include <array>
#include <cmath>
#include <cstdint>

#include "msp0g_spi.hpp"

namespace {

/** @brief Track SPI peripherals already initialized during this program run. */
std::array<bool, 2> spi_init = { false, false };

constexpr std::uintptr_t spi0_base = 0x40468000;
constexpr std::uintptr_t spi1_base = 0x4046A000;

constexpr std::uintptr_t pwren_offset = 0x800;
constexpr std::uintptr_t rstctl_offset = 0x804;

// constexpr std::uint32_t clkdiv_offset = 0x1000;
constexpr std::uint32_t clksel_offset = 0x1004;
constexpr std::uintptr_t ctl0_offset = 0x1100;
constexpr std::uintptr_t ctl1_offset = 0x1104;
constexpr std::uintptr_t clkctl_offset = 0x1108;

constexpr std::uintptr_t stat_offset = 0x1110;

constexpr std::uintptr_t rxdata_offset = 0x1130;
constexpr std::uintptr_t txdata_offset = 0x1140;

constexpr std::uint32_t pwren_mask = 0x2600'0001;
constexpr std::uint32_t rstctl_mask = 0xB100'0001;

// clksel
constexpr std::uint32_t sysclk_mask = (0x1 << 3);

// ctl0
constexpr std::uint32_t modes_mask = (0b11 << 8);
constexpr std::uint32_t modes[4] = { (0b00 << 8),
                                     (0b10 << 8),
                                     (0b01 << 8),
                                     (0b11 << 8) };
constexpr std::uint32_t data_size_8_mask = (0x7);

// ctl1
constexpr std::uint32_t msb_mask = (0x1 << 4);
constexpr std::uint32_t CP_mask = (0x1 << 2);
constexpr std::uint32_t ena_mask = 0x1;

// stat
constexpr std::uint32_t busy_mask = (0x1 << 4);
constexpr std::uint32_t tnf_mask = (0x1 << 1);  // transfer FIFO not full
constexpr std::uint32_t rfe_mask = (0x1 << 2);  // receive FIFO empty

volatile std::uint32_t* reg(std::uintptr_t p_base, std::uintptr_t p_offset)
{

  return reinterpret_cast<volatile uint32_t*>(p_base + p_offset);
}

void initialize_spi(std::uintptr_t p_base)
{

  auto* pwren = reg(p_base, pwren_offset);
  auto* rstctl = reg(p_base, rstctl_offset);

  *rstctl = rstctl_mask;
  *pwren = pwren_mask;

  for (volatile int i = 0; i < 16; i++)
    ;
}

}  // namespace

namespace msp0g {

msp0g_spi::msp0g_spi(const settings& p_settings, spix p_spix)
{

  if (spi_init[static_cast<uint8_t>(p_spix)] ||
      (p_settings.clock_rate < 15625)) {
    m_valid = false;
    return;
  }

  if (p_spix == spix::spi0) {
    m_spi_base = spi0_base;
  } else {
    m_spi_base = spi1_base;
  }

  initialize_spi(m_spi_base);

  auto* clksel = reg(m_spi_base, clksel_offset);
  auto* ctl0 = reg(m_spi_base, ctl0_offset);
  auto* ctl1 = reg(m_spi_base, ctl1_offset);

  *clksel = sysclk_mask;
  *ctl0 = data_size_8_mask;
  *ctl1 = msb_mask | CP_mask;

  m_valid = true;
  driver_configure(p_settings);

  *ctl1 |= ena_mask;

  spi_init[static_cast<uint8_t>(p_spix)] = true;
}

bool msp0g_spi::driver_configure(const settings& p_settings)
{

  if (!m_valid || (p_settings.clock_rate < 15625)) {
    return false;
  }

  auto* ctl0 = reg(m_spi_base, ctl0_offset);
  auto* clkctl = reg(m_spi_base, clkctl_offset);

  *ctl0 &= ~modes_mask;
  *ctl0 |= modes[static_cast<uint8_t>(p_settings.bus_mode)];

  /*
  SCR calculation:

  clkrate = sclk/((SCR+1)*2)
  min clkrate = SCR = 1023 => 15.625 kHz
  max clkrate = SCR = 0 => 16 MHz
  */
  uint32_t clkrate = p_settings.clock_rate;
  std::uint32_t SCR;

  if (clkrate > 16'000'000) {

    SCR = 0;

  } else {

    // ceil needed to ensure act clkrate <= req clkrate (higher SCR = lower
    // clkrate)

    SCR = static_cast<std::uint32_t>(
      std::ceil((32'000'000.0 / (static_cast<double>(clkrate) * 2.0)) - 1.0));
  }

  *clkctl = SCR;

  return true;
}
void msp0g_spi::driver_transfer(std::span<const std::uint8_t> p_data_out,
                                std::span<std::uint8_t> p_data_in,
                                std::uint8_t p_filler)
{

  if (!m_valid) {
    return;
  }

  std::size_t loop_size = (p_data_out.size() > p_data_in.size())
                            ? p_data_out.size()
                            : p_data_in.size();

  auto* stat = reg(m_spi_base, stat_offset);
  auto* rxdata = reg(m_spi_base, rxdata_offset);
  auto* txdata = reg(m_spi_base, txdata_offset);

  for (std::size_t i = 0; i < loop_size; i++) {

    while ((*stat & tnf_mask) == 0)
      ;

    if (i < p_data_out.size()) {

      *txdata = p_data_out[i];

    } else {

      *txdata = p_filler;
    }

    while (*stat & rfe_mask)
      ;

    if (i < p_data_in.size()) {

      p_data_in[i] = static_cast<uint8_t>(*rxdata);

    } else {

      (void)*rxdata;
    }
  }

  while (*stat & busy_mask)
    ;
}

}  // namespace msp0g
