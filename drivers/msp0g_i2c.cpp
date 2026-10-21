#include <array>
#include <cstdint>
#include <cstdlib>

#include "msp0g_i2c.hpp"

namespace {

constexpr std::array<std::uintptr_t, 2> i2c_base = { 0x400F0000, 0x400F2000 };

constexpr std::uintptr_t pwren_offset = 0x800;
constexpr std::uintptr_t rstctl_offset = 0x804;

constexpr std::uintptr_t clksel_offset = 0x1004;    // bit 3 = busclk
constexpr std::uintptr_t gfctl_offset = 0x1200;     // glitch filter
constexpr std::uintptr_t csa_offset = 0x1210;       // controller target address
constexpr std::uintptr_t cctr_offset = 0x1214;      // controller ctrl
constexpr std::uintptr_t csr_offset = 0x1218;       // controller status
constexpr std::uintptr_t crxdata_offset = 0x121C;   // controller rxdata
constexpr std::uintptr_t ctxdata_offset = 0x1220;   // controller txdata
constexpr std::uintptr_t ctpr_offset = 0x1224;      // controller timer period
constexpr std::uintptr_t ccr_offset = 0x1228;       // controller config
constexpr std::uintptr_t cfifictl_offset = 0x1238;  // controller fifo ctrl
constexpr std::uintptr_t cfifosr_offset = 0x123c;   // controller fifo stat

constexpr std::uintptr_t cpuint_ris_offset = 0x1030;
constexpr std::uintptr_t cpuint_iclr_offset = 0x1048;

constexpr std::uint32_t pwren_mask = 0x2600'0001;
constexpr std::uint32_t pwren_enable_mask = 0x1;
constexpr std::uint32_t rstctl_mask = 0xB100'0001;

constexpr std::uint32_t i2c_clk = 32'000'000;

// clksel
constexpr std::uint32_t busclk_mask = (0x1 << 3);

// controller target address
constexpr std::uint32_t receive_mask = 0x1;

// controller control
constexpr std::uint32_t cblen_shift = 16;
constexpr std::uint32_t stop_mask = (0x1 << 2);
constexpr std::uint32_t start_mask = (0x1 << 1);
constexpr std::uint32_t burstrun_mask = 0x1;

// controller status
constexpr std::uint32_t busbsy_mask = (0x1 << 6);
constexpr std::uint32_t idle_mask = (0x1 << 5);
constexpr std::uint32_t err_mask = (0x1 << 1);
constexpr std::uint32_t busy_mask = 0x1;

// controller config
constexpr std::uint32_t active_mask = 0x1;

// controller fifo control
constexpr std::uint32_t tx_flush_mask = (0x1 << 7);
constexpr std::uint32_t rx_flush_mask = (0x1 << 15);

// controller fifo stat
constexpr std::uint32_t txfifocnt_shift = 8;
constexpr std::uint32_t txfifocnt_mask = (0xF << txfifocnt_shift);
constexpr std::uint32_t rxfifocnt_shift = 0;
constexpr std::uint32_t rxfifocnt_mask = (0xF << rxfifocnt_shift);

// ris & iclr
constexpr std::uint32_t carblost_mask = (0x1 << 10);
constexpr std::uint32_t cnack_mask = (0x1 << 7);
constexpr std::uint32_t ctxfifotrg_mask = (0x1 << 3);
constexpr std::uint32_t crxfifotrg_mask = (0x1 << 2);
constexpr std::uint32_t ctxdone_mask = (0x1 << 1);
constexpr std::uint32_t crxdone_mask = 0x1;

volatile std::uint32_t* reg(std::uintptr_t p_base, std::uintptr_t p_offset)
{

  return reinterpret_cast<volatile uint32_t*>(p_base + p_offset);
}

/**
 * @brief Reset and power an I2C peripheral that is not already initialized.
 *
 * @param p_base Base address of the selected I2C peripheral.
 * @return true when initialization was performed; false if the peripheral was
 * already powered.
 */
bool initialize_i2c(std::uintptr_t p_base)
{

  auto* pwren = reg(p_base, pwren_offset);
  auto* rstctl = reg(p_base, rstctl_offset);

  if ((*pwren & pwren_enable_mask) != 0) {

    return false;
  }

  *rstctl = rstctl_mask;
  *pwren = pwren_mask;

  for (volatile int i = 0; i < 16; i++)
    ;

  return true;
}

/**
 * @brief Check whether a clock rate has an exact hardware divider.
 *
 * @param freq Requested I2C bus clock in hertz.
 * @return true when the rate is within the supported range and divides the
 * peripheral clock exactly; otherwise false.
 */
bool check_valid_freq(std::uint32_t freq)
{

  // Highest TPR value of 127 results in a i2c_freq of 25kHz
  // Lowest TPR value of 0 results in i2c_freq of 3.2 MHz
  // But, i2c_clk >= (20 * i2c_freq) so max freq = 1.6Mhz using
  // 32MHz clk

  if ((freq < 25'000) || (freq > 1'600'000)) {

    return false;
  }

  std::uint32_t denominator = freq * 10;

  // requested freq has to match exactly
  if ((i2c_clk % denominator) != 0) {

    return false;
  }

  return true;
}

}  // namespace

namespace msp0g {

msp0g_i2c::msp0g_i2c(const settings& p_settings, i2cx p_i2cx)
{

  m_i2c_base = i2c_base[static_cast<std::uint8_t>(p_i2cx)];

  if (!check_valid_freq(p_settings.clock_rate) || !initialize_i2c(m_i2c_base)) {

    std::abort();
  }

  auto* clksel = reg(m_i2c_base, clksel_offset);

  *clksel = busclk_mask;

  apply_config(p_settings);
}

bool msp0g_i2c::driver_configure(const settings& p_settings)
{

  if (!check_valid_freq(p_settings.clock_rate)) {

    return false;
  }

  apply_config(p_settings);

  return true;
}

bool msp0g_i2c::driver_transaction(std::uint8_t p_address,
                                   std::span<const std::uint8_t> p_data_out,
                                   std::span<std::uint8_t> p_data_in)
{

  if (p_address > 0x7F)
    return false;  // 7-bit addressing
  if (p_data_out.size() > 0xFFF || p_data_in.size() > 0xFFF)
    return false;
  if (p_data_out.empty() && p_data_in.empty())
    return false;

  auto* csr = reg(m_i2c_base, csr_offset);

  while (((*csr & idle_mask) == 0) || ((*csr & busbsy_mask) != 0))
    ;

  auto* csa = reg(m_i2c_base, csa_offset);
  auto* cctr = reg(m_i2c_base, cctr_offset);
  auto* crxdata = reg(m_i2c_base, crxdata_offset);
  auto* ctxdata = reg(m_i2c_base, ctxdata_offset);
  auto* cfifoctl = reg(m_i2c_base, cfifictl_offset);
  auto* cfifosr = reg(m_i2c_base, cfifosr_offset);
  auto* ris = reg(m_i2c_base, cpuint_ris_offset);
  auto* iclr = reg(m_i2c_base, cpuint_iclr_offset);

  *csa = p_address << 1;  // target address at bits 10-1

  *cfifoctl |= tx_flush_mask | rx_flush_mask;

  while ((((*cfifosr & txfifocnt_mask) >> txfifocnt_shift) != 8) ||
         (((*cfifosr & rxfifocnt_mask) >> rxfifocnt_shift) != 0))
    ;

  *cfifoctl &= ~(tx_flush_mask | rx_flush_mask);

  if (!p_data_out.empty()) {

    std::uint32_t tx_config =
      (static_cast<uint32_t>(p_data_out.size()) << cblen_shift) | start_mask |
      burstrun_mask;

    if (p_data_in.empty()) {
      tx_config |= stop_mask;
    }

    *csa &= ~receive_mask;

    std::size_t index = 0;

    for (; (index < 8 && index < p_data_out.size()); index++) {

      *ctxdata = p_data_out[index];
    }

    *iclr = carblost_mask | cnack_mask | ctxfifotrg_mask | ctxdone_mask;

    *cctr = tx_config;

    while (index < p_data_out.size()) {

      while ((*ris & ctxfifotrg_mask) == 0) {
        if ((*csr & err_mask) != 0) {

          return false;
        }
      }

      if ((*csr & err_mask) != 0) {

        return false;
      }

      for (std::size_t i = 0; (i < 8 && index < p_data_out.size()); i++) {

        *ctxdata = p_data_out[index++];
      }

      *iclr = ctxfifotrg_mask;
    }

    while ((*ris & ctxdone_mask) == 0) {
      if (((*csr & err_mask) != 0) ||
          ((*ris & (cnack_mask | carblost_mask)) != 0)) {

        return false;
      }
    }

    if ((*csr & err_mask) != 0) {

      return false;
    }

    *iclr = ctxdone_mask;
  }

  if (!p_data_in.empty()) {

    *csa |= receive_mask;
    *iclr = crxfifotrg_mask | crxdone_mask;

    *cctr = (static_cast<uint32_t>(p_data_in.size()) << cblen_shift) |
            stop_mask | start_mask | burstrun_mask;

    std::size_t index = 0;
    while (index < p_data_in.size()) {

      while ((*ris & crxfifotrg_mask) == 0) {  // while rxfifo is empty

        if ((*csr & err_mask) != 0) {
          return false;
        }
      }

      p_data_in[index++] = static_cast<std::uint8_t>(*crxdata);
      *iclr = crxfifotrg_mask;
    }

    while ((*ris & crxdone_mask) == 0) {
      if ((*csr & err_mask) != 0) {

        return false;
      }
    }

    if ((*csr & err_mask) != 0) {

      return false;
    }

    *iclr = crxdone_mask;
  }

  return true;
}

void msp0g_i2c::apply_config(const settings& p_settings)
{

  auto* csr = reg(m_i2c_base, csr_offset);

  while ((*csr & idle_mask) == 0)
    ;

  auto* ccr = reg(m_i2c_base, ccr_offset);
  auto* ctpr = reg(m_i2c_base, ctpr_offset);

  *ccr &= ~active_mask;

  /*
  CTPR.TPR calculation:

  i2c_freq = i2c_clk / ((1+TPR) * (SCL_LP + SCL_HP))
  TPR = (i2c_clk / (i2c_freq * (SCL_LP + SCL_HP))) - 1

  SCL_LP fixed at 6, SCL_HP fixed at 4,
  i2c_clk = 32MHz with BUSSCLK (ULPCLK because i2c is in PD0)
  i2c_freq = target_freq

  TPR = (32,000,000 / (targ_freq * (6 + 4))) - 1

  */

  *ctpr = (i2c_clk / (p_settings.clock_rate * 10)) - 1;

  *ccr |= active_mask;
}
}  // namespace msp0g
