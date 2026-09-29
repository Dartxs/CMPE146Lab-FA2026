#include <cstdint>

#include "msp0g_iomux.hpp"

namespace {
constexpr std::uintptr_t iomux_base = 0x4042'8000;

constexpr std::uint32_t pc_mask = 1U << 7;
constexpr std::uint32_t pipd_mask = 1U << 16;
constexpr std::uint32_t pipu_mask = 1U << 17;
constexpr std::uint32_t inena_mask = 1U << 18;
constexpr std::uint32_t hiz1_mask = 1U << 25;

// NOLINTNEXTLINE(performance-no-int-to-ptr): MMIO register access
auto* iomux_registers =
  reinterpret_cast<volatile std::uint32_t*>(iomux_base + 0x4);

// iomux_registers[x] corresponds to PINCM(x+1), this helpers hides the indexing
// issue and ensures PINCM is within boundary
volatile std::uint32_t* pincm(std::uint8_t p_pincm)
{

  if (p_pincm < 1U || p_pincm > 60U) {
    return nullptr;
  }

  return &iomux_registers[p_pincm - 1U];
}

}  // namespace

namespace msp0g {

msp0g_iomux::msp0g_iomux(const iomux_config& p_iomux_config,
                         const iomux_settings& p_iomux_settings)
{

  m_pincm = p_iomux_config.pincm;
  m_pf_sel = p_iomux_config.pf_sel;

  if (m_pincm > 58 || m_pf_sel > 0x3F) {

    m_valid = false;
    return;
  }

  auto* pincm_reg = pincm(m_pincm);

  *pincm_reg = m_pf_sel | pc_mask;

  if (static_cast<bool>(p_iomux_config.inena)) {

    *pincm_reg |= inena_mask;
  }

  m_valid = true;

  config(p_iomux_settings);
}

void msp0g_iomux::config(const iomux_settings& p_iomux_settings)
{

  if (m_valid) {

    auto* pincm_reg = pincm(m_pincm);

    if (p_iomux_settings.resistor == lab1::pin_resistor::pull_down) {

      *pincm_reg |= pipd_mask;

    } else if (p_iomux_settings.resistor == lab1::pin_resistor::pull_up) {

      *pincm_reg |= pipu_mask;

    } else {

      *pincm_reg = *pincm_reg & ~pipu_mask & ~pipd_mask;
    }

    if (p_iomux_settings.open_drain) {

      *pincm_reg |= hiz1_mask;

    } else {

      *pincm_reg &= ~hiz1_mask;
    }
  }
}

bool msp0g_iomux::valid_pincm()
{

  return m_valid;
}

}  // namespace msp0g
