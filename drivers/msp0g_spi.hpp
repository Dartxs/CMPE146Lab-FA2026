#pragma once

#include <cstdint>

#include "../hal/spi.hpp"

namespace msp0g {

/** @brief Select the SPI peripheral used by the driver. */
enum class spix : std::uint8_t
{
  spi0 = 0,
  spi1
};

/**
 * @brief Provide blocking, polled SPI controller transfers on SPI0 or SPI1.
 *
 * Transfers use 8-bit words, MSB first. Configure SCLK, PICO, and POCI
 * through IOMUX separately; the caller controls GPIO chip select.
 * Clock calculations assume a 32 MHz SYSCLK.
 */
class msp0g_spi : public lab3::spi
{

public:
  /**
   * @brief Reset, power, configure, and enable the selected SPI peripheral.
   *
   * @param p_settings Initial clock rate and SPI bus mode.
   * @param p_spix Peripheral to initialize: SPI0 or SPI1.
   * @note If the peripheral was already initialized by this driver, or
   *       the requested clock is below 15,625 Hz, this object is invalid:
   *       configuration returns false and transfers do nothing.
   *       Successful initialization reserves the peripheral for the rest
   *       of the program run.
   */
  msp0g_spi(const settings& p_settings, spix p_spix);

private:
  std::uintptr_t m_spi_base;
  bool m_valid;

  /**
   * @brief Apply the SPI bus mode and calculate the serial clock divider.
   *
   * @param p_settings Requested clock rate and SPI bus mode.
   * @return True if applied; false if the object is invalid or the
   *         requested clock is below 15,625 Hz.
   * @note Rates above 16 MHz are clamped to 16 MHz. Other accepted rates
   *       are rounded down to a realizable rate, assuming 32 MHz SYSCLK.
   */
  bool driver_configure(const settings& p_settings) override;
  /**
   * @brief Exchange bytes through the TX/RX FIFOs and wait until SPI is idle.
   *
   * @param[in] p_data_out Bytes to send; an empty span sends only filler.
   * @param[out] p_data_in Buffer for received bytes; an empty span discards
   *                      all received bytes.
   * @param p_filler Byte sent after the outgoing span is exhausted.
   * @note Transfers the larger span size, saving received bytes only
   *       while the incoming span has room. Every transmitted byte's RX
   *       entry is read, even when discarded. Chip select is not changed.
   *       Invalid objects return immediately; FIFO polling has no timeout.
   */
  void driver_transfer(std::span<const std::uint8_t> p_data_out,
                       std::span<std::uint8_t> p_data_in,
                       std::uint8_t p_filler) override;
};

}  // namespace msp0g
