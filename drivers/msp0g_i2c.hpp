#pragma once

#include <cstdint>

#include "../hal/i2c.hpp"

namespace msp0g {

/**
 * @brief Available MSPM0 I2C peripheral instances.
 */
enum class i2cx : std::uint8_t
{

  i2c0 = 0,
  i2c1

};

/**
 * @brief Polling I2C controller driver for an MSPM0G peripheral.
 *
 * Supports 7-bit target addresses and write-only, read-only, or combined
 * write/read transactions.
 */
class msp0g_i2c : public lab4::i2c
{

public:
  /**
   * @brief Construct and initialize an I2C controller.
   *
   * @param p_settings Initial controller settings.
   * @param p_i2cx I2C peripheral instance to use.
   *
   * @note Construction aborts if the requested clock rate is invalid or the
   * selected peripheral has already been initialized.
   */
  msp0g_i2c(const settings& p_settings, i2cx p_i2cx);

private:
  std::uintptr_t m_i2c_base;

  /**
   * @brief Update the controller clock rate.
   *
   * @param p_settings Requested controller settings.
   * @return true if the settings were valid and applied; otherwise false.
   */
  bool driver_configure(const settings& p_settings) override;

  /**
   * @brief Perform a polling transaction with a 7-bit I2C target.
   *
   * When both spans contain data, the write phase is followed by a repeated
   * START and a read phase. An empty span omits its corresponding phase.
   *
   * @param p_address Seven-bit target address.
   * @param p_data_out Bytes to transmit.
   * @param p_data_in Buffer that receives bytes from the target.
   * @return true when every requested phase completes without a controller
   * error; otherwise false.
   */
  bool driver_transaction(std::uint8_t p_address,
                          std::span<const std::uint8_t> p_data_out,
                          std::span<std::uint8_t> p_data_in) override;

  /**
   * @brief Apply previously validated settings while the controller is idle.
   *
   * @param p_settings Settings used to calculate the controller timer period.
   */
  void apply_config(const settings& p_settings);
};

}  // namespace msp0g
