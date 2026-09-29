#pragma once

#include <cstdint>

#include "../hal/timer.hpp"

namespace msp0g {

/**
 * @brief Set of gp timers that can be used for pwm
 *
 * timg0, timg6, timg7, timg8
 *
 * timg12 not available due to lack of prescaler
 *
 */
enum class pwm_timer : std::uint8_t
{
  timg0 = 0,
  timg6,
  timg7,
  timg8
};

/**
 * @brief Set of channels that can be used for pwm
 *
 * cc0, cc1
 *
 */
enum class pwm_timer_channel : std::uint8_t
{
  cc0 = 0,
  cc1
};

struct pwm_config
{

  std::uint32_t freq;
  pwm_timer timer;
  pwm_timer_channel cc;
};

class msp0g_steady_clock : public lab2::steady_clock
{

public:
  /**
   * @brief Constructor for steady clock using timg12.
   *
   * Only one instance should exist, creating another instance will invalidate
   * the previous.
   *
   */
  msp0g_steady_clock();

private:
  std::uint64_t m_uptime;
  std::uint32_t m_prev_count;

  std::uint32_t driver_frequency() override;
  std::uint64_t driver_uptime() override;
};

class msp0g_pwm : public lab2::pwm
{

public:
  /**
   * @brief Constructorr for pwm
   *
   * @param p_config Struct containing pwm config {frequency, timer, channel}.
   *
   * Subsequent instances of this object using the same timer needs to use the
   * same frequency and different channel (if the other channel isn't already
   * occupied). Subsequent instances breaking this rule will not work, but will
   * have no affect on the existing instance.
   *
   * Requested frequency (as passed in p_config) is not guaranteed due to
   * truncation when calculating the value to load into timer. In this case,
   * frequency will be higher than requested (lower load value = higher freq).
   * Also frequency math assumes BUSCLK = MCLK = ULPCLK = 32MHz.
   *
   * Duty cycle is at 0% by default.
   *
   */
  msp0g_pwm(const pwm_config& p_config);

private:
  std::uint32_t m_pwm_freq;
  std::uint32_t m_per_count;
  std::uintptr_t m_timer_base;
  std::uint8_t m_channel;
  bool m_valid;

  std::uint32_t driver_frequency() override;

  /**
   * @brief Function to modify pwm duty cycle
   *
   * @param p_duty_cycle value from 0-65535 with 0 = 0%, 65535 = 100%, etc.
   *
   * The requested value will get adjusted with respect to the load/max value
   * thus precision depends on the pwm frequency. Lower pwm frequency = higher
   * precision (load value closer to 65535)
   *
   *
   */
  void driver_duty_cycle(std::uint16_t p_duty_cycle) override;
};

}  // namespace msp0g
