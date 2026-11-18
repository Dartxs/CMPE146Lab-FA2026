#pragma once

#include <cstdint>

#include "../../hal/adc.hpp"

/**
 * @brief DriverLib ADC14 driver for the external TMP36 on P6.1 (A14).
 *
 * One read() fills the caller's buffer with a single DMA transfer on channel
 * 7 and sleeps the calling task until DMA_INT1_IRQHandler wakes it.
 * The constructor leaves ADC14, REF_A, and the DMA channel ready to use.
 */
class msp432_adc_driver : public lab5::adc16_stream
{
public:
  msp432_adc_driver();

  msp432_adc_driver(msp432_adc_driver const&) = delete;
  msp432_adc_driver& operator=(msp432_adc_driver const&) = delete;

  /**
   * @brief Wake the task blocked in read() after DMA channel 7 completes.
   *
   * Called only from DMA_INT1_IRQHandler().
   */
  static void handle_interrupt();

private:
  void driver_read(std::span<std::uint16_t> p_buffer) override;
};

/**
 * @brief Convert one upscaled ADC sample to units of 0.1 degC.
 *
 * The ADC reference is 2.5 V and read() has already expanded the 14-bit code
 * to the 16-bit range, so the TMP36 voltage is
 * `2.5 V * p_upscaled / 65535`. The sensor outputs 500 mV at 0 degC and
 * rises by 10 mV per degC. 0.1 degC steps are finer than the required
 * 0.5 degC.
 *
 * @param p_upscaled 16-bit sample produced by msp432_adc_driver::read().
 * @return Temperature in units of 0.1 degC. 250 is 25.0 degC.
 */
constexpr std::int16_t tmp36_temperature_tenths(std::uint16_t p_upscaled)
{
  constexpr std::int64_t reference_microvolts = 2'500'000;
  constexpr std::int64_t full_scale = 65535;
  constexpr std::int64_t zero_celsius_microvolts = 500'000;
  constexpr std::int64_t microvolts_per_tenth = 1'000;

  std::int64_t const microvolts =
    static_cast<std::int64_t>(p_upscaled) * reference_microvolts / full_scale;

  return static_cast<std::int16_t>(
    (microvolts - zero_celsius_microvolts) / microvolts_per_tenth);
}
