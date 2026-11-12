#pragma once

// cspell:ignore SYSOSC VDDA TSTRIM SLASEX SLAU DMASZ

// =============================================================================
// Role 1 (ADC): DMA-backed driver for the on-chip temperature sensor, plus the
// raw-sample -> temperature conversion.
//
// This file is intentionally self-contained and separate from
// lab5_adc_uart_crc.cpp so Role 1 can be reviewed and merged on its own. See
// "Integrating" at the bottom of this file for the three small edits the main
// file needs once the other roles are ready.
//
// Sources used throughout Role 1 (cited inline as [README], [DS], [TRM]):
//   [README] lab5_adc_uart_crc/README.md, "Team roles" -> "1. ADC" guide, and
//            the "Packet format" paragraph of "Overview".
//   [DS]     MSPM0G3507 datasheet (datasheets/mspm0g3507_datasheet.pdf,
//            SLASEX6C): section numbers and printed page numbers.
//   [TRM]    MSPM0 G-Series Technical Reference Manual
//            (datasheets/mspm0g_slau846e.pdf, SLAU846E): section numbers and
//            printed page numbers.
//   [HAL]    hal/adc.hpp, the lab5::adc16_stream interface this class
//            implements.
//   [STYLE]  ../style.md, and the msp0g_* drivers on the lab5-kt branch of
//            the CMPE146Lab-FA2026 repo (msp0g_uart.hpp/.cpp): namespace
//            `msp0g`, class named `msp0g_<peripheral>`, constructor fully
//            configures the peripheral, an interrupt entry point named
//            `handle_interrupt()`, Doxygen on every public API.
// =============================================================================

#include <cstddef>
#include <cstdint>

#include "../hal/adc.hpp"

namespace msp0g {

// -----------------------------------------------------------------------------
// Temperature encoding
// -----------------------------------------------------------------------------

/**
 * @brief One converted temperature reading, in units of 0.1 degC.
 *
 * For example, 250 means 25.0 degC and -50 means -5.0 degC.
 *
 * [README] "Overview" -> "Packet format": the encoding is the team's choice
 * as long as it resolves at least 0.5 degC (a whole-degree integer cannot).
 * 0.1 degC steps satisfy that with room to spare and still fit an int16_t
 * (range -3276.8 to +3276.7 degC). 250 is also the placeholder value the
 * stand-in sampler_task already writes, so the display board keeps working
 * unchanged.
 *
 * @note Both boards must agree on this encoding AND on byte order. The
 * packet sends this type's bytes as-is (little endian on this MCU).
 */
using temperature_sample_t = std::int16_t;

/** @brief How many `temperature_sample_t` counts make up one degree C. */
inline constexpr std::int32_t temperature_units_per_degree = 10;

// -----------------------------------------------------------------------------
// Pure conversion helpers (no hardware access, so they are host-testable)
// -----------------------------------------------------------------------------

/**
 * @brief Upscale a 12-bit ADC code to the 16-bit range by bit duplication.
 *
 * Shifts the 12 sampled bits to the top of the value, then repeats the top
 * bits into the freed low bits, so 0x000 maps to 0x0000 and 0xFFF maps to
 * 0xFFFF.
 *
 * [HAL] adc16_stream::read() docs: "If the ADC's resolution is below
 * 16-bits, the implementation is required to upscale each sample via bit
 * duplication ... so that a sample of 0 always maps to 0 and the maximum
 * sample always maps to 0xFFFF."
 * [README] 1. ADC -> "Each read()" step 5.
 * [TRM] Table 18-4 (section 18.2.11, p. 1027): unsigned 12-bit results are
 * right-justified in MEMRES, range 0x000 to 0xFFF.
 *
 * @param p_code 12-bit ADC code. Bits above bit 11 are ignored.
 * @return The same reading expressed on the 16-bit scale.
 */
constexpr std::uint16_t upscale_12_to_16(std::uint16_t p_code)
{
  std::uint16_t const code = p_code & 0x0FFFU;
  return static_cast<std::uint16_t>((code << 4) | (code >> 8));
}

/**
 * @brief Convert one upscaled temperature-sensor sample to 0.1 degC units.
 *
 * Applies the datasheet's single-point linear model around the factory trim:
 *
 *   T = TSTRIM + (V_sample - V_trim) / TSc
 *
 * where V = 3.3 V * code / 0xFFFF for both the sample and the trim code.
 * Working in 16-bit counts means both codes use the same units, which is
 * what [README] asks for ("scale either them or the calibration value so
 * both use the same units").
 *
 * Constants and where each comes from:
 *   - TSTRIM = 30 degC and TSc = -1.8 mV/degC (typical):
 *     [DS] 7.14 "Temperature Sensor" (p. 41); [README] 1. ADC ->
 *     "Temperature conversion".
 *   - The 3.3 V reference: [DS] 7.14 gives the factory trim conditions as
 *     "VRSEL = 0h (VDD = 3.3V)" and 8.13 (p. 62) repeats that the trim code
 *     was measured in 12-bit mode against the 3.3 V VDD reference. The
 *     driver converts against VDDA, so the LaunchPad's 3.3 V supply is
 *     assumed.
 *   - The equation shape: [TRM] Chapter 17 (pp. 1014-1015), Equation 17.
 *     That chapter's Equation 18 subtracts 0.5 LSB from each code before
 *     converting to volts. The same offset appears in V_sample and V_trim,
 *     so it cancels in (V_sample - V_trim) and is left out here.
 *
 * Resolution check: one 12-bit LSB is 3.3 V / 4096 = 0.806 mV, which is
 * about 0.45 degC at 1.8 mV/degC, so the sensor itself meets the lab's
 * "at least 0.5 degC" requirement ([README] Overview -> Packet format and
 * Grading Rubric -> Role 1, "Temperature conversion").
 *
 * @param p_sample 16-bit-scaled ADC sample (output of the ADC driver).
 * @param p_trim 16-bit-scaled factory calibration code at 30 degC (see
 * `temperature_trim_sample()`).
 * @return Temperature in units of 0.1 degC (see `temperature_sample_t`).
 */
constexpr temperature_sample_t to_temperature(std::uint16_t p_sample,
                                              std::uint16_t p_trim)
{
  // Physical constants, in integers so no floating point is needed on the
  // Cortex-M0+ (it has no FPU).
  constexpr std::int64_t supply_microvolts = 3'300'000;  // [DS] 7.14
  constexpr std::int64_t sensor_microvolts_per_degree = 1'800;  // |TSc|
  constexpr std::int64_t trim_temperature = 30;  // TSTRIM, [DS] 7.14
  constexpr std::int64_t full_scale = 0xFFFF;

  // Positive when the sample is above the trim voltage. The sensor has a
  // NEGATIVE coefficient, so a positive difference means "colder".
  std::int64_t const delta_counts =
    static_cast<std::int64_t>(p_sample) - static_cast<std::int64_t>(p_trim);

  // delta_degrees = -(delta_counts * 3.3 V / full_scale) / 1.8 mV
  // Multiply by the units-per-degree up front so we divide only once.
  std::int64_t const numerator =
    delta_counts * supply_microvolts * temperature_units_per_degree;
  std::int64_t const denominator = full_scale * sensor_microvolts_per_degree;

  // Round to nearest (not toward zero) so the error stays symmetric.
  std::int64_t const half_denominator = denominator / 2;
  std::int64_t const rounded_delta =
    (numerator >= 0 ? numerator + half_denominator
                    : numerator - half_denominator) /
    denominator;

  return static_cast<temperature_sample_t>(
    trim_temperature * temperature_units_per_degree - rounded_delta);
}

/**
 * @brief Read the factory temperature calibration code, scaled to 16 bits.
 *
 * TEMP_SENSE0 (address 0x41C4003C) holds the 12-bit code the factory
 * measured from the sensor at 30 degC. The code is upscaled with the same
 * bit duplication as the live samples so that `to_temperature()` can compare
 * them directly.
 *
 * [README] 1. ADC -> "Temperature conversion", first bullet.
 * [TRM] 1.5.2.16 TEMP_SENSE0 (p. 159), and Chapter 17 (p. 1014): the code is
 * "based upon 12-bit sampling mode" with the reference used at the factory.
 *
 * @return 16-bit-scaled calibration code. Read it once and reuse it; it
 * never changes at run time.
 */
std::uint16_t temperature_trim_sample();

// -----------------------------------------------------------------------------
// ADC + DMA driver
// -----------------------------------------------------------------------------

/**
 * @brief DMA-backed driver that samples the on-chip temperature sensor.
 *
 * One `read()` fills the whole caller-provided buffer with a single DMA
 * transfer. The calling FreeRTOS task sleeps for the whole transfer and is
 * woken once by the DMA-done interrupt, instead of waking per sample.
 *
 * The constructor configures ADC0 and its DMA channel completely, so there
 * is no separate initialize() step ([README] "Rules": "Each driver configures
 * its peripheral completely in its constructor"). All DriverLib calls stay
 * inside this class ([README] "Rules": "All DriverLib calls go inside your
 * driver classes. Don't configure peripherals in SysConfig.").
 *
 * @note Construction gives this object exclusive use of ADC0 and DMA
 * channel 0. Only one instance should exist, and only one task may call
 * `read()` at a time.
 */
class msp0g_adc : public lab5::adc16_stream
{
public:
  /**
   * @brief Power, configure, and enable ADC0 + DMA channel 0 for sampling
   * the internal temperature sensor (ADC input channel 11).
   *
   * Assumes the reset-default 32 MHz SYSOSC feeds the ADC clock
   * ([DS] 7.9.1, p. 32; [TRM] 18.2.5, p. 1021).
   */
  msp0g_adc();

  // The object owns a peripheral, so copying it would make no sense.
  msp0g_adc(msp0g_adc const&) = delete;
  msp0g_adc& operator=(msp0g_adc const&) = delete;

  /**
   * @brief Wake the task sleeping in `read()` when the DMA transfer is done.
   *
   * Call this from `ADC0_IRQHandler()` and nothing else. It checks that the
   * pending interrupt is DMA-done, notifies the saved task handle, and
   * requests a context switch so the woken task runs as soon as the handler
   * returns.
   *
   * [README] 1. ADC -> "Interrupt handler", steps 1-4. This mirrors the
   * `msp0g_uart::handle_interrupt()` entry point in the FA2026 repo's UART
   * driver [STYLE].
   */
  static void handle_interrupt();

private:
  /**
   * @brief Fill p_buffer with one sample per element using a single DMA
   * transfer, then upscale each 12-bit sample to 16 bits.
   *
   * Blocks the calling task (no busy-wait) until the DMA-done interrupt.
   *
   * @param p_buffer Destination for consecutive samples. Must hold between 1
   * and 65535 elements (the DMA size register is 16 bits); otherwise nothing
   * is sampled and the buffer is left untouched.
   */
  void driver_read(std::span<std::uint16_t> p_buffer) override;
};

}  // namespace msp0g

// =============================================================================
// Integrating (do this after the other roles' work is merged)
// =============================================================================
//
// In lab5_adc_uart_crc.cpp:
//
//  1. Replace the empty `class adc_driver {};` with
//       using adc_driver = msp0g::msp0g_adc;
//     (same pattern the file already uses for `serial_driver`).
//  2. Replace the empty ADC0_IRQHandler body with
//       extern "C" void ADC0_IRQHandler()
//       {
//         msp0g::msp0g_adc::handle_interrupt();
//       }
//     The vector must stay in the application file only; defining it here as
//     well would be a duplicate-symbol link error while the stub exists.
//  3. Replace the stand-in `sampler_task` with msp0g::sampler_task from
//     sampler_task.hpp, passing a static msp0g::sampler_context holding
//     `sample_request` and `batch_queue` as the task parameter.
