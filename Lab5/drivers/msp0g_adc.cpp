// cspell:ignore SYSOSC VDDA IIDX MEMRES SCOMP FRANGE SCLKDIV DMADONE DMASZ

// =============================================================================
// Role 1 (ADC): implementation of the DMA-backed temperature-sensor driver.
//
// High-level data flow of one read():
//
//   task calls read(buffer)
//     -> arm DMA: "copy the next N ADC results into `buffer`"
//     -> start the ADC (it converts again and again by itself)
//     -> task SLEEPS (ulTaskNotifyTake)
//   ADC finishes a conversion  -> DMA copies the result into the buffer
//   (repeats N times with zero CPU involvement)
//   DMA copied the N-th sample -> ADC raises the DMA-done interrupt
//     -> ADC0_IRQHandler() -> handle_interrupt() wakes the task
//   task wakes up -> stops the ADC -> upscales 12-bit samples to 16-bit
//
// Citation keys ([README], [DS], [TRM], [HAL], [STYLE]) are defined at the top
// of msp0g_adc.hpp.
// =============================================================================

#include <cstddef>
#include <cstdint>

#include "FreeRTOS.h"
#include "task.h"

#include <ti/driverlib/driverlib.h>

#include "msp0g_adc.hpp"

namespace {

// -----------------------------------------------------------------------------
// Hardware choices
// -----------------------------------------------------------------------------

// DMA channel used for the ADC transfers. Channels 0-2 are "full" channels and
// 3-6 are "basic" channels; single-transfer mode (what we use) works on both.
// [DS] 8.5 "DMA" (pp. 55-56), Table 8-2. [TRM] 5.2.3, Table 5-2 (p. 528).
constexpr std::uint32_t dma_channel = 0;

// How many ADC results one DMA trigger moves. The ADC raises one trigger per
// finished conversion, and we want exactly one result per trigger, so each
// sample is a separate single-transfer DMA step.
// [README] 1. ADC -> "Constructor" step 6 (DL_ADC12_setDMASamplesCnt(ADC0, 1)).
// [TRM] 18.2.12.3 (p. 1029): "SAMPCNT must be programmed to 1 by SW for DMA
// operation" in single/repeat-single conversion mode with FIFO disabled.
constexpr std::uint32_t samples_per_dma_trigger = 1;

// The DMA size register (DMASZ) is 16 bits wide, so one transfer can move at
// most this many samples.
// [DS] 8.5 (p. 56): "Transfer counter block size supports up to 64k
// transfers". [TRM] 5.2.3.1 (p. 528): DMASZ sets the transfer count.
constexpr std::size_t max_samples_per_read = 0xFFFF;

// -----------------------------------------------------------------------------
// ADC timing
// -----------------------------------------------------------------------------

// ADC clock source frequency. After reset the system oscillator runs at its
// 32 MHz base frequency and we select it as the ADC clock with divide-by-1.
// [DS] 7.9.1 "System Oscillator (SYSOSC)" (p. 32): 32 MHz at SYSOSCCFG.FREQ=00.
// [TRM] 18.2.5 "ADC Clocking" (p. 1021).
// [README] 1. ADC -> "Constructor" step 2 (SYSOSC as the clock source).
constexpr std::uint32_t adc_clock_hertz = 32'000'000;

// The sensor needs its output to settle before the ADC samples it: at least
// 12.5 us. We add 5 us of ADC wake-up time because the ADC powers itself down
// between conversions by default, and the wake-up has to fit inside the sample
// window. 12.5 us + 5 us = 17.5 us.
// [DS] 7.14 (p. 41): tSET,TS = 12.5 us, "the minimum required ADC sampling
// time when measuring the temperature sensor"; 7.12.1 (p. 39): Twakeup = 5 us.
// [TRM] 18.2.8.1 note (p. 1024): SCOMPx must exceed wake-up time + the
// sample window; 18.2.9 (p. 1025).
// [README] 1. ADC -> "Constructor" step 5 ("at least 12.5 us. Compute the
// cycle count from your ADC clock").
constexpr std::uint32_t sample_time_nanoseconds = 17'500;

// Convert the time to ADC clock cycles, rounding UP so we never undershoot.
// For a 32 MHz clock this is 560 cycles.
constexpr std::uint32_t sample_time_cycles =
  (adc_clock_hertz / 1'000 * sample_time_nanoseconds + 999'999) / 1'000'000;

// 12.5 us at 32 MHz is 400 cycles; the sample timer (SCOMP0) is 10 bits wide.
// [TRM] 18.2.8.1 (p. 1023): "The sample timer is 10-bits wide".
static_assert(sample_time_cycles >= 400,
              "sample time must be at least 12.5 us (datasheet 7.14)");
static_assert(sample_time_cycles <= 0x3FF,
              "sample time must fit the 10-bit SCOMP0 register");

// Cycles to wait after enabling power before touching the peripheral. This is
// the same value TI's generated ti_msp_dl_config.c uses
// (POWER_STARTUP_DELAY = 16), and the same one crc.cpp uses for the CRC.
// [README] 1. ADC -> "Constructor" step 1 ("wait for it to power up").
constexpr std::uint32_t power_startup_delay_cycles = 16;

// -----------------------------------------------------------------------------
// Shared with the interrupt handler
// -----------------------------------------------------------------------------

// The task currently sleeping inside driver_read(). The interrupt handler
// needs it to know whom to wake. It lives at file scope (not as a member)
// because handle_interrupt() is static and called from ADC0_IRQHandler(),
// which has no access to the msp0g_adc object. `volatile` because it is
// written by a task and read from an interrupt.
// [README] 1. ADC -> "Each read()" step 3: "Save the calling task's handle
// from xTaskGetCurrentTaskHandle() somewhere the interrupt handler can reach,
// like a file-scope variable."
TaskHandle_t volatile waiting_task = nullptr;

}  // namespace

namespace msp0g {

std::uint16_t temperature_trim_sample()
{
  // DL_SYSCTL_getTempCalibrationConstant() reads TEMP_SENSE0.DATA (the code
  // the factory measured at 30 degC). Only the low 12 bits are meaningful.
  // [README] 1. ADC -> "Temperature conversion", first bullet.
  // [TRM] 1.5.2.16 TEMP_SENSE0 (p. 159); Chapter 17 (p. 1014).
  auto const trim_code =
    static_cast<std::uint16_t>(DL_SYSCTL_getTempCalibrationConstant());

  // Put the calibration code on the same 16-bit scale as the live samples.
  return upscale_12_to_16(trim_code);
}

msp0g_adc::msp0g_adc()
{
  // --- Constructor step 1: reset and power the ADC ---------------------------
  // [README] 1. ADC -> "Constructor" step 1. Reset first so the peripheral
  // starts from a known state, then enable its power domain and give it a few
  // cycles to come up.
  DL_ADC12_reset(ADC0);
  DL_ADC12_enablePower(ADC0);
  DL_Common_delayCycles(power_startup_delay_cycles);

  // --- Constructor step 2: pick the ADC clock --------------------------------
  // SYSOSC as the source, no division. The frequency-range field must match
  // the real ADC clock or the ADC mis-times its end-of-conversion signal:
  // 24-32 MHz is the range that contains 32 MHz.
  // [README] 1. ADC -> "Constructor" step 2.
  // [TRM] 18.2.5, Table 18-2 (p. 1021): FRANGE = 5 for ">24 to 32" MHz.
  DL_ADC12_ClockConfig clock_config{};
  clock_config.clockSel = DL_ADC12_CLOCK_SYSOSC;
  clock_config.divideRatio = DL_ADC12_CLOCK_DIVIDE_1;
  clock_config.freqRange = DL_ADC12_CLOCK_FREQ_RANGE_24_TO_32;
  DL_ADC12_setClockConfig(ADC0, &clock_config);

  // --- Constructor step 3: conversion mode -----------------------------------
  // Repeat mode: after one software "start", the ADC keeps converting the same
  // channel back-to-back until told to stop. That is what lets a single
  // DMA transfer collect 64 consecutive samples with no per-sample CPU work.
  // Software trigger, automatic sampling timer, 12-bit, unsigned (so results
  // are plain 0..4095 codes).
  // [README] 1. ADC -> "Constructor" step 3.
  // [TRM] 18.2.10 (p. 1026): "Repeat single channel conversion ... repeatedly
  // sampled and converted until ENC is cleared"; 18.2.11 (p. 1027): data
  // formats.
  DL_ADC12_initSingleSample(ADC0,
                            DL_ADC12_REPEAT_MODE_ENABLED,
                            DL_ADC12_SAMPLING_SOURCE_AUTO,
                            DL_ADC12_TRIG_SRC_SOFTWARE,
                            DL_ADC12_SAMP_CONV_RES_12_BIT,
                            DL_ADC12_SAMP_CONV_DATA_FORMAT_UNSIGNED);

  // --- Constructor step 4: what MEM0 converts --------------------------------
  // Memory slot 0 reads channel 11 (the temperature sensor) against VDDA
  // (the supply). The "auto next" trigger mode makes the repeat mode above
  // start the next conversion on its own. No hardware averaging and no window
  // comparator; we want every raw sample.
  // [README] 1. ADC -> "Constructor" step 4.
  // [DS] 8.12 "ADC", Table 8-8 (p. 62): ADC0 channel 11 is the temperature
  // sensor (also 7.14, p. 41: "ADC CHANNEL=11"). 8.13 (p. 62): the factory
  // trim was taken with the 3.3 V VDD reference, so we use VDDA too.
  DL_ADC12_configConversionMem(ADC0,
                               DL_ADC12_MEM_IDX_0,
                               DL_ADC12_INPUT_CHAN_11,
                               DL_ADC12_REFERENCE_VOLTAGE_VDDA,
                               DL_ADC12_SAMPLE_TIMER_SOURCE_SCOMP0,
                               DL_ADC12_AVERAGING_MODE_DISABLED,
                               DL_ADC12_BURN_OUT_SOURCE_DISABLED,
                               DL_ADC12_TRIGGER_MODE_AUTO_NEXT,
                               DL_ADC12_WINDOWS_COMP_MODE_DISABLED);

  // --- Constructor step 5: sample time ---------------------------------------
  // Hold the input for 17.5 us (560 cycles) so the sensor can settle.
  // [README] 1. ADC -> "Constructor" step 5.
  DL_ADC12_setSampleTime0(ADC0, sample_time_cycles);

  // --- Constructor step 6: connect the ADC to the DMA ------------------------
  // Tell the ADC to issue one DMA trigger each time MEM0 receives a new
  // result, and to raise the DMA-done interrupt when the DMA channel has
  // moved its programmed number of samples. Then let that interrupt reach the
  // CPU through the NVIC.
  // [README] 1. ADC -> "Constructor" step 6.
  // [TRM] 18.2.12.3 (pp. 1028-1029): MEMRESIFGx can generate the DMA trigger;
  // the "DONE status" signal generates the DMA DONE interrupt.
  // [TRM] 5.2.11 (p. 533): DMA trigger event status lets the peripheral
  // raise an interrupt when its assigned DMA channel finishes.
  DL_ADC12_setDMASamplesCnt(ADC0, samples_per_dma_trigger);
  DL_ADC12_enableDMATrigger(ADC0, DL_ADC12_DMA_MEM0_RESULT_LOADED);
  DL_ADC12_enableInterrupt(ADC0, DL_ADC12_INTERRUPT_DMA_DONE);
  NVIC_ClearPendingIRQ(ADC0_INT_IRQn);
  NVIC_EnableIRQ(ADC0_INT_IRQn);

  // --- Constructor step 7: configure the DMA channel -------------------------
  // Single-transfer mode: every ADC trigger moves exactly one item, and the
  // channel switches itself off after the programmed count. Both ends are
  // half-words (16 bits) because the ADC result register holds a 12-bit code
  // in its low half. The source address never changes (always MEM0's result
  // register); the destination address increments so samples land in
  // consecutive buffer slots.
  // [README] 1. ADC -> "Constructor" step 7.
  // [DS] 8.5, Table 8-3 (p. 56): trigger 23 is "ADC0 Publisher 2", which
  // DriverLib names DMA_ADC0_EVT_GEN_BD_TRIG.
  // [TRM] 5.2.3.1 (p. 528): single transfer, DMAEN clears after DMASZ
  // transfers; 5.2.5 (p. 532): each channel has its own trigger source.
  DL_DMA_Config dma_config{};
  dma_config.transferMode = DL_DMA_SINGLE_TRANSFER_MODE;
  dma_config.extendedMode = DL_DMA_NORMAL_MODE;
  dma_config.destIncrement = DL_DMA_ADDR_INCREMENT;
  dma_config.srcIncrement = DL_DMA_ADDR_UNCHANGED;
  dma_config.destWidth = DL_DMA_WIDTH_HALF_WORD;
  dma_config.srcWidth = DL_DMA_WIDTH_HALF_WORD;
  dma_config.trigger = DMA_ADC0_EVT_GEN_BD_TRIG;
  dma_config.triggerType = DL_DMA_TRIGGER_TYPE_EXTERNAL;
  DL_DMA_initChannel(DMA, dma_channel, &dma_config);

  // The source never changes, so it is set once here rather than every read.
  DL_DMA_setSrcAddr(
    DMA, dma_channel, DL_ADC12_getMemResultAddress(ADC0, DL_ADC12_MEM_IDX_0));

  // --- Constructor step 8: allow conversions ---------------------------------
  // ENC must be set before the ADC accepts a start command. All of the
  // configuration above had to happen before this (the config registers are
  // locked while conversions are enabled).
  // [README] 1. ADC -> "Constructor" step 8.
  // [TRM] 18.2.10 step 8 (p. 1026): "Set the ENC bit in the CTL0 register".
  DL_ADC12_enableConversions(ADC0);
}

void msp0g_adc::driver_read(std::span<std::uint16_t> p_buffer)
{
  // Nothing to do, or more than the 16-bit DMA size register can describe.
  // Returning early also prevents sleeping forever waiting for an interrupt
  // that can never come.
  if (p_buffer.empty() || p_buffer.size() > max_samples_per_read) {
    return;
  }

  // Throw away any stale wake-up left over from an earlier read so we do not
  // return before this transfer actually completes. A zero timeout makes this
  // a non-blocking check.
  ulTaskNotifyTake(pdTRUE, 0);

  // --- read() step 1: re-arm the DMA channel for this buffer -----------------
  // The channel must be disabled while its addresses and size are changed. The
  // DMA clears its own enable bit when a transfer completes, so this has to be
  // redone for every read.
  // [README] 1. ADC -> "Each read()" step 1.
  // [TRM] 5.2.6 (p. 532): DMAEN is cleared to stop; 5.2.3.1 (p. 528-529):
  // DMAEN is cleared automatically when DMASZ reaches zero and must be set
  // again for another transfer.
  DL_DMA_disableChannel(DMA, dma_channel);
  auto const destination =
    reinterpret_cast<std::uintptr_t>(p_buffer.data());
  DL_DMA_setDestAddr(DMA, dma_channel, static_cast<std::uint32_t>(destination));
  DL_DMA_setTransferSize(
    DMA, dma_channel, static_cast<std::uint16_t>(p_buffer.size()));
  DL_DMA_enableChannel(DMA, dma_channel);

  // --- read() step 2: arm the ADC side of the DMA handshake ------------------
  // The ADC clears this enable bit by itself when the DMA reports done, so it
  // has to be set again for each read.
  // [README] 1. ADC -> "Each read()" step 2.
  // [TRM] 18.2.12.3 (p. 1028): "The DMAEN bit is cleared by ADC hardware when
  // the DMA 'DONE status' signal is asserted. Software is expected to re-enable
  // the DMA using DMAEN".
  DL_ADC12_enableDMA(ADC0);

  // --- read() step 3: remember who to wake -----------------------------------
  // Must be saved BEFORE starting the conversion: the interrupt could in theory
  // fire right after the start, and it would find a null handle otherwise.
  // [README] 1. ADC -> "Each read()" step 3.
  waiting_task = xTaskGetCurrentTaskHandle();

  // --- read() step 4: start sampling and sleep until the DMA is done ---------
  // The task blocks here without using the CPU. If the interrupt already ran
  // before we got here, the notification is still pending and this returns
  // immediately, so there is no lost-wakeup race.
  // [README] 1. ADC -> "Each read()" step 4.
  DL_ADC12_startConversion(ADC0);
  ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

  // --- read() step 5: stop and convert ---------------------------------------
  // The ADC is in repeat mode and would keep converting forever, so stop it.
  // [README] 1. ADC -> "Each read()" step 5.
  DL_ADC12_stopConversion(ADC0);
  waiting_task = nullptr;

  // Samples arrived as 12-bit codes (0..4095). [HAL] requires 16-bit samples
  // where 0 -> 0 and max -> 0xFFFF, so upscale by bit duplication.
  // [HAL] hal/adc.hpp, adc16_stream::read(); [README] 1. ADC -> "Each read()"
  // step 5.
  for (auto& sample : p_buffer) {
    sample = upscale_12_to_16(sample);
  }
}

void msp0g_adc::handle_interrupt()
{
  // --- Interrupt handler steps 1-2 -------------------------------------------
  // ADC0 has a single interrupt vector shared by many causes, so ask which one
  // fired. Reading the index register also clears that pending flag. We only
  // care about "the DMA finished its programmed transfer".
  // [README] 1. ADC -> "Interrupt handler" steps 1-2.
  // [TRM] 5.2.10 (p. 533): reading IIDX acknowledges the highest-priority
  // pending interrupt.
  if (DL_ADC12_getPendingInterrupt(ADC0) != DL_ADC12_IIDX_DMA_DONE) {
    return;
  }

  // --- Interrupt handler step 3 ----------------------------------------------
  // Wake the task blocked in driver_read(). The FromISR variant is the only
  // FreeRTOS call allowed in an interrupt; it also reports whether the woken
  // task has a higher priority than the one that was interrupted.
  // [README] 1. ADC -> "Interrupt handler" step 3.
  BaseType_t higher_priority_task_woken = pdFALSE;
  TaskHandle_t const task = waiting_task;
  if (task != nullptr) {
    vTaskNotifyGiveFromISR(task, &higher_priority_task_woken);
  }

  // --- Interrupt handler step 4 ----------------------------------------------
  // If the woken task outranks the interrupted one, switch to it right as the
  // handler returns instead of waiting for the next tick.
  // [README] 1. ADC -> "Interrupt handler" step 4.
  portYIELD_FROM_ISR(higher_priority_task_woken);
}

}  // namespace msp0g
