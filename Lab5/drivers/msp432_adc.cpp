#include <cstddef>
#include <cstdint>

#include "FreeRTOS.h"
#include "task.h"

#include <ti/devices/msp432p4xx/driverlib/driverlib.h>

#include "msp432_adc.hpp"

namespace {

constexpr std::uint32_t dma_channel = 7;
constexpr std::uint32_t max_dma_transfers = 1024;

// Primary control table for all 32 DMA channels (16 bytes each). The uDMA
// controller requires 1024-byte alignment.
alignas(1024) std::uint8_t dma_control_table[1024];

// Task blocked inside read(). The DMA interrupt has no driver object.
TaskHandle_t volatile waiting_task = nullptr;

constexpr std::uint16_t upscale_14_to_16(std::uint16_t p_code)
{
  std::uint16_t const code = p_code & 0x3FFFU;
  return static_cast<std::uint16_t>((code << 2) | (code >> 12));
}

}  // namespace

msp432_adc_driver::msp432_adc_driver()
{
  // The LaunchPad watchdog is running after reset. Hold it so a 64-sample
  // transfer is not cut short by a reset.
  WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;

  // 1. P6.1 is ADC14 input A14 in its tertiary function.
  GPIO_setAsPeripheralModuleFunctionInputPin(
    GPIO_PORT_P6, GPIO_PIN1, GPIO_TERTIARY_MODULE_FUNCTION);

  // 2. ADC14 clock is the internal oscillator divided down to a stable rate.
  ADC14_enableModule();
  ADC14_initModule(ADC_CLOCKSOURCE_ADCOSC,
                   ADC_PREDIVIDER_4,
                   ADC_DIVIDER_4,
                   0);

  // 3. Unsigned 14-bit, repeated single-channel conversions. A zero internal
  // channel mask leaves the die-temperature sensor disconnected.
  ADC14_setResolution(ADC_14BIT);
  ADC14_configureSingleSampleMode(ADC_MEM0, true);
  ADC14_enableSampleTimer(ADC_AUTOMATIC_ITERATION);

  // 4. Memory 0 reads A14 against the internal buffered reference.
  ADC14_configureConversionMemory(
    ADC_MEM0, ADC_VREFPOS_INTBUF_VREFNEG_VSS, ADC_INPUT_A14, false);

  // 5. 192 ADC clocks of sample-and-hold. At this divided clock that is well
  // above the TMP36 and 14-bit settling time.
  ADC14_setSampleHoldTime(ADC_PULSE_WIDTH_192, ADC_PULSE_WIDTH_192);

  // 6. Turn on the 2.5 V reference and wait until it is ready to sample.
  REF_A_setReferenceVoltage(REF_A_VREF2_5V);
  REF_A_enableReferenceVoltage();
  while (REF_A_isRefGenBusy()) {
  }
  while (!REF_A_getVariableReferenceVoltageStatus()) {
  }

  // 7. DMA channel 7 moves one half-word from ADC14 memory 0 per trigger.
  // The source stays on that register and the destination advances.
  DMA_enableModule();
  DMA_setControlBase(dma_control_table);
  DMA_assignChannel(DMA_CH7_ADC14);
  DMA_setChannelControl(UDMA_PRI_SELECT | DMA_CH7_ADC14,
                        UDMA_SIZE_16 | UDMA_SRC_INC_NONE | UDMA_DST_INC_16 |
                          UDMA_ARB_1);

  // 8. DMA completion on channel 7 raises DMA_INT1. ADC14 itself has no
  // per-sample interrupt.
  DMA_assignInterrupt(DMA_INT1, dma_channel);
  DMA_clearInterruptFlag(dma_channel);
  // FreeRTOS only allows FromISR calls from interrupts at this priority or
  // lower. 0 is the highest priority and would halt the scheduler.
  Interrupt_setPriority(INT_DMA_INT1, 6);
  Interrupt_enableInterrupt(INT_DMA_INT1);
}

void msp432_adc_driver::driver_read(std::span<std::uint16_t> p_buffer)
{
  if (p_buffer.empty() || p_buffer.size() > max_dma_transfers) {
    return;
  }

  ulTaskNotifyTake(pdTRUE, 0);

  // 1. Stop a previous run, take the DMA channel offline, then wait until
  // ADC14 has finished any conversion already in progress.
  ADC14_disableConversion();
  DMA_disableChannel(dma_channel);
  while (ADC14_isBusy()) {
  }

  // 2. This transfer writes the caller's buffer and then stops.
  DMA_setChannelControl(UDMA_PRI_SELECT | DMA_CH7_ADC14,
                        UDMA_SIZE_16 | UDMA_SRC_INC_NONE | UDMA_DST_INC_16 |
                          UDMA_ARB_1);
  DMA_setChannelTransfer(UDMA_PRI_SELECT | DMA_CH7_ADC14,
                         UDMA_MODE_BASIC,
                         const_cast<void*>(
                           static_cast<void const volatile*>(&ADC14->MEM[0])),
                         p_buffer.data(),
                         static_cast<std::uint32_t>(p_buffer.size()));

  // 3. Publish the sleeper before the conversion can complete.
  waiting_task = xTaskGetCurrentTaskHandle();

  // 4. Arm the channel, then start the repeated conversion.
  DMA_enableChannel(dma_channel);
  ADC14_enableConversion();
  ADC14_toggleConversionTrigger();

  // 5. Sleep until DMA_INT1_IRQHandler gives the notification.
  ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

  // 6. Stop the repeat mode and expand each 14-bit code to 16 bits.
  ADC14_disableConversion();
  waiting_task = nullptr;

  for (auto& sample : p_buffer) {
    sample = upscale_14_to_16(sample);
  }
}

void msp432_adc_driver::handle_interrupt()
{
  bool const channel_7_interrupt =
    (DMA_getInterruptStatus() & DMA_INT1) != 0;
  bool const transfer_complete =
    DMA_getChannelMode(UDMA_PRI_SELECT | DMA_CH7_ADC14) == UDMA_MODE_STOP;

  if (!channel_7_interrupt && !transfer_complete) {
    return;
  }

  DMA_clearInterruptFlag(dma_channel);

  BaseType_t higher_priority_task_woken = pdFALSE;
  TaskHandle_t const task = waiting_task;
  if (task != nullptr) {
    vTaskNotifyGiveFromISR(task, &higher_priority_task_woken);
  }
  portYIELD_FROM_ISR(higher_priority_task_woken);
}

extern "C" void DMA_INT1_IRQHandler()
{
  msp432_adc_driver::handle_interrupt();
}
