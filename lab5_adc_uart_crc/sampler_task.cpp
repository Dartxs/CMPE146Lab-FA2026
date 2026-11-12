#include <array>
#include <cstddef>
#include <cstdint>

#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"

#include "msp0g_adc.hpp"
#include "sampler_task.hpp"

namespace msp0g {

void sampler_task(void* p_context)
{
  auto* context = static_cast<sampler_context*>(p_context);

  // A FreeRTOS task function must never return. If main() forgot to pass the
  // context there is nothing useful to do, so sleep forever (this still lets
  // the other tasks run, unlike a busy loop).
  if (context == nullptr) {
    while (true) {
      vTaskDelay(portMAX_DELAY);
    }
  }

  // [README] 1. ADC -> "sampler_task" (intro) and the TODO above the stand-in
  // sampler_task: the task constructs the ADC driver. The constructor does
  // all configuration (power, clock, DMA), so there is no second setup call.
  // The driver is not shared with any other task, so living on this task's
  // stack is fine (the README's "static storage duration" rule is for drivers
  // *shared* between tasks).
  msp0g_adc adc;

  // Read the factory calibration code once. It is a constant for this chip and
  // reading it is cheap, but there is no reason to repeat it for every sample.
  // [README] 1. ADC -> "Temperature conversion" (first bullet).
  std::uint16_t const trim_sample = temperature_trim_sample();

  // Raw 16-bit samples filled by the DMA. Kept separate from the converted
  // batch because the queue item type is temperature_sample_t, not uint16_t.
  // [README] 1. ADC -> "sampler_task" step 2: "std::array<std::uint16_t, 64>".
  std::array<std::uint16_t, sampler_samples_per_batch> raw_samples{};

  // Converted batch: this is exactly what batch_queue carries.
  std::array<temperature_sample_t, sampler_samples_per_batch> batch{};

  while (true) {
    // --- sampler_task step 1: wait for a request --------------------------
    // Sleeps (no CPU use) until request_task sees sample_request_command on
    // the UART and gives the semaphore.
    // [README] 1. ADC -> "sampler_task" step 1.
    xSemaphoreTake(context->sample_request, portMAX_DELAY);

    // --- sampler_task step 2: sample ---------------------------------------
    // One call = one DMA transfer of all 64 samples. The task sleeps inside
    // read() until the DMA-done interrupt wakes it. Samples come back already
    // upscaled to 16 bits.
    // [README] 1. ADC -> "sampler_task" step 2; "How the sensor tasks pass
    // data" ("The task notification lives inside adc.read()").
    adc.read(raw_samples);

    // --- sampler_task step 3: convert ---------------------------------------
    // Raw ADC code -> temperature in 0.1 degC units, one sample at a time.
    // [README] 1. ADC -> "sampler_task" step 3 and "Temperature conversion".
    for (std::size_t i = 0; i < raw_samples.size(); i++) {
      batch[i] = to_temperature(raw_samples[i], trim_sample);
    }

    // --- sampler_task step 4: hand off to transmit_task ----------------------
    // xQueueSend copies the whole array into the queue, so this task can start
    // refilling `batch` on the next request without sharing a buffer with
    // transmit_task.
    // [README] 1. ADC -> "sampler_task" step 4; "How the sensor tasks pass
    // data" (each item is a full copy of the 64 samples).
    xQueueSend(context->batch_queue, &batch, portMAX_DELAY);
  }
}

}  // namespace msp0g
