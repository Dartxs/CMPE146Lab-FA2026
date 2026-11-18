#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdio>

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

  if (context == nullptr) {
    while (true) {
      vTaskDelay(portMAX_DELAY);
    }
  }

  msp0g_adc adc;
  std::uint16_t const trim_sample = temperature_trim_sample();
  std::array<std::uint16_t, sampler_samples_per_batch> raw_samples{};
  std::array<temperature_sample_t, sampler_samples_per_batch> batch{};

  while (true) {
    xSemaphoreTake(context->sample_request, portMAX_DELAY);
    adc.read(raw_samples);

    std::int32_t sum = 0;
    for (std::size_t i = 0; i < raw_samples.size(); i++) {
      batch[i] = to_temperature(raw_samples[i], trim_sample);
      sum += batch[i];
    }

    // Remove once the display board shows the temperature.
    std::int32_t const average =
      sum / static_cast<std::int32_t>(raw_samples.size());
    std::printf("batch raw[0]=%u raw[63]=%u  %ld.%ld C\n",
                raw_samples[0],
                raw_samples.back(),
                static_cast<long>(average / 10),
                static_cast<long>(average % 10));

    xQueueSend(context->batch_queue, &batch, portMAX_DELAY);
  }
}

}  // namespace msp0g
