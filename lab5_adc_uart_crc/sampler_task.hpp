#pragma once

#include <cstddef>

#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"

namespace msp0g {

/** @brief Samples per batch. Must match samples_per_batch in the app file. */
inline constexpr std::size_t sampler_samples_per_batch = 64;

/**
 * @brief Semaphore and queue passed into sampler_task.
 *
 * The instance must have static storage duration.
 */
struct sampler_context
{
  SemaphoreHandle_t sample_request;
  QueueHandle_t batch_queue;
};

/**
 * @brief Wait for a request, sample, convert, and queue the batch.
 *
 * @param p_context Pointer to a static sampler_context. A null pointer parks
 * the task; a FreeRTOS task must not return.
 */
void sampler_task(void* p_context);

}  // namespace msp0g
