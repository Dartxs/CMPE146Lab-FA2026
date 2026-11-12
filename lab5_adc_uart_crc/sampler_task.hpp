#pragma once

// =============================================================================
// Role 1 (ADC): the sensor board's `sampler_task`.
//
// Kept in its own file so it can be reviewed separately from the other roles'
// tasks (request_task, transmit_task, display_task). It only touches the two
// FreeRTOS objects Role 4 creates in main(), and receives them through its
// task parameter instead of reaching into lab5_adc_uart_crc.cpp's private
// (anonymous-namespace) globals.
//
// Citation keys ([README], [DS], [TRM], [HAL], [STYLE]) are defined at the top
// of msp0g_adc.hpp.
// =============================================================================

#include <cstddef>

#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"

namespace msp0g {

/**
 * @brief Number of temperature samples in one batch.
 *
 * [README] "Overview" step 2 and "Packet format": 64 samples per packet. This
 * must equal the `samples_per_batch` in lab5_adc_uart_crc.cpp, because
 * `batch_queue`'s item size is `sizeof(std::array<temperature_sample_t, 64>)`.
 */
inline constexpr std::size_t sampler_samples_per_batch = 64;

/**
 * @brief The two FreeRTOS objects `sampler_task` needs.
 *
 * The task parameter must point at an instance with static storage duration
 * (it is read for the lifetime of the task).
 */
struct sampler_context
{
  /**
   * @brief Binary semaphore given by `request_task` to mean "take a batch
   * now". Carries no data. [README] "How the sensor tasks pass data".
   */
  SemaphoreHandle_t sample_request;

  /**
   * @brief Queue that receives a full copy of each converted batch. Its item
   * size must be `sampler_samples_per_batch * sizeof(temperature_sample_t)`.
   * [README] "How the sensor tasks pass data".
   */
  QueueHandle_t batch_queue;
};

/**
 * @brief Sensor-board task: wait for a request, sample the temperature sensor
 * with one DMA transfer, convert, and hand the batch to `transmit_task`.
 *
 * Constructs the ADC driver itself, so the task owns the ADC from the moment
 * it starts and no other code has to initialize it.
 *
 * @param p_context Pointer to a static `sampler_context`. If null (a wiring
 * mistake in main()), the task parks itself asleep forever instead of
 * returning, because a FreeRTOS task function must never return.
 */
void sampler_task(void* p_context);

}  // namespace msp0g
