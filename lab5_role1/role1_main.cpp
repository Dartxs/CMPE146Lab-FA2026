#include <array>
#include <cstdint>

#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"

#include "msp0g_adc.hpp"
#include "sampler_task.hpp"

// DMA completion wakes the task blocked in adc.read().
extern "C" void ADC0_IRQHandler()
{
  msp0g::msp0g_adc::handle_interrupt();
}

namespace {
constexpr std::size_t samples_per_batch = msp0g::sampler_samples_per_batch;
using temperature_batch =
  std::array<msp0g::temperature_sample_t, samples_per_batch>;

StaticSemaphore_t sample_request_storage;
SemaphoreHandle_t sample_request = nullptr;

StaticQueue_t batch_queue_storage;
std::uint8_t batch_queue_buffer[sizeof(temperature_batch)];
QueueHandle_t batch_queue = nullptr;

constexpr std::uint32_t task_stack_words = 512;
StaticTask_t sampler_task_tcb;
StackType_t sampler_task_stack[task_stack_words];
StaticTask_t request_task_tcb;
StackType_t request_task_stack[task_stack_words];

// One board, no display. Give the sampler a request once a second and drop
// the previous batch so the queue does not stay full.
void role1_request_task(void*)
{
  temperature_batch discarded{};
  while (true) {
    xQueueReceive(batch_queue, &discarded, 0);
    xSemaphoreGive(sample_request);
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}
}  // namespace

int main()
{
  sample_request = xSemaphoreCreateBinaryStatic(&sample_request_storage);
  batch_queue = xQueueCreateStatic(1,
                                    sizeof(temperature_batch),
                                    batch_queue_buffer,
                                    &batch_queue_storage);

  static msp0g::sampler_context sampler{sample_request, batch_queue};

  xTaskCreateStatic(msp0g::sampler_task,
                    "sampler_task",
                    task_stack_words,
                    &sampler,
                    tskIDLE_PRIORITY + 1,
                    sampler_task_stack,
                    &sampler_task_tcb);
  xTaskCreateStatic(role1_request_task,
                    "role1_request",
                    task_stack_words,
                    nullptr,
                    tskIDLE_PRIORITY + 1,
                    request_task_stack,
                    &request_task_tcb);

  vTaskStartScheduler();
  while (true) {
  }
}
