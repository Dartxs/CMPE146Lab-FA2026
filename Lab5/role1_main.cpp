#include <ti/devices/msp432p4xx/driverlib/driverlib.h>

#include <array>
#include <cstdint>
#include <cstdio>

#include <FreeRTOS.h>
#include <queue.h>
#include <semphr.h>
#include <task.h>

#include "drivers/msp432_adc.hpp"

// GNU rdimon writes printf to the CCS console. The debugger check keeps a
// board that is running alone from stopping inside a semihosting call.
extern "C" void initialise_monitor_handles(void);

namespace {
bool semihosting_ready = false;
std::array<char, 256> semihosting_stdout_buffer{};

bool debugger_connected()
{
  return (CoreDebug->DHCSR & CoreDebug_DHCSR_C_DEBUGEN_Msk) != 0;
}

template<typename... Args>
void debug_printf(char const* format, Args... args)
{
  if (semihosting_ready && debugger_connected()) {
    std::printf(format, args...);
  }
}

constexpr std::size_t samples_per_batch = 64;
using temperature_batch = std::array<std::int16_t, samples_per_batch>;

struct sampler_context
{
  SemaphoreHandle_t sample_request;
  QueueHandle_t batch_queue;
};

StaticSemaphore_t sample_request_storage;
SemaphoreHandle_t sample_request = nullptr;

StaticQueue_t batch_queue_storage;
std::uint8_t batch_queue_buffer[sizeof(temperature_batch)];
QueueHandle_t batch_queue = nullptr;

void sampler_task(void* p_context)
{
  auto* context = static_cast<sampler_context*>(p_context);
  if (context == nullptr) {
    while (true) {
      vTaskDelay(portMAX_DELAY);
    }
  }

  msp432_adc_driver adc;
  std::array<std::uint16_t, samples_per_batch> raw_samples{};
  temperature_batch batch{};

  while (true) {
    xSemaphoreTake(context->sample_request, portMAX_DELAY);
    adc.read(raw_samples);

    std::int32_t sum = 0;
    for (std::size_t i = 0; i < raw_samples.size(); i++) {
      batch[i] = tmp36_temperature_tenths(raw_samples[i]);
      sum += batch[i];
    }

    std::int32_t const average =
      sum / static_cast<std::int32_t>(raw_samples.size());
    std::int32_t const whole = average / 10;
    std::int32_t const fraction =
      average < 0 ? -(average % 10) : (average % 10);
    debug_printf("TMP36 batch raw[0]=%u raw[63]=%u  %ld.%ld C\n",
                 raw_samples[0],
                 raw_samples.back(),
                 static_cast<long>(whole),
                 static_cast<long>(fraction));

    // Nothing receives the batch on this one-board build. A blocking send
    // would stop the prints after the queue filled.
    xQueueSend(context->batch_queue, &batch, 0);
  }
}

void role1_single_board_task(void*)
{
  while (true) {
    xSemaphoreGive(sample_request);
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void blinker_task(void*)
{
  GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0);
  while (true) {
    GPIO_toggleOutputOnPin(GPIO_PORT_P1, GPIO_PIN0);
    vTaskDelay(pdMS_TO_TICKS(250));
  }
}

constexpr std::uint32_t task_stack_words = 512;
constexpr std::uint32_t blinker_stack_words = 256;

StaticTask_t sampler_task_tcb;
StackType_t sampler_task_stack[task_stack_words];
StaticTask_t role1_single_board_task_tcb;
StackType_t role1_single_board_task_stack[task_stack_words];
StaticTask_t blinker_tcb;
StackType_t blinker_stack[blinker_stack_words];
}  // namespace

int main()
{
  WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;

  if (debugger_connected()) {
    initialise_monitor_handles();
    std::setvbuf(stdout,
                 semihosting_stdout_buffer.data(),
                 _IOLBF,
                 semihosting_stdout_buffer.size());
    semihosting_ready = true;
  }

  sample_request = xSemaphoreCreateBinaryStatic(&sample_request_storage);
  batch_queue = xQueueCreateStatic(1,
                                    sizeof(temperature_batch),
                                    batch_queue_buffer,
                                    &batch_queue_storage);

  static sampler_context sampler = {};
  sampler.sample_request = sample_request;
  sampler.batch_queue = batch_queue;

  configASSERT(xTaskCreateStatic(sampler_task,
                                 "sampler_task",
                                 task_stack_words,
                                 &sampler,
                                 tskIDLE_PRIORITY + 1,
                                 sampler_task_stack,
                                 &sampler_task_tcb) != nullptr);
  configASSERT(xTaskCreateStatic(role1_single_board_task,
                                 "role1_single_board",
                                 task_stack_words,
                                 nullptr,
                                 tskIDLE_PRIORITY + 1,
                                 role1_single_board_task_stack,
                                 &role1_single_board_task_tcb) != nullptr);
  configASSERT(xTaskCreateStatic(blinker_task,
                                 "blinker",
                                 blinker_stack_words,
                                 nullptr,
                                 tskIDLE_PRIORITY + 1,
                                 blinker_stack,
                                 &blinker_tcb) != nullptr);

  debug_printf("Role 1 single-board test running.\n");
  vTaskStartScheduler();

  configASSERT(false);
  while (true) {
  }
}

extern "C" void vApplicationGetIdleTaskMemory(
  StaticTask_t** ppxIdleTaskTCBBuffer,
  StackType_t** ppxIdleTaskStackBuffer,
  uint32_t* pulIdleTaskStackSize)
{
  static StaticTask_t idle_task_tcb;
  static StackType_t idle_task_stack[configMINIMAL_STACK_SIZE];

  *ppxIdleTaskTCBBuffer = &idle_task_tcb;
  *ppxIdleTaskStackBuffer = idle_task_stack;
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}

extern "C" void vApplicationGetTimerTaskMemory(
  StaticTask_t** ppxTimerTaskTCBBuffer,
  StackType_t** ppxTimerTaskStackBuffer,
  uint32_t* pulTimerTaskStackSize)
{
  static StaticTask_t timer_task_tcb;
  static StackType_t timer_task_stack[configTIMER_TASK_STACK_DEPTH];

  *ppxTimerTaskTCBBuffer = &timer_task_tcb;
  *ppxTimerTaskStackBuffer = timer_task_stack;
  *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
}

extern "C" void vApplicationMallocFailedHook(void)
{
  while (true) {
  }
}

extern "C" void vApplicationStackOverflowHook(TaskHandle_t, char*)
{
  while (true) {
  }
}
