#include <array>
#include <cinttypes>
#include <cstdint>
#include <cstdio>
#include <limits>

#include "../drivers/msp0g_iomux.hpp"
#include "../drivers/msp0g_timer.hpp"
#include "../hal/gpio.hpp"
#include "../hal/timer.hpp"
#include "../hal/timer_util.hpp"

namespace {

struct led_config
{

  msp0g::pwm_config pwm_config;
  msp0g::iomux_config iomux_config;
};

constexpr uint32_t led_freq = 500U;

constexpr led_config red_led = { { .freq = led_freq,
                                   .timer = msp0g::pwm_timer::timg6,
                                   .cc = msp0g::pwm_timer_channel::cc0 },
                                 { .pincm = 57, .pf_sel = 5 } };

constexpr led_config green_led = { { .freq = led_freq,
                                     .timer = msp0g::pwm_timer::timg6,
                                     .cc = msp0g::pwm_timer_channel::cc1 },
                                   { .pincm = 58, .pf_sel = 5 } };

constexpr led_config blue_led = { { .freq = led_freq,
                                    .timer = msp0g::pwm_timer::timg8,
                                    .cc = msp0g::pwm_timer_channel::cc1 },
                                  { .pincm = 50, .pf_sel = 3 } };

constexpr double custom_cosine(double x)
{  // software cosine estimate, taken from lab snippet

  double cos{ 1 }, pow{ x };

  for (auto fac{ 1ull }, n{ 1ull }; n != 19; fac *= ++n, pow *= x)
    if ((n & 1) == 0)
      cos += (n & 2 ? -pow : pow) / fac;

  return cos;
}

template<size_t CycleSteps>
constexpr std::array<std::uint16_t, CycleSteps> generate_cosine_table()
{

  std::array<std::uint16_t, CycleSteps> samples{};
  constexpr auto max = std::numeric_limits<std::uint16_t>::max();
  constexpr double pi = M_PI;
  constexpr double phase_step = (2.0 * pi) / CycleSteps;
  for (std::size_t x = 0; x < CycleSteps; x++) {
    const auto y = (custom_cosine(phase_step * x) + 1.0) / 2.0;
    samples[x] = y * max;
  }

  return samples;
}

constexpr std::uint16_t table_size = 628;
constexpr std::uint16_t offset1 = table_size / 3;
constexpr std::uint16_t offset2 = 2 * (table_size / 3);

const auto uint16_cosine2 = generate_cosine_table<table_size>();

}  // namespace

int main()
{
  using namespace std::chrono_literals;

  msp0g::msp0g_steady_clock clock;

  msp0g::msp0g_pwm red_led_pwm(red_led.pwm_config);
  msp0g::msp0g_iomux red_led_iomux(red_led.iomux_config, {});

  msp0g::msp0g_pwm green_led_pwm(green_led.pwm_config);
  msp0g::msp0g_iomux green_led_iomux(green_led.iomux_config, {});

  msp0g::msp0g_pwm blue_led_pwm(blue_led.pwm_config);
  msp0g::msp0g_iomux blue_led_iomux(blue_led.iomux_config, {});

  while (true) {

    for (int i = 0; i < table_size; i++) {

      red_led_pwm.duty_cycle(uint16_cosine2[i]);

      green_led_pwm.duty_cycle(uint16_cosine2[(i + offset1) % table_size]);

      blue_led_pwm.duty_cycle(uint16_cosine2[(i + offset2) % table_size]);

      lab2::delay(clock, 5ms);
    }
  }
}
