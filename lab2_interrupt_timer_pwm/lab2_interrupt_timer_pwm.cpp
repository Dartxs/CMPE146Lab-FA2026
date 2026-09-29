#include <array>
#include <cinttypes>
#include <cstdint>
#include <cstdio>
#include <limits>

#include "../drivers/msp0g_gpio.hpp"
#include "../drivers/msp0g_iomux.hpp"
#include "../drivers/msp0g_timer.hpp"
#include "../hal/gpio.hpp"
#include "../hal/timer.hpp"
#include "../hal/timer_util.hpp"

// Test double for lab2::steady_clock. Does not touch real hardware - it just
// increments a counter every time uptime() is called, as if a timer tick had
// elapsed. Useful for exercising delay logic before your real Timer_A-backed
// steady_clock exists.
// class fake_steady_clock : public lab2::steady_clock
// {
// public:
//   fake_steady_clock() = default;

// private:
//   std::uint32_t driver_frequency() override
//   {
//     // Use a small frequency to reduce the amount of ticks needed
//     // for the delay function.
//     return 1'000'000;
//   }

//   std::uint64_t driver_uptime() override
//   {
//     return m_count++;
//   }

//   std::uint64_t m_count = 0;
// };

namespace {

struct led_config
{

  msp0g::pwm_config pwm_config;
  msp0g::iomux_config iomux_config;
};

constexpr uint32_t led_freq = 500U;

constexpr led_config red_led = {
  { led_freq, msp0g::pwm_timer::timg6, msp0g::pwm_timer_channel::cc0 },
  { 57, 5 }
};

constexpr led_config green_led = {
  { led_freq, msp0g::pwm_timer::timg6, msp0g::pwm_timer_channel::cc1 },
  { 58, 5 }
};

constexpr led_config blue_led = {
  { led_freq, msp0g::pwm_timer::timg8, msp0g::pwm_timer_channel::cc1 },
  { 50, 3 }
};

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
  for (int x = 0; x < CycleSteps; x++) {
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

  // TODO(lab2, step 1): Implement lab2::steady_clock

  // TODO(lab2, step 2): Pass your steady clock to lab2::delay() from
  // hal/timer_util.hpp and test it with printf or blinking an LED - your
  // choice. Put a printf on either side of the delay and confirm the gap
  // between them matches the duration you asked for. If a 1s delay is not
  // taking 1 second, your frequency() is wrong.

  // TODO(lab2, step3): Implement lab2::pwm using what you learned from
  // lab2::steady_clock

  // TODO(lab2, step4): Test against an LED and see if you can control the
  // brightness

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

    // TODO(lab2, step 5): Use the steady clock together with your PWM driver to
    // animate the RGB LED as a continuous color wheel, as described in
    // README.md.
  }
}
