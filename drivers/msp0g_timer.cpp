#include <cstdint>

#include "msp0g_timer.hpp"

namespace {

// to keep track of the state for each timer
struct timer_state
{

  std::uint32_t freq;
  bool initiated;
  std::uint8_t channels;
};

// index 0 corresponds to timg0, 1 = timg6, 2 = timg7, 3 = timg8
timer_state timers[4];

// array to store base addresses for timg0, timg6, timg7, timg8
constexpr std::uintptr_t timg_bases[4] = { 0x4008'4000,
                                           0x4086'8000,
                                           0x4086'A000,
                                           0x4009'0000 };

constexpr std::uint32_t steady_clock_frequency = 4'000'000;
constexpr std::uint32_t pwm_timer_frequency = 1'000'000;

/* pwm frequency has to be high enough for the counter's resolution.
for a timer frequency of 1MHz and counter resolution of 16 bits, the
pwm has to operate > 1,000,000/65535 = 15.26Hz, rounding up to 16
*/
constexpr std::uint32_t pwm_min_frequency = 16;

constexpr std::uintptr_t timg12_base = 0x4087'0000;

constexpr std::uintptr_t pwren_offset = 0x0800;
constexpr std::uintptr_t rstctl_offset = 0x0804;

// set low 3 bits to 0-7 to divide clock by 1-8. ex. 0 = no divide,
// 7 = divide by 8
constexpr std::uintptr_t clkdiv_offset = 0x1000;
constexpr std::uintptr_t clksel_offset =
  0x1008;  // set bit 3 = BUSCLK, 2 = MFCLK, 1 = LFCLK

constexpr std::uintptr_t ccpd_offset =
  0x1100;  // configure CCP direction (input or output)
constexpr std::uintptr_t cps_offset =
  0x110C;  // set low byte to further divide clock
constexpr std::uintptr_t ctr_offset = 0x1800;  // register storing timer count

// see data sheet, configures start value, count up/down, and more
constexpr std::uintptr_t ctrctl_offset = 0x1804;
constexpr std::uintptr_t load_offset =
  0x1808;  // set low 2 bytes, value loaded to timer

// offsets for channel 0/1 capture/compare registers, add 4 for channel 1
constexpr std::uintptr_t cc_01_offset = 0x1810;
constexpr std::uintptr_t ccact_01_offset = 0x1870;

constexpr std::uint32_t pwren_mask = 0x2600'0001;
constexpr std::uint32_t rstctl_mask = 0xB100'0001;

// clksel
constexpr std::uint32_t busclk_mask = 1 << 3;

// ctrctl
constexpr std::uint32_t cvae_zero = 0x2 << 28;  // value set after enabling, 2=0
constexpr std::uint32_t cm_up = 0x2 << 4;       // count mode 2=up
constexpr std::uint32_t repeat_cont =
  0x1 << 1;  // whether to repeat if 0, 0 = no repeat, 1 = continue
constexpr std::uint32_t en_mask = 0x1;  // enables counter, set to enable

// ccact
constexpr std::uint32_t cuact_setlow =
  0x2 << 9;  // set output low during compare event while counting up
constexpr std::uint32_t zact_sethigh =
  0x1;  // set output high when zero event occurs

volatile std::uint32_t* reg(std::uintptr_t p_base, std::uintptr_t p_offset)
{

  return reinterpret_cast<volatile uint32_t*>(p_base + p_offset);
}

void initialize_timer(std::uintptr_t p_base)
{

  auto* pwren = reg(p_base, pwren_offset);
  auto* rstctl = reg(p_base, rstctl_offset);

  *rstctl = rstctl_mask;
  *pwren = pwren_mask;

  for (volatile int i = 0; i < 16; i++)
    ;  // required delay after setting PWREN
}
}  // namespace

namespace msp0g {

// steady clock
msp0g_steady_clock::msp0g_steady_clock()
{

  m_prev_count = 0;

  initialize_timer(
    timg12_base);  // using timg12 because it has 32-bit resolution, but no cps

  auto* clkdiv = reg(timg12_base, clkdiv_offset);
  auto* clksel = reg(timg12_base, clksel_offset);
  auto* ctrctl = reg(timg12_base, ctrctl_offset);
  auto* load = reg(timg12_base, load_offset);

  *clksel = busclk_mask;

  *clkdiv = 0x7;  // divide 32MHz clk down to 32/(7+1=8) = 4MHz
  *load = 0xFFFFFFFF;

  *ctrctl = cvae_zero | cm_up | repeat_cont | en_mask;
}

std::uint32_t msp0g_steady_clock::driver_frequency()
{

  return steady_clock_frequency;
}

std::uint64_t msp0g_steady_clock::driver_uptime()
{

  auto* ctr = reg(timg12_base, ctr_offset);

  uint32_t current_count = *ctr;

  m_uptime += (current_count - m_prev_count);
  m_prev_count = current_count;

  return m_uptime;
}

// pwm
msp0g_pwm::msp0g_pwm(const pwm_config& p_config)
{

  auto index = static_cast<std::uint8_t>(p_config.timer);

  m_per_count = pwm_timer_frequency / p_config.freq;

  // could be different from requested freq due to truncation
  m_pwm_freq = pwm_timer_frequency / m_per_count;
  m_timer_base = timg_bases[index];
  m_channel = static_cast<std::uint8_t>(p_config.cc);

  uint32_t cc_mask = (0x1 << m_channel);

  if (timers[index].initiated) {
    if (m_pwm_freq != timers[index].freq ||
        (timers[index].channels & cc_mask)) {
      m_valid = false;
      return;
    }
  } else {

    if (m_pwm_freq > pwm_timer_frequency || m_pwm_freq < pwm_min_frequency) {
      m_valid = false;
      return;
    }

    initialize_timer(m_timer_base);
    auto* clksel = reg(m_timer_base, clksel_offset);
    auto* cps = reg(m_timer_base, cps_offset);
    auto* ctrctl = reg(m_timer_base, ctrctl_offset);
    auto* load = reg(m_timer_base, load_offset);

    *clksel = busclk_mask;
    *cps = 31U;  // divide clock by 31+1 for 1MHz clock

    *load = (m_per_count - 1);

    *ctrctl = cvae_zero | cm_up | repeat_cont | en_mask;

    timers[index].initiated = true;
    timers[index].freq = m_pwm_freq;
  }

  auto* ccpd = reg(m_timer_base, ccpd_offset);
  auto* ccact_01 = reg(m_timer_base, ccact_01_offset + (m_channel * 4));

  *ccpd |= cc_mask;
  *ccact_01 = cuact_setlow | zact_sethigh;

  timers[index].channels |= cc_mask;
}

std::uint32_t msp0g_pwm::driver_frequency()
{

  if (!m_valid) {
    return 0;
  }

  return m_pwm_freq;
}

void msp0g_pwm::driver_duty_cycle(std::uint16_t p_duty_cycle)
{

  if (!m_valid) {
    return;
  }

  auto* cc_01 = reg(m_timer_base, cc_01_offset + (m_channel * 4));

  *cc_01 =
    ((static_cast<std::uint32_t>(p_duty_cycle)) * (m_per_count - 1)) / 65535;
}

}  // namespace msp0g
