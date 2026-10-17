#include <cstdint>
#include <cstdio>

#include "../drivers/msp0g_gpio.hpp"
#include "../hal/gpio.hpp"

constexpr msp0g::gpio_pin sw1_pin{ msp0g::gpio_port::a, 18, 40 };
constexpr msp0g::gpio_pin sw2_pin{ msp0g::gpio_port::b, 21, 49 };
constexpr msp0g::gpio_pin blue_led_pin{ msp0g::gpio_port::b, 22, 50 };
constexpr msp0g::gpio_pin red_led_pin{ msp0g::gpio_port::b, 26, 57 };
constexpr msp0g::gpio_pin green_led_pin{ msp0g::gpio_port::b, 27, 58 };

int main()
{

  // msp0g::initialize_gpio_port(msp0g::gpio_port::a);
  // msp0g::initialize_gpio_port(msp0g::gpio_port::b);

  lab1::input_pin::settings sw1_settings{ lab1::pin_resistor::none };

  msp0g::msp0g_inpin sw1{ sw1_pin, sw1_settings };
  msp0g::msp0g_inpin sw2{ sw2_pin };

  msp0g::msp0g_outpin blue_led{ blue_led_pin };
  msp0g::msp0g_outpin red_led{ red_led_pin };
  msp0g::msp0g_outpin green_led{ green_led_pin };

  while (true) {

    if (sw1.level() && !sw2.level()) {  // sw2 active low

      blue_led.level(true);
      red_led.level(true);
      green_led.level(true);

    } else if (sw1.level()) {

      blue_led.level(false);
      red_led.level(true);
      green_led.level(true);

    } else if (!sw2.level()) {

      blue_led.level(true);
      red_led.level(true);
      green_led.level(false);

    } else {

      blue_led.level(false);
      red_led.level(false);
      green_led.level(false);
    }
  }
}
