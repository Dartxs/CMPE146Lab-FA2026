#include <cstdio>

#include "../hal/gpio.hpp"
//volatile uint32_t s1_state = 0;
class mspm0_output_pin: public lab1::output_pin{
private:
  volatile uint32_t* pincm;
  volatile uint32_t* dout;//sets pin as high or low
  volatile uint32_t* doe;//enables bit as output
  volatile uint32_t* din;
  volatile uint32_t* pwren;
  uint32_t bit;

  bool driver_configure(settings const& p_settings) override{
    if (p_settings.open_drain)
    {
        return false;
    }
     *pwren = 0x26000001;//enable gpio peripheral
      // Configure gpio function
    uint32_t pincm_value = (1u << 7) | 1u;//bit 7 selects gpio peripheral and 0 bit enables it

    // Configure pull resistor
    if (p_settings.resistor == lab1::pin_resistor::pull_down)
    {
        pincm_value |= (1u << 16);
    }
    else if (p_settings.resistor == lab1::pin_resistor::pull_up)
    {
        pincm_value |= (1u << 17);
    }

    *pincm = pincm_value;
    *dout &= ~(1u<<bit);//initally dout is not enabled
    *doe |= (1u << bit);//enables the pin-bit 0= 1 enables it
    return true;
  }
  void driver_level(bool p_high) override{
    if(p_high)
    {
      *dout |= (1u << bit);
    }
    else 
    {
      *dout&= ~(1u << bit);
    }
  }
  bool driver_level() override 
  {
     return (((*din >> bit) & 1u) != 0);
  }
  
public:
  mspm0_output_pin(volatile uint32_t* p_pincm, volatile uint32_t* p_dout, volatile uint32_t* p_doe, volatile uint32_t* p_din, volatile uint32_t* p_pwren, uint32_t p_bit):pincm(p_pincm),
  dout(p_dout),doe(p_doe), din(p_din),pwren(p_pwren),bit(p_bit){}

};
class mspm0_input_pin: public lab1::input_pin{
private:
  volatile uint32_t* din;
  volatile uint32_t* pincm;
  volatile uint32_t* pwren;
  uint32_t bit;
bool driver_configure(settings const& p_settings) override
{
  *pwren = 0x26000001;//enable gpio peripheral
  uint32_t pincm_value = (1u << 7) | (1u << 18) | 1u;//bit 7 for PC, bit 18 for INENA and bit 0 for PF
// Configure pull resistor
    if (p_settings.resistor == lab1::pin_resistor::pull_down)
    {
        pincm_value |= (1u << 16);
    }
    else if (p_settings.resistor == lab1::pin_resistor::pull_up)
    {
        pincm_value |= (1u << 17);
    }
  *pincm = pincm_value;
return true;
}
bool driver_level() override
{
  return (((*din >> bit) & 1u) != 0);
}
public:
mspm0_input_pin(
    volatile uint32_t* p_pincm,
    volatile uint32_t* p_din,
    volatile uint32_t* p_pwren,
    uint32_t p_bit)
    : din(p_din),
      pincm(p_pincm),
      pwren(p_pwren),
      bit(p_bit)
{
}
};

int main()
{
  //std::printf("Hello, World\n");
  constexpr uintptr_t pwren_offset = 0x800;
  constexpr uintptr_t gpio_base_addr = 0x400A2000;
  constexpr uintptr_t iomux_addr = 0x40428000;
  constexpr uintptr_t pincm_addr_red_led = 0x404280E4;
  constexpr uintptr_t pincm_addr_green_led = (iomux_addr + (4*58));
  constexpr uintptr_t pincm_addr_blue_led = (iomux_addr + (4*50));
  constexpr uintptr_t dout31_0 = 0x1280;
  constexpr uintptr_t doe31_0 = 0x12C0;

  //for using button s1 as input
  constexpr uintptr_t gpioa_base_addr = 0x400A0000;
  constexpr uintptr_t din31_0 = 0x1380;
  constexpr uintptr_t gpioa_din = gpioa_base_addr + din31_0;
  constexpr uintptr_t gpioa_pwren = gpioa_base_addr + pwren_offset;
  constexpr uintptr_t s1_pincm = 0x404280A0;
  //for s2 button
  constexpr uintptr_t s2_pincm = (iomux_addr + (4*49));
  mspm0_output_pin red_led(
    reinterpret_cast<volatile uint32_t*>(pincm_addr_red_led),
    reinterpret_cast<volatile uint32_t*>(gpio_base_addr + dout31_0),
    reinterpret_cast<volatile uint32_t*>(gpio_base_addr + doe31_0),
    reinterpret_cast<volatile uint32_t*>(gpio_base_addr + din31_0),
    reinterpret_cast<volatile uint32_t*>(gpio_base_addr + pwren_offset),
    26
);
  mspm0_output_pin green_led(
    reinterpret_cast<volatile uint32_t*>(pincm_addr_green_led),
    reinterpret_cast<volatile uint32_t*>(gpio_base_addr + dout31_0),
    reinterpret_cast<volatile uint32_t*>(gpio_base_addr + doe31_0),
    reinterpret_cast<volatile uint32_t*>(gpio_base_addr + din31_0),
    reinterpret_cast<volatile uint32_t*>(gpio_base_addr + pwren_offset),
    27
);
  mspm0_output_pin blue_led(
    reinterpret_cast<volatile uint32_t*>(pincm_addr_blue_led),
    reinterpret_cast<volatile uint32_t*>(gpio_base_addr + dout31_0),
    reinterpret_cast<volatile uint32_t*>(gpio_base_addr + doe31_0),
    reinterpret_cast<volatile uint32_t*>(gpio_base_addr + din31_0),
    reinterpret_cast<volatile uint32_t*>(gpio_base_addr + pwren_offset),
    22
);
mspm0_input_pin s1(
    reinterpret_cast<volatile uint32_t*>(s1_pincm),
    reinterpret_cast<volatile uint32_t*>(gpioa_din),
    reinterpret_cast<volatile uint32_t*>(gpioa_pwren),
    18
);
//s2 is port B
mspm0_input_pin s2(
    reinterpret_cast<volatile uint32_t*>(s2_pincm),
    reinterpret_cast<volatile uint32_t*>(gpio_base_addr + din31_0),
    reinterpret_cast<volatile uint32_t*>(gpio_base_addr + pwren_offset),
    21
);

    lab1::output_pin::settings led_settings;
    led_settings.resistor = lab1::pin_resistor::none;
    led_settings.open_drain = false;

    red_led.configure(led_settings);
    //red_led.level(true);
    green_led.configure(led_settings);
    //green_led.level(true);
    blue_led.configure(led_settings);
    //blue_led.level(true);

  lab1::input_pin::settings s1_settings;
  s1_settings.resistor = lab1::pin_resistor::pull_down;

  lab1::input_pin::settings s2_settings;
  s2_settings.resistor = lab1::pin_resistor::pull_up;
  
s1.configure(s1_settings);
s2.configure(s2_settings);
  // TODO(lab1, step 2): Configure the RGB LED pins as outputs and the two
  // push-buttons (S1, S2) as inputs using the gpio:: driver you write in
  // gpio.cpp. See README.md for pin assignments and reference material.

  // TIP: the switches need pull resistor. See the user schematic of the launch
  // pad for which is needed for S1 and S2.
  //red_led.level(true);
  //green_led.level(true);
  //blue_led.level(false);
volatile bool s1_state = false;
volatile bool s2_state = false;
while (true)
{
  s1_state = s1.level();
  s2_state = !s2.level();
  /*red_led.level(!s1_state && s2_state);
  green_led.level(!s2_state && s1_state);
  blue_led.level(s1_state && s2_state);*/
   if (s1_state && !s2_state)
    {
        // Yellow
        red_led.level(true);
        green_led.level(true);
        blue_led.level(false);
    }
    else if (!s1_state && s2_state)
    {
        // white
        red_led.level(true);
        green_led.level(true);
        blue_led.level(true);
    }
    else if (s1_state && s2_state)
    {
        // Blue
        red_led.level(false);
        green_led.level(false);
        blue_led.level(true);
    }
    else
    {
        // OFF
        red_led.level(false);
        green_led.level(false);
        blue_led.level(false);
    }
   
    // TODO(lab1, step 2): implement the button -> LED color behavior
    // described in README.md
  }
}
