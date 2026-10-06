#include <cstddef>
#include <cstdint>
#include <cstdio>

#include "../hal/gpio.hpp"
#include "../hal/timer.hpp"
#include "../hal/timer_util.hpp"


//Addresses of the three timers. Red and green use TIMG6, blue uses TIMG8, the clock uses TIMG12.
constexpr std::uintptr_t TIMG6  = 0x40868000;
constexpr std::uintptr_t TIMG8  = 0x40090000;
constexpr std::uintptr_t TIMG12 = 0x40870000;

// steady clock
// TIMG12 counts up by itself, and we just read its value.
class my_clock : public lab2::steady_clock
{
public:
    my_clock()
    {
        volatile std::uint32_t* power  = (volatile std::uint32_t*)(TIMG12 + 0x0800);
        volatile std::uint32_t* clksel = (volatile std::uint32_t*)(TIMG12 + 0x1008);
        volatile std::uint32_t* clkdiv = (volatile std::uint32_t*)(TIMG12 + 0x1000);
        volatile std::uint32_t* clkctl = (volatile std::uint32_t*)(TIMG12 + 0x1108);
        volatile std::uint32_t* load   = (volatile std::uint32_t*)(TIMG12 + 0x1808);
        volatile std::uint32_t* ctrctl = (volatile std::uint32_t*)(TIMG12 + 0x1804);

        *power  = 0x26000001;// turn the timer on (0x26 is a required key)
        *clksel = 0x8;// use the 32 MHz bus clock
        *clkdiv = 7;// divide by 8, so the timer runs at 4 MHz
        *clkctl = 1; // turn on the timer's clock
        *load   = 0xFFFFFFFF;// count all the way up to the biggest 32-bit number
        *ctrctl = 0x20000023; // start at 0, count up, repeat, enable
    }

private:
    // number of ticks happen every second
    std::uint32_t driver_frequency() override
    {
        return 4000000;
    }

    // How many ticks have happened so far.
    std::uint64_t driver_uptime() override
    {
        volatile std::uint32_t* counter = (volatile std::uint32_t*)(TIMG12 + 0x1800);
        return *counter;
    }
};

// The PWM driver (controls the brightness of one color).
// The timer counts from 0 up to "load". The pin is ON until the
// count reaches "compare", then OFF for the rest of the period.
// A bigger compare value means the LED is on longer, so it is brighter.
class my_pwm : public lab2::pwm
{
public:
    my_pwm(std::uintptr_t timer, int channel, std::uintptr_t pin_address, int pin_function)
    {
        load_reg    = (volatile std::uint32_t*)(timer + 0x1808);
        compare_reg = (volatile std::uint32_t*)(timer + 0x1810 + channel * 4);

        volatile std::uint32_t* power   = (volatile std::uint32_t*)(timer + 0x0800);
        volatile std::uint32_t* clksel  = (volatile std::uint32_t*)(timer + 0x1008);
        volatile std::uint32_t* clkdiv  = (volatile std::uint32_t*)(timer + 0x1000);
        volatile std::uint32_t* prescal = (volatile std::uint32_t*)(timer + 0x110C);
        volatile std::uint32_t* clkctl  = (volatile std::uint32_t*)(timer + 0x1108);
        volatile std::uint32_t* pin     = (volatile std::uint32_t*)(pin_address);
        volatile std::uint32_t* outdir  = (volatile std::uint32_t*)(timer + 0x1100);
        volatile std::uint32_t* ccctl   = (volatile std::uint32_t*)(timer + 0x1830 + channel * 4);
        volatile std::uint32_t* octl    = (volatile std::uint32_t*)(timer + 0x1850 + channel * 4);
        volatile std::uint32_t* ccact   = (volatile std::uint32_t*)(timer + 0x1870 + channel * 4);
        volatile std::uint32_t* ctrctl  = (volatile std::uint32_t*)(timer + 0x1804);

        *power   = 0x26000001;// turn the timer on
        *clksel  = 0x8;// use the 32 MHz bus clock
        *clkdiv  = 7; // divide by 8, timer runs at 4 MHz
        *prescal = 0; // no extra dividing
        *clkctl  = 1; // turn on the timer's clock

        *pin = 0x80 | pin_function; // connect the timer to the LED pin

        *outdir |= (1 << channel);// this channel is an output
        *ccctl   = 0;// compare mode 
        *octl    = 0;// use the timer's output signal
        *ccact   = 0x401;// ON at zero, OFF at compare match

        *load_reg    = 3999;// 4,000,000 / 4000 ticks = 1000 Hz (1 kHz)
        *compare_reg = 2000;// start at 50% brightness

        *ctrctl  = 0x20000023; // start at 0, count up, repeat, enable
    }

private:
    std::uint32_t driver_frequency() override
    {
        return 1000;
    }

    // duty is 0 (off) to 65535 (full brightness).
    void driver_duty_cycle(std::uint16_t duty) override
    {
        std::uint32_t load = *load_reg;
        *compare_reg = (load * duty) / 65535;
    }

    volatile std::uint32_t* load_reg;
    volatile std::uint32_t* compare_reg;
};


//A cosine function, made from a math formula, gives a smooth wave that goes up and down between -1 and 1
double my_cosine(double x)
{
    double result = 1;
    double power = x;
    double factorial = 1;

    for (int n = 1; n < 19; n++) {
        factorial = factorial * n;
        if (n % 2 == 0) {
            if (n % 4 == 2) {
                result = result - power / factorial;
            } else {
                result = result + power / factorial;
            }
        }
        power = power * x;
    }
    return result;
}

int main()
{
    using namespace std::chrono_literals;

    my_clock clock;

    // Timer, channel, pin address, pin function
    my_pwm red(TIMG6, 0, 0x404280E4, 5); // pin PB26
    my_pwm green(TIMG6, 1, 0x404280E8, 5);// pin PB27
    my_pwm blue(TIMG8, 1, 0x404280C8, 3);// pin PB22

    // Build a table of 628 brightness values (a full cosine wave).
    const int steps = 628;
    std::uint16_t table[steps];
    double pi = 3.14159265358979;

    for (int i = 0; i < steps; i++) {
        double wave = my_cosine((2.0 * pi / steps) * i);// -1 to 1
        double zero_to_one = (wave + 1.0) / 2.0;// 0 to 1
        table[i] = (std::uint16_t)(zero_to_one * 65535);// 0 to 65535
    }

    // Green and blue are shifted so the three colors are out of step.
    // One third of the table is a 120 degree shift.
    int shift = steps / 3;

    while (true) {
        for (int i = 0; i < steps; i++) {
            red.duty_cycle(table[i]);
            green.duty_cycle(table[(i + shift) % steps]);
            blue.duty_cycle(table[(i + 2 * shift) % steps]);

            lab2::delay(clock, 5ms);// wait 5 milliseconds
        }
    }
}