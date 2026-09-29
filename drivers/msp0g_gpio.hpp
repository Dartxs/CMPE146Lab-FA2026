#pragma once

#include <cstdint>

#include "../hal/gpio.hpp"
#include "msp0g_iomux.hpp"

// Peripheral Base Addresses:
// GPIOA = 0x400A0000, GPIOB = 0x400A2000
// IOMUX = 0x40428000

// GPIO Register Offsets:
// PWREN = 0x800, RSTCTL = 0x804,
// DOUTSET31_0 = 0x1290, DOUTCLR31_0 = 0x12A0,
// DOESET31_0 = 0x12D0, DOECLR31_0 = 0x12E0, DIN31_0 = 0x1380

// Needed IOMUX Register Offsets:
// SW1 = PA18 = PINCM40, offset 0xA0
// SW2 = PB21 = PINCM49, offset 0xC4
// BLUE = PB22 = PINCM50, offset 0xC8
// RED = PB26 = PINCM57, offset 0xE4
// GREEN = PB27 = PINCM58, offset 0xE8

// Needed GPIO Bit Mapping:
// PWREN -> bit 0 = enable, key = 0x26
// RSTCTL -> bit 0 = reset assert, key = 0x1
// note: key enables writing to the enable bits and goes into
// the highest byte of the reg

// Needed IOMUX Bit Mapping:
// PF = bits 0-5, controls which peripheral to select (like GPIO)
// PC = bit 7, allows connection to from mux to peripheral
// PIPD = bit 16, enables pulldown
// PIPU = bit 17, enables pullup
// INENA = bit 18, allows the peripheral to be read by MCU
// HIZ1 = bit 25, high impedance/open-drain

namespace msp0g{

    enum class gpio_port : std::uint8_t{
        a,
        b
    };

    struct gpio_pin{
        gpio_port port;
        std::uint8_t pin;
        std::uint8_t pincm;
    };

    class msp0g_outpin : public lab1::output_pin
    {

    public:
        /**
         * @brief Constructor for output pin. 
         *
         * @param p_gpio_pin GPIO pin description passed as struct
         * @param p_settings Initial settings, default if omittec.
         *                 
         * If p_gpio_pin contains out of bounds values, configuration 
         * is ignored and the pin will not work. 
         */
        msp0g_outpin(const gpio_pin& p_gpio_pin, const settings& p_settings = {});

    private:
        bool driver_configure(const settings& p_settings) override;

        void driver_level(bool p_high) override;

        bool driver_level() override;

        msp0g_iomux m_pincm;
        std::uint32_t m_pin_mask = 0U;
        gpio_port m_port;
        bool m_valid_pin = false;
    };

    class msp0g_inpin : public lab1::input_pin
    {

    public:
        /**
         * @brief Constructor for input pin.
         *
         * @param p_gpio_pin GPIO pin description passed as struct
         * @param p_settings Initial settings, default if omitted.
         *                 
         * If p_gpio_pin contains out of bounds values, configuration 
         * is ignored and the pin will not work.                
         */
        msp0g_inpin(const gpio_pin& p_gpio_pin, const settings& p_settings = {});

    private:
        bool driver_configure(const settings& p_settings) override;

        bool driver_level() override;

        msp0g_iomux m_pincm;
        std::uint32_t m_pin_mask = 0U;
        gpio_port m_port;
        bool m_valid_pin = false;
    };

    /**
    * @brief Powers on and resets GPIO port.
    *
    * @param p_port Port to initialize.
    *                             
    */
    void initialize_gpio_port(gpio_port p_port);

}