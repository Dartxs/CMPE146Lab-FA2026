#include <cstdint>

#include "msp0g_gpio.hpp"

namespace {

    constexpr std::uintptr_t gpio_a_base = 0x400A'0000;
    constexpr std::uintptr_t gpio_b_base = 0x400A'2000;
    constexpr std::uintptr_t iomux_base = 0x4042'8000;

    constexpr std::uintptr_t pwren_offset = 0x0800;
    constexpr std::uintptr_t rstctl_offset = 0x804;
    constexpr std::uintptr_t doutset_offset = 0x1290; // doutset31_0
    constexpr std::uintptr_t doutclr_offset = 0x12A0; // doutclr31_0
    constexpr std::uintptr_t doeset_offset = 0x12D0; // doeset31_0
    constexpr std::uintptr_t doeclr_offset = 0x12E0; // doeclr31_0
    constexpr std::uintptr_t din_offset = 0x1380; // din31_0

    constexpr std::uint32_t pwren_mask = 0x2600'0001;
    constexpr std::uint32_t rstctl_mask = 0xB100'0001;

    constexpr std::uint32_t pf_gpio_mask = 0x0000'0001;
    constexpr std::uint32_t pc_mask = 1U << 7;
    constexpr std::uint32_t pipd_mask = 1U << 16;
    constexpr std::uint32_t pipu_mask = 1U << 17;
    constexpr std::uint32_t inena_mask = 1U << 18;
    constexpr std::uint32_t hiz1_mask = 1U << 25;

    volatile std::uint32_t* gpio_reg(msp0g::gpio_port p_port, std::uintptr_t p_offset){

        std::uintptr_t base;

        if(p_port == msp0g::gpio_port::a){

            base = gpio_a_base;

        } else{

            base = gpio_b_base;

        }
        // NOLINTNEXTLINE(performance-no-int-to-ptr): MMIO register access
        return reinterpret_cast<volatile std::uint32_t*>(base + p_offset);

    }

    // NOLINTNEXTLINE(performance-no-int-to-ptr): MMIO register access
    auto* iomux_registers = reinterpret_cast<volatile std::uint32_t*>(iomux_base + 0x4); 

    // iomux_registers[x] corresponds to PINCM(x+1), this helpers hides the indexing issue and ensures PINCM is within boundary
    volatile std::uint32_t* pincm(std::uint8_t p_pincm){

        if(p_pincm < 1U || p_pincm > 60U){
            return nullptr;
        }

        return &iomux_registers[p_pincm - 1U];
    }

    bool valid_pin(msp0g::gpio_port p_port, std::uint8_t p_pin)
    {

        if (p_port == msp0g::gpio_port::a) {
            return (p_pin <= 31);
        }

        return (p_pin <= 27);
    }

}

namespace msp0g{


    msp0g_outpin::msp0g_outpin(const gpio_pin& p_gpio_pin, const settings& p_settings) : m_gpio_pin(p_gpio_pin){

        m_valid_pin = pincm(m_gpio_pin.pincm) != nullptr || 
                    valid_pin(m_gpio_pin.port, m_gpio_pin.pin);

        if(m_valid_pin){

            m_pin_mask = 1U << m_gpio_pin.pin;

            configure(p_settings);

            auto* doeset = gpio_reg(p_gpio_pin.port, doeset_offset);

            *doeset = m_pin_mask;

        }
        
    }

    bool msp0g_outpin::driver_configure(const settings& p_settings){
        
        if (!m_valid_pin){
            return false;
        }

        auto* pincm_reg = pincm(m_gpio_pin.pincm);

        *pincm_reg = pf_gpio_mask | pc_mask | inena_mask;

        if (p_settings.resistor == lab1::pin_resistor::pull_down) {

            *pincm_reg |= pipd_mask;

        } else if (p_settings.resistor == lab1::pin_resistor::pull_up) {

            *pincm_reg |= pipu_mask;

        } else {

            *pincm_reg = *pincm_reg & ~pipu_mask & ~pipd_mask;

        }

        if (p_settings.open_drain) {

            *pincm_reg |= hiz1_mask;

        } else {

            *pincm_reg &= ~hiz1_mask;

        }

        return true;

    }

    void msp0g_outpin::driver_level(bool p_high){

        if (!m_valid_pin){

            return;

        }

        if (p_high){

            auto* doutset = gpio_reg(m_gpio_pin.port, doutset_offset);
            
            *doutset = m_pin_mask;

        } else{

            auto* doutclr = gpio_reg(m_gpio_pin.port, doutclr_offset);

            *doutclr = m_pin_mask;

        }

    }

    bool msp0g_outpin::driver_level(){

        if(!m_valid_pin){

            return false;

        }

        auto* din = gpio_reg(m_gpio_pin.port, din_offset);

        return (*din & m_pin_mask);

    }

    msp0g_inpin::msp0g_inpin(const gpio_pin& p_gpio_pin, const settings& p_settings) : m_gpio_pin(p_gpio_pin){

        m_valid_pin = pincm(m_gpio_pin.pincm) != nullptr || 
                    valid_pin(m_gpio_pin.port, m_gpio_pin.pin);

        if(m_valid_pin){

            m_pin_mask = 1U << m_gpio_pin.pin;

            configure(p_settings);

            auto* doeclr = gpio_reg(p_gpio_pin.port, doeclr_offset);

            *doeclr = m_pin_mask;

        }

    }

    bool msp0g_inpin::driver_configure(const settings& p_settings){

        if (!m_valid_pin){
            return false;
        }

        auto* pincm_reg = pincm(m_gpio_pin.pincm);

        *pincm_reg = pf_gpio_mask | pc_mask | inena_mask;

        if (p_settings.resistor == lab1::pin_resistor::pull_down) {

            *pincm_reg |= pipd_mask;

        } else if (p_settings.resistor == lab1::pin_resistor::pull_up) {

            *pincm_reg |= pipu_mask;

        } else {

            *pincm_reg = *pincm_reg & ~pipu_mask & ~pipd_mask;

        }

        return true;

    }

    bool msp0g_inpin::driver_level(){

        if(!m_valid_pin){

            return false;

        }

        auto* din = gpio_reg(m_gpio_pin.port, din_offset);

        return (*din & m_pin_mask);
        
    }

    void initialize_gpio_port(gpio_port p_port){

        auto* rstctl = gpio_reg(p_port, rstctl_offset);
        auto* pwren = gpio_reg(p_port, pwren_offset);

        *rstctl = rstctl_mask;
        *pwren = pwren_mask;

        for(volatile int i = 0; i < 16; i++); // required delay after setting PWREN

    }

}
