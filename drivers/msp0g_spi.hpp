#pragma once

#include <cstdint>

#include "../hal/spi.hpp"

namespace msp0g{

    enum class spix : std::uint8_t{
        spi0 = 0,
        spi1
    };

    class msp0g_spi : public lab3::spi{

    public:

        msp0g_spi(settings const& p_settings, spix p_spix);

    private:

        std::uintptr_t m_spi_base;
        bool m_valid;

        bool driver_configure(settings const& p_settings) override;
        void driver_transfer(std::span<std::uint8_t const> p_data_out,      
                             std::span<std::uint8_t> p_data_in, 
                             std::uint8_t p_filler) override;

    };

}