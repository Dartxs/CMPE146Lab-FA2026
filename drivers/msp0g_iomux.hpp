#pragma once

#include <cstdint>

#include "../hal/gpio.hpp"

namespace msp0g{

    enum class input_enable{
        disable = 0,
        enable
    };

    struct iomux_settings{
      
        lab1::pin_resistor resistor;
        bool open_drain;

    };

    struct iomux_config{

        uint8_t pincm;
        uint8_t pf_sel;
        input_enable inena;

    };

    class msp0g_iomux{

        public:

            msp0g_iomux(const iomux_config& p_iomux_config, const iomux_settings& p_iomux_settings);

            void config(const iomux_settings& p_iomux_settings);

            bool valid_pincm();

        private:
            std::uint8_t m_pincm;
            std::uint8_t m_pf_sel;
            bool m_valid;

    };
}