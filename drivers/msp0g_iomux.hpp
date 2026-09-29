#pragma once

#include <cstdint>

#include "../hal/gpio.hpp"

namespace msp0g{

    /**
     * @brief Removes the need for a boolean inena
     *
     */
    enum class input_enable{
        disable = 0,
        enable
    };

    struct iomux_settings{
      
        lab1::pin_resistor resistor = lab1::pin_resistor::none;
        bool open_drain = false;

    };

    struct iomux_config{

        uint8_t pincm;
        uint8_t pf_sel;
        input_enable inena;

    };

    class msp0g_iomux{

        public:

            /**
             * @brief Constructor for iomux. 
             *
             * @param p_iomux_config Struct containing initial settings {pincm, pf_sel, inena}.
             * @param p_iomux_settings Struct containing initial configuration {resistor, open_drain}, 
             * defaults to no resistor and open_drain disable. 
             *                 
             * pincm is boundary checked resulting in an invalid and unusable pin. 
             * pf_sel is checked to ensure it's <= 0x3F, but doesn't check whether a peripheral actually
             * maps to that sel. 
             * 
             */
            msp0g_iomux(const iomux_config& p_iomux_config, const iomux_settings& p_iomux_settings);

            /**
             * @brief Function to modify resistor/open drain settings for pin
             *
             * @param p_iomux_settings Struct containing settings {pincm, pf_sel, inena}.
             *                 
             * 
             */
            void config(const iomux_settings& p_iomux_settings);

            /**
             * @brief Check whether pin configuration is valid
             *
             * @return bool - TRUE = valid pin, FALSE = invalid pin, object will not control hardware
             * 
             */
            bool valid_pincm();

        private:
            std::uint8_t m_pincm;
            std::uint8_t m_pf_sel;
            bool m_valid;

    };
}