#include <cstdio>

#include "../hal/gpio.hpp"
#include "../hal/spi.hpp"
#include "../drivers/msp0g_gpio.hpp"
#include "../drivers/msp0g_iomux.hpp"
#include "../drivers/msp0g_spi.hpp"

namespace{

  //PA11
  msp0g::gpio_pin cs_flash_pin = {msp0g::gpio_port::a, 11, 22};

  //PA12
  msp0g::iomux_config sclk_config = {34, 3, msp0g::input_enable::disable};

  //controller rx, PA13
  msp0g::iomux_config poci_config = {35, 3, msp0g::input_enable::enable}; 

  //controller tx, PB17
  msp0g::iomux_config pico_config = {43, 3, msp0g::input_enable::disable};

  constexpr uint8_t jedec_id_command = 0x9F;
  constexpr uint8_t read_command = 0x03;
  constexpr uint8_t write_ena_command = 0x06;
  constexpr uint8_t write_command = 0x02;

}

/**
 * @brief Generic SPI flash memory driver
 *
 * This driver works with common SPI flash memory with JEDEC ID
 * This driver needs both an SPI peripheral and an output pin for chip select.
 */
class flash_memory
{
public:
  /**
   * @brief Construct a new flash memory object
   *
   * @param p_spi - spi port connected to the spi flash memory
   * @param p_chip_select - pin connected to spi flash memory chip select
   */
  flash_memory(lab3::spi* p_spi, lab1::output_pin* p_chip_select)
    : m_spi(p_spi)
    , m_chip_select(p_chip_select)
  {
  }

  // TODO(lab3, step 2): Add APIs to access various parts of the flash memory as
  // required by the lab.

  void print_manufacturerID(){

    std::uint8_t tx_buffer[1] = {jedec_id_command};
    std::uint8_t rx_buffer[1];

    m_chip_select->level(false);

    m_spi->transfer(tx_buffer, {});
    m_spi->transfer({}, rx_buffer);

    m_chip_select->level(true);

    std::printf("Manufacturer ID: 0x%02X\n", rx_buffer[0]);

  }

  // void read(std::uint32_t address, std::span<std::uint8_t> destination){

  //   std::uint8_t tx_buffer[4] = {read_command, (address & 0x0F00 >> 16), (address & 0x00F0) >> 8, (address & 0x000F)};

  //   m_chip_select->level(false);

  //   m_spi->transfer(tx_buffer, {});
  //   m_spi->transfer({}, destination);

    

  // }

  // void write(std::uint32_t address, std::span<const std::uint8_t> source){

  // }

private:
  lab3::spi* m_spi = nullptr;
  lab1::output_pin* m_chip_select = nullptr;

};

int main()
{
  
  msp0g::msp0g_iomux sclk_iomux(sclk_config, {});
  msp0g::msp0g_iomux poci_iomux(poci_config, {});
  msp0g::msp0g_iomux pico_iomux(pico_config, {});

  msp0g::initialize_gpio_port(msp0g::gpio_port::a);
  msp0g::msp0g_outpin cs_flash(cs_flash_pin, {});

  msp0g::msp0g_spi flash_spi({}, msp0g::spix::spi0);

  flash_memory flash(&flash_spi, &cs_flash);

  flash.print_manufacturerID();

  // TODO(lab3, step 3): Construct an spi driver object and gpio driver object
  // in order to construct a flash memory object. Use the flash memory object to
  // retrieve and print the manufacturer ID, capacity, and the first 32-byte
  // block of the flash memory. Then take user input and write up to 32 bytes to
  // that block.

  while (true) {
    continue;  // loop here forever
  }
}
