#include <cstdio>
#include <array>

#include "../hal/gpio.hpp"
#include "../hal/spi.hpp"
#include "../drivers/msp0g_gpio.hpp"
#include "../drivers/msp0g_iomux.hpp"
#include "../drivers/msp0g_spi.hpp"

namespace{

  //PB12
  msp0g::gpio_pin cs_flash_pin = {msp0g::gpio_port::b, 12, 29};

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
  constexpr uint8_t read_stat1_command = 0x05;
  constexpr uint8_t sector_erase_command = 0x20;

  //flash stat reg bit masks
  constexpr uint8_t busy_mask = 1;
  constexpr uint8_t wel_mask = (1 << 1);
}

/**
 * @brief Access SPI flash identification, data, and erase/program operations.
 *
 * Uses JEDEC ID and standard read, page program, and 4 KB sector erase commands.
 * The caller supplies the SPI peripheral and an active-low GPIO chip select.
 * Both objects must remain valid for the lifetime of this driver.
 */
class flash_memory
{
public:
  /**
   * @brief Associate the flash with an SPI peripheral and chip select.
   *
   * @param p_spi Non-null pointer to the configured SPI peripheral.
   * @param p_chip_select Non-null pointer to the configured output pin.
   * @pre Chip select is high before the first transaction.
   * @note The pointers are stored without taking ownership of either object.
   */
  flash_memory(lab3::spi* p_spi, lab1::output_pin* p_chip_select)
    : m_spi(p_spi)
    , m_chip_select(p_chip_select)
  {
  }

  // TODO(lab3, step 2): Add APIs to access various parts of the flash memory as
  // required by the lab.

  /**
   * @brief Read the JEDEC ID and print its manufacturer byte in hexadecimal.
   */
  void print_manufacturerID(){

    std::array<std::uint8_t, 1> tx_buffer = {jedec_id_command};
    std::array<std::uint8_t, 3> rx_buffer;

    m_chip_select->level(false);

    m_spi->transfer(tx_buffer, {});
    m_spi->transfer({}, rx_buffer);

    m_chip_select->level(true);

    std::printf("Manufacturer ID: 0x%02X\n", rx_buffer[0]);
    // std::printf("RX: %02X %02X %02X\n",
    //         rx_buffer[0], rx_buffer[1],
    //         rx_buffer[2]);

  }

  /**
   * @brief Read the JEDEC capacity code and print the capacity in bytes.
   *
   * Interprets the capacity code as the exponent in 2^code bytes.
   */
  void print_capacity(){

    std::array<uint8_t, 1> tx_buffer = {jedec_id_command};
    std::array<uint8_t, 3> rx_buffer;

    m_chip_select->level(false);

    m_spi->transfer(tx_buffer, {});
    m_spi->transfer({}, rx_buffer);

    m_chip_select->level(true);

    uint64_t capacity_bytes = 1U << rx_buffer[2];

    std::printf("Capacity: %llu bytes\n", static_cast<unsigned long long>(capacity_bytes));

    // std::printf("RX: %02X %02X %02X\n",
    //     rx_buffer[0], rx_buffer[1],
    //     rx_buffer[2]);

  }

  /**
   * @brief Read 32 consecutive bytes and print them as characters.
   *
   * @param address Starting byte address; only the lower 24 bits are sent.
   * @pre The flash is ready to accept a read command.
   * @note Chip select remains low across the command, address, and data.
   *       The output is followed by a newline; non-text bytes may not display.
   */
  void block_read(std::uint32_t address){

    std::array<std::uint8_t, 4> tx_buffer = 
                      {read_command, 
                      static_cast<std::uint8_t>((address >> 16) & 0xFF), 
                      static_cast<std::uint8_t>((address >> 8) & 0xFF), 
                      static_cast<std::uint8_t>(address & 0xFF)};

    std::array<std::uint8_t, 32> rx_buffer;

    m_chip_select->level(false);

    m_spi->transfer(tx_buffer, {});
    m_spi->transfer({}, rx_buffer);

    m_chip_select->level(true);

    for(int i = 0; i < rx_buffer.size(); i++){

      std::printf("%c", static_cast<char>(rx_buffer[i]));

    }

    std::printf("\n");

  }

  /**
   * @brief Read up to 32 console bytes, erase their sector, and program them.
   *
   * Input ends at newline, EOF, or 32 bytes. Backspace and Delete remove the
   * previous buffered byte. Empty input leaves the flash unchanged. Otherwise,
   * the entire containing 4 KB sector is erased before the input is programmed.
   * The newline is not stored, and no null terminator is appended.
   *
   * @param address Starting byte address; only the lower 24 bits are sent.
   * @pre The flash is ready, and the input fits within one 256-byte page.
   * @pre CCS console I/O is active before this function reads input.
   * @note Blocks until programming completes. Other data in the erased sector
   *       is not preserved.
   */
  void block_write(std::uint32_t address){

    std::array<std::uint8_t, 4> tx_addr_buffer = 
              {write_command, 
              static_cast<std::uint8_t>((address >> 16) & 0xFF), 
              static_cast<std::uint8_t>((address >> 8) & 0xFF), 
              static_cast<std::uint8_t>(address & 0xFF)};

    std::array<std::uint8_t, 32> tx_data_buffer;

    std::uint8_t length = 0;

    std::printf("Data to write(up to 32): ");
    std::fflush(stdout);

    while(length < tx_data_buffer.size()){
      int ch = std::getchar();

      if(ch == 0x08 || ch == 0x7F){
        if(length > 0){
          length--;
        }
        continue;
      }

      if(ch == EOF || ch == '\n'){

        break;
      
      }

      tx_data_buffer[length++] = static_cast<std::uint8_t>(ch);

    }

    if(length == 0){
      
      return;

    }

    sector_erase(address);

    ena_write();

    m_chip_select->level(false);

    m_spi->transfer(tx_addr_buffer, {});
    m_spi->transfer(std::span<const std::uint8_t>(tx_data_buffer.data(), length), {});

    m_chip_select->level(true);

    wait_until_ready(busy_mask, false);

  }

  /**
   * @brief Erase the 4 KB sector containing the address and wait for completion.
   *
   * @param address Any byte address within the sector; only the lower 24 bits
   *                are sent. The address need not be sector-aligned.
   * @pre The flash is ready to accept an erase command.
   * @note Enables writes before erasing. All bytes in the sector become 0xFF.
   */
  void sector_erase(std::uint32_t address){

    std::array<std::uint8_t, 4> tx_buffer = 
        {sector_erase_command, 
        static_cast<std::uint8_t>((address >> 16) & 0xFF), 
        static_cast<std::uint8_t>((address >> 8) & 0xFF), 
        static_cast<std::uint8_t>(address & 0xFF)};

    ena_write();

    m_chip_select->level(false);

    m_spi->transfer(tx_buffer, {});

    m_chip_select->level(true);

    wait_until_ready(busy_mask, false);

  }

private:
  lab3::spi* m_spi = nullptr;
  lab1::output_pin* m_chip_select = nullptr;

  /**
   * @brief Send Write Enable and wait for the write-enable latch to be set.
   *
   * The command ends with chip select high before status polling begins.
   */
  void ena_write(){

    m_chip_select->level(false);

    m_spi->transfer(std::span<const std::uint8_t>(&write_ena_command, 1), {});

    m_chip_select->level(true);

    wait_until_ready(wel_mask, true);

  }

  /**
   * @brief Poll Status Register 1 until the selected bits match the expectation.
   *
   * @param mask Status bits to inspect, such as BUSY or WEL.
   * @param expected True waits for any masked bit to be set; false waits for
   *                 all masked bits to be clear.
   * @note Reads at least one status byte and keeps chip select low while
   *       polling. There is no timeout if the expected state is never reached.
   */
  void wait_until_ready(std::uint8_t mask, bool expected){

    m_chip_select->level(false);

    m_spi->transfer(std::span<const std::uint8_t>(&read_stat1_command, 1), {});

    std::array<uint8_t, 1> rx_stat_buffer = {0};

    do{

      m_spi->transfer({}, rx_stat_buffer);

    }
    while(((rx_stat_buffer[0] & mask) != 0) != expected);

    m_chip_select->level(true);

  }

};

/**
 * @brief Gate console access until the CCS debugger is ready to service I/O.
 *
 * Set this flag to true through CCS Expressions while execution is paused,
 * then resume. It starts false so a standalone boot waits before console input
 * can trigger an erase or write.
 */
volatile bool console_ready = false;

/**
 * @brief Demonstrate flash identification, reading, and console-driven writing.
 *
 * Waits for console_ready, configures the SPI pins and GPIO chip select, then
 * prints identification and capacity, reads address zero, writes user input,
 * and reads back the result. Remains in an infinite loop afterward.
 */
int main()
{

  while(!console_ready);
  
  msp0g::msp0g_outpin cs_flash(cs_flash_pin, {});
  cs_flash.level(true);

  msp0g::msp0g_iomux sclk_iomux(sclk_config, {});
  msp0g::msp0g_iomux poci_iomux(poci_config, {});
  msp0g::msp0g_iomux pico_iomux(pico_config, {});

  msp0g::msp0g_spi flash_spi({}, msp0g::spix::spi0);

  flash_memory flash(&flash_spi, &cs_flash);

  flash.print_manufacturerID();

  flash.print_capacity();

  flash.block_read(0);

  flash.block_write(0);

  flash.block_read(0);

  // TODO(lab3, step 3): Construct an spi driver object and gpio driver object
  // in order to construct a flash memory object. Use the flash memory object to
  // retrieve and print the manufacturer ID, capacity, and the first 32-byte
  // block of the flash memory. Then take user input and write up to 32 bytes to
  // that block.

  while (true) {
    continue;  // loop here forever
  }
}
