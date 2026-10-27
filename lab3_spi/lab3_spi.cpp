#include <cstdio>
#include <cstdint>
#include <array>
#include <span>
#include "../hal/gpio.hpp"
#include "../hal/spi.hpp"

// GPIOB base address
constexpr uintptr_t gpio_b_base = 0x400A2000UL;
// IOMUX base address
constexpr uintptr_t iomux_base = 0x40428000UL;
// GPIOB register offsets
constexpr uintptr_t gpio_pwren_offset  = 0x0800UL;
constexpr uintptr_t gpio_dout_offset   = 0x1280UL;
constexpr uintptr_t gpio_doutset_offset = 0x1290UL;
constexpr uintptr_t gpio_doutclr_offset = 0x12A0UL;
constexpr uintptr_t gpio_doeset_offset = 0x12D0UL;
// PB12 is bit 12 of GPIOB
constexpr uint32_t cs_bit = 12U;
constexpr uint32_t cs_mask = 1U << cs_bit;
// PB12 corresponds to PINCM29
constexpr uintptr_t cs_pincm_address = iomux_base + (4U * 29U);
// Register pointers
volatile uint32_t* const gpio_b_pwren = reinterpret_cast<volatile uint32_t*>(gpio_b_base + gpio_pwren_offset);
volatile uint32_t* const gpio_b_dout = reinterpret_cast<volatile uint32_t*>(gpio_b_base + gpio_dout_offset);
volatile uint32_t* const gpio_b_doutset = reinterpret_cast<volatile uint32_t*>(gpio_b_base + gpio_doutset_offset);
volatile uint32_t* const gpio_b_doutclr = reinterpret_cast<volatile uint32_t*>(gpio_b_base + gpio_doutclr_offset);
volatile uint32_t* const gpio_b_doeset = reinterpret_cast<volatile uint32_t*>(gpio_b_base + gpio_doeset_offset);
volatile uint32_t* const cs_pincm = reinterpret_cast<volatile uint32_t*>(cs_pincm_address);
// PINCM configuration:
// PF = GPIO function(bit 0), PC = input/output control(bit 7).
constexpr uint32_t cs_gpio_config = (1U << 7) | 1U;
// PB17: SPI0_PICO, PINCM43
volatile uint32_t* const pico_pincm = reinterpret_cast<volatile uint32_t*>(iomux_base + 4UL * 43UL);
// PB18: SPI0_SCK, PINCM44
volatile uint32_t* const sck_pincm = reinterpret_cast<volatile uint32_t*>(iomux_base + 4UL * 44UL);
// PB19: SPI0_POCI, PINCM45
volatile uint32_t* const poci_pincm = reinterpret_cast<volatile uint32_t*>(iomux_base + 4UL * 45UL);
// GPIO configuration values
// PINCM fields
constexpr uint32_t pc_mask      = 1U << 7;
constexpr uint32_t inena_mask   = 1U << 18;
// GPIO output configuration
// SPI pin configuration
constexpr uint32_t pf_spi0 = 3U;
constexpr uint32_t spi_output_config = pf_spi0 | pc_mask;
constexpr uint32_t spi_input_config = pf_spi0 | pc_mask | inena_mask;
// SPI0 registers
constexpr uintptr_t spi0_base = 0x40468000UL;
volatile uint32_t* const spi0_pwren = reinterpret_cast<volatile uint32_t*>(spi0_base + 0x0800UL);
volatile uint32_t* const spi0_rstctl = reinterpret_cast<volatile uint32_t*>(spi0_base + 0x0804UL);
volatile uint32_t* const spi0_clkdiv = reinterpret_cast<volatile uint32_t*>(spi0_base + 0x1000UL);
volatile uint32_t* const spi0_clksel = reinterpret_cast<volatile uint32_t*>(spi0_base + 0x1004UL);
volatile uint32_t* const spi0_ctl0 = reinterpret_cast<volatile uint32_t*>(spi0_base + 0x1100UL);
volatile uint32_t* const spi0_ctl1 = reinterpret_cast<volatile uint32_t*>(spi0_base + 0x1104UL);
volatile uint32_t* const spi0_clkctl = reinterpret_cast<volatile uint32_t*>(spi0_base + 0x1108UL);
volatile uint32_t* const spi0_stat = reinterpret_cast<volatile uint32_t*>(spi0_base + 0x1110UL);
volatile uint32_t* const spi0_rxdata =reinterpret_cast<volatile uint32_t*>(spi0_base + 0x1130UL);
volatile uint32_t* const spi0_txdata = reinterpret_cast<volatile uint32_t*>(spi0_base + 0x1140UL);
// SPI0 STAT register bit masks
constexpr uint32_t stat_tnf_mask  = 1U << 1;  // Bit 1: Transmit FIFO not full
constexpr uint32_t stat_rfe_mask  = 1U << 2;  // Bit 2: Receive FIFO empty
constexpr uint32_t stat_busy_mask = 1U << 4;  // Bit 4: SPI busy
// Chip-select output pin implementation
class gpio_b12_chip_select : public lab1::output_pin
{
public:
    bool driver_configure(settings const& p_settings) override
    {
        // Open-drain mode is not implemented in this driver.
        if (p_settings.open_drain)
        {
            return false;
        }
        // Enable GPIOB power.
        *gpio_b_pwren = 0x26000001UL;

        // Configure PB12 as GPIO.
        *cs_pincm = cs_gpio_config;

        // Set the output HIGH before enabling output.
        // HIGH means the SPI flash is deselected.
        *gpio_b_doutset = cs_mask;

        // Enable PB12 as an output.
        *gpio_b_doeset = cs_mask;

        return true;
    }
    void driver_level(bool p_high) override
    {
        if (p_high)
        {
            // CS HIGH: deselect the flash.
            *gpio_b_doutset = cs_mask;
        }
        else
        {
            // CS LOW: select the flash.
            *gpio_b_doutclr = cs_mask;
        }
    }

    bool driver_level() override
    {
        // Read the GPIO output data register.
        return ((*gpio_b_dout & cs_mask) != 0U);
    }
};
// Generic SPI flash memory driver
class flash_memory
{
public:
    flash_memory(
        lab3::spi* p_spi,
        lab1::output_pin* p_chip_select)
        : m_spi(p_spi),
          m_chip_select(p_chip_select)
    {
    }
    bool read_jedec_id(
    std::uint8_t& manufacturer_id,
    std::uint8_t& memory_type,
    std::uint8_t& capacity)
{
    if (m_spi == nullptr || m_chip_select == nullptr)
    {
        return false;
    }
    // Command 0x9F: Read JEDEC ID.
    const std::array<std::uint8_t, 1> command = {0x9F};

    // Receive the command response and three ID bytes.
    std::array<std::uint8_t, 4> received{};

    // Select the flash.
    m_chip_select->level(false);

    // Send 0x9F and three filler bytes.
    m_spi->transfer(command, received, 0xFF);

    // Deselect the flash.
    m_chip_select->level(true);

    // Ignore received[0]; the next three bytes contain the ID.
    manufacturer_id = received[1];
    memory_type     = received[2];
    capacity        = received[3];
    return true;
}
     // Read data starting from a specified flash address.
    bool read(
        std::uint32_t address,
        std::span<std::uint8_t> data)
    {
        if (m_spi == nullptr ||
            m_chip_select == nullptr ||
            data.empty())
        {
            return false;
        }

        // Command 0x03: Read Data.
        // The flash expects a 24-bit address after the command.
        const std::array<std::uint8_t, 4> command = {
            0x03,
            static_cast<std::uint8_t>(address >> 16),
            static_cast<std::uint8_t>(address >> 8),
            static_cast<std::uint8_t>(address)
        };

        // Receive bytes while sending the command and address.
        std::array<std::uint8_t, 4> ignored{};

        // Select the flash.
        m_chip_select->level(false);

        // Send the read command and address.
        m_spi->transfer(command, ignored, 0xFF);

        // Send filler bytes to generate clocks and receive flash data.
        std::span<std::uint8_t const> no_output{};

        m_spi->transfer(no_output, data, 0xFF);

        // Deselect the flash.
        m_chip_select->level(true);

        return true;
    }
    bool erase_sector(std::uint32_t address)
    {
    // Check that the address is valid and sector-aligned.
    if (m_spi == nullptr ||
        m_chip_select == nullptr ||
        address >= flash_size ||
        address % 4096U != 0U)
    {
        return false;
    }

    //Enable writing.
    if (!write_enable())
    {
        return false;
    }

    //Prepare the Sector Erase command (0x20).
    const std::array<std::uint8_t, 4> command = {
        0x20,
        static_cast<std::uint8_t>(address >> 16),
        static_cast<std::uint8_t>(address >> 8),
        static_cast<std::uint8_t>(address)
    };

    std::array<std::uint8_t, 4> ignored{};

    //Select the flash and send the command.
    m_chip_select->level(false);

    m_spi->transfer(command, ignored, 0xFF);

    m_chip_select->level(true);

    // Wait until erasing finishes.
    return wait_until_ready();
    }
// Write up to 32 bytes to flash.
    bool write(
    std::uint32_t address,
    std::span<std::uint8_t const> data)
    {
    // Check the address and input size.
    if (m_spi == nullptr ||
        m_chip_select == nullptr ||
        data.empty() ||
        data.size() > 32U ||
        address >= flash_size ||
        data.size() > flash_size - address)
    {
        return false;
    }

    //Page Program operation must not cross a 256-byte page.
    if ((address % 256U) + data.size() > 256U)
    {
        return false;
    }

    //Enable writing.
    if (!write_enable())
    {
        return false;
    }

    // Prepare the Page Program command (0x02).
    std::array<std::uint8_t, 36> command{};
    std::array<std::uint8_t, 36> ignored{};

    command[0] = 0x02;

    // Add the 24-bit address.
    command[1] = static_cast<std::uint8_t>(address >> 16);
    command[2] = static_cast<std::uint8_t>(address >> 8);
    command[3] = static_cast<std::uint8_t>(address);

    // Copy user data after the command and address.
    for (std::size_t i = 0; i < data.size(); ++i)
    {
        command[4 + i] = data[i];
    }

    const std::size_t transfer_size = 4U + data.size();

    //Send the command, address, and data.
    m_chip_select->level(false);

    m_spi->transfer(
        std::span<std::uint8_t const>(
            command.data(), transfer_size),
        std::span<std::uint8_t>(
            ignored.data(), transfer_size),
        0xFF);

    m_chip_select->level(true);

    // Wait until programming finishes.
    return wait_until_ready();
    }

    // TODO(lab3, step 2): Add APIs to access various parts of
    // the flash memory as required by the lab.
private:
    lab3::spi* m_spi = nullptr;
    lab1::output_pin* m_chip_select = nullptr;
    // Total capacity of the W25Q32JV: 4 MiB.
    static constexpr std::uint32_t flash_size = 4U * 1024U * 1024U;
// Send Write Enable command (0x06).
    bool write_enable()
    {
    const std::array<std::uint8_t, 1> command = {0x06};
    std::array<std::uint8_t, 1> ignored{};

    m_chip_select->level(false);

    m_spi->transfer(command, ignored, 0xFF);

    m_chip_select->level(true);

    // Confirm that WEL (Status Register bit 1) is set.
    return (read_status() & 0x02U) != 0U;
    }
// Read Status Register-1 using command 0x05.
    std::uint8_t read_status()
    {
    const std::array<std::uint8_t, 1> command = {0x05};
    std::array<std::uint8_t, 2> received{};

    m_chip_select->level(false);

    m_spi->transfer(command, received, 0xFF);

    m_chip_select->level(true);

    // The second received byte contains the status register.
    return received[1];
    }
// Wait until the flash is no longer busy.
    bool wait_until_ready()
    {
    for (std::uint32_t attempt = 0;
         attempt < 50000U;
         ++attempt)
    {
        // WIP is bit 0 of Status Register-1.
        if ((read_status() & 0x01U) == 0U)
        {
            return true;
        }
    }

    return false;
    }
};
class mspm0_spi0 : public lab3::spi
{
private:
    bool driver_configure(
        lab3::spi::settings const& p_settings) override;

    void driver_transfer(
        std::span<std::uint8_t const> p_data_out,
        std::span<std::uint8_t> p_data_in,
        std::uint8_t p_filler) override;
};

std::uint32_t calculate_scr(
    std::uint32_t functional_clock,
    std::uint32_t requested_clock)
{
    if (functional_clock == 0 || requested_clock == 0)
    {
        return 1023U; // Safest slowest setting
    }

    // The SPI clock cannot exceed half the functional clock.
    if (requested_clock >= functional_clock / 2U)
    {
        return 0U;
    }

    // Round the divisor upward to keep the actual clock
    // less than or equal to the requested clock.
    const std::uint64_t denominator =
        2ULL * requested_clock;

    const std::uint64_t divisor =
        (functional_clock + denominator - 1U) / denominator;

    // SCR = divisor - 1, limited to the hardware range.
    const std::uint64_t scr = divisor - 1U;

    if (scr > 1023U)
    {
        return 1023U;
    }

    return static_cast<std::uint32_t>(scr);
}
bool mspm0_spi0::driver_configure(
    lab3::spi::settings const& p_settings)
{
    constexpr std::uint32_t functional_clock = 32'000'000U;

    // using SPI Mode 0.
    if (p_settings.bus_mode != lab3::spi::mode::m0)
    {
        return false;
    }

    if (p_settings.clock_rate == 0U)
    {
        return false;
    }
    *spi0_rstctl = 0xB1000003UL;//Reset SPI0 and clear its sticky reset status
    // Enable SPI0 power.
    *spi0_pwren = 0x26000001UL;
    //Short delay after enabling power
    volatile std::uint32_t delay_counter = 0U;

    while (delay_counter < 10000U)
    {
        delay_counter = delay_counter + 1U;
    }

    // Disable SPI0 while configuring it.
    *spi0_ctl1 = 0U;

    //Select the SPI0 clock source.
    *spi0_clksel = 0x00000008UL;

    // Set the clock divider to divide by 1.
    *spi0_clkdiv = 0U;

    //Configure the SPI0 pins.
    *pico_pincm = spi_output_config;
    *sck_pincm  = spi_output_config;
    *poci_pincm = spi_input_config;

    // Configure CTL0.
    // SPH = 0, SPO = 0, PACKEN = 0.
    // FRF = 00, DSS = 00111 (8-bit data).
    *spi0_ctl0 = 0x07U;

    // Configure CTL1 while SPI is disabled.
    // CP = 1: Controller mode.
    // MSB = 1: MSB-first transmission.
    // ENABLE = 0: SPI remains disabled.
    *spi0_ctl1 = (1U << 2) | (1U << 4);

    // Calculate SCR using your existing function.
    const std::uint32_t scr =
        calculate_scr(functional_clock, p_settings.clock_rate);

    *spi0_clkctl = scr;

    //Enable SPI0.
    *spi0_ctl1 |= (1U << 0);

    return true;
}
void mspm0_spi0::driver_transfer(
    std::span<std::uint8_t const> p_data_out,
    std::span<std::uint8_t> p_data_in,
    std::uint8_t p_filler)
{
    // Find how many bytes must be transferred.
    // We must transfer enough bytes to fill the larger buffer.
    std::size_t transfer_size = p_data_out.size();

    if (p_data_in.size() > transfer_size)
    {
        transfer_size = p_data_in.size();
    }

    // Remove any old data from the receive FIFO.
    while ((*spi0_stat & stat_rfe_mask) == 0U)
    {
        (void)*spi0_rxdata;
    }

    //Transfer the bytes one at a time.
    for (std::size_t i = 0U; i < transfer_size; ++i)
    {
        // Choose the byte to send.
        std::uint8_t data_to_send = p_filler;

        if (i < p_data_out.size())
        {
            data_to_send = p_data_out[i];
        }

        // Wait until the transmit FIFO can accept a byte.
        while ((*spi0_stat & stat_tnf_mask) == 0U)
        {
        }

        // Send one byte.
        *spi0_txdata = data_to_send;

        // Wait until a received byte is available.
        while ((*spi0_stat & stat_rfe_mask) != 0U)
        {
        }

        // Read the received byte.
        const std::uint8_t received_byte =
            static_cast<std::uint8_t>(*spi0_rxdata & 0xFFU);

        // Store the byte if the input buffer has space.
        if (i < p_data_in.size())
        {
            p_data_in[i] = received_byte;
        }
    }
    //Wait until SPI finishes transmitting.
    while ((*spi0_stat & stat_busy_mask) != 0U)
    {
    }
}
bool get_user_input(
    std::array<std::uint8_t, 32>& data,
    std::size_t& length)
{
    char input[35]{};

    std::printf("Enter 1 to 32 characters:\n");

    if (std::fgets(input, sizeof(input), stdin) == nullptr)
    {
        return false;
    }

    // Count characters, excluding the newline.
    length = 0;

    while (length < sizeof(input) &&
           input[length] != '\0' &&
           input[length] != '\n' &&
           input[length] != '\r')
    {
        ++length;
    }

    // Require 1 to 32 characters.
    if (length == 0U || length > 32U)
    {
        std::printf("Input must contain 1 to 32 characters.\n");
        return false;
    }

    // Convert characters to bytes.
    for (std::size_t i = 0; i < length; ++i)
    {
        data[i] = static_cast<std::uint8_t>(
            static_cast<unsigned char>(input[i]));
    }
    return true;
}
void print_block(const std::array<std::uint8_t, 32>& data)
{
    for (std::size_t i = 0; i < data.size(); ++i)
    {
        if (i % 16U == 0U)
        {
            std::printf(
                "\n%06X: ",
                static_cast<unsigned>(i));
        }

        std::printf("%02X ", static_cast<unsigned>(data[i]));
    }

    std::printf("\n");
}
int main()
{
    mspm0_spi0 spi0;  // 1. Create the SPI0 driver object.
    lab3::spi::settings spi_settings{}; // 2. Configure SPI settings.
    spi_settings.clock_rate = 100'000;
    spi_settings.bus_mode = lab3::spi::mode::m0;
    if (!spi0.configure(spi_settings))
{
    std::printf("SPI configuration failed!\n");

    while (true)
    {
    }
}
    gpio_b12_chip_select chip_select;
    lab1::output_pin::settings cs_settings{};
     if (!chip_select.configure(cs_settings))
    {
        std::printf("CS configuration failed!\n");
        while (true) {}
    }
    flash_memory flash(&spi0, &chip_select);
std::uint8_t manufacturer_id = 0;
std::uint8_t memory_type = 0;
std::uint8_t capacity = 0;

if (!flash.read_jedec_id(
        manufacturer_id,
        memory_type,
        capacity))
{
    std::printf("Failed to read flash JEDEC ID.\n");
    while (true) {}
}
std::printf(
    "Manufacturer ID: 0x%02X\n",
    static_cast<unsigned>(manufacturer_id));

std::printf(
    "Memory Type:     0x%02X\n",
    static_cast<unsigned>(memory_type));

std::printf(
    "Capacity Code:   0x%02X\n",
    static_cast<unsigned>(capacity));

// Calculate flash capacity in bytes using the JEDEC capacity code.
const std::uint64_t capacity_bytes = std::uint64_t{1} << capacity;

std::printf(
    "Flash Capacity: %llu bytes\n",
    static_cast<unsigned long long>(capacity_bytes));
std::array<std::uint8_t, 32> data{}; // Buffer to store the first 32 bytes of flash.
// Read from address 0x000000.
if (!flash.read(
        0x000000U,
        std::span<std::uint8_t>(data)))
{
    std::printf("Flash read failed.\n");

    while (true)
    {
    }
}
// Print all 32 bytes.
std::printf("First 32 bytes of flash:\n");
print_block(data);
std::printf("\n");
  // Get data from the user.
std::array<std::uint8_t, 32> user_data{};
std::size_t input_length = 0;

if (!get_user_input(user_data, input_length))
{
    std::printf("Invalid input.\n");
    while (true) {}
}

// Erase the sector containing address 0x000000.
if (!flash.erase_sector(0x000000U))
{
    std::printf("Sector erase failed.\n");
    while (true) {}
}
// Write the user's data at address 0x000000.
std::span<std::uint8_t const> data_to_write(
    user_data.data(), input_length);

if (!flash.write(0x000000U, data_to_write))
{
    std::printf("Flash write failed.\n");
    while (true) {}
}

// Read back into the existing 32-byte buffer.
data.fill(0);

if (!flash.read(0x000000U, data))
{
    std::printf("Flash read-back failed.\n");
    
    while (true)
    {
    }
}
std::printf("\nFlash contents after writing:\n");
print_block(data);

// Compare the written data against the user's input.
bool match = true;

for (std::size_t i = 0; i < input_length; ++i)
{
    if (data[i] != user_data[i])
    {
        match = false;
        break;
    }
}
if (match)
{
    std::printf("\nWrite verified!\n");
}
else
{
    std::printf("\nWrite MISMATCH!\n");
}
while (true)
{
}
} 