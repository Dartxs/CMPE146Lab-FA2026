#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>

#include "../drivers/msp0g_i2c.hpp"
#include "../drivers/msp0g_iomux.hpp"
#include "../drivers/msp0g_timer.hpp"
#include "../hal/accelerometer.hpp"
#include "../hal/i2c.hpp"
#include "../hal/timer_util.hpp"

using namespace std::chrono_literals;

// TODO(lab4, step 2): Make this class inherit from and implement the
// `lab4::accelerometer` interface (`accelerometer.hpp`). Note that the
// BMI160 boots into a low-power suspend mode, so before its acceleration
// registers hold live data you must write a "set accelerometer to normal
// power mode" command to its command register, and you may also want to set
// its output data rate/range via its accelerometer configuration register.
// See the BMI160 datasheet's register map (command, power mode, and
// acceleration data registers).

namespace {

constexpr std::uint8_t bmi160_addr = 0x69;  // 0x68 if SA0 pin pull low

// PA28
msp0g::iomux_config sda_config = { .pincm = 3,
                                   .pf_sel = 3,
                                   .inena = msp0g::input_enable::enable };

// PA31
msp0g::iomux_config scl_config = { .pincm = 6,
                                   .pf_sel = 3,
                                   .inena = msp0g::input_enable::enable };

msp0g::iomux_settings i2c_settings = { .resistor = lab1::pin_resistor::none,
                                       .open_drain = true };

constexpr float rad_to_deg = 180.0F / 3.14159265F;

}  // namespace

/**
 * @brief BMI160 accelerometer implementation using an I2C controller.
 */
class bmi160 : public lab4::accelerometer
{

public:
  /**
   * @brief Construct the sensor and place its accelerometer in normal mode.
   *
   * Allows the sensor to start, requests normal accelerometer operation, and
   * verifies the resulting power-management state.
   *
   * @param p_i2c I2C controller connected to the BMI160.
   * @param p_clock Clock used for the required sensor startup delays.
   *
   * @note Construction aborts if the sensor cannot be configured or its power
   * mode cannot be verified.
   */
  bmi160(lab4::i2c& p_i2c, lab2::steady_clock& p_clock)
    : m_i2c(p_i2c)
    , m_clock(p_clock)
  {

    lab2::delay(m_clock, 5ms);

    if (!write_reg(cmd_address, acc_set_pmu_mode_normal)) {

      std::printf("could not set accelerometer\n");
      std::abort();

    } else {

      lab2::delay(m_clock, 5ms);

      std::uint8_t pmu_status;
      if (!read_reg(pmu_status_address, pmu_status)) {

        std::printf("could not read pmu status\n");
        std::abort();

      } else if ((pmu_status & 0x30) != 0x10) {

        std::printf("Accelerometer not in proper power mode\n");
        std::abort();
      }
    }
  }

private:
  // static constexpr std::uint8_t chip_id_address = 0x00;
  static constexpr std::uint8_t pmu_status_address = 0x03;
  static constexpr std::uint8_t cmd_address = 0x7E;
  static constexpr std::uint8_t acc_data_base_address =
    0x12;  // acc data in 0x12-0x17

  static constexpr std::uint8_t acc_set_pmu_mode_normal = 0x11;

  lab4::i2c& m_i2c;
  lab2::steady_clock& m_clock;

  /**
   * @brief Read and convert the three accelerometer axes.
   *
   * Reads the six consecutive little-endian axis registers and converts the
   * signed samples from the configured +/-2 g range into units of g.
   *
   * @return Latest acceleration along the sensor's X, Y, and Z axes. A zero
   * vector is returned if the I2C transaction fails.
   */
  read_t driver_read() override
  {

    std::array<std::uint8_t, 1> tx_buffer = { acc_data_base_address };

    // rx should read 6 bytes: x[7:0], x[15:8], y[7:0], y[15:8], z[7:0], z[15:8]
    std::array<std::uint8_t, 6> rx_buffer = {};

    if (!m_i2c.transaction(bmi160_addr, tx_buffer, rx_buffer)) {

      return { 0, 0, 0 };
    }

    std::uint16_t raw_x =
      (static_cast<std::uint16_t>(rx_buffer[1]) << 8) | rx_buffer[0];
    std::uint16_t raw_y =
      (static_cast<std::uint16_t>(rx_buffer[3]) << 8) | rx_buffer[2];
    std::uint16_t raw_z =
      (static_cast<std::uint16_t>(rx_buffer[5]) << 8) | rx_buffer[4];

    std::int16_t acc_x = static_cast<std::int16_t>(raw_x);
    std::int16_t acc_y = static_cast<std::int16_t>(raw_y);
    std::int16_t acc_z = static_cast<std::int16_t>(raw_z);

    // std::printf("%d, %d, %d\n", acc_x, acc_y, acc_z);
    constexpr float counts_per_g = 16384.0F;

    float acc_x_g = static_cast<float>(acc_x) / counts_per_g;
    float acc_y_g = static_cast<float>(acc_y) / counts_per_g;
    float acc_z_g = static_cast<float>(acc_z) / counts_per_g;

    return { acc_x_g, acc_y_g, acc_z_g };
  }

  /**
   * @brief Read one BMI160 register.
   *
   * @param p_reg_address Register address to read.
   * @param p_value Destination for the received register value.
   * @return true when the combined write/read transaction succeeds; otherwise
   * false.
   */
  bool read_reg(std::uint8_t p_reg_address, std::uint8_t& p_value)
  {

    std::array<std::uint8_t, 1> tx_buffer = { p_reg_address };
    std::array<std::uint8_t, 1> rx_buffer = {};

    if (!m_i2c.transaction(bmi160_addr, tx_buffer, rx_buffer)) {

      return false;
    }

    p_value = rx_buffer[0];
    return true;
  }

  /**
   * @brief Write one BMI160 register.
   *
   * @param p_reg_address Register address to write.
   * @param p_value Value to store in the register.
   * @return true when the write transaction succeeds; otherwise false.
   */
  bool write_reg(std::uint8_t p_reg_address, std::uint8_t p_value)
  {

    std::array<std::uint8_t, 2> tx_buffer = { p_reg_address, p_value };

    return m_i2c.transaction(bmi160_addr, tx_buffer, {});
  }
};

int main()
{

  // TODO(lab4, step 3): Construct an i2c driver object and use it to
  // construct a bmi160 object.

  msp0g::msp0g_iomux sda(sda_config, i2c_settings);
  msp0g::msp0g_iomux scl(scl_config, i2c_settings);
  msp0g::msp0g_i2c i2c_0({}, msp0g::i2cx::i2c0);
  msp0g::msp0g_steady_clock clock;

  bmi160 imu(i2c_0, clock);

  while (true) {

    const auto data = imu.read();

    float roll = std::atan2(data.y, data.z) * rad_to_deg;
    float pitch =
      std::atan2(-data.x, std::sqrt(data.y * data.y + data.z * data.z)) *
      rad_to_deg;

    std::printf("Roll: %.2f, Pitch: %.2f\n", roll, pitch);

    lab2::delay(clock, 20ms);
  }
}
