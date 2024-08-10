#pragma once

#include <libhal-util/i2c.hpp>


namespace sjsu::science {
  class sht21 {
    public:
      const static hal::byte sht21_i2c_address = 0x40;

      /**
       * @brief Resolution settings. Used with `set_resolution`
       * rh_12bit_temp_14bit is 12 bits of resolution for relative humidity and 14 bit of resolution for temperature.
       */
      enum resolution : hal::byte {
        rh_12bit_temp_14bit = 0b00000000,
        rh_8bit_temp_12bit = 0b00000001,
        rh_10bit_temp_13bit = 0b10000000,
        rh_11bit_temp_11bit = 0b10000001,
      };

      /**
       * @brief Create a sht21 sensor driver
       * 
       * @param p_bus I2C bus the device in on.
       * @param p_address I2C address of the device
       * @return hal::result<sht21> 
       */
      static sht21 create(hal::i2c& p_bus, hal::byte p_address = sht21_i2c_address);

      sht21(hal::i2c& p_bus, hal::byte p_address);
      
      /**
       * @brief Soft reset the sensor.
       * 
       * @return hal::status 
       */
      void soft_reset();

      /**
       * @brief Set the resolution of the relative humidity and temperature sensor.
       * (Untested)
       * 
       * @param p_resolution 
       * @return hal::status 
       */
      void set_resolution(resolution p_resolution);

      /**
       * @brief Returns true when VDD on the sensor is less than 2.25 V
       * (Untested)
       * 
       * @return hal::result<bool> 
       */
      bool is_low_battery();

      /**
       * @brief Enable or disable the on-chip heater.
       * (Untested)
       * 
       * @param p_enabled Defaults to true (enable)
       * @return hal::status 
       */
      void enable_heater(bool p_enabled = true);

      /**
       * @brief Perform a single relative humidity measurement. This will block until the measurement is ready.
       * 
       * @return hal::result<double> 
       */
      double get_relative_humidity();

      /**
       * @brief Perform a single relative humidity measurement. This will block until the measurement is ready.
       * 
       * @return hal::result<double> 
       */
      double get_temperature();

    private:
      
      enum command {
        hold_temperature_measurement_command = 0b11100011,
        hold_relative_humidity_measurement_command = 0b11100101,
        no_hold_temperature_measurement_command = 0b11110011,
        no_hold_relative_humidity_measurement_command = 0b11110101,

        write_user_register_command = 0b11100110,
        read_user_register_command = 0b11100111,

        soft_reset_command = 0b11111110,
      };

      /**
       * @brief I2C Bus
       */
      hal::i2c* m_i2c;

      /**
       * @brief Device address
       */
      hal::byte m_address;
  
  };
};