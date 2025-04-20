#pragma once
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal-util/i2c.hpp>
#include <libhal/units.hpp>

namespace sjsu::drivers{
    class tmag5273{
    private:
        hal::i2c& m_i2c;
        hal::steady_clock& m_clock;

        enum device_addresses{
            i2c_address = 0x22,
            write_address = 0x6A,
            read_address = 0x6B,
            DEVICE_CONFIG_1 = 0x0,
            DEVICE_CONFIG_2 = 0x1,
            SENSOR_CONFIG_1 = 0x2,
            SENSOR_CONFIG_2 = 0x3,
            X_THR_CONFIG = 0x4,
            Y_THR_CONFIG = 0x5,
            Z_THR_CONFIG = 0x6,
            T_CONFIG = 0x7,
            INT_CONFIG_1 = 0x8,
            MAG_GAIN_CONFIG = 0x9,
            MAG_OFFSET_CONFIG_1 = 0xA,
            MAG_OFFSET_CONFIG_2 = 0xB,
            NEW_I2C_ADDRESS = 0xC,
            DEVICE_ID = 0xD,
            MANUFACTURER_ID_LSB = 0xE,
            MANUFACTURER_ID_MSB = 0xF,
            T_MSB_RESULT = 0x10,
            T_LSB_RESULT = 0x11,
            X_MSB_RESULT = 0x12,
            X_LSB_RESULT = 0x13,
            Y_MSB_RESULT = 0x14,
            Y_LSB_RESULT = 0x15,
            Z_MSB_RESULT = 0x16,
            Z_LSB_RESULT = 0x17,
            CONV_STATUS = 0x18,
            ANGLE_RESULT_MSB = 0x19,
            ANGLE_RESULT_LSB = 0x1A,
            MAGNITUDE_RESULT = 0x1B,
            DEVICE_STATUS = 0x1c
        };


    public:
        struct data{
            float temperature;
            float x_field;
            float y_field;
            float z_field;
            float angle;
            float magnitude;
        };

        tmag5273(hal::i2c& p_i2c,  hal::steady_clock&  p_clock);
        
        void defualt_config();
        void config_device(hal::byte config_1, hal::byte config_2);
        void config_sensor(hal::byte config_1, hal::byte config_2);
        // void change_i2c_address(hal::byte new_address);
        // void config_temp_sensor(hal::byte configurations);
        // void config_device_offset();
        data read();
        hal::byte device_status();

    };

}