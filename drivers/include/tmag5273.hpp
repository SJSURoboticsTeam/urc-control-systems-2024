#pragma once
#include <libhal-util/serial.hpp>
#include <libhal-util/steady-clock.hpp>
#include <libhal-soft/bit_bang_i2c>
#include <libhal/units.hpp>

namespace sjsu::drivers{
    class tmag5273{
    private:
        hal::i2c& m_i2c;
        hal::steady_clock& m_clock;
        enum device_addresses{
            i2c_address = 0x35,
            write_address = 0x6A,
            read_address = 0x6B,
        };


    public:
        struct data{
            float temperature
            float x_field,
            float y_field,
            float z_field
        };

        // tmag5273(h){}
    };

}