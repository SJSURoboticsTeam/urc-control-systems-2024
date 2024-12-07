// #pragma once

#include "../include/scd40.hpp"

using namespace std::chrono_literals;
namespace sjsu::drivers {

scd40::scd40(hal::i2c& p_i2c, hal::steady_clock& p_clock) : m_i2c(p_i2c) ,m_clock(p_clock){
    start();
}


void scd40::start(){
    std::array<hal::byte, 2> start_address =  { start_periodic_measurement_first_half, start_periodic_measurement_second_half };
    hal::write(m_i2c, addresses::device_address, start_address, hal::never_timeout());
}

scd40::scd40_read_data scd40::read() {
    std::array<hal::byte, 2> read_address = {read_measurement_first_half, read_measurement_second_half };
    std::array<hal::byte, 9> buffer;

    hal::write(m_i2c, addresses::device_address, read_address);
    hal::delay(m_clock, 1ms);
    hal::read(m_i2c, addresses::device_address, buffer, hal::never_timeout());
    
    scd40::scd40_read_data rd;
    rd.co2 = buffer[0] << 8 | buffer[1];
    rd.temp = (-45 + 175.0*(buffer[3] << 8 | buffer[4])/ (1 << 16));
    rd.rh = 100.0 * (buffer[6] << 8 | buffer[7]) / (1 << 16);

    return rd;
}

void scd40::stop() {
    std::array<hal::byte, 2> stop_address =  { stop_periodic_measurement_first_half, stop_periodic_measurement_second_half };
    hal::write(m_i2c, addresses::device_address, stop_address, hal::never_timeout());
}

scd40::scd40_settings scd40::get_settings() {
    std::array<hal::byte, 2> read_address_temp = {get_temperature_offset_first_half, get_temperature_offset_second_half };
    std::array<hal::byte, 2> read_address_alt = { get_sensor_altitude_first_half, get_sensor_altitude_second_half };
    std::array<hal::byte, 6> buffer;

    hal::write(m_i2c,addresses::device_address, read_address_temp);
    hal::delay(m_clock, 1ms);
    hal::read(m_i2c, addresses::device_address, buffer, hal::never_timeout());
    
    scd40::scd40_settings get;
     
    get.temp_offset = 175 *(buffer[0] << 8 | buffer[1])/(1 << 16);

    hal::write(m_i2c,addresses::device_address, read_address_alt);
    hal::delay(m_clock, 1ms);
    hal::read(m_i2c, addresses::device_address, buffer, hal::never_timeout());
    get.altitude = buffer[0]<<8 | buffer[1];

    return get;
}

void scd40::set_settings( struct settings setting) {
    if(setting.set_temp != 0){
        int temp = int(((setting.set_temp*(1<<16)) / 175)) ;
        hal::byte temp_first_half = (temp) >> 8;
        hal::byte temp_second_half = temp & 0xff;
        std::array<hal::byte, 4> set_temp_address = {set_temperature_offset_first_half, set_temperature_offset_second_half, temp_first_half, temp_second_half};
        hal::write(m_i2c,addresses::device_address,set_temp_address);
        hal::delay(m_clock, 1ms);
    }
    if(setting.set_pressure != -1){
        int pressure = int(setting.set_pressure/100);
        hal::byte pressure_first_half = (pressure) >> 8;
        hal::byte pressure_second_half = pressure & 0xff;
        std::array<hal::byte, 4> set_pressure_address = {set_ambient_pressure_first_half, set_ambient_pressure_second_half, pressure_first_half, pressure_second_half};
        hal::write(m_i2c,addresses::device_address,set_pressure_address);
        hal::delay(m_clock, 1ms);
    } else if(setting.set_alt != 0){
        int alt = int(setting.set_alt);
        hal::byte alt_first_half = (alt) >> 8;
        hal::byte alt_second_half = alt & 0xff;
        std::array<hal::byte, 4> set_alt_address = {set_sensor_altitude_first_half, set_sensor_altitude_second_half, alt_first_half, alt_second_half};
        hal::write(m_i2c,addresses::device_address,set_alt_address);
        hal::delay(m_clock, 1ms);
    }

}

hal::byte scd40::generate_crc(std::array<hal::byte, 2> data){
    uint16_t current_byte;
    uint8_t crc_bit;
    uint8_t crc = 0xFF;
    uint8_t CRC8_POLYNOMIAL = 0x31;

    for (current_byte = 0; current_byte < 2 ; ++current_byte) {
        crc ^= (data[current_byte]);
        for (crc_bit = 8; crc_bit > 0; --crc_bit) {
            if (crc & 0x80)
                crc = (crc << 1) ^ CRC8_POLYNOMIAL;
            else
                crc = (crc << 1);
        }
    }
    return crc;
}

bool scd40::validate_crc(std::array<hal::byte, 3> data) {
    uint16_t current_byte;
    uint8_t crc_bit;
    uint8_t crc = 0xFF;
    uint8_t CRC8_POLYNOMIAL = 0x31;

    for (current_byte = 0; current_byte < 3 ; ++current_byte) {
        crc ^= (data[current_byte]);
        for (crc_bit = 8; crc_bit > 0; --crc_bit) {
            if (crc & 0x80)
                crc = (crc << 1) ^ CRC8_POLYNOMIAL;
            else
                crc = (crc << 1);
        }
    }
    return crc == 0;
}
};