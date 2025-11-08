#pragma once
#include "../../drivers/include/h_bridge.hpp"
#include <libhal-arm-mcu/stm32_generic/quadrature_encoder.hpp>
#include <libhal/pointers.hpp>
#include <libhal/rotation_sensor.hpp>
#include <libhal/units.hpp>

namespace sjsu::perseus {
    
class can_perseus 
{

public: 
    can_perseus(hal::u16 curr_servo_addr); 


    enum class action : hal::byte
    {
    // actuators
    set_position = 0x12,
    set_velocity = 0x13,
    stop = 0x22,  // hard stop the servo to be 0
                    // readers
    read_position = 0x20,
    read_velocity = 0x21,
    // setters
    clamp_speed = 0x30,
    set_pid_position = 0x31,
    set_pid_velocity = 0x32,
    // heartbeat 
    heartbeat = 0x0E
    };
    enum servo_address : hal::u16
    {
    track_servo = 0x120,
    shoulder_servo = 0x121,
    elbow_servo = 0x122,
    wrist_pitch = 0x123,
    wrist_roll = 0x124,
    clamp = 0x125
    };

    void set_curr_servo_addr(hal::u16 servo_addr); 

    void print_can_message(hal::serial& p_console,
                        hal::can_message const& p_message); 
    void process_can_message(hal::can_message const& p_message,
                            hal::v5::strong_ptr<bldc_perseus> bldc,
                                [[maybe_unused]] hal::v5::strong_ptr<hal::can_message> response); 

    // heartbeat response  (heartbeat response = servo addr - 0x50)
    // receive pid (16 bits, check document for breakdown )


    private: 
    hal::u16 m_curr_servo_addr; 
}; 
} // namespace sjsu::perseus