#pragma once
#include <libhal-arm-mcu/stm32_generic/quadrature_encoder.hpp>
#include <libhal/can.hpp>
#include <libhal/pointers.hpp>
#include <libhal/rotation_sensor.hpp>
#include <libhal/units.hpp>

#include <bldc_servo.hpp>

namespace sjsu::perseus {
    
class can_perseus 
{

public: 
    can_perseus(
        hal::u16 p_servo_addr,
        hal::u32 p_baudrate,
        hal::v5::strong_ptr<hal::can_transceiver> p_can_transceiver,
        hal::v5::strong_ptr<hal::can_bus_manager> p_can_bus_manager,
        hal::v5::strong_ptr<hal::can_identifier_filter> p_can_identifier_filter
    ); 


    enum class action : uint32_t
    {
    // top priority
    power_off_reset = 0x0C,  // hard stop the servo to be 0
    heartbeat = 0x0E, // are you alive

    // actuators
    homing = 0x11, 
    set_position_target = 0x12,
    set_position_reading = 0x13, 
    set_velocity_target = 0x14, 
//     set_velocity_reading = 0x15,
    set_power = 0x16, 
    set_pid_position_config = 0x17,
    set_pid_velocity_config = 0x18,

    // readers
    read_homing_status = 0x21, 
    read_position_target = 0x22,
    read_position_reading = 0x23,
    read_velocity_target = 0x24,
    read_velocity_reading = 0x25, 
    read_power = 0x26, 
    read_pid_position_config = 0x27,
    read_pid_velocity_config = 0x28,

    // servo to servo 
    prev_joint_actual_position = 0x41, 
    prev_joint_position_response = 0x51, 
    };
    enum servo_address : hal::u16
    {
    track_servo = 0x121,
    shoulder_servo = 0x122,
    elbow_servo = 0x123,
    wrist_left = 0x124,
    wrist_right = 0x125,
    clamp = 0x126,

    from_shoulder = 0x321, 
    from_elbow = 0x322

    };

    void set_curr_servo_addr(hal::u16 servo_addr); 

    void print_can_message(hal::serial& p_console,
                        hal::can_message const& p_message); 
    float fixed_to_floating_point_32(hal::byte b0, hal::byte b1, hal::byte b2, hal::byte b3, int exponent); 
    float fixed_to_floating_point_16(hal::byte b0, hal::byte b1, int exponent); 
    float floating_to_position(float floating);
    float position_to_floating(float position); 
    hal::i32 floating_to_fixed_point_32(float n, int exponent); 
    hal::i16 floating_to_fixed_point_16(float n, int exponent); 
    void process_can_message(hal::can_message const& p_message,
                        hal::v5::strong_ptr<bldc_perseus> bldc);
    void periodic_action(uint32_t curr_action, 
                        hal::v5::strong_ptr<bldc_perseus> bldc); 
    std::optional<hal::can_message> check_for_mc_message(); 
    std::optional<hal::can_message> check_for_joint_message(); 

private: 
    hal::u16 m_self_servo_addr;
    hal::u32 m_baudrate;
    hal::v5::strong_ptr<hal::can_transceiver> m_can_transceiver;
    hal::v5::strong_ptr<hal::can_bus_manager> m_can_bus_manager;
    hal::v5::strong_ptr<hal::can_identifier_filter> m_can_identifier_filter;
    hal::can_message_finder m_mc_message_finder;
    hal::can_message_finder m_mc_all_message_finder;
}; 
} // namespace sjsu::perseus