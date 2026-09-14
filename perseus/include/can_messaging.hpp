#pragma once
#include <libhal/can.hpp>
#include <libhal-util/can.hpp>
#include <libhal/pointers.hpp>
#include <libhal/units.hpp>

#include <serial_commands.hpp>
#include <bldc_servo.hpp>

namespace sjsu::perseus {

class can_perseus 
{

public: 
    can_perseus(
        hal::u16 p_servo_addr,
        hal::u32 p_baudrate,
        hal::u8 p_listen_prev,
        hal::v5::strong_ptr<hal::can_transceiver> p_can_transceiver,
        hal::v5::strong_ptr<hal::can_bus_manager> p_can_bus_manager,
        hal::v5::strong_ptr<hal::can_identifier_filter> p_can_identifier_filter
    ); 


    enum class action : uint8_t
    {
    // top priority
    power_off_reset = 0x0C,  
    heartbeat = 0x0E, 

    // actuators
    homing = 0x11, 
    set_position_target = 0x12,
    set_position_reading = 0x13, 
    set_velocity_target = 0x14, 
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
    end_effector = 0x126
    };

    void set_curr_servo_addr(hal::u16 servo_addr); 

    void print_can_message(hal::serial& p_console,
                        hal::can_message const& p_message); 
    float floating_to_position(float floating);
    float position_to_floating(float position); 
    float float_setter(action act, 
                        hal::can_message const& p_message, 
                        hal::can_message& r_message); 
    bldc_perseus::PID_settings pid_settings_setter(action act, 
                        hal::can_message const& p_message, 
                        hal::can_message& r_message); 
    void float_getter(action act, 
                        float read_value, 
                        hal::i16 exponent, 
                        hal::can_message& r_message); 
    void pid_settings_getter(action act, 
                        bldc_perseus::PID_settings settings, 
                        hal::can_message& r_message);
    void create_response(hal::can_message& r_message, 
                        hal::u16 r_id, hal::byte r_len, 
                        hal::byte r0, hal::byte r1, hal::byte r2, hal::byte r3, 
                        hal::byte r4, hal::byte r5, hal::byte r6, hal::byte r7); 
    void process_can_message(hal::can_message const& p_message,
                        hal::v5::strong_ptr<bldc_perseus> const& bldc);
    std::optional<hal::can_message> check_for_mc_message(); 

private: 
    hal::u16 m_self_servo_addr;
    hal::u32 m_baudrate;
    hal::u8 m_listen_prev; // 0 = doesn't need info from prev joint, 1 = from prev, 2 = from prev prev
    hal::v5::strong_ptr<hal::can_transceiver> m_can_transceiver;
    hal::v5::strong_ptr<hal::can_bus_manager> m_can_bus_manager;
    hal::v5::strong_ptr<hal::can_identifier_filter> m_can_identifier_filter;
    hal::can_message_finder m_mc_message_finder;
    hal::can_message_finder m_mc_all_message_finder;
}; 
} // namespace sjsu::perseus