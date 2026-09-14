// #include "can_messaging.hpp"
#include "bldc_servo.hpp"
#include <array>
#include <cmath>
#include <cstdint>
#include <libhal-util/can.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/can.hpp>
#include <libhal/pointers.hpp>
#include <libhal/units.hpp>
#include <optional>

#include <can_util.hpp>
#include <resource_list.hpp>
// #include <bldc_servo.hpp>
// #include <can_messaging.hpp>

using namespace std::chrono_literals;
namespace sjsu::perseus {

can_perseus::can_perseus(
    hal::u16 p_curr_servo_addr,
    hal::u32 p_baudrate,
    hal::u8 p_listen_prev, 
    hal::v5::strong_ptr<hal::can_transceiver> p_can_transceiver,
    hal::v5::strong_ptr<hal::can_bus_manager> p_can_bus_manager,
    hal::v5::strong_ptr<hal::can_identifier_filter> p_can_identifier_filter
  ) 
  : 
    m_self_servo_addr(p_curr_servo_addr), 
    m_baudrate(p_baudrate),
    m_listen_prev(p_listen_prev),
    m_can_transceiver(p_can_transceiver),
    m_can_bus_manager(std::move(p_can_bus_manager)),
    m_can_identifier_filter(p_can_identifier_filter), 
    m_mc_message_finder(hal::can_message_finder(*m_can_transceiver, m_self_servo_addr)),
    m_mc_all_message_finder(hal::can_message_finder(*m_can_transceiver, 0x110))
{
  auto console = resources::console();
  m_can_identifier_filter->allow(m_self_servo_addr);
  m_can_bus_manager->baud_rate(m_baudrate); 
  hal::print<32>(*console,
                 "Receiver buffer size = %zu\n",
                 m_can_transceiver->receive_buffer().size());
  hal::print<64>(
    *console, "🆔 Allowing ID [0x%lX] through the filter!\n", m_self_servo_addr);
};

// helpers for decode message from mission control 
// floating point to position 
float can_perseus::floating_to_position(float floating) {
  if (m_self_servo_addr == servo_address::track_servo) {
    return floating * 30; 
  }
  return floating * 360; 
}
// position to floating point 
float can_perseus::position_to_floating(float position) {
  if (m_self_servo_addr == servo_address::track_servo) {
    return position / 30; 
  }
  return position / 360; 
}

// helper function for setters which set floats  
float can_perseus::float_setter(action act, 
                          hal::can_message const& p_message, 
                          hal::can_message& r_message) {
  float set_value = 0; 
  if (act == action::set_power) {
    std::array<hal::byte, 2> to_set_array = {p_message.payload[2], p_message.payload[3]}; 
    hal::i16 number = drivers::can_util::byte_array_to_int16_big_endian(to_set_array); 
    set_value = drivers::can_util::fixed_to_floating_point_16(
                    number, static_cast<hal::i16>(p_message.payload[1])); 
  }
  else {
    std::array<hal::byte, 4> to_set_array = {p_message.payload[2], p_message.payload[3], 
            p_message.payload[4], p_message.payload[5]}; 
    hal::i32 number = drivers::can_util::byte_array_to_int32_big_endian(to_set_array); 
    set_value = drivers::can_util::fixed_to_floating_point_32(
                    number, static_cast<hal::i16>(p_message.payload[1])); 
  }
  set_value = floating_to_position(set_value); 
  create_response(r_message, m_self_servo_addr + 0x100, 6, 
                        static_cast<hal::byte>(act), 
                        p_message.payload[1], 
                        p_message.payload[2], 
                        p_message.payload[3], 
                        p_message.payload[4], 
                        p_message.payload[5], 
                        0x00, 0x00); 
  return set_value; 
}
// helper function for setting pid settings
bldc_perseus::PID_settings can_perseus::pid_settings_setter(action act, 
                          hal::can_message const& p_message, 
                          hal::can_message& r_message) {
  std::array<float, 3> k_values; 
  for (int i = 1; i < 4; i++) {
    std::array<hal::byte, 2> to_set_array = {p_message.payload[i*2], p_message.payload[i*2+1]} ;
    hal::i16 number_from_array = drivers::can_util::byte_array_to_int16_big_endian(to_set_array); 
    float set_value = drivers::can_util::fixed_to_floating_point_16(
                    number_from_array, static_cast<hal::i16>(p_message.payload[1])); 
    k_values[i-1] = set_value; 
  }
  bldc_perseus::PID_settings settings = {
        .kp = k_values[0],
        .ki = k_values[1],
        .kd = k_values[2]
      };
  create_response(r_message, m_self_servo_addr + 0x100, 6, 
                        static_cast<hal::byte>(act), 
                        p_message.payload[1], 
                        p_message.payload[2], 
                        p_message.payload[3], 
                        p_message.payload[4], 
                        p_message.payload[5], 
                        0x00, 0x00); 
  return settings; 
}
// helper function for getters which get floats 
void can_perseus::float_getter(action act, 
                          float read_value, 
                          hal::i16 exponent, 
                          hal::can_message& r_message) {
  if (act == action::read_power) {
    hal::i16 fixed_pt = drivers::can_util::floating_to_fixed_point_16(
                        read_value, exponent); 
    hal::byte dir = 0x00; 
    if (fixed_pt < 0) {
      dir = static_cast<hal::byte>(-1); 
    }
    else {
      dir = 0x01; 
    }
    create_response(r_message, m_self_servo_addr + 0x100, 4, 
                      static_cast<hal::byte>(act), 
                      dir, 
                      static_cast<hal::byte>(fixed_pt >> 8) & 0xFF, 
                      static_cast<hal::byte>(fixed_pt >> 0) & 0xFF, 
                      0x00, 0x00, 0x00, 0x00); 
  }
  else {
    hal::i32 fixed_pt = drivers::can_util::floating_to_fixed_point_32(
                        read_value, exponent); 
    create_response(r_message, m_self_servo_addr + 0x100, 6, 
                        static_cast<hal::byte>(act), 
                        static_cast<hal::byte>(exponent),
                        static_cast<hal::byte>(fixed_pt >> 24) & 0xFF,
                        static_cast<hal::byte>(fixed_pt >> 16) & 0xFF,
                        static_cast<hal::byte>(fixed_pt >> 8) & 0xFF,
                        static_cast<hal::byte>(fixed_pt >> 0) & 0xFF,
                        0X00, 0X00); 
  }
}
// helper function for reading pid settings
void can_perseus::pid_settings_getter(action act, 
                          bldc_perseus::PID_settings settings, 
                          hal::can_message& r_message) {
  hal::i16 exponent = 14;
  hal::i16 kp = drivers::can_util::floating_to_fixed_point_16(settings.kp, exponent); 
  hal::i16 ki = drivers::can_util::floating_to_fixed_point_16(settings.ki, exponent); 
  hal::i16 kd = drivers::can_util::floating_to_fixed_point_16(settings.kd, exponent); 
  create_response(r_message, m_self_servo_addr + 0x100, 8, 
                        static_cast<hal::byte>(act), 
                        exponent,
                        static_cast<hal::byte>(kp >> 8) & 0xFF,
                        static_cast<hal::byte>(kp >> 0) & 0xFF,
                        static_cast<hal::byte>(ki >> 8) & 0xFF,
                        static_cast<hal::byte>(ki >> 0) & 0xFF,
                        static_cast<hal::byte>(kd >> 8) & 0xFF,
                        static_cast<hal::byte>(kd >> 0) & 0xFF); 
}



void can_perseus::print_can_message(hal::serial& p_console,
                       hal::can_message const& p_message)
{
  hal::print<256>(p_console,
                  "Received Message from ID: 0x%lX, length: %u \n"
                  "payload = [ 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X, "
                  "0x%02X, 0x%02X, 0x%02X ]\n",
                  p_message.id,
                  p_message.length,
                  p_message.payload[0],
                  p_message.payload[1],
                  p_message.payload[2],
                  p_message.payload[3],
                  p_message.payload[4],
                  p_message.payload[5],
                  p_message.payload[6],
                  p_message.payload[7]);
}

void can_perseus::create_response(hal::can_message& r_message,
                                    hal::u16 id, hal::byte len, 
                                    hal::byte b0, hal::byte b1, hal::byte b2, hal::byte b3, 
                                    hal::byte b4, hal::byte b5, hal::byte b6, hal::byte b7) 
                                  {
  r_message.id = id; 
  r_message.length = len; 
  r_message.payload[0] = b0;
  r_message.payload[1] = b1; 
  r_message.payload[2] = b2; 
  r_message.payload[3] = b3; 
  r_message.payload[4] = b4; 
  r_message.payload[5] = b5; 
  r_message.payload[6] = b6; 
  r_message.payload[7] = b7; 
  
}

void can_perseus::process_can_message(hal::can_message const& p_message,
                        hal::v5::strong_ptr<bldc_perseus> const& bldc)
{   
  hal::can_message response = {
    .id = 0x000,
    .extended=false,
    .remote_request=false,
    .length = 0,
    .payload = {},
  };
  auto console = resources::console();
  auto current_action = static_cast<action>(p_message.payload[0]); 
  switch (current_action) {
    // major 
    case action::power_off_reset:{
      bldc->stop(); 
      break;
    }
    case action::heartbeat: {
      create_response(response, m_self_servo_addr + 0x100, 1, 
                        m_self_servo_addr + 0x50, 0x00, 0x00, 0x00,
                        0x00, 0x00, 0x00, 0x00 
                      ); 
      bldc->set_active_action(static_cast<uint32_t>(action::heartbeat)); 
      break; 
    }
    case action::homing: {
      create_response(response, m_self_servo_addr + 0x100, 1, 
                        0x11, 0x00, 0x00, 0x00,
                        0x00, 0x00, 0x00, 0x00); 
      bldc->set_active_action(static_cast<uint32_t>(action::homing)); 
      break; 
    }
    // setters 
    case action::set_position_target: {
      float target_position = float_setter(current_action, p_message, response); 
      bldc->set_target_position(target_position);
      hal::print<64>(*console, "Target = %f\n", target_position);
      bldc->set_active_action(static_cast<uint32_t>(action::set_position_target)); 
      // get previous joint's target position 
      hal::can_message request {
        .id = 0x000,
        .extended=false,
        .remote_request=false,
        .length = 0,
        .payload = {},
      };
      if (m_listen_prev > 0) {
        request.id = m_self_servo_addr - m_listen_prev; 
        request.length = 3; 
        request.payload[0] = static_cast<hal::byte>(action::prev_joint_actual_position); 
        request.payload[1] = static_cast<hal::byte>(m_self_servo_addr >> 8) & 0xFF; 
        request.payload[2] = static_cast<hal::byte>(m_self_servo_addr >> 0) & 0xFF; 
        m_mc_message_finder.transceiver().send(request);
      } 
      break;
    }
    case action::set_position_reading: {
      float reading_position = float_setter(current_action, p_message, response); 
      float new_angle_offset = bldc->get_actual_position() - reading_position + bldc->get_angle_offset(); 
      bldc->set_angle_offset(new_angle_offset); 
      hal::print<64>(*console, "reading = %f\n", reading_position);
      bldc->set_active_action(static_cast<uint32_t>(action::set_position_reading)); 
      break;
    }
    case action::set_velocity_target: {
      float target_velocity = float_setter(current_action, p_message, response); 
      bldc->set_target_position(target_velocity);
      hal::print<64>(*console, "Target = %f\n", target_velocity);
      bldc->set_active_action(static_cast<uint32_t>(action::set_velocity_target)); 
      break;
    }
    case action::set_power: {
      float power = float_setter(current_action, p_message, response);  
      bldc->set_power(power); 
      bldc->set_active_action(static_cast<uint32_t>(action::set_power)); 
      break; 
    }
    case action::set_pid_position_config: {
      bldc_perseus::PID_settings settings = pid_settings_setter(current_action, 
                          p_message, response); 
      bldc->update_pid_position(settings);
      bldc->set_active_action(static_cast<uint32_t>(action::set_pid_position_config)); 
      break;
    }
    case action::set_pid_velocity_config: {
      bldc_perseus::PID_settings settings = pid_settings_setter(current_action, 
                          p_message, response); 
      bldc->update_pid_position(settings);
      bldc->set_active_action(static_cast<uint32_t>(action::set_pid_velocity_config)); 
      break;
    }
    // readers 
    case action::read_position_target: {
      float read_value = position_to_floating(bldc->get_target_position()); 
      float_getter(current_action, read_value, 14, response); 
      break;
    }
    case action::read_position_reading: {
      float read_value = position_to_floating(bldc->get_actual_position()); 
      float_getter(current_action, read_value, 14, response); 
      break;
    }
    case action::read_velocity_target: {
      float read_value = position_to_floating(bldc->get_target_velocity()); 
      float_getter(current_action, read_value, 14, response); 
      break;
    }
    case action::read_velocity_reading: {
      float read_value = position_to_floating(bldc->get_reading_velocity()); 
      float_getter(current_action, read_value, 14, response); 
      break;
    }
    case action::read_power: {
      float read_value = bldc->get_power(); 
      float_getter(current_action, read_value, 14, response); 
      break;
    }
    case action::read_pid_position_config: {
      bldc_perseus::PID_settings settings = bldc->get_pid_settings(); 
      pid_settings_getter(current_action, settings, response); 
      break;
    }
    case action::read_pid_velocity_config: {
      bldc_perseus::PID_settings settings = bldc->get_pid_settings(); 
      pid_settings_getter(current_action, settings, response); 
      break;
    } 
    case action::prev_joint_actual_position: {
      float read_value = position_to_floating(bldc->get_actual_position()); 
      float_getter(current_action, read_value, 14, response); 
      hal::print<64>(*console, "Actual position = %d\n", read_value);
      break;
    }
    case action::prev_joint_position_response: {
      hal::i32 prev_join_response = drivers::can_util::byte_array_to_int32_big_endian(
        {p_message.payload[2], p_message.payload[3],
                p_message.payload[4], p_message.payload[5]}); 
      float prev_joint_fixed = drivers::can_util::fixed_to_floating_point_32(prev_join_response, p_message.payload[1]); 
      float prev_joint_pos = floating_to_position(prev_joint_fixed); 
      if (bldc->get_servo_values().flipped_direction == false){ 
        prev_joint_pos = prev_joint_pos * -1; 
      }
      bldc->set_prev_joint_position(prev_joint_pos);
      hal::print<64>(*console, "prev_pos = %f\n", 0.0f);
      break;
    }
    default:
      create_response(response, m_self_servo_addr + 0x100, 1, 
                        static_cast<hal::byte>(p_message.payload[0]) + 0x100, 
                        0X00, 0X00, 0X00, 0X00,
                        0X00, 0X00, 0X00
                      ); 
      throw hal::operation_not_supported(nullptr); 
      break; 
  }
  m_mc_message_finder.transceiver().send(response);
  print_can_message(*console, response);
  hal::print<64>(*console, "finished transmission\n");
}

std::optional<hal::can_message> can_perseus::check_for_mc_message() {
  auto msg = m_mc_message_finder.find();
  auto console = resources::console(); 
  if (msg) {
    return msg; 
  }
  return std::nullopt; 
}


} // namespace sjsu::perseus