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
#include <utility>

using namespace std::chrono_literals;
namespace can_util = sjsu::drivers::can_util;
namespace sjsu::perseus {

can_perseus::can_perseus(
    hal::u16 p_curr_servo_addr,
    hal::u16 p_listen_prev, 
    hal::u32 p_baudrate,
    hal::v5::strong_ptr<hal::can_transceiver> p_can_transceiver,
    hal::v5::strong_ptr<hal::can_bus_manager> p_can_bus_manager,
    hal::v5::strong_ptr<hal::can_identifier_filter> p_can_identifier_filter, 
    hal::v5::strong_ptr<hal::can_mask_filter> p_can_mask_filter 
  ) 
  : 
    m_self_servo_addr(p_curr_servo_addr), 
    m_prev_servo_addr(p_listen_prev),
    m_baudrate(p_baudrate),
    m_can_transceiver(p_can_transceiver),
    m_can_bus_manager(p_can_bus_manager),
    m_can_identifier_filter(p_can_identifier_filter), 
    m_can_mask_filter(p_can_mask_filter),
    m_command_message_finder(hal::can_message_finder(*m_can_transceiver, m_self_servo_addr)),
    m_group_command_message_finder(hal::can_message_finder(*m_can_transceiver, 0x120))
{
  auto console = resources::console();
  m_can_identifier_filter->allow(m_self_servo_addr);
  m_can_mask_filter->allow(hal::can_mask_filter::pair(0x120, 0x006));
  m_can_bus_manager->baud_rate(m_baudrate); 
  hal::print<32>(*console,
                 "Receiver buffer size = %zu\n",
                 m_can_transceiver->receive_buffer().size());
  hal::print<64>(
    *console, "🆔 Allowing ID [0x%lX] through the filter!\n", m_self_servo_addr);
};

// TODO make documentation more thorough

// helpers for decode message from mission control 
// rotations to position (degree or mm)
float can_perseus::rotations_to_position(float p_rotations) {
  if (m_self_servo_addr == servo_address::track_servo) {
    return p_rotations * 30; 
  }
  return p_rotations * 360; 
}
// rotations to position (degree or mm)
float can_perseus::position_to_rotations(float p_position) {
  if (m_self_servo_addr == servo_address::track_servo) {
    return p_position / 30; 
  }
  return p_position / 360; 
}

// helper function for setters which set floats  
float can_perseus::float_setter(action p_act, 
                          hal::can_message const& p_message, 
                          hal::can_message& p_response) {
  float set_value = 0; 
  if (p_act == action::set_power) {
    std::array<hal::byte, 2> to_set_array = {p_message.payload[2], p_message.payload[3]}; 
    hal::i16 number = can_util::byte_array_to_int16_big_endian(to_set_array); 
    set_value = can_util::fixed_to_floating_point_16(
                    number, static_cast<hal::i16>(p_message.payload[1])); 
  }
  else {
    std::array<hal::byte, 4> to_set_array = {p_message.payload[2], p_message.payload[3], 
            p_message.payload[4], p_message.payload[5]}; 
    hal::i32 number = can_util::byte_array_to_int32_big_endian(to_set_array); 
    set_value = can_util::fixed_to_floating_point_32(
                    number, static_cast<hal::i16>(p_message.payload[1])); 
    set_value = rotations_to_position(set_value); 
  }
  hal::u16 r_id = m_self_servo_addr + 0x100; 
  p_response = hal::can_message {
    .id = r_id,
    .length = 6, 
    .payload = {static_cast<hal::byte>(p_act), 
                p_message.payload[1], 
                p_message.payload[2], 
                p_message.payload[3], 
                p_message.payload[4], 
                p_message.payload[5], 
                0x00, 0x00}
  }; 
  return set_value; 
}
// helper function for setting pid settings
bldc_perseus::PID_settings can_perseus::pid_settings_setter(action p_act, 
                          hal::can_message const& p_message, 
                          hal::can_message& p_response) {
  std::array<float, 3> k_values; 
  for (int i = 1; i < 4; i++) {
    std::array<hal::byte, 2> to_set_array = {p_message.payload[i*2], p_message.payload[i*2+1]} ;
    hal::i16 number_from_array = can_util::byte_array_to_int16_big_endian(to_set_array); 
    float set_value = can_util::fixed_to_floating_point_16(
                    number_from_array, static_cast<hal::i16>(p_message.payload[1])); 
    k_values[i-1] = set_value; 
  }
  bldc_perseus::PID_settings settings = {
        .kp = k_values[0],
        .ki = k_values[1],
        .kd = k_values[2]
      };
  hal::u16 r_id = m_self_servo_addr + 0x100; 
  p_response = hal::can_message {
    .id = r_id,
    .length = 6, 
    .payload = {
              static_cast<hal::byte>(p_act), 
              p_message.payload[1], 
              p_message.payload[2], 
              p_message.payload[3], 
              p_message.payload[4], 
              p_message.payload[5], 
              0x00, 0x00
            }
  }; 
  return settings; 
}
// helper function for getters which get floats 
void can_perseus::float_getter(action p_act, 
                          float p_read_value, 
                          hal::i16 p_exponent, 
                          hal::can_message& p_response) {
  if (p_act == action::read_power) {
    hal::i16 fixed_pt = can_util::floating_to_fixed_point_16(
                        p_read_value, p_exponent); 
    hal::byte dir = 0x00; 
    if (fixed_pt < 0) {
      dir = static_cast<hal::byte>(-1); 
    }
    else {
      dir = 0x01; 
    }
    p_response = hal::can_message{
                .id = static_cast<hal::u16>(m_self_servo_addr + 0x100), 
                .length = 4, 
                .payload = {
                  static_cast<hal::byte>(p_act), 
                  dir, 
                  static_cast<hal::byte>(fixed_pt >> 8), 
                  static_cast<hal::byte>(fixed_pt >> 0), 
                  0x00, 0x00, 0x00, 0x00
                }
    }; 
  }
  else {
    hal::i32 fixed_pt = can_util::floating_to_fixed_point_32(
                        p_read_value, p_exponent); 
    p_response = hal::can_message{
                        .id = static_cast<hal::u16>(m_self_servo_addr + 0x100), 
                        .length = 6, 
                        .payload = {
                          static_cast<hal::byte>(p_act), 
                          static_cast<hal::byte>(p_exponent),
                          static_cast<hal::byte>(fixed_pt >> 24),
                          static_cast<hal::byte>(fixed_pt >> 16),
                          static_cast<hal::byte>(fixed_pt >> 8),
                          static_cast<hal::byte>(fixed_pt >> 0),
                          0X00, 0X00
                        } 
                };
  }
}
// helper function for reading pid settings
void can_perseus::pid_settings_getter(action p_act, 
                          bldc_perseus::PID_settings p_settings, 
                          hal::can_message& p_response) {
  hal::i16 exponent = 14;
  hal::i16 kp = can_util::floating_to_fixed_point_16(p_settings.kp, exponent); 
  hal::i16 ki = can_util::floating_to_fixed_point_16(p_settings.ki, exponent); 
  hal::i16 kd = can_util::floating_to_fixed_point_16(p_settings.kd, exponent); 
  p_response = hal::can_message {
                      .id = static_cast<hal::u16>(m_self_servo_addr + 0x100), 
                      .length = 8, 
                      .payload = {
                        static_cast<hal::byte>(p_act), 
                        static_cast<hal::byte>(exponent),
                        static_cast<hal::byte>(kp >> 8),
                        static_cast<hal::byte>(kp >> 0),
                        static_cast<hal::byte>(ki >> 8),
                        static_cast<hal::byte>(ki >> 0),
                        static_cast<hal::byte>(kd >> 8),
                        static_cast<hal::byte>(kd >> 0)
                      }
              }; 
}

void can_perseus::process_can_message(hal::can_message const& p_message,
                                        bldc_perseus& p_bldc)
{   
  hal::can_message response;
  auto console = resources::console();
  auto current_action = static_cast<action>(p_message.payload[0]); 
  switch (current_action) {
    // major 
    case action::power_off_reset:{
      p_bldc.stop(); 
      break;
    }
    case action::heartbeat: {
      response = hal::can_message{
                      .id = static_cast<hal::u16>(m_self_servo_addr + 0x100), 
                      .length = 1, 
                      .payload = {
                        static_cast<hal::byte>(m_self_servo_addr + 0x50), 0x00, 0x00, 0x00,
                        0x00, 0x00, 0x00, 0x00 
                        }
                    }; 
      p_bldc.set_active_action(static_cast<uint32_t>(action::heartbeat)); 
      break; 
    }
    case action::homing: {
      response = hal::can_message{
                          .id = static_cast<hal::u16>(m_self_servo_addr + 0x100), 
                          .length = 1, 
                          .payload = {
                          0x11, 0x00, 0x00, 0x00,
                          0x00, 0x00, 0x00, 0x00
                          }
                        }; 
      p_bldc.set_active_action(static_cast<uint32_t>(action::homing)); 
      break; 
    }
    // setters 
    case action::set_position_target: {
      float target_position = float_setter(current_action, p_message, response); 
      p_bldc.set_target_position(target_position);
      hal::print<64>(*console, "Target = %f\n", target_position);
      p_bldc.set_active_action(static_cast<uint32_t>(action::set_position_target)); 
      // get previous joint's target position 
      if (m_prev_servo_addr > 0) {
        auto request = hal::can_message {
                        .id = static_cast<hal::u16>(m_prev_servo_addr), 
                        .length = 3, 
                        .payload = {
                            static_cast<hal::byte>(action::prev_joint_actual_position), 
                            static_cast<hal::byte>(m_self_servo_addr >> 8), 
                            static_cast<hal::byte>(m_self_servo_addr >> 0), 
                            0x00, 0x00, 0x00, 0x00, 0x00
                          }
                      };
        m_can_transceiver->send(request);
      } 
      break;
    }
    case action::set_position_reading: {
      float reading_position = float_setter(current_action, p_message, response); 
      float new_angle_offset = p_bldc.get_actual_position() - reading_position + p_bldc.get_angle_offset(); 
      p_bldc.set_angle_offset(new_angle_offset); 
      hal::print<64>(*console, "reading = %f\n", reading_position);
      p_bldc.set_active_action(static_cast<uint32_t>(action::set_position_reading)); 
      break;
    }
    case action::set_velocity_target: {
      float target_velocity = float_setter(current_action, p_message, response); 
      p_bldc.set_target_position(target_velocity);
      hal::print<64>(*console, "Target = %f\n", target_velocity);
      p_bldc.set_active_action(static_cast<uint32_t>(action::set_velocity_target)); 
      break;
    }
    case action::set_power: {
      float power = float_setter(current_action, p_message, response);  
      p_bldc.set_power(power); 
      p_bldc.set_active_action(static_cast<uint32_t>(action::set_power)); 
      break; 
    }
    case action::set_pid_position_config: {
      bldc_perseus::PID_settings settings = pid_settings_setter(current_action, 
                          p_message, response); 
      p_bldc.update_pid_position(settings);
      p_bldc.set_active_action(static_cast<uint32_t>(action::set_pid_position_config)); 
      break;
    }
    case action::set_pid_velocity_config: {
      bldc_perseus::PID_settings settings = pid_settings_setter(current_action, 
                          p_message, response); 
      p_bldc.update_pid_position(settings);
      p_bldc.set_active_action(static_cast<uint32_t>(action::set_pid_velocity_config)); 
      break;
    }
    // readers 
    case action::read_position_target: {
      float read_value = position_to_rotations(p_bldc.get_target_position()); 
      float_getter(current_action, read_value, 14, response); 
      break;
    }
    case action::read_position_reading: {
      float read_value = position_to_rotations(p_bldc.get_actual_position()); 
      float_getter(current_action, read_value, 14, response); 
      break;
    }
    case action::read_velocity_target: {
      float read_value = position_to_rotations(p_bldc.get_target_velocity()); 
      float_getter(current_action, read_value, 14, response); 
      break;
    }
    case action::read_velocity_reading: {
      float read_value = position_to_rotations(p_bldc.get_reading_velocity()); 
      float_getter(current_action, read_value, 14, response); 
      break;
    }
    case action::read_power: {
      float read_value = p_bldc.get_power(); 
      float_getter(current_action, read_value, 14, response); 
      break;
    }
    case action::read_pid_position_config: {
      bldc_perseus::PID_settings settings = p_bldc.get_pid_settings(); 
      pid_settings_getter(current_action, settings, response); 
      break;
    }
    case action::read_pid_velocity_config: {
      bldc_perseus::PID_settings settings = p_bldc.get_pid_settings(); 
      pid_settings_getter(current_action, settings, response); 
      break;
    } 
    case action::prev_joint_actual_position: {
      float read_value = position_to_rotations(p_bldc.get_actual_position()); 
      float_getter(current_action, read_value, 14, response); 
      hal::print<64>(*console, "Actual position = %d\n", read_value);
      break;
    }
    case action::prev_joint_position_response: {
      hal::i32 prev_join_response = can_util::byte_array_to_int32_big_endian(
        {p_message.payload[2], p_message.payload[3],
                p_message.payload[4], p_message.payload[5]}); 
      float prev_joint_fixed = can_util::fixed_to_floating_point_32(prev_join_response, p_message.payload[1]); 
      float prev_joint_pos = rotations_to_position(prev_joint_fixed); 
      if (p_bldc.get_physical_servo_values().clockwise_positive == false){ 
        prev_joint_pos = prev_joint_pos * -1; 
      }
      p_bldc.set_prev_joint_position(prev_joint_pos);
      hal::print<64>(*console, "prev_pos = %f\n", 0.0f);
      break;
    }
    default:
      response = hal::can_message{
                        .id = static_cast<hal::u16>(m_self_servo_addr + 0x100), 
                        .length = 1, 
                        .payload = {
                        static_cast<hal::byte>(p_message.payload[0]), 
                        0X00, 0X00, 0X00, 0X00,
                        0X00, 0X00, 0X00
                        }
                      }; 
      throw hal::operation_not_supported(nullptr); 
      break; 
  }
  m_can_transceiver->send(response);
  drivers::can_util::print_can_message(*console, response);
  hal::print<64>(*console, "finished transmission\n");
}

std::optional<hal::can_message> can_perseus::check_for_mc_message() {
  auto msg = m_command_message_finder.find();
  auto console = resources::console(); 
  if (msg) {
    return msg; 
  }
  return std::nullopt; 
}


} // namespace sjsu::perseus