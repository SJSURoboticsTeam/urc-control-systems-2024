#include <cmath>
#include <libhal-util/can.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/can.hpp>
#include <libhal/pointers.hpp>
#include <libhal/units.hpp>
#include <optional>

#include <resource_list.hpp>
#include <bldc_servo.hpp>
#include <can_messaging.hpp>

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

// decode message from mission control 
float can_perseus::fixed_to_floating_point_32(hal::byte b0, hal::byte b1, hal::byte b2, hal::byte b3, float exponent) {
  auto console = resources::console();
  hal::i32 initial = (static_cast<hal::i32>(b0) << 24)
                      | (static_cast<hal::i32>(b1) << 16)
                      | (static_cast<hal::i32>(b2) << 8)
                      | (static_cast<hal::i32>(b3)); 
  hal::print<64>(*console, "pure_binary: %x -- ", initial); 
  float shifted = static_cast<float>(initial) / powf(2, exponent); 
  hal::print<64>(*console, "float cast: %f\n", shifted); 
  return shifted; 
}
float can_perseus::fixed_to_floating_point_16(hal::byte b0, hal::byte b1, float exponent) {
  auto console = resources::console();
  hal::i32 initial = (static_cast<hal::i32>(b0) << 8)
                      | (static_cast<hal::i32>(b1)); 
  hal::print<64>(*console, "pure_binary: %x -- ", initial); 
  float shifted = static_cast<float>(initial) / powf(2, exponent); 
  hal::print<64>(*console, "float cast: %f\n", shifted); 
  return shifted; 
}
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
// endcode message to mission control 
hal::i32 can_perseus::floating_to_fixed_point_32(float n, float exponent) {
  auto console = resources::console(); 
  float initial = n * powf(2, exponent); 
  hal::print<64>(*console, "moved: %f -- ", initial); 
  auto shifted = static_cast<hal::i32>(initial); 
  hal::print<64>(*console, "int cast: %d\n", shifted); 
  return(shifted);
}
hal::i16 can_perseus::floating_to_fixed_point_16(float n, float exponent) {
  auto console = resources::console(); 
  float initial = n * powf(2, exponent); 
  hal::print<64>(*console, "moved: %f -- ", initial); 
  auto shifted = static_cast<hal::i16>(initial); 
  hal::print<64>(*console, "int cast: %d\n", shifted); 
  return(shifted);
  // return (static_cast<hal::i16>(n) << exponent); 
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

void can_perseus::create_response(hal::v5::strong_ptr<hal::can_message> const& r_message,
                                    hal::u16 id, hal::byte len, 
                                    hal::byte b0, hal::byte b1, hal::byte b2, hal::byte b3, 
                                    hal::byte b4, hal::byte b5, hal::byte b6, hal::byte b7) 
                                  {
  r_message->id = id; 
  r_message->length = len; 
  r_message->payload[0] = b0;
  r_message->payload[1] = b1; 
  r_message->payload[2] = b2; 
  r_message->payload[3] = b3; 
  r_message->payload[4] = b4; 
  r_message->payload[5] = b5; 
  r_message->payload[6] = b6; 
  r_message->payload[7] = b7; 
  
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
  auto response_ptr = hal::v5::make_strong_ptr<decltype(response)>(resources::driver_allocator(), response); 
  auto console = resources::console();
  switch (static_cast<action>(p_message.payload[0])) {
    // major 
    case action::power_off_reset:{
      bldc->stop(); 
      bldc->set_active_action(static_cast<uint32_t>(action::power_off_reset)); 
      break;
    }
    case action::heartbeat: {
      create_response(response_ptr, m_self_servo_addr + 0x100, 1, 
                        m_self_servo_addr + 0x50, 0x00, 0x00, 0x00,
                        0x00, 0x00, 0x00, 0x00 
                      ); 
      bldc->set_active_action(static_cast<uint32_t>(action::heartbeat)); 
      break; 
    }
    case action::homing: {
      create_response(response_ptr, m_self_servo_addr + 0x100, 1, 
                        0x11, 0x00, 0x00, 0x00,
                        0x00, 0x00, 0x00, 0x00); 
      bldc->set_active_action(static_cast<uint32_t>(action::homing)); 
      break; 
    }
    // setters 
    case action::set_position_target: {
      float target_position = floating_to_position(
          fixed_to_floating_point_32(p_message.payload[2], p_message.payload[3],
                p_message.payload[4], p_message.payload[5],
          p_message.payload[1]) 
        );
      bldc->set_target_position(target_position);
      hal::print<64>(*console, "Target = %f\n", target_position);
      create_response(response_ptr, m_self_servo_addr + 0x100, 6, 
                        static_cast<hal::byte>(action::set_position_target), 
                        p_message.payload[1], 
                        p_message.payload[2], 
                        p_message.payload[3], 
                        p_message.payload[4], 
                        p_message.payload[5], 
                        0x00, 0x00); 
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
      float reading_position = floating_to_position(
          fixed_to_floating_point_32(p_message.payload[2], p_message.payload[3],
                p_message.payload[4], p_message.payload[5],
          p_message.payload[1]) 
        );
      float new_angle_offset = bldc->get_actual_position() - reading_position + bldc->get_angle_offset(); 
      bldc->set_angle_offset(new_angle_offset); 
      hal::print<64>(*console, "reading = %f\n", reading_position);
      create_response(response_ptr, m_self_servo_addr + 0x100, 6, 
                        static_cast<hal::byte>(action::set_position_reading), 
                        p_message.payload[1], 
                        p_message.payload[2], 
                        p_message.payload[3], 
                        p_message.payload[4], 
                        p_message.payload[5], 
                        0x00, 0x00); 
      bldc->set_active_action(static_cast<uint32_t>(action::set_position_reading)); 
      break;
    }
    case action::set_velocity_target: {
      float target_velocity = floating_to_position(
          fixed_to_floating_point_32(p_message.payload[2], p_message.payload[3],
                p_message.payload[4], p_message.payload[5],
          p_message.payload[1]) 
        );
      bldc->set_target_position(target_velocity);
      hal::print<64>(*console, "Target = %f\n", target_velocity);
      create_response(response_ptr, m_self_servo_addr + 0x100, 6, 
                        static_cast<hal::byte>(action::set_velocity_target), 
                        p_message.payload[1], 
                        p_message.payload[2], 
                        p_message.payload[3], 
                        p_message.payload[4], 
                        p_message.payload[5], 
                        0x00, 0x00); 
      bldc->set_active_action(static_cast<uint32_t>(action::set_velocity_target)); 
      break;
    }
    case action::set_power: {
      float power = fixed_to_floating_point_16(p_message.payload[2], p_message.payload[3], 0) * static_cast<float>(p_message.payload[1]); 
      bldc->set_power(power); 
      create_response(response_ptr, m_self_servo_addr + 0x100, 4, 
                        static_cast<hal::byte>(action::set_power), 
                        p_message.payload[1], 
                        p_message.payload[2], 
                        p_message.payload[3], 
                        0x00, 0x00, 0x00, 0x00); 
      bldc->set_active_action(static_cast<uint32_t>(action::set_power)); 
      break; 
    }
    case action::set_pid_position_config: {
      bldc_perseus::PID_settings settings = {
        .kp = fixed_to_floating_point_16(p_message.payload[2], p_message.payload[3], p_message.payload[1]),
        .ki = fixed_to_floating_point_16(p_message.payload[4], p_message.payload[5], p_message.payload[1]),
        .kd = fixed_to_floating_point_16(p_message.payload[6], p_message.payload[7], p_message.payload[1])
      };
      bldc->update_pid_position(settings);
      create_response(response_ptr, m_self_servo_addr + 0x100, 8, 
                        static_cast<hal::byte>(action::set_pid_position_config), 
                        p_message.payload[1], 
                        p_message.payload[2], 
                        p_message.payload[3], 
                        p_message.payload[4], 
                        p_message.payload[5], 
                        p_message.payload[6], 
                        p_message.payload[7]); 
      bldc->set_active_action(static_cast<uint32_t>(action::set_pid_position_config)); 
      break;
    }
    case action::set_pid_velocity_config: {
      bldc_perseus::PID_settings settings = {
        .kp = fixed_to_floating_point_16(p_message.payload[2], p_message.payload[3], p_message.payload[1]),
        .ki = fixed_to_floating_point_16(p_message.payload[4], p_message.payload[5], p_message.payload[1]),
        .kd = fixed_to_floating_point_16(p_message.payload[6], p_message.payload[7], p_message.payload[1])
      };
      bldc->update_pid_position(settings);
      create_response(response_ptr, m_self_servo_addr + 0x100, 8, 
                        static_cast<hal::byte>(action::set_pid_position_config), 
                        p_message.payload[1], 
                        p_message.payload[2], 
                        p_message.payload[3], 
                        p_message.payload[4], 
                        p_message.payload[5], 
                        p_message.payload[6], 
                        p_message.payload[7]); 
      bldc->set_active_action(static_cast<uint32_t>(action::set_pid_velocity_config)); 
      break;
    }
    // readers 
    case action::read_position_target: {
      float exponent = 14.0; 
      hal::i32 t = floating_to_fixed_point_32(position_to_floating(bldc->get_target_position()), exponent);  
      hal::print<64>(*console, "Target = %f\n", t);
      create_response(response_ptr, m_self_servo_addr + 0x100, 6, 
                        static_cast<hal::byte>(action::read_position_target), 
                        static_cast<hal::byte>(exponent),
                        static_cast<hal::byte>(t >> 24) & 0xFF,
                        static_cast<hal::byte>(t >> 16) & 0xFF,
                        static_cast<hal::byte>(t >> 8) & 0xFF,
                        static_cast<hal::byte>(t >> 0) & 0xFF,
                        0X00, 0X00); 
      bldc->set_active_action(static_cast<uint32_t>(action::read_position_target)); 
      break;
    }
    case action::read_position_reading: {
      float exponent = 14.0; 
      hal::i32 t = floating_to_fixed_point_32(position_to_floating(bldc->get_actual_position()), exponent); 
      hal::print<64>(*console, "reading = %i\n", t);
      create_response(response_ptr, m_self_servo_addr + 0x100, 6, 
                        static_cast<hal::byte>(action::read_position_reading), 
                        static_cast<hal::byte>(exponent),
                        static_cast<hal::byte>(t >> 24) & 0xFF,
                        static_cast<hal::byte>(t >> 16) & 0xFF,
                        static_cast<hal::byte>(t >> 8) & 0xFF,
                        static_cast<hal::byte>(t >> 0) & 0xFF,
                        0X00, 0X00); 
      bldc->set_active_action(static_cast<uint32_t>(action::read_position_reading)); 
      break;
    }
    case action::read_velocity_target: {
      float exponent = 14.0; 
      hal::i32 t = floating_to_fixed_point_32(position_to_floating(bldc->get_target_velocity()), exponent); 
      create_response(response_ptr, m_self_servo_addr + 0x100, 6, 
                        static_cast<hal::byte>(action::read_velocity_target), 
                        static_cast<hal::byte>(exponent),
                        static_cast<hal::byte>(t >> 24) & 0xFF,
                        static_cast<hal::byte>(t >> 16) & 0xFF,
                        static_cast<hal::byte>(t >> 8) & 0xFF,
                        static_cast<hal::byte>(t >> 0) & 0xFF,
                        0X00, 0X00); 
      bldc->set_active_action(static_cast<uint32_t>(action::read_velocity_target)); 
      break;
    }
    case action::read_velocity_reading: {
      float exponent = 14.0; 
      hal::i32 t = floating_to_fixed_point_32(position_to_floating(bldc->get_reading_velocity()), exponent); 
      create_response(response_ptr, m_self_servo_addr + 0x100, 6, 
                        static_cast<hal::byte>(action::read_velocity_target), 
                        static_cast<hal::byte>(exponent),
                        static_cast<hal::byte>(t >> 24) & 0xFF,
                        static_cast<hal::byte>(t >> 16) & 0xFF,
                        static_cast<hal::byte>(t >> 8) & 0xFF,
                        static_cast<hal::byte>(t >> 0) & 0xFF,
                        0X00, 0X00); 
      bldc->set_active_action(static_cast<uint32_t>(action::read_velocity_reading)); 
      break;
    }
    case action::read_power: {
      hal::i16 t = floating_to_fixed_point_16(bldc->get_power(), 0); 
      hal::byte dir = 0x00; 
      if (t < 0) {
        dir = static_cast<hal::byte>(-1); 
      }
      else {
        dir = 0x01; 
      }
      create_response(response_ptr, m_self_servo_addr + 0x100, 4, 
                        static_cast<hal::byte>(action::read_power), 
                        dir, 
                        static_cast<hal::byte>(t >> 8) & 0xFF, 
                        static_cast<hal::byte>(t >> 8) & 0xFF, 
                        0x00, 0x00, 0x00, 0x00); 
      bldc->set_active_action(static_cast<uint32_t>(action::read_power)); 
      break;
    }
    case action::read_pid_position_config: {
      bldc_perseus::PID_settings settings = bldc->get_pid_settings(); 
      hal::i16 t_kp = floating_to_fixed_point_16(settings.kp, p_message.payload[1]); 
      hal::i16 t_ki = floating_to_fixed_point_16(settings.ki, p_message.payload[1]); 
      hal::i16 t_kd = floating_to_fixed_point_16(settings.kd, p_message.payload[1]); 
      create_response(response_ptr, m_self_servo_addr + 0x100, 8, 
                        static_cast<hal::byte>(action::read_pid_position_config), 
                        p_message.payload[1],
                        static_cast<hal::byte>(t_kp >> 8) & 0xFF,
                        static_cast<hal::byte>(t_kp >> 0) & 0xFF,
                        static_cast<hal::byte>(t_ki >> 8) & 0xFF,
                        static_cast<hal::byte>(t_ki >> 0) & 0xFF,
                        static_cast<hal::byte>(t_kd >> 8) & 0xFF,
                        static_cast<hal::byte>(t_kd >> 0) & 0xFF); 
      bldc->set_active_action(static_cast<uint32_t>(action::read_pid_position_config)); 
      break;
    }
    case action::read_pid_velocity_config: {
      bldc_perseus::PID_settings settings = bldc->get_pid_settings(); 
      hal::i16 t_kp = floating_to_fixed_point_16(settings.kp, p_message.payload[1]); 
      hal::i16 t_ki = floating_to_fixed_point_16(settings.ki, p_message.payload[1]); 
      hal::i16 t_kd = floating_to_fixed_point_16(settings.kd, p_message.payload[1]); 
      create_response(response_ptr, m_self_servo_addr + 0x100, 8, 
                        static_cast<hal::byte>(action::read_pid_velocity_config), 
                        p_message.payload[1],
                        static_cast<hal::byte>(t_kp >> 8) & 0xFF,
                        static_cast<hal::byte>(t_kp >> 0) & 0xFF,
                        static_cast<hal::byte>(t_ki >> 8) & 0xFF,
                        static_cast<hal::byte>(t_ki >> 0) & 0xFF,
                        static_cast<hal::byte>(t_kd >> 8) & 0xFF,
                        static_cast<hal::byte>(t_kd >> 0) & 0xFF); 
      bldc->set_active_action(static_cast<uint32_t>(action::read_pid_velocity_config)); 
      break;
    } 
    case action::prev_joint_actual_position: {
      float exponent = 14.0; 
      hal::i32 t = floating_to_fixed_point_32(position_to_floating(bldc->get_actual_position()), exponent); 
      hal::print<64>(*console, "Actual position = %d\n", t);
      hal::u16 next_addr = (static_cast<hal::u16>(p_message.payload[1]) << 8) 
                            | static_cast<hal::u16>(p_message.payload[2]); 
      create_response(response_ptr, next_addr, 6, 
                        static_cast<hal::byte>(action::prev_joint_position_response), 
                        static_cast<hal::byte>(exponent),
                        static_cast<hal::byte>(t >> 24) & 0xFF,
                        static_cast<hal::byte>(t >> 16) & 0xFF,
                        static_cast<hal::byte>(t >> 8) & 0xFF,
                        static_cast<hal::byte>(t >> 0) & 0xFF,
                        0X00, 0X00); 
      break;
    }
    case action::prev_joint_position_response: {
      float prev_joint_pos = floating_to_position(
          fixed_to_floating_point_32(p_message.payload[2], p_message.payload[3],
                p_message.payload[4], p_message.payload[5],
          p_message.payload[1])
        );
      if (m_self_servo_addr == wrist_left){ 
        prev_joint_pos = prev_joint_pos * -1; 
      }
      bldc->set_prev_joint_position(prev_joint_pos);
      bldc->set_actual_position();
      hal::print<64>(*console, "prev_pos = %f\n", prev_joint_pos);
      break;
    }
    default:
      create_response(response_ptr, m_self_servo_addr + 0x100, 1, 
                        static_cast<hal::byte>(p_message.payload[0]) + 0x100, 
                        0X00, 0X00, 0X00, 0X00,
                        0X00, 0X00, 0X00
                      ); 
      throw hal::operation_not_supported(nullptr); 
      break; 
  }
  m_mc_message_finder.transceiver().send(*response_ptr);
  print_can_message(*console, *response_ptr);
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