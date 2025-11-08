#include <cmath>
#include <libhal-util/can.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/can.hpp>
#include <libhal/error.hpp>
#include <libhal/pointers.hpp>


#include <type_traits>

#include "../hardware_map.hpp"
#include "../include/bldc_servo.hpp"

#include "../include/can_messaging.hpp"

using namespace std::chrono_literals;
namespace sjsu::perseus {

can_perseus::can_perseus(hal::u16 curr_servo_addr) : m_curr_servo_addr(curr_servo_addr) {}


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

void can_perseus::process_can_message(hal::can_message const& p_message,
                         hal::v5::strong_ptr<bldc_perseus> bldc,
                         [[maybe_unused]] hal::v5::strong_ptr<hal::can_message> response)
{
  switch (static_cast<action>(p_message.payload[0])) {
    // case action::read_position: {
    //   auto current_position = bldc->get_current_position();
    //   response->length = 3;
    //   response->payload[0] =
    //     static_cast<hal::byte>(action::read_position) + 0x50;
    //   response->payload[1] = (current_position >> 8) & 0xff;
    //   response->payload[1] = current_position & 0xff;
    //   break;
    // }
    case action::set_position: {
      auto target_position = p_message.payload[1] << 8 | p_message.payload[2];
      bldc->set_target_position(target_position);
      break;
    }
    case action::set_velocity: { 
      auto target_velocity = p_message.payload[1] << 8 | p_message.payload[2];
      bldc->set_target_velocity(target_velocity);
      break;
    }
    case action::clamp_speed: {
      auto target_speed = p_message.payload[1] << 8 | p_message.payload[2];
      bldc->set_clamped_speed(target_speed);
      break;
    }
    case action::read_velocity: {
      // auto current_velocity = bldc->get_current_velocity_percentage();
      // response->payload[0] =
      //   static_cast<hal::byte>(action::read_velocity) + 0x50;
      // response->payload[1] =
      //   (current_velocity >> 8) & 0xFF;  // HIGH BYTE FIRST // HIGH BYTE FIRST
      // response->payload[2] = current_velocity & 0xFF;  // LOW BYTE SECOND
      break;
    }
    case action::stop:
      bldc->set_current_velocity(0);
      break;
    case action::set_pid_position: {
      // change u16 to float 
      hal::u16 b_kp = (p_message.payload[3] << 8 | p_message.payload[2]); 
      hal::u16 b_ki = (p_message.payload[5] << 8 | p_message.payload[4]); 
      hal::u16 b_kd = (p_message.payload[7] << 8 | p_message.payload[6]); 
      // proper division way 
      hal::u16 flip_kp = 0; 
      hal::u16 flip_ki = 0; 
      hal::u16 flip_kd = 0; 
      
      for (int i = 0; i < 16-p_message.payload[1]; i++) {
        flip_kp = (flip_kp | ((b_kp & (1 << i)) >> i)) << 1; 
        flip_ki = (flip_ki | ((b_ki & (1 << i)) >> i)) << 1;
        flip_kd = (flip_kd | ((b_kd & (1 << i)) >> i)) << 1;
      }
      // float val_kp = static_cast<float>(b_kp >> p_message.payload[1]) + (static_cast<float>(flip_kp) / pow(2, p_message.payload[1])); 
      // float val_ki = static_cast<float>(b_ki >> p_message.payload[1]) + (static_cast<float>(flip_ki) / pow(2, p_message.payload[1])); 
      // float val_kd = static_cast<float>(b_kd >> p_message.payload[1]) + (static_cast<float>(flip_kd) / pow(2, p_message.payload[1])); 
      
      // decimal shift way 
      float val_kp = static_cast<float>(b_kp >> p_message.payload[1]) + static_cast<float>(b_kp & (0xff >> (16 - p_message.payload[1]))) / pow(10, p_message.payload[1]);
      float val_ki = static_cast<float>(b_ki >> p_message.payload[1]) + static_cast<float>(b_ki & (0xff >> (16 - p_message.payload[1]))) / pow(10, p_message.payload[1]);
      float val_kd = static_cast<float>(b_kd >> p_message.payload[1]) + static_cast<float>(b_kd & (0xff >> (16 - p_message.payload[1]))) / pow(10, p_message.payload[1]);

      bldc_perseus::PID_settings settings = {
        .kp = val_kp,
          //static_cast<float>(p_message.payload[1] << 8 | p_message.payload[2]),
        .ki = val_ki,
          //static_cast<float>(p_message.payload[3] << 8 | p_message.payload[4]),
        .kd = val_kd,
          //static_cast<float>(p_message.payload[5] << 8 | p_message.payload[6])
      };
      bldc->update_pid_position(settings);
      response->id = static_cast<hal::byte>(action::set_pid_position) + 0x100;
      response->length = 1;
      response->payload[0] = static_cast<hal::byte>(action::set_pid_position) + 0x50;
      break;
    }
    case action::set_pid_velocity: {
      // change u16 to float 
      hal::u16 b_kp = (p_message.payload[3] << 8 | p_message.payload[2]); 
      hal::u16 b_ki = (p_message.payload[5] << 8 | p_message.payload[4]); 
      hal::u16 b_kd = (p_message.payload[7] << 8 | p_message.payload[6]); 
      // proper division way 
      hal::u16 flip_kp = 0; 
      hal::u16 flip_ki = 0; 
      hal::u16 flip_kd = 0; 
      
      for (int i = 0; i < 16-p_message.payload[1]; i++) {
        flip_kp = (flip_kp | ((b_kp & (1 << i)) >> i)) << 1; 
        flip_ki = (flip_ki | ((b_ki & (1 << i)) >> i)) << 1;
        flip_kd = (flip_kd | ((b_kd & (1 << i)) >> i)) << 1;
      }
      // float val_kp = static_cast<float>(b_kp >> p_message.payload[1]) + (static_cast<float>(flip_kp) / pow(2, p_message.payload[1])); 
      // float val_ki = static_cast<float>(b_ki >> p_message.payload[1]) + (static_cast<float>(flip_ki) / pow(2, p_message.payload[1])); 
      // float val_kd = static_cast<float>(b_kd >> p_message.payload[1]) + (static_cast<float>(flip_kd) / pow(2, p_message.payload[1])); 
      
      // decimal shift way 
      float val_kp = static_cast<float>(b_kp >> p_message.payload[1]) + static_cast<float>(b_kp & (0xff >> (16 - p_message.payload[1]))) / pow(10, p_message.payload[1]);
      float val_ki = static_cast<float>(b_ki >> p_message.payload[1]) + static_cast<float>(b_ki & (0xff >> (16 - p_message.payload[1]))) / pow(10, p_message.payload[1]);
      float val_kd = static_cast<float>(b_kd >> p_message.payload[1]) + static_cast<float>(b_kd & (0xff >> (16 - p_message.payload[1]))) / pow(10, p_message.payload[1]);

      bldc_perseus::PID_settings settings = {
        .kp = val_kp,
          //static_cast<float>(p_message.payload[1] << 8 | p_message.payload[2]),
        .ki = val_ki,
          //static_cast<float>(p_message.payload[3] << 8 | p_message.payload[4]),
        .kd = val_kd,
          //static_cast<float>(p_message.payload[5] << 8 | p_message.payload[6])
      };
      bldc->update_pid_velocity(settings);
      response->id = static_cast<hal::byte>(action::set_pid_velocity) + 0x100;
      response->length = 1;
      response->payload[0] = static_cast<hal::byte>(action::set_pid_velocity) + 0x50;
      break;
    }
    case action::heartbeat: {
      response->id = static_cast<hal::byte>(action::heartbeat) + 0x100;
      response->length = 1;
      response->payload[0] = m_curr_servo_addr + 0x50;
      break; 
    }
    default:
      hal::operation_not_supported(nullptr);
  }
}


} // namespace sjsu::perseus