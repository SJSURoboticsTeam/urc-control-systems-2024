// copied from drivers/applications/h_bridge_demo.cpp

#include <libhal-util/can.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/can.hpp>
#include <libhal/error.hpp>
#include <libhal/pointers.hpp>

#include <bldc_servo.hpp>

#include "../hardware_map.hpp"

#include <serial_commands.hpp>
#include "../include/bldc_servo.hpp"


using namespace std::chrono_literals;

namespace sjsu::perseus {

void application()
{
  using namespace std::chrono_literals;
  using namespace hal::literals;
  auto console = resources::console();
  auto clock = resources::clock();
  auto h_bridge = resources::h_bridge();
  auto encoder = resources::encoder();
  hal::print(*console, "CAN message finder initialized...\n");
  bldc_perseus servo(h_bridge, encoder);
  hal::print(*console, "BLDC Servo created...\n");

  auto servo_ptr = hal::v5::make_strong_ptr<decltype(servo)>(resources::driver_allocator(), std::move(servo));
  
  hal::print(*console, "BLDC Servo initialized...\n");
  bldc_perseus::PID_settings pid_settings = {
    .kp = 0.0025,
    // .ki = 0.05,
    .ki = 0,
    .kd = 0,
  };
  servo_ptr->update_pid_position(pid_settings);
  servo_ptr->set_target_position(-30);
  std::array cmd_defs = {
    drivers::serial_commands::def{
      "setpos",
      [&console, &servo_ptr](auto params) {
        if (params.size() != 1) {
          throw hal::argument_out_of_domain(nullptr);
        }
        float position = drivers::serial_commands::parse_float(params[0]);
        servo_ptr->set_target_position(position);
        hal::print<32>(*console, "Set Position to: %f\n", position);
      },
    },
    drivers::serial_commands::def{
      "setkp",
      [&console, &servo_ptr](auto params) {
        if (params.size() != 1) {
          throw hal::argument_out_of_domain(nullptr);
        }
        float kp = drivers::serial_commands::parse_float(params[0]);
        auto current_settings = servo_ptr->get_pid_settings();
        current_settings.kp = kp;
        servo_ptr->update_pid_position(current_settings);
        hal::print<32>(*console, "Set Kp to: %f\n", kp);
      },
    },
    drivers::serial_commands::def{
      "setki",
      [&console, &servo_ptr](auto params) {
        if (params.size() != 1) {
          throw hal::argument_out_of_domain(nullptr);
        }
        float ki = drivers::serial_commands::parse_float(params[0]);
        auto current_settings = servo_ptr->get_pid_settings();
        current_settings.ki = ki;
        servo_ptr->update_pid_position(current_settings);
        hal::print<32>(*console, "Set Ki to: %f\n", ki);
      },
    },
    drivers::serial_commands::def{
      "setkd",
      [&console, &servo_ptr](auto params) {
        if (params.size() != 1) {
          throw hal::argument_out_of_domain(nullptr);
        }
        float kd = drivers::serial_commands::parse_float(params[0]);
        auto current_settings = servo_ptr->get_pid_settings();
        current_settings.kd = kd;
        servo_ptr->update_pid_position(current_settings);
        hal::print<32>(*console, "Set Kd to: %f\n", kd);
      },
    },
    drivers::serial_commands::def{
      "maxpower",
      [&console, &servo_ptr](auto params) {
        if (params.size() != 1) {
          throw hal::argument_out_of_domain(nullptr);
        }
        float power = drivers::serial_commands::parse_float(params[0]);
        servo_ptr->set_max_power(power);
        hal::print<32>(*console, "Set max power: %f\n", power);
      },
    },
    // drivers::serial_commands::def{ "" },
  };
  sjsu::drivers::serial_commands::handler cmd{ console };
  
  while (true) {
    // csv output for easy graphing
    
    // servo_ptr->update_position_noff();
    // hal::print<128>(*console, "Current Position: %.2f\n", servo_ptr->get_current_position());

    // hal::delay(*clock, 10ms);

    try {
      cmd.handle(cmd_defs);
    } catch (hal::exception e) {
      switch (e.error_code()) {
        case std::errc::argument_out_of_domain:
          hal::print(*console, "Error: invalid argument length or type\n");
          break;
        default:
          hal::print<32>(*console, "Error code: %d\n", e.error_code());
          break;
      }
    }
    servo_ptr->update_position_noff();
  } 
}
}  // namespace sjsu::perseus