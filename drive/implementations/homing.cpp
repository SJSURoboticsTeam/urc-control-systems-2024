#include "homing.hpp"
#include <libhal-actuator/smart_servo/rmd/mc_x_v2.hpp>
#include <libhal/can.hpp>
#include <libhal/units.hpp>

using namespace std::chrono_literals;
using namespace hal::literals;
namespace sjsu::drive {

void send_custom_message(hal::u32 p_id,
                         hal::can_transceiver& p_can,
                         hal::u8 p_length,
                         std::array<hal::byte, 8>&& p_payload)
{
  const hal::can_message message = { .id = p_id,
                                     .length = p_length,
                                     .payload = p_payload };
  p_can.send(message);
}

void home(std::span<steering_module> legs,
          std::span<start_wheel_setting> setting_span,
          // hal::can_transceiver& can,
          hal::steady_clock& clock,
          hal::serial& terminal)
{
  for (size_t i = 0; i < legs.size(); i++) {
    auto& mc_x = legs[i].steer;
    mc_x->feedback_request(hal::actuator::rmd_mc_x_v2::read::multi_turns_angle);
    float start_angle = mc_x->feedback().angle();
    hal::print<128>(terminal, "start angle: %f\n", start_angle);

    while (!legs[i].limit_switch.value()->level()) {
      if (setting_span[i].reversed) {
        mc_x->velocity_control(1.0);
        hal::delay(clock, 10ms);
      } else {
        mc_x->velocity_control(-1.0);
        hal::delay(clock, 10ms);
      }
    }
    mc_x->feedback_request(hal::actuator::rmd_mc_x_v2::read::multi_turns_angle);
    float stop_angle = mc_x->feedback().angle();

    mc_x->velocity_control(0); // stops 
    hal::delay(clock, 1000ms);
    mc_x->position_control(stop_angle + setting_span[i].homing_offset, 1);
    stop_angle = mc_x->feedback().angle();

    hal::print<128>(terminal, "Stopped angle: %f\n", stop_angle);
    setting_span[i].homing_angle = stop_angle;
    
  }
}
}  // namespace sjsu::drive