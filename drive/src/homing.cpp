#include "../include/homing.hpp"
#include <libhal-util/steady_clock.hpp>

using namespace std::chrono_literals;
using namespace hal::literals;
namespace sjsu::drive {

void home(std::span<swerve_module> legs,
          // hal::can_transceiver& can,
          hal::steady_clock& clock,
          hal::serial& terminal)
{
  for (int i = 0; i < 4; i++) {
    auto& mc_x = legs[i].hardware.steer;
    mc_x->feedback_request(hal::actuator::rmd_mc_x_v2::read::multi_turns_angle);
    float start_angle = mc_x->feedback().angle();
    hal::print<128>(terminal, "start angle: %f\n", start_angle);

    while (!legs[i].hardware.limit_switch->level()) {
      if (legs[i].reversed) {
        mc_x->velocity_control(-1);
        hal::delay(clock, 10ms);
      } else {
        mc_x->velocity_control(1);
        hal::delay(clock, 10ms);
      }
    }
    mc_x->feedback_request(hal::actuator::rmd_mc_x_v2::read::multi_turns_angle);
    float stop_angle = mc_x->feedback().angle();

    mc_x->velocity_control(0);  // stops
    hal::delay(clock, 1000ms);

    if(legs[i].reversed){
      mc_x->position_control(stop_angle + 90, 1);
    }else{
      mc_x->position_control(stop_angle - 90, 1);
    }
    stop_angle = mc_x->feedback().angle();

    hal::print<128>(terminal, "Stopped angle: %f\n", stop_angle);
    legs[i].homing_offset = stop_angle;
  }
}
}  // namespace sjsu::drive