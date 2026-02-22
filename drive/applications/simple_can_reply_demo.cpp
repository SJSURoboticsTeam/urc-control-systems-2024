#include <drivetrain.hpp>
#include <drivetrain_math.hpp>
#include <libhal-exceptions/control.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/error.hpp>
#include <libhal/units.hpp>
#include <mission_control_manager.hpp>
#include <optional>
#include <resource_list.hpp>
#include <swerve_module.hpp>

namespace sjsu::drive {

void application()
{
  constexpr hal::time_duration cycle_time = 50ms;
  auto clock = resources::clock();
  auto console = resources::console();
  auto can_transceiver = resources::can_transceiver();
  mission_control_manager mcm(can_transceiver);

  hal::print(*console, "appstart\n");
  while (true) {
    hal::u64 frame_end = hal::future_deadline(*clock, cycle_time);

    bool home_req = mcm.read_homing_request();
    std::optional<chassis_velocities_request> cvr =
      mcm.read_set_velocity_request();
    if (cvr) {
      mcm.reply_set_velocity_request(cvr.value());
      hal::print<64>(*console,
                     "\nset vel: %f, %f, %f, %d\n",
                     cvr->chassis_vels.translation.x,
                     cvr->chassis_vels.translation.y,
                     cvr->chassis_vels.rotational_vel,
                     cvr->module_conflicts);
    } else if (home_req) {
      hal::print(*console, "\nhoming\n");
      mcm.reply_homing_request();
    }

    mcm.reply_heartbeat();

    while (clock->uptime() < frame_end)
      ;
  }
}
}  // namespace sjsu::drive
