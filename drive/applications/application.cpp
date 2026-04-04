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
  auto console = resources::console();
  auto clock = resources::clock();
  constexpr hal::time_duration cycle_time = 250ms;
  constexpr sec cycle_time_sec = hal_time_duration_to_sec(cycle_time);
  drivetrain dt(resources::swerve_modules(), cycle_time_sec);
  mission_control_manager mcm(resources::can_transceiver());

  dt.hard_home();  // TODO: move homing into main loop
  while (true) {
    hal::u64 frame_end = hal::future_deadline(*clock, cycle_time);

    try {
      dt.periodic();
    } catch (hal::exception e) {
      hal::print(*console, "\nexception thrown in periodic\n");
      throw;
    }

    hal::print(*console, "\ncommand handle\n");
    bool home_req = mcm.read_homing_request();
    std::optional<chassis_velocities_request> cvr =
      mcm.read_set_velocity_request();
    if (cvr) {
      hal::print<64>(*console,
                     "\nset vel: %f, %f, %f, %d\n",
                     cvr->chassis_vels.translation.x,
                     cvr->chassis_vels.translation.y,
                     cvr->chassis_vels.rotational_vel,
                     cvr->module_conflicts);
      if (home_req) {
        cvr->chassis_vels = { { 0, 0 }, 0 };
      }
      bool resolved =
        dt.set_target_state(cvr->chassis_vels, cvr->module_conflicts);
      cvr->module_conflicts = resolved;
      mcm.reply_set_velocity_request(cvr.value());
    } else if (home_req) {
      hal::print(*console, "\nhoming\n");
      mcm.reply_homing_request();
      // dt.hard_home();// TODO: replace with interuptable homing later
    }
    hal::print(*console, "\ndata req\n");
    mcm.fulfill_data_requests(dt);
    hal::print(*console, "\nheartbeat\n");
    mcm.reply_heartbeat();

    while (clock->uptime() < frame_end)
      ;
  }
}
}  // namespace sjsu::drive
