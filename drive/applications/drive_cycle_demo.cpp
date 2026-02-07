#include <drivetrain.hpp>
#include <drivetrain_math.hpp>
#include <libhal-exceptions/control.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/error.hpp>
#include <libhal/units.hpp>
#include <resource_list.hpp>
#include <swerve_module.hpp>

namespace sjsu::drive {

struct state_and_duration
{
  chassis_velocities velocities;
  hal::time_duration duration;
};
constexpr state_and_duration states[] = {
  { chassis_velocities(vector2d(0, 0), 0), 2s },
  { chassis_velocities(vector2d(1, 0), 0), 2s },
  { chassis_velocities(vector2d(0.5, 0), 0), 2s },
  { chassis_velocities(vector2d(-1, 0), 0), 2s },
  { chassis_velocities(vector2d(-0.5, 0.5), 0), 2s },
  { chassis_velocities(vector2d(0, 1), 0), 2s },
  { chassis_velocities(vector2d(0, 0), 0), 2s },
  { chassis_velocities(vector2d(0, 0), 1), 2s },
  { chassis_velocities(vector2d(0, 0), -1), 2s }
};

void application()
{
  constexpr hal::time_duration cycle_time = 50ms;
  constexpr sec cycle_time_sec = hal_time_duration_to_sec(cycle_time);
  auto clock = resources::clock();
  drivetrain dt(resources::swerve_modules(), cycle_time_sec);

  dt.hard_home();

  hal::time_duration loop_duration = 0ns;
  for (unsigned int i = 0; i < sizeof(states) / sizeof(states[0]); i++) {
    loop_duration += states[i].duration;
  }

  int state_index = 0;
  hal::time_duration state_end_time = states[0].duration;
  hal::time_duration previous_time = 0ns;
  while (true) {
    hal::u64 frame_end = hal::future_deadline(*clock, cycle_time);

    // convert clock to hal::time_duration
    hal::time_duration cur_time = get_clock_time(*clock) % loop_duration;
    if (cur_time < previous_time) {
      state_index = 0;
      state_end_time = states[0].duration;
    }
    while (cur_time < state_end_time) {
      state_index++;
      state_end_time += states[state_index].duration;
    }
    dt.set_target_state(states[state_index].velocities, true);
    dt.periodic();

    previous_time = cur_time;

    while (clock->uptime() < frame_end) {
    }
  }
}
}  // namespace sjsu::drive
