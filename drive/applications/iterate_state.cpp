#include "vector2d.hpp"
#include <ctime>
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
bool loop = false;
constexpr state_and_duration states[] = {
  { chassis_velocities(vector2d(0, 0), 0), 2s },
  { chassis_velocities(vector2d(0.40, 0), 0), 8s },
  { chassis_velocities(vector2d(0, 0), 0), 4s },
  { chassis_velocities(vector2d(0, 0), 0.5), 17500ms },
  { chassis_velocities(vector2d(0, 0), 0), 4s }
};

void application()
{
  constexpr hal::time_duration cycle_time = 50ms;
  constexpr sec cycle_time_sec = hal_time_duration_to_sec(cycle_time);
  auto clock = resources::clock();
  hal::delay(*clock, 1s);
  [[maybe_unused]] auto console = resources::console();
  drivetrain dt(resources::swerve_modules(), cycle_time_sec);
  hal::print(*console,"homming\n");
  dt.hard_home();

  hal::time_duration loop_duration = 0ns;
  for (unsigned int i = 0; i < sizeof(states) / sizeof(states[0]); i++) {
    loop_duration += states[i].duration;
  }
  hal::print<64>(
    *console, "loop_dur:%lld\n", static_cast<long long>(loop_duration / 1ns));

  int state_index = 0;
  hal::time_duration start_time = get_clock_time(*clock);
  hal::time_duration state_end_time = states[0].duration;
  hal::time_duration previous_time = 0ns;
  while (true) {
    hal::print(*console, "Cycle Start\n");
    hal::u64 frame_end = hal::future_deadline(*clock, cycle_time);

    // convert clock to hal::time_duration
    hal::time_duration cur_time = get_clock_time(*clock) - start_time;
    hal::print<64>(
      *console, "cur_time:%lld\n", static_cast<long long>(cur_time / 1ns));
    hal::print<64>(*console,
                   "state_end_time:%lld\n",
                   static_cast<long long>(state_end_time / 1ns));
    if (!loop && cur_time > start_time + loop_duration) {
      dt.stop();
      break;
    } else if (cur_time % loop_duration < previous_time % loop_duration) {
      state_index = 0;
      state_end_time = states[0].duration;
    }

    while (cur_time > state_end_time) {
      state_index++;
      state_end_time += states[state_index].duration;
      hal::print<64>(
        *console,
        "new state: (%f,%f,%f), time:%lld\n",
        states[state_index].velocities.translation.x,
        states[state_index].velocities.translation.y,
        states[state_index].velocities.rotational_vel,
        static_cast<long long>(states[state_index].duration / 1ns));
      hal::print<64>(*console,
                     "state_end_time:%lld\n",
                     static_cast<long long>(state_end_time / 1ns));
    }
    dt.set_target_state(states[state_index].velocities, true);
    try {
      dt.periodic();
    } catch (hal::exception e) {
      print<64>(*console, "periodic failed, error code: %d\n", e.error_code());
      throw;
    }

    previous_time = cur_time;

    while (clock->uptime() < frame_end) {
    }
  }
}
}  // namespace sjsu::drive
