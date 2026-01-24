#include <drivetrain.hpp>
#include <drivetrain_math.hpp>
#include <libhal-exceptions/control.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/error.hpp>
#include <resource_list.hpp>

namespace sjsu::drive {
void application()
{
  auto clock = resources::clock();
  auto console = resources::console();
  try {
  using namespace std::chrono_literals;
  constexpr auto refresh_rate = 250ms;
  constexpr auto transition_time = 5s;
  constexpr int delay_cycles = transition_time / refresh_rate;
  drivetrain dt(resources::swerve_modules(),
                hal_time_duration_to_sec(refresh_rate));

  dt.async_home_begin();
  while (!dt.async_home_poll()) {
    hal::delay(*clock, 250ms);
  }

  dt.set_target_state(chassis_velocities(vector2d(0, 0), 1), true);
  hal::print<128>(*console,
                  "v:%f,%f,%f\n",
                  dt.get_target_state().translation.x,
                  dt.get_target_state().translation.y,
                  dt.get_target_state().rotational_vel);
  for (int i = 0; i < delay_cycles; i++) {
    dt.periodic();
    hal::delay(*clock, refresh_rate);
  }
  dt.set_target_state(chassis_velocities(vector2d(0, 0), 0), true);
  hal::print<128>(*console,
                  "v:%f,%f,%f\n",
                  dt.get_target_state().translation.x,
                  dt.get_target_state().translation.y,
                  dt.get_target_state().rotational_vel);
  for (int i = 0; i < delay_cycles; i++) {
    dt.periodic();
    hal::delay(*clock, refresh_rate);
  }
  dt.set_target_state(chassis_velocities(vector2d(1, 0), 0), true);
  hal::print<128>(*console,
                  "v:%f,%f,%f\n",
                  dt.get_target_state().translation.x,
                  dt.get_target_state().translation.y,
                  dt.get_target_state().rotational_vel);
  for (int i = 0; i < delay_cycles; i++) {
    dt.periodic();
    hal::delay(*clock, refresh_rate);
  }
  dt.set_target_state(chassis_velocities(vector2d(0, 0), 0), true);
  hal::print<128>(*console,
                  "v:%f,%f,%f\n",
                  dt.get_target_state().translation.x,
                  dt.get_target_state().translation.y,
                  dt.get_target_state().rotational_vel);
  for (int i = 0; i < delay_cycles; i++) {
    dt.periodic();
    hal::delay(*clock, refresh_rate);
  }
  dt.set_target_state(chassis_velocities(vector2d(0, 1), 0), true);
  hal::print<128>(*console,
                  "v:%f,%f,%f\n",
                  dt.get_target_state().translation.x,
                  dt.get_target_state().translation.y,
                  dt.get_target_state().rotational_vel);
  for (int i = 0; i < delay_cycles; i++) {
    dt.periodic();
    hal::delay(*clock, refresh_rate);
  }
  dt.set_target_state(chassis_velocities(vector2d(0, 0), 0), true);
  hal::print<128>(*console,
                  "v:%f,%f,%f\n",
                  dt.get_target_state().translation.x,
                  dt.get_target_state().translation.y,
                  dt.get_target_state().rotational_vel);
  for (int i = 0; i < delay_cycles; i++) {
    dt.periodic();
    hal::delay(*clock, refresh_rate);
  }
  dt.set_target_state(chassis_velocities(vector2d(0, 1), 0), true);
  hal::print<128>(*console,
                  "v:%f,%f,%f\n",
                  dt.get_target_state().translation.x,
                  dt.get_target_state().translation.y,
                  dt.get_target_state().rotational_vel);
  for (int i = 0; i < delay_cycles; i++) {
    dt.periodic();
    hal::delay(*clock, refresh_rate);
  }
  dt.set_target_state(chassis_velocities(vector2d(1, 0), 0), true);
  hal::print<128>(*console,
                  "v:%f,%f,%f\n",
                  dt.get_target_state().translation.x,
                  dt.get_target_state().translation.y,
                  dt.get_target_state().rotational_vel);
  for (int i = 0; i < delay_cycles; i++) {
    dt.periodic();
    hal::delay(*clock, refresh_rate);
  }
  dt.set_target_state(chassis_velocities(vector2d(0, 0), 1), true);
  hal::print<128>(*console,
                  "v:%f,%f,%f\n",
                  dt.get_target_state().translation.x,
                  dt.get_target_state().translation.y,
                  dt.get_target_state().rotational_vel);
  for (int i = 0; i < delay_cycles; i++) {
    dt.periodic();
    hal::delay(*clock, refresh_rate);
  }
  dt.set_target_state(chassis_velocities(vector2d(0, 0), 0), true);
  hal::print<128>(*console,
                  "v:%f,%f,%f\n",
                  dt.get_target_state().translation.x,
                  dt.get_target_state().translation.y,
                  dt.get_target_state().rotational_vel);
  for (int i = 0; i < delay_cycles; i++) {
    dt.periodic();
    hal::delay(*clock, refresh_rate);
  }
  hal::print(*console,"fin!");
  } catch (hal::exception e) {
    hal::print<128>(*console, "Exception code %d\n", e.error_code());
  }
}
}  // namespace sjsu::drive
