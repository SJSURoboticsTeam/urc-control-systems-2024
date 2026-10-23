#include <libhal-util/can.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/can.hpp>

#include <damiao_motor.hpp>
#include <libhal/error.hpp>
#include <resource_list.hpp>

using namespace std::chrono_literals;
namespace sjsu::drivers {

void print_data(damiao_motor& motor,
                hal::v5::strong_ptr<hal::steady_clock> clock,
                hal::time_duration time,
                hal::v5::optional_ptr<hal::serial> console);

void application()
{
  auto clock = resources::clock();
  auto console = resources::console();
  auto can_transceiver = resources::can_transceiver();

  hal::print(*console, "Starting Demo");

  damiao_motor_settings set{
    .master_id = 0x34,
    .can_id = 0x0012,
    .minimum_position = -14400.0f,
    .maximum_position = 14400.0f,
    .minimum_velocity = -240.0f,
    .maximum_velocity = 240.0f,
    .minimum_proportional_gain = 0.0f,
    .maximum_proportional_gain = 500.0f,
    .minimum_derivative_gain = 0.0f,
    .maximum_derivative_gain = 5.0f,
    .minimum_torque = -200.0f,
    .maximum_torque = 200.0f,
  };

  mit_pd_mode_parameters mit_pd_param{ .position = 150.0f,
                                       .velocity = 0.0f,
                                       .proportional_gain = 6.0f,
                                       .derivative_gain = 1.2f,
                                       .torque_feed_forward = 0.0f };

  force_position_hybrid_mode_parameters force_position_hybrid_param{
    .position = 0.0f, .velocity = 30.0f, .torque_current_limit = 0.8f
  };

  damiao_motor motor(can_transceiver, set, clock);
  try {
    motor.enable();
    motor.mit_pd_mode(mit_pd_param);
    print_data(motor, clock, std::chrono::milliseconds(1500), console);
    hal::delay(*clock, 300ms);

    motor.position_velocity_mode(550, 20);
    print_data(motor, clock, std::chrono::milliseconds(5000), console);
    hal::delay(*clock, 300ms);

    motor.velocity_mode_start(-50);
    print_data(motor, clock, std::chrono::milliseconds(3000), console);
    motor.velocity_mode_stop();
    hal::delay(*clock, 300ms);

    motor.force_position_hybrid_mode(force_position_hybrid_param);
    print_data(motor, clock, std::chrono::milliseconds(3000), console);

    motor.disable();
  } catch (hal::timed_out e) {
    hal::print(*console, "Something Timed Out");
  }
}

void print_data(damiao_motor& motor,
                hal::v5::strong_ptr<hal::steady_clock> clock,
                hal::time_duration time,
                hal::v5::optional_ptr<hal::serial> console)
{
  auto const deadline = hal::future_deadline(*clock, time);
  while (clock->uptime() < deadline) {
    damiao_motor_data dat = motor.read_encoder();
    hal::print<96>(*console,
                   "pos = %f degrees, vel = %f rpm, torque = %f Nm \n",
                   dat.position,
                   dat.velocity,
                   dat.torque);
  }
}
}  // namespace sjsu::drivers
