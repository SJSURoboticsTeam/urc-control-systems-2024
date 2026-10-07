#include <libhal-util/can.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/can.hpp>

#include <damiao_motor.hpp>
#include <libhal/error.hpp>
#include <resource_list.hpp>

using namespace std::chrono_literals;
namespace sjsu::drivers {

void print_data(damiao_motor& motor, hal::v5::strong_ptr<hal::steady_clock> clock, hal::time_duration time, hal::v5::optional_ptr<hal::serial> console);

void application()
{
  auto clock = resources::clock();
  auto console = resources::console();
  auto can_transceiver = resources::can_transceiver();

  hal::print(*console, "Starting Demo");

  damiao_motor_settings set{
    .master_id = 0x34,
    .can_id = 0x0012,
    .pos_max = 14400.0f,
    .pos_min = -14400.0f,
    .vel_min = -240.0f,
    .vel_max = 240.0f,
    .Kp_min = 0.0f,
    .Kp_max = 500.0f,
    .Kd_min = 0.0f,
    .Kd_max = 5.0f,
    .torque_min = -200.0f,
    .torque_max = 200.0f,
  };

  damiao_motor motor(can_transceiver, set, clock);
  try{
  motor.enable();
  motor.mit(150.0f, 0.0f, 6.0f, 1.2f, 0.0f);
  print_data(motor, clock, std::chrono::milliseconds(1500), console);
  hal::delay(*clock, 300ms);

  motor.position_velocity(550, 20);
  print_data(motor, clock, std::chrono::milliseconds(5000), console);
  hal::delay(*clock, 300ms);

  motor.velocity_start(-50);
  print_data(motor, clock, std::chrono::milliseconds(3000), console);
  motor.velocity_stop();
  hal::delay(*clock, 300ms);
  
  motor.force_position_hybrid(0, 30, 0.8);
  print_data(motor, clock, std::chrono::milliseconds(3000), console);

  motor.disable();
  }
  catch(hal::timed_out e){
    hal::print(*console, "Something Timed Out");
  }
}

void print_data(damiao_motor& motor, hal::v5::strong_ptr<hal::steady_clock> clock, hal::time_duration time, hal::v5::optional_ptr<hal::serial> console){ 
    auto const deadline = hal::future_deadline(*clock, time);
    while(clock->uptime() < deadline){
      damiao_motor_data dat = motor.read_encoder();
      hal::print<96>(*console, "pos = %f degrees, vel = %f rpm, torque = %f Nm \n", dat.position, dat.velocity, dat.torque);
    }
}
}  // namespace sjsu::drive