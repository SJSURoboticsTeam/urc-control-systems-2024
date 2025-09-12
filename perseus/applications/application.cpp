//copied from drivers/applications/h_bridge_demo.cpp

#include "../hardware_map.hpp"
#include <h_bridge.hpp>
#include <libhal-util/steady_clock.hpp>

using namespace std::chrono_literals;
namespace sjsu::perseus {


void application()
{
  // perseus shoulder_motor(apram1, param2);
  // institialize can, hbridge etc.

  

  // shoulder_motor.set_position(50.0_degrees);

  // on_can_receive(0x)
    //-> perform set_position task
}
}  // namespace sjsu::drivers