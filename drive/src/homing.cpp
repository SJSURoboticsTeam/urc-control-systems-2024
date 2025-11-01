#include "../include/homing.hpp"
#include <libhal-util/steady_clock.hpp>

using namespace std::chrono_literals;
using namespace hal::literals;
namespace sjsu::drive {

void home(hal::v5::strong_ptr<std::array<hal::v5::strong_ptr<swerve_module>, 4> > modules,
          hal::v5::strong_ptr<hal::serial> terminal)
{
  for (int i = 0; i < 4; i++) {
    try{
      (*modules)[i]->home();
      hal::print<64>(*terminal, "Homed wheel: %d\n", i);
    }catch(hal::exception e){
      hal::print<64>(*terminal, "Wheel throwing error %d\n", i);
    }

  }
}
}  // namespace sjsu::drive