#include <libhal-util/can.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/can.hpp>
#include <libhal/pointers.hpp>

#include <bldc_servo_settings.hpp>
#include <can_util.hpp>
#include <resource_list.hpp>

using namespace std::chrono_literals;
namespace sjsu::perseus {

void application()
{
  using namespace hal::literals;
  using namespace std::chrono_literals;

  // general
  auto clock = resources::clock();
  auto console = resources::console();

  // variables used to initialize can_perseus
  can_perseus::servo_address allowed_id;
  hal::u16 listen_prev = 0x00;
  auto switches_ptr = resources::switches();
  allowed_id = static_cast<can_perseus::servo_address>(
    static_cast<hal::u16>(switches_ptr->read_switch_value()) + 0x120);
  hal::u16 group_id = 0x120;

  // create can_perseus
  auto can_transceiver = resources::can_transceiver();
  auto can_bus_manager = resources::can_bus_manager();
  auto can_id_filter = resources::get_new_can_filter();
  auto can_group_filter = resources::get_new_can_filter();
  can_perseus servo_can(allowed_id,
                        listen_prev,
                        group_id,
                        1_MHz,
                        can_transceiver,
                        can_bus_manager,
                        can_id_filter,
                        can_group_filter);

  hal::print(*console, "Begin.\n");

  // start loop
  while (true) {
    // receive message
    std::optional<hal::can_message> msg =
      servo_can.check_for_individual_message();
    // react to message
    if (msg) {
      drivers::can_util::print_can_message(*console, *msg);
    }

    // check group message message
    msg = servo_can.check_for_group_message();
    // react to message
    if (msg) {
      drivers::can_util::print_can_message(*console, *msg);
    }
  }
}
}  // namespace sjsu::perseus
