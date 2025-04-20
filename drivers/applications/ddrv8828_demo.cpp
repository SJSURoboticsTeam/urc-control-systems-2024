// #include <libhal-armcortex/dwt_counter.hpp>
// #include <libhal-armcortex/startup.hpp>
// #include <libhal-armcortex/system_control.hpp>
// #include <libhal-util/serial.hpp>
// #include <libhal-util/steady_clock.hpp>
// #include <libhal/units.hpp>

// #include "../hardware_map.hpp"
// #include <drv8825.hpp>

// using namespace hal::literals;
// using namespace std::chrono_literals;
// namespace sjsu::drivers {

// void application(application_framework& p_framework)
// {
//   // configure drivers
//   //   auto& i2c2 = *p_framework.i2c;
//   auto& step_pin = *p_framework.out_pin0;
//   auto& dir_pin = *p_framework.out_pin1;
//   auto& clock = *p_framework.steady_clock;
//   auto& terminal = *p_framework.terminal;

//   auto drv8825_driver = arm::drv8825::create(
//     dir_pin, step_pin, clock, arm::drv8825::step_factor::one, 2024);

//   // hal::print<64>(terminal, "Hello World!");

//   while (true) {

//     drv8825_driver.step(100000);
//     hal::delay(clock, 500ms)
//   }
// }
// }  // namespace sjsu::drivers