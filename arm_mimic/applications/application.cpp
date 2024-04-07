#include <array>
#include <cstdio>

#include <libhal-armcortex/dwt_counter.hpp>
#include <libhal-lpc40/clock.hpp>
#include <libhal-lpc40/constants.hpp>
#include <libhal-lpc40/i2c.hpp>
#include <libhal-lpc40/uart.hpp>
#include <libhal-util/i2c.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>

#include "../implementations/degree_manipulation.hpp"
#include "../implementations/mission_control_handler.hpp"
#include "../implementations/tla2528.hpp"

#include "application.hpp"
namespace sjsu::arm_mimic {

void application(application_framework& p_framework)
{
  try {
    using namespace hal::literals;

    auto& steady_clock = *p_framework.steady_clock;
    auto& uart0 = *p_framework.terminal;
    auto& i2c2 = *p_framework.i2c2;
    tla::tla2528 TLA2528(i2c2, steady_clock);

    const uint8_t N = 6;
    std::array<float, N> scrambled_results = {};
    std::array<float, N> results = {};
    float true_degree;
    float max_volt = 4096;
    float max_deg = 360;

    while (true) {
      // auto results = HAL_CHECK(TLA2528.read_one(channel));
      // hal::print<64>(uart0, "voltage: %d\n", results);

      auto buffer_of_results = TLA2528.read_all();
      for (int i = 0; i < N; i++) {
        true_degree =
          tla::voltage_to_degree(buffer_of_results[i], max_volt, max_deg);
        scrambled_results[i] = true_degree;
        hal::print<64>(uart0, "voltage %d: %d \n", i, (int)true_degree);
      }
      // hal::print<64>(uart0, "\n");

      results[0] = 0;                              // ROTUNDA
      results[1] = scrambled_results[2] - 261;     // SHOULDER
      results[2] = -(scrambled_results[1] - 151);  // ELBOW
      // results[3] = scrambled_results[0];  // WRIST PITCH
      // results[4] = scrambled_results[5];  // WRIST ROLL
      results[3] = 0;
      results[4] = 0;
      results[5] = 0;

      //   tla::send_data_to_mc(*p_framework.terminal, results);
    }
  } catch (...) {
    
    hal::halt();
  }
}
}  // namespace sjsu::arm_mimic