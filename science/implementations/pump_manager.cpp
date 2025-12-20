#include "../include/pump_manager.hpp"

namespace sjsu::science {

pump_manager::pump_manager(
  hal::v5::strong_ptr<hal::steady_clock> p_clock,
  hal::v5::strong_ptr<hal::output_pin> p_deionized_water_pump,
  hal::v5::strong_ptr<hal::output_pin> p_benedict_reagent_pump,
  hal::v5::strong_ptr<hal::output_pin> p_biuret_reagent,
  hal::v5::strong_ptr<hal::output_pin> p_kalling_reagent_pump)
  : m_clock(p_clock)
  , m_pumps{ p_deionized_water_pump,
             p_benedict_reagent_pump,
             p_biuret_reagent,
             p_kalling_reagent_pump} {};

void pump_manager::pump(pumps pump, hal::time_duration duration)
{
  (m_pumps[static_cast<int>(pump)])->level(true);
  hal::delay(*m_clock, duration);
  (m_pumps[static_cast<int>(pump)])->level(false);
}

}  // namespace sjsu::science