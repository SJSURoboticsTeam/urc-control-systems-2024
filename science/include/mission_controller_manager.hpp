#pragma once
#include <array>
#include <cstddef>
#include <cstdint>
#include <libhal-util/can.hpp>
#include <libhal/can.hpp>
#include <libhal/pointers.hpp>
#include <optional>

#include "science_state_machine.hpp"

namespace sjsu::sceince {

class mission_controller_manager
{
public:
  mission_controller_manager(hal::v5::strong_ptr<hal::can_transceiver> p_can_transceiver);

  bool read_stop_request();

  void reply_heartbeat();

  bool read_step_request();
  void reply_finish_step();

  void reply_soil_amount(float amount);
  
  // void reply_color_sensor();

private:
  hal::v5::strong_ptr<hal::can_transceiver> m_can_transceiver;
  hal::can_message_finder m_stop_finder;
  hal::can_message_finder m_step_finder;
  hal::can_message_finder m_heartbeat_finder;
  hal::can_message_finder m_finish_ack_finder;
  hal::can_message_finder m_soil_amount_ack_finder;
  hal::can_message_finder m_color_sensor_ack_finder;
};

}  // namespace sjsu::sceince
