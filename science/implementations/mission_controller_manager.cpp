#include "../include/mission_controller_manager.hpp"

namespace sjsu::sceince {

enum class can_message_id : uint32_t
{
  stop = 0x00,
  heartbeat = 0x32,
  heartbeat_reply = 0x33,
  step = 0x34,
  finish_step_ack = 0x35,
  color_senor = 0x36,
  color_senor_ack = 0x37,
  soil_amount = 0x36,
  soil_amount_ack = 0x37
};

mission_controller_manager::mission_controller_manager(
  hal::v5::strong_ptr<hal::can_transceiver> p_can_transceiver)
  : m_can_transceiver(p_can_transceiver)
  , m_stop_finder(
      hal::can_message_finder(*m_can_transceiver,
                              static_cast<uint32_t>(can_message_id::stop)))
  , m_step_finder(
      hal::can_message_finder(*m_can_transceiver,
                              static_cast<uint32_t>(can_message_id::step)))
  , m_heartbeat_finder(
      hal::can_message_finder(*m_can_transceiver,
                              static_cast<uint32_t>(can_message_id::heartbeat)))
  , m_finish_ack_finder(hal::can_message_finder(
      *m_can_transceiver,
      static_cast<uint32_t>(can_message_id::finish_step_ack)))
  , m_soil_amount_ack_finder(hal::can_message_finder(
      *m_can_transceiver,
      static_cast<uint32_t>(can_message_id::soil_amount_ack)))
  , m_color_sensor_ack_finder(hal::can_message_finder(
      *m_can_transceiver,
      static_cast<uint32_t>(can_message_id::color_senor_ack)))
{
}

bool mission_controller_manager::read_stop_request()
{
  while (true) {
    if (m_stop_finder.find()) {
      return true;
    }
  }
  return false;
}

void mission_controller_manager::reply_heartbeat()
{
  if (m_heartbeat_finder.find()) {
    m_can_transceiver->send(
      { .id = static_cast<uint32_t>(can_message_id::heartbeat_reply),
        .length = 0 });
  }
}

bool mission_controller_manager::read_step_request()
{
  while (true) {
    if (m_step_finder.find()) {
      return true;
    }
  }
  return false;
}

void mission_controller_manager::reply_finish_step()
{
  while (!m_finish_ack_finder.find()) {
    m_can_transceiver->send(
      { .id = static_cast<uint32_t>(can_message_id::finish_step_ack),
        .length = 0 });
  }
}

void mission_controller_manager::reply_soil_amount([[maybe_unused]] float amount)
{
  while (!m_soil_amount_ack_finder.find()) {
    m_can_transceiver->send(
      { .id = static_cast<uint32_t>(can_message_id::soil_amount),
        .length = 0 });
  }
}

}  // namespace sjsu::sceince
