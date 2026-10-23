#include <damiao_motor.hpp>

namespace sjsu::drivers {

constexpr uint8_t MIT = 1;
constexpr uint8_t POSITION_VELOCITY = 2;
constexpr uint8_t VELOCITY = 3;
constexpr uint8_t POSITION_HYBRID = 4;

constexpr float CONVERT_TO_RADIANS = 0.01745329f;
constexpr float CONVERT_TO_RAD_PER_SECOND = (6.283185f / 60.0f);

damiao_motor::damiao_motor(
  hal::v5::strong_ptr<hal::can_transceiver> p_can_transceiver,
  struct damiao_motor_settings p_set,
  hal::v5::strong_ptr<hal::steady_clock> p_clock,
  hal::time_duration p_max_response_time)
  : m_can_transceiver(p_can_transceiver)
  , m_set(p_set)
  , m_clock(p_clock)
  , m_max_response_time(p_max_response_time)
{
  /// Converting Degrees to Rad
  m_set.maximum_position *= CONVERT_TO_RADIANS;
  m_set.minimum_position *= CONVERT_TO_RADIANS;

  /// Converting RPM to Rad/s
  m_set.minimum_velocity *= CONVERT_TO_RAD_PER_SECOND;
  m_set.maximum_velocity *= CONVERT_TO_RAD_PER_SECOND;

  m_recent_mit_frame.id = m_set.can_id;
  m_recent_mit_frame.length = 8;

  /// Default dummy CAN frame, for encoder readings, unless changed in mit_pd()
  m_recent_mit_frame.payload = {
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00
  };
}

void damiao_motor::enable()
{
  hal::can_message enable_message;
  enable_message.id = m_set.can_id;
  enable_message.length = 8;

  /// Enable Message Bytes
  enable_message.payload = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc };

  send_can_data(enable_message);
}

void damiao_motor::disable()
{
  hal::can_message disable_message;
  disable_message.id = m_set.can_id;
  disable_message.length = 8;

  /// Disable Message Bytes
  disable_message.payload = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfb };

  send_can_data(disable_message);
}

void damiao_motor::mit_pd_mode(mit_pd_mode_parameters p_param)
{
  uint16_t position_bytes, velocity_bytes, proportional_gain_bytes,
    derivative_gain_bytes, torque_feed_forward_bytes;

  /// converting to values the motor can use
  p_param.position *= CONVERT_TO_RADIANS;
  p_param.velocity *= CONVERT_TO_RAD_PER_SECOND;

  /// Converting from float to 16 Bit fixed point signed integer using ranges
  position_bytes = static_cast<uint16_t>(
    std::clamp((p_param.position - m_set.minimum_position) /
                 (m_set.maximum_position - m_set.minimum_position),
               0.0f,
               1.0f) *
    65535.0f);

  /// Converting from float to 12 bit fixed point signed integer using ranges
  velocity_bytes = static_cast<uint16_t>(
    std::clamp(((p_param.velocity - m_set.minimum_velocity) /
                (m_set.maximum_velocity - m_set.minimum_velocity)),
               0.0f,
               1.0f) *
    4095.0f);
  proportional_gain_bytes = static_cast<uint16_t>(
    std::clamp(
      (p_param.proportional_gain - m_set.minimum_proportional_gain) /
        (m_set.maximum_proportional_gain - m_set.minimum_proportional_gain),
      0.0f,
      1.0f) *
    4095.0f);
  derivative_gain_bytes = static_cast<uint16_t>(
    std::clamp(
      (p_param.derivative_gain - m_set.minimum_derivative_gain) /
        (m_set.maximum_derivative_gain - m_set.maximum_derivative_gain),
      0.0f,
      1.0f) *
    4095.0f);
  torque_feed_forward_bytes = static_cast<uint16_t>(
    std::clamp((p_param.torque_feed_forward - m_set.minimum_torque) /
                 (m_set.maximum_torque - m_set.minimum_torque),
               0.0f,
               1.0f) *
    4095.0f);

  /// Setting the payload bytes based on documentation for MIT mode
  m_recent_mit_frame.payload[0] = position_bytes >> 8;
  m_recent_mit_frame.payload[1] = position_bytes & 0xFF;
  m_recent_mit_frame.payload[2] = velocity_bytes >> 4;
  m_recent_mit_frame.payload[3] =
    ((velocity_bytes & 0x0F) << 4) | (proportional_gain_bytes >> 8);
  m_recent_mit_frame.payload[4] = proportional_gain_bytes & 0xFF;
  m_recent_mit_frame.payload[5] = derivative_gain_bytes >> 4;
  m_recent_mit_frame.payload[6] =
    ((derivative_gain_bytes & 0x0F) << 4) | (torque_feed_forward_bytes >> 8);
  m_recent_mit_frame.payload[7] = torque_feed_forward_bytes & 0xFF;

  mode_set(MIT);
  send_can_data(m_recent_mit_frame);
}

void damiao_motor::position_velocity_mode(float p_position, float p_velocity)
{
  uint32_t position_bytes, velocity_bytes;
  hal::can_message set_value;

  p_position = std::clamp(p_position * CONVERT_TO_RADIANS,
                          m_set.minimum_position,
                          m_set.maximum_position);
  p_velocity = std::clamp(p_velocity * CONVERT_TO_RAD_PER_SECOND,
                          m_set.minimum_velocity,
                          m_set.maximum_velocity);

  /// The motor expects IEEE float values for both position and velocity, hence
  /// the values are simply being bit casted here
  position_bytes = std::bit_cast<uint32_t>(p_position);
  velocity_bytes = std::bit_cast<uint32_t>(p_velocity);

  set_value.id = m_set.can_id + 0x100;
  set_value.length = 8;
  for (int i = 0; i < 8; i++) {
    if (i < 4) {
      set_value.payload[i] = position_bytes & 0xff;
      position_bytes >>= 8;
    } else {
      set_value.payload[i] = velocity_bytes & 0xff;
      velocity_bytes >>= 8;
    }
  }

  mode_set(POSITION_VELOCITY);
  send_can_data(set_value);
}

void damiao_motor::velocity_mode_start(float p_velocity)
{
  uint32_t velocity_bytes;
  hal::can_message set_value;

  p_velocity = std::clamp(p_velocity * CONVERT_TO_RAD_PER_SECOND,
                          m_set.minimum_velocity,
                          m_set.maximum_velocity);
  /// The motor expects IEEE float values for velocity, hence why the bit cast
  velocity_bytes = std::bit_cast<uint32_t>(p_velocity);

  set_value.id = m_set.can_id + 0x200;
  set_value.length = 4;
  for (int i = 0; i < 4; i++) {
    set_value.payload[i] = velocity_bytes & 0xff;
    velocity_bytes >>= 8;
  }

  mode_set(VELOCITY);
  send_can_data(set_value);
}

void damiao_motor::velocity_mode_stop()
{
  hal::can_message stop;
  stop.id = m_set.can_id + 0x200;
  stop.length = 4;
  stop.payload = { 0x00, 0x00, 0x00, 0x00 };

  send_can_data(stop);
}

void damiao_motor::force_position_hybrid_mode(
  force_position_hybrid_mode_parameters p_param)
{
  hal::can_message set_value;

  p_param.position = std::clamp(p_param.position * CONVERT_TO_RADIANS,
                                m_set.minimum_position,
                                m_set.maximum_position);
  p_param.velocity =
    std::clamp(p_param.velocity * CONVERT_TO_RAD_PER_SECOND * 100,
               0.0f,
               10000.0f);  // This mode takes in velocity data scaled by 100
  p_param.torque_current_limit = std::clamp(
    p_param.torque_current_limit * 10000,
    0.0f,
    10000.0f);  // This mode takes in torque current limit data scaled by 100

  /// The motor expects IEEE float values for position, hence why the bit cast
  uint32_t position_bytes = std::bit_cast<uint32_t>(p_param.position);
  uint16_t velocity_bytes = static_cast<uint16_t>(p_param.velocity);
  uint16_t torque_current_limit_bytes =
    static_cast<uint16_t>(p_param.torque_current_limit);

  set_value.id = m_set.can_id + 0x300;
  set_value.length = 8;
  for (int i = 0; i < 8; i++) {
    if (i < 4) {
      set_value.payload[i] = position_bytes;
      position_bytes >>= 8;
    } else if (i < 6) {
      set_value.payload[i] = velocity_bytes;
      velocity_bytes >>= 8;
    } else {
      set_value.payload[i] = torque_current_limit_bytes;
      torque_current_limit_bytes >>= 8;
    }
  }

  mode_set(POSITION_HYBRID);
  send_can_data(set_value);
}

damiao_motor_data damiao_motor::read_encoder()
{
  damiao_motor_data data;
  hal::can_message message;

  message = send_can_data(m_recent_mit_frame);

  data.position = 0;
  data.velocity = 0;
  data.torque = 0;

  uint16_t position_bytes = (message.payload[1] << 8) | message.payload[2];
  uint16_t velocity_bytes =
    (message.payload[3] << 4) | (message.payload[4] >> 4);
  uint16_t torque_bytes = (message.payload[4] & 0x0F) << 8 | message.payload[5];

  data.position =
    (((float)position_bytes *
      ((m_set.maximum_position - m_set.minimum_position) / 65535.0f)) +
     m_set.minimum_position) /
    CONVERT_TO_RADIANS;
  data.velocity =
    (((float)velocity_bytes *
      ((m_set.maximum_velocity - m_set.minimum_velocity) / 4095.0f)) +
     m_set.minimum_velocity) /
    CONVERT_TO_RAD_PER_SECOND;
  data.torque = (((float)torque_bytes *
                  ((m_set.maximum_torque - m_set.minimum_torque) / 4095.0f)) +
                 m_set.minimum_torque);

  return data;
}

hal::can_message damiao_motor::send_can_data(
  hal::can_message const& sent_message)
{
  hal::can_message_finder message_finder =
    hal::can_message_finder(*m_can_transceiver, m_set.master_id);

  m_can_transceiver->send(sent_message);

  auto const deadline = hal::future_deadline(*m_clock, m_max_response_time);
  while (m_clock->uptime() < deadline) {
    if (auto const received_message = message_finder.find();
        received_message.has_value()) {
      return received_message.value();
    }
  }
  throw hal::timed_out(this);
}

void damiao_motor::mode_set(uint8_t const mode)
{
  hal::can_message set_mode;

  set_mode.id = 0x7ff;
  set_mode.length = 8;
  set_mode.payload = { 0x00, 0x00, 0x55, 0x0a, 0x00, 0x00, 0x00, 0x00 };
  set_mode.payload[0] = m_set.can_id;
  set_mode.payload[1] = m_set.can_id >> 8;
  set_mode.payload[4] = mode;

  send_can_data(set_mode);
}
}  // namespace sjsu::drivers
