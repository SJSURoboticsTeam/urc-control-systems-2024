#include <damiao_motor.hpp>

namespace {

constexpr float degrees_to_radians = (std::numbers::pi / 180.0f);
constexpr float radians_to_degrees = (180.0f / std::numbers::pi);

constexpr float rpm_to_rad_per_second = ((2 * std::numbers::pi) / 60.0f);
constexpr float rad_per_second_to_rpm = (60.0f / (2 * std::numbers::pi));

}  // namespace

namespace sjsu::drivers {

damiao_motor::damiao_motor(
  hal::v5::strong_ptr<hal::can_transceiver> p_can_transceiver,
  damiao_motor_settings& p_settings,
  hal::v5::strong_ptr<hal::steady_clock> p_clock,
  hal::time_duration p_max_response_time)
  : m_can_transceiver(p_can_transceiver)
  , m_message_finder(
      hal::can_message_finder(*p_can_transceiver, p_settings.master_id))
  , m_settings(p_settings)
  , m_clock(p_clock)
  , m_max_response_time(p_max_response_time)
{
  m_recent_mit_frame.id = m_settings.can_id;
}

void damiao_motor::enable(bool p_state)
{
  hal::can_message state_message{
    .id = m_settings.can_id,
    .length = 8,
    /// Enable Message Bytes
    .payload = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfb }
  };
  /// payload for enable message is 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
  /// 0xfc
  if (p_state)
    state_message.payload[7]++;

  send_can_data(state_message);
}

void damiao_motor::mit_pd_mode(mit_pd_mode_parameters& p_param)
{
  /// enable motor at the start if the user doesn't
  enable(true);

  /// converting to values the motor can use
  float converted_position = p_param.position * degrees_to_radians;
  float converted_minimum_position =
    m_settings.minimum_position * degrees_to_radians;
  float converted_maximum_position =
    m_settings.maximum_position * degrees_to_radians;

  float converted_velocity = p_param.velocity * rpm_to_rad_per_second;
  float converted_minimum_velocity =
    m_settings.minimum_velocity * rpm_to_rad_per_second;
  float converted_maximum_velocity =
    m_settings.maximum_velocity * rpm_to_rad_per_second;

  /// Converting from float to 16 Bit fixed point signed integer using ranges
  uint16_t position_bytes = float_to_fixed_point(converted_position,
                                                 converted_minimum_position,
                                                 converted_maximum_position,
                                                 16);

  /// Converting from float to 12 bit fixed point signed integer using ranges
  uint16_t velocity_bytes = float_to_fixed_point(converted_velocity,
                                                 converted_minimum_velocity,
                                                 converted_maximum_velocity,
                                                 12);

  uint16_t proportional_gain_bytes =
    float_to_fixed_point(p_param.proportional_gain,
                         m_settings.minimum_proportional_gain,
                         m_settings.maximum_proportional_gain,
                         12);

  uint16_t derivative_gain_bytes =
    float_to_fixed_point(p_param.derivative_gain,
                         m_settings.minimum_derivative_gain,
                         m_settings.maximum_derivative_gain,
                         12);

  uint16_t torque_feed_forward_bytes =
    float_to_fixed_point(p_param.torque_feed_forward,
                         m_settings.minimum_torque,
                         m_settings.maximum_torque,
                         12);

  /// Setting the payload bytes based on documentation for MIT mode
  m_recent_mit_frame.payload = {
    static_cast<hal::byte>(position_bytes >> 8),
    static_cast<hal::byte>(position_bytes),
    static_cast<hal::byte>(velocity_bytes >> 4),
    static_cast<hal::byte>(((velocity_bytes & 0x0F) << 4) |
                           (proportional_gain_bytes >> 8)),
    static_cast<hal::byte>(proportional_gain_bytes),
    static_cast<hal::byte>(derivative_gain_bytes >> 4),
    static_cast<hal::byte>(((derivative_gain_bytes & 0x0F) << 4) |
                           (torque_feed_forward_bytes >> 8)),
    static_cast<hal::byte>(torque_feed_forward_bytes)
  };

  mode_set(operating_mode::mit);
  send_can_data(m_recent_mit_frame);
}

void damiao_motor::position_velocity_mode(hal::degrees p_position,
                                          hal::rpm p_velocity)
{
  /// enable motor at the start if the user doesn't
  enable(true);

  /// converting to values the motor can use
  float converted_position = p_position * degrees_to_radians;
  float converted_minimum_position =
    m_settings.minimum_position * degrees_to_radians;
  float converted_maximum_position =
    m_settings.maximum_position * degrees_to_radians;

  float converted_velocity = p_velocity * rpm_to_rad_per_second;
  float converted_minimum_velocity =
    m_settings.minimum_velocity * rpm_to_rad_per_second;
  float converted_maximum_velocity =
    m_settings.maximum_velocity * rpm_to_rad_per_second;

  p_position = std::clamp(
    converted_position, converted_minimum_position, converted_maximum_position);
  p_velocity = std::clamp(
    converted_velocity, converted_minimum_velocity, converted_maximum_velocity);

  /// The motor expects IEEE float values for both position and velocity, hence
  /// the values are simply being bit casted here
  uint32_t position_bytes = std::bit_cast<uint32_t>(p_position);
  uint32_t velocity_bytes = std::bit_cast<uint32_t>(p_velocity);

  hal::can_message set_value{ .id = m_settings.can_id + 0x100u,
                              .length = 8,
                              .payload = {
                                static_cast<hal::byte>(position_bytes),
                                static_cast<hal::byte>(position_bytes >> 8),
                                static_cast<hal::byte>(position_bytes >> 16),
                                static_cast<hal::u8>(position_bytes >> 24),
                                static_cast<hal::u8>(velocity_bytes),
                                static_cast<hal::u8>(velocity_bytes >> 8),
                                static_cast<hal::u8>(velocity_bytes >> 16),
                                static_cast<hal::u8>(velocity_bytes >> 24) } };

  mode_set(operating_mode::position_velocity);
  send_can_data(set_value);
}

void damiao_motor::velocity_mode(hal::rpm p_velocity)
{
  /// enable motor at the start if the user doesn't
  enable(true);

  float converted_velocity = p_velocity * rpm_to_rad_per_second;
  float converted_minimum_velocity =
    m_settings.minimum_velocity * rpm_to_rad_per_second;
  float converted_maximum_velocity =
    m_settings.maximum_velocity * rpm_to_rad_per_second;

  p_velocity = std::clamp(
    converted_velocity, converted_minimum_velocity, converted_maximum_velocity);

  /// The motor expects IEEE float values for velocity, hence why the bit cast
  uint32_t velocity_bytes = std::bit_cast<uint32_t>(p_velocity);

  hal::can_message set_value{
    .id = m_settings.can_id + 0x200u,
    .length = 4,
    .payload = { static_cast<hal::byte>(velocity_bytes),
                 static_cast<hal::byte>(velocity_bytes >> 8),
                 static_cast<hal::byte>(velocity_bytes >> 16),
                 static_cast<hal::byte>(velocity_bytes >> 24) }
  };

  mode_set(operating_mode::velocity);
  send_can_data(set_value);
}

void damiao_motor::force_position_hybrid_mode(
  force_position_hybrid_mode_parameters& p_param)
{
  /// enable motor at the start if the user doesn't
  enable(true);

  float converted_position = p_param.position * degrees_to_radians;
  float converted_minimum_position =
    m_settings.minimum_position * degrees_to_radians;
  float converted_maximum_position =
    m_settings.maximum_position * degrees_to_radians;

  float converted_velocity = p_param.velocity * rpm_to_rad_per_second;

  p_param.position = std::clamp(
    converted_position, converted_minimum_position, converted_maximum_position);
  p_param.velocity = std::clamp(
    converted_velocity * 100,
    0.0f,
    10000.0f);  // This mode takes in velocity data scaled by 100, 0 to 100
                // rad/s is the range of velocity the motor accepts
  p_param.torque_current_limit = std::clamp(
    p_param.torque_current_limit * 10000,
    0.0f,
    10000.0f);  // This mode takes in torque current limit data scaled by 100
  // 0 to 10000 is the range the motor takes

  /// The motor expects IEEE float values for position, hence why the bit cast
  uint32_t position_bytes = std::bit_cast<uint32_t>(p_param.position);
  uint16_t velocity_bytes = static_cast<uint16_t>(p_param.velocity);
  uint16_t torque_current_limit_bytes =
    static_cast<uint16_t>(p_param.torque_current_limit);

  hal::can_message set_value{
    .id = m_settings.can_id + 0x300u,
    .length = 8,
    .payload = { static_cast<hal::byte>(position_bytes),
                 static_cast<hal::byte>(position_bytes >> 8),
                 static_cast<hal::byte>(position_bytes >> 16),
                 static_cast<hal::byte>(position_bytes >> 24),
                 static_cast<hal::byte>(velocity_bytes),
                 static_cast<hal::byte>(velocity_bytes >> 8),
                 static_cast<hal::byte>(torque_current_limit_bytes >> 16),
                 static_cast<hal::byte>(torque_current_limit_bytes >> 8) }
  };

  mode_set(operating_mode::force_position_hybrid);
  send_can_data(set_value);
}

damiao_motor_data damiao_motor::read_encoder()
{
  /**
   * If read_encoder() is called before mit_pd(), m_recent_mit_frame
   * contains dummy payload bytes initialized at the start. This is NOT a bug.
   * The dummy frame will not be able to do anything to the motor unless theres
   * another CAN message sent to the motor explicitly switching the mode to MIT
   * mode. That CAN message is ONLY sent in mit_pd(), which also changes the
   * dummy frame to an actual MIT frame.
   *
   * If the user calls any other mode after mit_pd(), a CAN message is sent
   * changing the mode of the motor, and thus the mit frame sent can no longer
   * affect the state of the motor, and you can still get readings out of the
   * motor.
   *
   */
  hal::can_message message = send_can_data(m_recent_mit_frame);

  uint16_t position_bytes = (message.payload[1] << 8) | message.payload[2];
  uint16_t velocity_bytes =
    (message.payload[3] << 4) | (message.payload[4] >> 4);
  uint16_t torque_bytes = (message.payload[4] & 0x0F) << 8 | message.payload[5];

  float converted_minimum_position =
    m_settings.minimum_position * degrees_to_radians;
  float converted_maximum_position =
    m_settings.maximum_position * degrees_to_radians;

  float converted_minimum_velocity =
    m_settings.minimum_velocity * rpm_to_rad_per_second;
  float converted_maximum_velocity =
    m_settings.maximum_velocity * rpm_to_rad_per_second;

  damiao_motor_data data{
    .position = (fixed_point_to_float(position_bytes,
                                      converted_minimum_position,
                                      converted_maximum_position,
                                      16) *
                 radians_to_degrees),
    .velocity = (fixed_point_to_float(velocity_bytes,
                                      converted_minimum_velocity,
                                      converted_maximum_velocity,
                                      12) *
                 rad_per_second_to_rpm),
    .torque = fixed_point_to_float(
      torque_bytes, m_settings.minimum_torque, m_settings.maximum_torque, 12)
  };

  return data;
}

hal::can_message damiao_motor::send_can_data(
  hal::can_message const& p_sent_message)
{
  /// Flush the buffer, which might hold unrelated messages before the
  /// transmission even starts
  while (m_message_finder.find().has_value())
    ;
  m_can_transceiver->send(p_sent_message);

  auto const deadline = hal::future_deadline(*m_clock, m_max_response_time);
  while (m_clock->uptime() < deadline) {
    if (auto const received_message = m_message_finder.find();
        (received_message.has_value() &&
         ((received_message->payload[0] & 0x0F) ==
          (m_settings.can_id & 0x0F)))) {
      return received_message.value();
    }
  }
  throw hal::timed_out(this);
}

void damiao_motor::mode_set(operating_mode p_mode)
{
  hal::can_message set_mode{ .id = 0x7ffu,
                             .length = 8,
                             .payload = {
                               static_cast<uint8_t>(m_settings.can_id),
                               static_cast<uint8_t>(m_settings.can_id >> 8),
                               0x55,
                               0x0a,
                               static_cast<uint8_t>(p_mode),
                               0x00,
                               0x00,
                               0x00 } };
  send_can_data(set_mode);
}

uint16_t damiao_motor::float_to_fixed_point(float p_value,
                                            float p_minimum,
                                            float p_maximum,
                                            uint8_t p_bit_width)
{
  float maximum_limit = static_cast<float>((1u << p_bit_width) - 1);
  float clamped_value =
    std::clamp(((p_value - p_minimum) / (p_maximum - p_minimum)), 0.0f, 1.0f);
  return static_cast<uint16_t>(clamped_value * maximum_limit);
}

float damiao_motor::fixed_point_to_float(uint16_t p_value,
                                         float p_minimum,
                                         float p_maximum,
                                         uint8_t p_bit_width)
{
  float maximum_limit = static_cast<float>(((1u << p_bit_width) - 1));
  float normalization_ratio = ((p_maximum - p_minimum)) / maximum_limit;
  return (p_minimum + (static_cast<float>(p_value) * normalization_ratio));
}
}  // namespace sjsu::drivers
