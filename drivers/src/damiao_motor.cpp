#include <damiao_motor.hpp>

#define MIT 1
#define POSITION_VELOCITY 2
#define VELOCITY 3
#define POSITION_HYBRID 4

#define CONVERT_TO_RADIANS 0.01745329f
#define CONVERT_TO_RAD_PER_SECOND (6.283185f / 60.0f)

namespace sjsu::drivers {

damiao_motor::damiao_motor(hal::v5::strong_ptr<hal::can_transceiver> p_can_transceiver,
                           struct damiao_motor_settings p_set,
                           hal::v5::strong_ptr<hal::steady_clock> p_clock,
                           hal::time_duration p_max_response_time)
  : m_can_transceiver(p_can_transceiver)
  , m_set(p_set)
  , m_clock(p_clock)
  , m_max_response_time(p_max_response_time)
{
  m_set.pos_max *= CONVERT_TO_RADIANS;  // Converting Degrees to Rad
  m_set.pos_min *= CONVERT_TO_RADIANS;
  m_set.vel_min *= CONVERT_TO_RAD_PER_SECOND;  // Converting RPM to Rad/s
  m_set.vel_max *= CONVERT_TO_RAD_PER_SECOND;

  m_recent_mit_frame.id = m_set.can_id;
  m_recent_mit_frame.length = 8;
  m_recent_mit_frame.payload = {
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00
  }; //Standard dummy MIT frame to send in read_encoder(), unless changed in mit() mode
}

void damiao_motor::enable()
{
  hal::can_message enable_message;
  enable_message.id = m_set.can_id;
  enable_message.length = 8;
  enable_message.payload = {
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc
  };  // Enable Message Bytes
  try {
    send_can_data(enable_message);
  } catch (hal::timed_out e) {
    throw;
  }
}

void damiao_motor::disable()
{
  hal::can_message disable_message;
  disable_message.id = m_set.can_id;
  disable_message.length = 8;
  disable_message.payload = {
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfb
  };  // Disable Message Bytes
  try {
    send_can_data(disable_message);
  } catch (hal::timed_out e) {
    throw;
  }
}

void damiao_motor::mit(float pos, float vel, float Kp, float Kd, float t_ff)
{
  uint16_t pos_bytes, vel_bytes, Kp_bytes, Kd_bytes, t_ff_bytes;

  pos *= CONVERT_TO_RADIANS;             //converting to values the motor can use
  vel *= CONVERT_TO_RAD_PER_SECOND;

  pos_bytes =
    static_cast<uint16_t>(std::clamp((pos - m_set.pos_min) / (m_set.pos_max - m_set.pos_min), 0.0f, 1.0f) *
                          65535.0f);  // Converting from float to 16 Bit Fixed
                                      // point Signed Integer Using Ranges
  vel_bytes =
    static_cast<uint16_t>(std::clamp(((vel - m_set.vel_min) / (m_set.vel_max - m_set.vel_min)), 0.0f, 1.0f) *
                          4095.0f);  // Converting from float to 12 Bit Fixed
                                     // point Signed Integer Using Ranges
  Kp_bytes = static_cast<uint16_t>
    (std::clamp((Kp - m_set.Kp_min) / (m_set.Kp_max - m_set.Kp_min), 0.0f, 1.0f) * 4095.0f);
  Kd_bytes = static_cast<uint16_t>(
    std::clamp((Kd - m_set.Kd_min) / (m_set.Kd_max - m_set.Kd_min), 0.0f, 1.0f) * 4095.0f);
  t_ff_bytes = static_cast<uint16_t>(
    std::clamp((t_ff - m_set.torque_min) / (m_set.torque_max - m_set.torque_min), 0.0f, 1.0f) * 4095.0f);

  m_recent_mit_frame.payload[0] =
    pos_bytes >>
    8;  // Setting the payload bytes based on documentation for MIT mode
  m_recent_mit_frame.payload[1] = pos_bytes & 0xFF;
  m_recent_mit_frame.payload[2] = vel_bytes >> 4;
  m_recent_mit_frame.payload[3] = ((vel_bytes & 0x0F) << 4) | (Kp_bytes >> 8);
  m_recent_mit_frame.payload[4] = Kp_bytes & 0xFF;
  m_recent_mit_frame.payload[5] = Kd_bytes >> 4;
  m_recent_mit_frame.payload[6] = ((Kd_bytes & 0x0F) << 4) | (t_ff_bytes >> 8);
  m_recent_mit_frame.payload[7] = t_ff_bytes & 0xFF;

  try {
    mode_set(MIT);
    send_can_data(m_recent_mit_frame);
  } catch (hal::timed_out e) {
    throw;
  }
}

void damiao_motor::position_velocity(float pos, float vel)
{
  uint32_t pos_bytes, vel_bytes;
  hal::can_message set_val;

  pos = std::clamp(pos * CONVERT_TO_RADIANS, m_set.pos_min, m_set.pos_max);
  vel = std::clamp(vel * CONVERT_TO_RAD_PER_SECOND, m_set.vel_min, m_set.vel_max);

  pos_bytes = std::bit_cast<uint32_t>(pos);
  vel_bytes = std::bit_cast<uint32_t>(vel);

  set_val.id = m_set.can_id + 0x100;
  set_val.length = 8;
  for (int i = 0; i < 8; i++) {
    if (i < 4) {
      set_val.payload[i] = pos_bytes & 0xff;
      pos_bytes >>= 8;
    } else {
      set_val.payload[i] = vel_bytes & 0xff;
      vel_bytes >>= 8;
    }
  }
  try {
    mode_set(POSITION_VELOCITY);
    send_can_data(set_val);
  } catch (hal::timed_out e) {
    throw;
  }
}

void damiao_motor::velocity_start(float vel)
{
  uint32_t vel_bytes;
  hal::can_message set_val;

  vel = std::clamp(vel * CONVERT_TO_RAD_PER_SECOND, m_set.vel_min, m_set.vel_max);
  vel_bytes = std::bit_cast<uint32_t>(vel);

  set_val.id = m_set.can_id + 0x200;
  set_val.length = 4;
  for (int i = 0; i < 4; i++) {
    set_val.payload[i] = vel_bytes & 0xff;
    vel_bytes >>= 8;
  }

  try {
    mode_set(VELOCITY);
    send_can_data(set_val);
  } catch (hal::timed_out e) {
    throw;
  }
}

void damiao_motor::velocity_stop()
{
  hal::can_message stop;
  stop.id = m_set.can_id + 0x200;
  stop.length = 4;
  stop.payload = { 0x00, 0x00, 0x00, 0x00 };
  try {
    mode_set(VELOCITY);
    send_can_data(stop);
  } catch (hal::timed_out e) {
    throw;
  }
}

void damiao_motor::force_position_hybrid(float pos,
                                   float vel,
                                   float torque_current_limit)
{
  hal::can_message set_val;

  pos = std::clamp(pos * CONVERT_TO_RADIANS, m_set.pos_min, m_set.pos_max);
  vel = std::clamp(vel * CONVERT_TO_RAD_PER_SECOND * 100, 0.0f, 10000.0f);  // This mode takes in velocity data scaled by 100
  torque_current_limit = std::clamp(torque_current_limit * 10000, 0.0f, 10000.0f);

  uint32_t pos_bytes = std::bit_cast<uint32_t>(pos);
  uint16_t vel_bytes = static_cast<uint16_t>(vel);
  uint16_t torque_current_limit_bytes = static_cast<uint16_t>(torque_current_limit);

  set_val.id = m_set.can_id + 0x300;
  set_val.length = 8;
  for (int i = 0; i < 8; i++) {
    if (i < 4) {
      set_val.payload[i] = pos_bytes;
      pos_bytes >>= 8;
    } else if (i < 6) {
      set_val.payload[i] = vel_bytes;
      vel_bytes >>= 8;
    } else {
      set_val.payload[i] = torque_current_limit_bytes;
      torque_current_limit_bytes >>= 8;
    }
  }

  try {
    mode_set(POSITION_HYBRID);
    send_can_data(set_val);
    
  } catch (hal::timed_out e) {
    throw;
  }
}

damiao_motor_data damiao_motor::read_encoder()
{
  damiao_motor_data dat;
  hal::can_message msg;
  try {
    msg = send_can_data(m_recent_mit_frame);
  } catch (hal::timed_out e) {
    throw;
  }

  dat.position = 0;
  dat.velocity = 0;
  dat.torque = 0;
  
  uint16_t pos_bytes = (msg.payload[1] << 8) | msg.payload[2];
  uint16_t vel_bytes = (msg.payload[3] << 4) | (msg.payload[4] >> 4);
  uint16_t torque_bytes = (msg.payload[4] & 0x0F) << 8 | msg.payload[5];

  dat.position =
    (((float)pos_bytes * ((m_set.pos_max - m_set.pos_min) / 65535.0f)) +
     m_set.pos_min) /
    CONVERT_TO_RADIANS;
  dat.velocity = (((float)vel_bytes * ((m_set.vel_max - m_set.vel_min) / 4095.0f)) +
                  m_set.vel_min) /
                  CONVERT_TO_RAD_PER_SECOND;                                             
  dat.torque =
    (((float)torque_bytes * ((m_set.torque_max - m_set.torque_min) / 4095.0f)) +
     m_set.torque_min);
  return dat;
}

hal::can_message damiao_motor::send_can_data(hal::can_message const& sent_message)
{
  hal::can_message_finder message_finder =
    hal::can_message_finder(*m_can_transceiver, m_set.master_id);
  m_can_transceiver->send(sent_message);
  auto const deadline = hal::future_deadline(*m_clock, m_max_response_time);
  while (m_clock->uptime() < deadline) {
    if (auto const received_message = message_finder.find(); received_message.has_value()) {
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
  try {
    send_can_data(set_mode);
  } catch (hal::timed_out e) {
    throw;
  }
}
}  // namespace sjsu::drive