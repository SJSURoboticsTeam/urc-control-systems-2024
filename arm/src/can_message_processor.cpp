#include "can_message_processor.hpp"
#include "can_ids.hpp"
#include <libhal-util/serial.hpp>
namespace sjsu::arm {
void process_message(hal::can_message const& p_message,
                     resources::arm_joints& arm_servos,
                     hal::serial& console)
{
  switch (p_message.id) {
    case arm_addresses::arm_set: {
      hal::print(console, "Processing arm_set message\n");
      if (p_message.payload[0] == 0x00) {
        // position
        hal::u16 track_pos = p_message.payload[2] << 8 | p_message.payload[3];
        arm_servos[0]->set_position(track_pos);
        hal::u16 shoulder_pos =
          p_message.payload[4] << 8 | p_message.payload[5];
        arm_servos[1]->set_position(shoulder_pos);
        hal::u16 elbow_pos = p_message.payload[6] << 8 | p_message.payload[7];
        arm_servos[2]->set_position(elbow_pos);
      } else if (p_message.payload[0] == 0x01) {
        // velocity
        hal::u8 exp_bits = p_message.payload[1];
        hal::i16 track_vel = p_message.payload[2] << 8 | p_message.payload[3];
        arm_servos[0]->set_velocity(track_vel, exp_bits);
        hal::i16 shoulder_vel =
          p_message.payload[4] << 8 | p_message.payload[5];
        arm_servos[1]->set_velocity(shoulder_vel, exp_bits);
        hal::i16 elbow_vel = p_message.payload[6] << 8 | p_message.payload[7];
        arm_servos[2]->set_velocity(elbow_vel, exp_bits);
      }
      break;
    case arm_addresses::end_effector_set: {
      hal::print(console, "Processing end_effector_set message\n");
      break;
    }
    case arm_addresses::stop: {
      hal::print(console, "Processing stop message\n");

      break;
    }
    case arm_addresses::heartbeat: {
      hal::print(console, "Processing heartbeat message\n");
      break;
    }
    case arm_addresses::pid_params: {
      hal::print(console, "Processing pid_params message\n");
      break;
    }
    default: {
      hal::print<128>(console, "Received unknown message ID: ", p_message.id, "\n");
      break;
    }
}
  }
}
}// namespace sjsu::arm