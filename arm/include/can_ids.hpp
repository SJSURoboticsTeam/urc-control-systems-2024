#include <libhal/units.hpp>

namespace sjsu::arm {
enum arm_addresses : hal::u16
{
  // these commands are coming from MC
  stop = 0x0C,
  heartbeat = 0xE,

  home = 0x111,     // settings address 1rst 2-7 (specify)
  arm_get = 0x112,  // track, shoulder, elbow byte[0] =
                    // position/velocity byte[1-2] = track
                    // byte[3-4]=shoulder
  arm_set = 0x114,
  end_effector_get = 0x115,  // roll, pitch, yaw
                             // MC: change PID parameters,
  end_effector_set = 0x116,
  pid_params = 0x119,  // position

  // these commands = 0x100 are being received
  track = 0x120,
  shoulder = 0x121,
  elbow = 0x122,
  wrist_1 = 0x123,
  wrist_2 = 0x124,
  clamp = 0x125,

};
}  // namespace sjsu::arm
