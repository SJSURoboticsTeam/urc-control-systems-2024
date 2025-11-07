#include <libhal/can.hpp>
#include "../hardware_map.hpp"
namespace sjsu::arm {
void process_message(hal::can_message const& p_message,
                     resources::arm_joints& arm_servos,
                     hal::serial& console);
}