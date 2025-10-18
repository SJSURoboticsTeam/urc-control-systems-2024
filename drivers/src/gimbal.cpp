#include <gimbal.hpp>
#include <libhal-expander/pca9685.hpp>
#include <libhal-sensor/imu/mpu6050.hpp>
#include <libhal/servo.hpp>

using namespace hal::literals;
using namespace std::chrono_literals;

namespace sjsu::drivers {
    gimbal::gimbal(hal::v5::strong_ptr<hal::i2c> p_i2c,
    hal::v5::strong_ptr<hal::steady_clock> p_clock,
    hal::v5::strong_ptr<hal::serial> p_terminal)
    :m_i2c(p_i2c)
    , m_clock(p_clock)
    , m_terminal(p_terminal)
    {

    };

    void set_filter_tau(float initTau);
    void update_gimbal();
    void set_target_axis(float initTargetPitch,
                       float initTargetRoll,
                       float initTargetYaw);
    void set_pid_pitch(float kp, float ki, float kd);
    void calculate_rotation_axis(float initDt);
    void calculate_control_var(float initDt);


}