cmake_minimum_required(VERSION 3.25)

# Set project name to demos
project(application LANGUAGES CXX)

libhal_build_demos(

DEMOS
    application
    # can_stability_test
    chassis_vel_demo
    iterate_state
    # poll_motor_connections
    prop_motor_test
    sequential_homing_demo
    serial_command_demo
    simple_can_reply_demo
    simple_state_set_demo
INCLUDES
    .
    ../drivers/include
    include
SOURCES
    ../drivers/src/serial_commands.cpp
    ../drivers/src/perseus_bldc.cpp
    ../drivers/src/velocity_motor_mock.cpp
    ../drivers/src/velocity_servo_mock.cpp
    src/drivetrain.cpp
    src/drivetrain_math.cpp
    src/mission_control_manager.cpp
    src/swerve_module.cpp
    src/propulsion_controller_mock.cpp
    src/propulsion_controller_rmd_x7.cpp
    src/steer_controller_mock.cpp
    src/steer_controller_perseus.cpp

PACKAGES
#    libhal-__device__
    libhal-actuator

LINK_LIBRARIES
#    libhal::__device__
    libhal::actuator
)
