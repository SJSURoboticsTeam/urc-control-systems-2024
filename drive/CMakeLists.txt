cmake_minimum_required(VERSION 3.25)

# Set project name to demos
project(application LANGUAGES CXX)

libhal_build_demos(

DEMOS
    application
    chassis_vel_demo
    iterate_state
    limit_switch_test
    mc_stability_test
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
    src/drivetrain.cpp
    src/drivetrain_math.cpp
    src/mission_control_manager.cpp
    src/swerve_module.cpp

PACKAGES
#    libhal-__device__
    libhal-actuator

LINK_LIBRARIES
#    libhal::__device__
    libhal::actuator
)
