cmake_minimum_required(VERSION 3.25)

# Set project name to demos
project(application LANGUAGES CXX)

libhal_build_demos(

DEMOS
    application
    
INCLUDES
    include
    .
SOURCES
    ./src/drivetrain_math.cpp
    ./src/drivetrain.cpp
    ./src/homing.cpp
    ./src/swerve_module.cpp

PACKAGES
#    libhal-__device__
    libhal-actuator

LINK_LIBRARIES
#    libhal::__device__
    libhal::actuator
)