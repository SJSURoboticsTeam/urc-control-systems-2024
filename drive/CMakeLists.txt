cmake_minimum_required(VERSION 3.25)

# Set project name to demos
project(application LANGUAGES CXX)

libhal_build_demos(

DEMOS
    application
INCLUDES
    include
SOURCE
    src/drivetrain_math.cpp
    src/drivetrain.cpp

PACKAGES
#    libhal-__device__
libhal-actuator

    LINK_LIBRARIES
#    libhal::__device__
libhal::actuator
)