cmake_minimum_required(VERSION 3.25)

# Set project name to demos
project(application LANGUAGES CXX)

libhal_build_demos(

DEMOS
    applications
    
INCLUDES
    include
    ../drivers/include
SOURCES
    ./src/implementation.cpp
    ../drivers/src/soil_sensor_sht21.cpp
    ../drivers/src/drv8825.cpp

PACKAGES
#    libhal-__device__
    libhal-actuator

LINK_LIBRARIES
#   libhal::__device__
    libhal::actuator
)