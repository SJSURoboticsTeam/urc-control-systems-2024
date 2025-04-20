
cmake_minimum_required(VERSION 3.20)

project(application LANGUAGES CXX)

libhal_build_demos(
    DEMOS
    application

    PACKAGES
    libhal-util
    libhal-actuator
    # libhal-pca

    LINK_LIBRARIES
    libhal::util
    libhal::actuator
    # libhal::pca


    INCLUDES
    ./include

    SOURCES
    implementations/ackermann_steering.cpp
    implementations/drive_configuration_updater.cpp
    implementations/wheel_router.cpp
    implementations/homing.cpp
) 
