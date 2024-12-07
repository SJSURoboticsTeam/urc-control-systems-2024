
cmake_minimum_required(VERSION 3.20)

project(application LANGUAGES CXX)

libhal_build_demos(
    DEMOS
    application
    # pump_manager_demo

    PACKAGES
    libhal-util
    libhal-soft
    # libhal-pca
    # libhal-esp8266

    LINK_LIBRARIES
    libhal::util
    libhal::soft
    # libhal::pca
    # libhal::esp8266

    INCLUDES
    ./include

    SOURCES
    implementations/ackermann_steering.cpp
    implementations/drive_configuration_updater.cpp
    implementations/wheel_router.cpp
) 
