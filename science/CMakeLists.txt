
cmake_minimum_required(VERSION 3.20)

project(application LANGUAGES CXX)

libhal_build_demos(
    DEMOS
    application

    PACKAGES
    libhal-util
    libhal-soft
    libhal-pca
    # libhal-esp8266

    LINK_LIBRARIES
    libhal::util
    libhal::soft
    libhal::pca
    # libhal::esp8266

    INCLUDES
    .

    SOURCES
    implementations/pump_manager.cpp
    implementations/revolver.cpp
    implementations/science_state_machine.cpp
) 