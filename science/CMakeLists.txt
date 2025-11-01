
cmake_minimum_required(VERSION 3.20)

project(application LANGUAGES CXX)

libhal_build_demos(
    DEMOS
    application

    INCLUDES
    .

    SOURCES
    implementations/pump_manager.cpp

    PACKAGES
    libhal-util
    libhal-actuator


    LINK_LIBRARIES
    libhal::util
    libhal::actuator

) 
