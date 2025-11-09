
cmake_minimum_required(VERSION 3.20)

project(application LANGUAGES CXX)

libhal_build_demos(
    DEMOS
    application
    carousel_demo

    INCLUDES
    .

    SOURCES
    implementations/pump_manager.cpp
    implementations/carousel.cpp

    PACKAGES
    libhal-util
    libhal-actuator


    LINK_LIBRARIES
    libhal::util
    libhal::actuator

) 
