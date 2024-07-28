
cmake_minimum_required(VERSION 3.20)

project(application LANGUAGES CXX)

libhal_build_demos(
    DEMOS
    application

    PACKAGES
    libhal-${platform_library}
    libhal-util
    libhal-rmd
    libhal-soft
    libhal-pca
    libhal-esp8266

    LINK_LIBRARIES
    libhal::${platform_library}
    libhal::util
    libhal::rmd
    libhal::soft
    libhal::pca
    libhal::esp8266

    SOURCES
    main.cpp
    applications/application.cpp
    platform-implementations/pump_manager.cpp
    platform-implementations/revolver.cpp
    platform-implementations/esp8266_mission_control.cpp
    platform-implementations/science_state_machine.cpp
    platforms/${platform}.cpp
) 