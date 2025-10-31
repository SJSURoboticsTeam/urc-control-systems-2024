cmake_minimum_required(VERSION 3.25)

# Set project name to demos
project(application LANGUAGES CXX)

libhal_build_demos(
    DEMOS
    application
    
    INCLUDES
    ../drivers/include
    include
    
    SOURCES
    src/home.cpp
    ../drivers/src/perseus_bldc.cpp
)