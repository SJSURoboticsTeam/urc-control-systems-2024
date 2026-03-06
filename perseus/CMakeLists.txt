# Copyright 2024 Khalil Estell
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.20)

project(demos LANGUAGES CXX)

libhal_build_demos(
  DEMOS
    application
    # can_test
    # can_test_wrist
    can_test_elbow 
    can_test_shoulder
    can_test_track
    can_test_wrist_l
    can_test_wrist_r
    can_test_endeff

  
  INCLUDES
    ../drivers/include/
    include
    .

SOURCES
    ../drivers/src/h_bridge.cpp
    src/bldc_servo.cpp
    ../drivers/src/serial_commands.cpp
    src/can_messaging.cpp
    applications/perseus_can_demo.cpp
    
PACKAGES
    #Template for importing libhal packages is "libhal-__device__"
    #replace "__device__" with the libhal device you wish to import
    #ex: to import from https://github.com/libhal/libhal-rmd the next line would be "libhal-rmd"

LINK_LIBRARIES
    #Template for linking libhal namespaces is "libhal::__device__"
    #replace "__device__" with the libhal device you wish to import
    #ex: to import from https://github.com/libhal/libhal-rmd the next line would be "libhal::rmd"
    
)