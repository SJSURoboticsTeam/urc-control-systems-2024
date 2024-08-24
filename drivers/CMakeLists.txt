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
  color_sensor_demo
  led_strip_demo
  pressure_sensor_demo
  #pump_manager_demo
  scd40_demo
  soil_sensor_demo

SOURCES
    platforms/micromod.cpp
    src/color_sensor_opt4048.cpp
    src/methane_sensor_mq4.cpp
    src/pressure_sensor_bme680.cpp
    src/pump_manager.cpp
    src/revolver.cpp
    src/scd40.cpp
    src/soil_sensor_sht21.cpp

    PACKAGES
#    libhal-__device__

    LINK_LIBRARIES
#    libhal::__device__
)