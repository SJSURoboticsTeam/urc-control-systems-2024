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

INCLUDES
  include
  .

#SOURCES

    PACKAGES
    libhal-actuator
    libhal-sensor

    #Template for importing libhal packages is "libhal-__device__"
    #replace "__device__" with the libhal device you wish to import
    #ex: to import from https://github.com/libhal/libhal-rmd the next line would be "libhal-rmd"

    LINK_LIBRARIES
    libhal::actuator
    libhal::sensor
    
    #Template for linking libhal namespaces is "libhal::__device__"
    #replace "__device__" with the libhal device you wish to import
    #ex: to import from https://github.com/libhal/libhal-rmd the next line would be "libhal::rmd"
    
)