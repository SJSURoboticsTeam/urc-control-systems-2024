// Copyright 2023 Google LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "hardware_map.hpp"
sjsu::science::application_framework platform_status{};

int main()
{
 
  try{
    platform_status = sjsu::science::initialize_platform();
  }catch(...){
    hal::halt();
  }
  sjsu::science::application(platform_status);

  return 0;
}

