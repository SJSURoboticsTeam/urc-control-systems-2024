#pragma once
#include <libhal-util/serial.hpp> //include for print and serial

namespace sjsu::science {

// commands are parsed from mission control
//this file is specific to last year's science project, so will be deleted or completely rewritten
//for current science plan
struct science_commands
{
  int state_step = 0;
  char mode = 'A';
  int is_operational = 0;
};

enum status
{
  not_started = 0,
  in_progress = 1,
  complete = 2
};

struct science_status
{
  int move_revolver_status = status::not_started;
  int seal_status = status::not_started;
  int depressurize_status = status::not_started;
  int inject_status = status::not_started;
  int clear_status = status::not_started;
  int unseal_status = status::not_started;
  void Print(hal::serial& terminal)
  {
    hal::print<200>(
      terminal,
      "move_revolver_status: %d | seal_status: %d | depressurize_status: %d | "
      "inject_status: %d | clear_status: %d | unseal_status: %d \n",
      move_revolver_status,
      seal_status,
      depressurize_status,
      inject_status,
      clear_status,
      unseal_status);
  }
};

// science data is what we send mission control
struct science_data
{
  science_status status;
  float methane_level = 0.0;
  float co2_level = 0.0;
  float pressure_level = 0.0;
};

const char kResponseBodyFormat[] =
  "{\"state_step\":%d,\"mode\":%c,\"is_operational\":%d}\n";

const char kGETRequestFormat[] =
  "science?rev_status=%d&seal_status=%d&depressure_status=%d&inj_status=%d&"
  "clear_status=%d&"
  "unseal_status=%d&methane_level=%d&co2_level=%d&pressure_level=%d";

}  // namespace science