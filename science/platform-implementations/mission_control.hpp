#pragma once
#include <libhal-util/serial.hpp>
#include <libhal/motor.hpp>
#include <libhal/serial.hpp>
#include <libhal/servo.hpp>
#include <libhal/steady_clock.hpp>
#include <libhal/timeout.hpp>

namespace sjsu::science {

static constexpr char kResponseBodyFormat[] =
  "{\"HB\":%d,\"IO\":%d,\"SR\":%d,\"PP\":\"%d\",\"CR\":\"%d\"}\n";

static constexpr char kGETRequestFormat[] =
  "science?heartbeat_count=%d&is_operational=%d&sample_recieved=%d&pause_play=%"
  "c&contianment_reset=%dnum_vials%dsample_finished%d";

static constexpr char get_request[] = "GET / %s HTTP/1.1\r\n"
                                      "Host: 192.168.0.211:5000\r\n"
                                      "Keep-Alive: timeout=1000\r\n"
                                      "Connection: keep-alive\r\n"
                                      "\r\n";

class mission_control
{
public:
  struct mc_commands
  {
    int heartbeat_count = 0;
    int is_operational = 0;
    int sample_recieved = 0;
    int pause_play = 0;
    int contianment_reset = 0;

    hal::status print(hal::serial* terminal)
    {
      hal::print<128>(*terminal,
                      "HB: %d\n"
                      "IS_OP: %d\n"
                      "Sample_Recieved: %d\n"
                      "Pause_Play: %d\n"
                      "Contianment_Reset: %d\n",
                      heartbeat_count,
                      is_operational,
                      sample_recieved,
                      pause_play,
                      contianment_reset);
      return hal::success();
    }
  };
  struct status
  {
    int heartbeat_count = 0;
    int is_operational = 1;
    int sample_recieved = 0;
    int pause_play = 0;
    int contianment_reset = 0;
    int num_vials_used = 0;
    int is_sample_finished = 0;
  };
  // // static status m_status;
  // status m_status;

  //! @note mission_control::GetStatus();
  //! @note mission_control::GetStatus() { return instance->m_status; }

  // static mission_control* instance;

  /**
   * @brief Get the command object
   *
   * This command is infallible and will always return a command.
   * If mission control is disconnected from the machine, this command will
   * return the last known value. The heartbeat_count can be used to indicate
   * when the connection has been re-established.
   *
   * It is the responsibility of this driver to re-establishing the connection.
   * This driver is allowed to utilize time during this call in order to
   * reestablish the connection.
   *
   * The timeout is used to break out of the function. This is useful, because
   * attempting to reestablish a connection to mission control could take a
   * considerable amount of time and thus, could stall out the application.
   *
   * @param p_timeout - the amount of time to either get the latest command or
   * attempt to reestablish a connection to mission control.
   * @return drive_commands - the last known driver command received. If no
   * commands have been received, then this should return the default
   * initialized command.
   */
  hal::result<mc_commands> get_command(
    hal::function_ref<hal::timeout_function> p_timeout)
  {
    return impl_get_command( p_timeout);
  }

  virtual ~mission_control() = default;

private:
  virtual hal::result<mc_commands> impl_get_command(
    hal::function_ref<hal::timeout_function> p_timeout) = 0;
};

}  // namespace sjsu::science