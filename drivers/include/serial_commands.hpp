#pragma once
#include <libhal-util/serial.hpp>
#include <libhal/error.hpp>
#include <libhal/pointers.hpp>
#include <string.h>

namespace sjsu::drivers::serial_commands {

/**
 * @brief def is a command definition
 */
struct def
{
  /**
   * @brief the command identifier
   *
   * It should contain no spaces and uniquely identify the command.
   */
  char const* m_prefix;
  /**
   * @brief the command callback
   *
   * @param params the parameters of the callback, effectively a list of
   * strings
   *
   * @throws hal::argument_out_of_domain - if the number of parameters given to
   * the callback are not the expected number, or if some parameters are
   * invalid
   */
  std::function<void(std::span<std::span<hal::byte>> params)> m_callback;
};

/**
 * @brief handler reads and executes commands from serial
 */
class handler
{
  std::array<hal::byte, 256> m_line;
  size_t m_cursor;
  hal::v5::strong_ptr<hal::serial> m_console;

  enum class read_status : hal::byte
  {
    complete,
    incomplete,
    overflow
  };

  read_status read();
  void parse(std::span<std::span<hal::byte>>& p_segments);
  bool prefix_match(char const* p_prefix, std::span<hal::byte> p_str);

public:
  /**
   * @brief non-blocking read from serial which may execute a command in
   * p_commands
   *
   * @param p_commands a list of command definitions available to execute by
   * the handler based on what is read from serial
   */
  void handle(std::span<def> p_commands);

  /**
   * @brief creates a new command handler
   *
   * @param p_console strong ptr to a serial console which will be used to
   * receive serial commands
   */
  handler(hal::v5::strong_ptr<hal::serial> p_console);
};

/**
 * @brief parses a string to an integer
 *
 * @param p_str the string to parse
 *
 * @throws hal::argument_out_of_domain - if p_str cannot be parsed as an
 * integer
 */
int parse_int(std::span<hal::byte> p_str);

/**
 * @brief parses a string to a float
 *
 * @param p_str the string to parse
 *
 * @throws hal::argument_out_of_domain - if p_str cannot be parsed as a float
 */
float parse_float(std::span<hal::byte> p_str);

}  // namespace sjsu::drivers::serial_commands
