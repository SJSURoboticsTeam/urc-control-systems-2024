#pragma once
#include <libhal-util/serial.hpp>
#include <libhal/error.hpp>
#include <libhal/pointers.hpp>
#include <string.h>
// #include <libhal-exceptions/control.hpp>
// #include <libhal/timeout.hpp>

namespace sjsu::drivers::serial_commands {
struct def
{
  char const* m_prefix;
  std::function<void(std::span<std::span<hal::byte>>)> m_callback;
};

class handler
{
  std::array<hal::byte, 256> m_line;
  size_t m_cursor;
  hal::v5::strong_ptr<hal::serial> m_console;

  enum class read_stat : hal::byte
  {
    complete,
    incomplete,
    overflow
  };

  read_stat read();
  void parse(std::span<std::span<hal::byte>>& p_segments);
  bool prefix_match(char const* p_prefix, std::span<hal::byte> p_str);

public:
  void handle(std::span<def> p_commands);
  handler(hal::v5::strong_ptr<hal::serial> p_console);
};

int parse_int(std::span<hal::byte> p_str);
float parse_float(std::span<hal::byte> p_str);
}  // namespace sjsu::drivers::serial_commands
