#include <charconv>
#include <libhal-exceptions/control.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/error.hpp>
#include <libhal/pointers.hpp>
#include <libhal/steady_clock.hpp>
#include <libhal/timeout.hpp>
#include <resource_list.hpp>
#include <system_error>
#include <variant>

// just a convenience type
using hal_serial = hal::v5::strong_ptr<hal::serial>;

void handle_command(hal_serial& console,
                    std::span<hal::byte> prefix,
                    std::variant<int, float> param)
{
  switch (prefix[0]) {
    case 'f':
      switch (prefix[1]) {
        case 'l':
          switch (prefix[2]) {
            case 's':
              hal::print<32>(*console, "FLS: %f\n", std::get<float>(param));
              break;
            case 'p':
              hal::print<32>(*console, "FLP: %f\n", std::get<float>(param));
              break;
          }
          break;
        case 'r':
          switch (prefix[2]) {
            case 's':
              hal::print<32>(*console, "FRS: %f\n", std::get<float>(param));
              break;
            case 'p':
              hal::print<32>(*console, "FRP: %f\n", std::get<float>(param));
              break;
          }
          break;
      }
      break;
    case 'b':
      switch (prefix[1]) {
        case 'l':
          switch (prefix[2]) {
            case 's':
              hal::print<32>(*console, "BLS: %f\n", std::get<float>(param));
              break;
            case 'p':
              hal::print<32>(*console, "BLP: %f\n", std::get<float>(param));
              break;
          }
          break;
        case 'r':
          switch (prefix[2]) {
            case 's':
              hal::print<32>(*console, "BRS: %f\n", std::get<float>(param));
              break;
            case 'p':
              hal::print<32>(*console, "BRP: %f\n", std::get<float>(param));
              break;
          }
          break;
      }
      break;
    case 'd':
      switch (prefix[1]) {
        case 'l':
          switch (prefix[2]) {
            case 's':
              hal::print<32>(*console, "DLS: %f\n", std::get<float>(param));
              break;
            case 'p':
              hal::print<32>(*console, "DLP: %f\n", std::get<float>(param));
              break;
          }
          break;
        case 'r':
          switch (prefix[2]) {
            case 's':
              hal::print<32>(*console, "DRS: %f\n", std::get<float>(param));
              break;
            case 'p':
              hal::print<32>(*console, "DRP: %f\n", std::get<float>(param));
              break;
          }
          break;
      }
      break;
  }
}

/**
 * @brief Reads a string of a MAX_SIZE up until the separator. (this means
 * MAX_SIZE does not include the separator)
 *
 * @param console The serial interface to use for reading.
 *
 * @param separator The separator ending the string.
 *
 * @param buffer The buffer to store the string.
 *
 * @return Returns the number of bytes read if read was successful. If
 * unsuccessful: -1 = the string overflowed the buffer before reaching the
 * separator.
 */
template<size_t MAX_SIZE>
[[nodiscard]] int read_segment(hal_serial& console,
                               std::span<hal::byte, MAX_SIZE> buffer,
                               char separator)
{
  /*
  // hal v5 code:

  size_t i = 0;

  auto recv = console->receive_buffer();
  auto prev = console->receive_cursor();

  while (true) {
    auto cur = console->receive_cursor();
    size_t n = cur - prev;
    if (n == 0) {
      continue;
    }

    // to receive new bytes one-by-one, you would increment prev every time at
    // least 1 byte is read
    prev++;
    auto c = recv[prev];
    if (c == separator) {
      return i;
    }

    // catch buffer overflow
    if (i >= MAX_SIZE) {
      return -1;
    }

    buffer[i] = c;
    i++;
  }
  */

  std::array<hal::byte, 1> cur;
  size_t i = 0;
  while (true) {
    auto res = console->read(cur);
    if (res.available != 1) {
      return -1;
    }
    auto c = cur[0];
    if (c == separator) {
      return i;
    }
    buffer[i] = c;
    i++;
    // only report overflow if i strictly greater than MAX_SIZE to include the
    // loop instance that reads the separator at i == MAX_SIZE
    if (i > MAX_SIZE) {
      return -2;
    }
  }
}

using command_parameter = std::variant<int, float>;

[[nodiscard]] std::optional<command_parameter> read_command(
  hal_serial& console,
  std::span<hal::byte, 3> prefix)
{
  // cannot use receive buffer directly because:
  // 1. can only access receive buffer in hal v5
  // 2. receive buffer is a circular buffer, which does not play nice with the
  // standard lib's string parsing functions

  // read a command in the format '{prefix}:{number}\n'

  int prefixn = read_segment<3>(console, prefix, ':');
  if (prefixn < 0) {
    return std::nullopt;
  }
  std::array<hal::v5::byte, 32> number;
  int numbern = read_segment<32>(console, number, '\n');
  if (numbern < 0) {
    return std::nullopt;
  }

  // parse either float or int32

  auto numstart = reinterpret_cast<char*>(number.begin());
  auto numend = reinterpret_cast<char*>(number.begin() + numbern);

  int intval = 0;
  auto res = std::from_chars(numstart, numend, intval);
  if (res.ec != std::errc{}) {
    return std::nullopt;
  }
  if (res.ptr == numend) {
    return intval;
  }

  float floatval = 0;
  res = std::from_chars(numstart, numend, floatval);
  if (res.ec != std::errc{}) {
    return std::nullopt;
  }
  if (res.ptr == numend) {
    return floatval;
  }
  return std::nullopt;
}

namespace sjsu::drive {
void application()
{
  using namespace std::chrono_literals;

  hal_serial console = resources::console();

  std::array<hal::v5::byte, 3> prefix;
  auto param = read_command(console, prefix);
  handle_command(console, prefix, param.value());
}
}  // namespace sjsu::drive
