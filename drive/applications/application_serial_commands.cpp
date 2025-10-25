#include <libhal-exceptions/control.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/error.hpp>
#include <libhal/pointers.hpp>
#include <libhal/steady_clock.hpp>
#include <libhal/timeout.hpp>
#include <resource_list.hpp>

// just a convenience type
using hal_serial = hal::v5::strong_ptr<hal::serial>;

enum class command_payload_type : uint8_t
{
  none,
  float32,
  int32
};

struct command_payload
{
  command_payload_type type;
  float f32;
  int i32;
};

void handle_command(hal_serial& console,
                    std::span<hal::byte> prefix,
                    command_payload payload)
{
  if (payload.type != command_payload_type::float32) {
    hal::print<48>(*console, "Error: non-float value is unsupported\n");
    return;
  }
  float value = payload.f32;
  switch (prefix[0]) {
    case 'f':
      switch (prefix[1]) {
        case 'l':
          switch (prefix[2]) {
            case 's':
              hal::print<32>(*console, "FLS: %f\n", value);
              return;
            case 'p':
              hal::print<32>(*console, "FLP: %f\n", value);
              return;
          }
          break;
        case 'r':
          switch (prefix[2]) {
            case 's':
              hal::print<32>(*console, "FRS: %f\n", value);
              return;
            case 'p':
              hal::print<32>(*console, "FRP: %f\n", value);
              return;
          }
          break;
      }
      break;
    case 'b':
      switch (prefix[1]) {
        case 'l':
          switch (prefix[2]) {
            case 's':
              hal::print<32>(*console, "BLS: %f\n", value);
              return;
            case 'p':
              hal::print<32>(*console, "BLP: %f\n", value);
              return;
          }
          break;
        case 'r':
          switch (prefix[2]) {
            case 's':
              hal::print<32>(*console, "BRS: %f\n", value);
              return;
            case 'p':
              hal::print<32>(*console, "BRP: %f\n", value);
              return;
          }
          break;
      }
      break;
    case 'd':
      switch (prefix[1]) {
        case 'l':
          switch (prefix[2]) {
            case 's':
              hal::print<32>(*console, "DLS: %f\n", value);
              return;
            case 'p':
              hal::print<32>(*console, "DLP: %f\n", value);
              return;
          }
          break;
        case 'r':
          switch (prefix[2]) {
            case 's':
              hal::print<32>(*console, "DRS: %f\n", value);
              return;
            case 'p':
              hal::print<32>(*console, "DRP: %f\n", value);
              return;
          }
          break;
      }
      break;
  }
  hal::print<48>(*console, "Error: invalid command %s\n", prefix);
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
 * @return Returns the number of bytes read if read was successful. Returns
 * -1 if buffer overflowed.
 */
template<int MAX_SIZE>
[[nodiscard]] int read_segment(hal_serial& console,
                               std::span<hal::byte> buffer,
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

  int i = 0;
  while (true) {
    auto cur = hal::read<1>(*console, hal::never_timeout());
    auto c = cur[0];
    console->write(cur);

    // skip carriage return, may slightly reduce CRLF issues with various
    // serial clients
    if (c == '\r') {
      continue;
    }

    // support backspace
    if (c == '\b') {
      i--;
      if (i < 0) {
        i = 0;
      }
      continue;
    }

    if (c == separator) {
      return i;
    }

    buffer[i] = c;
    i++;
    // only report overflow if i strictly greater than MAX_SIZE to include the
    // loop instance that reads the separator at i == MAX_SIZE
    if (i > MAX_SIZE) {
      return -1;
    }
  }
}

template<int prefix_size, int payload_size>
[[nodiscard]] command_payload read_command(
  hal_serial& console,
  std::span<hal::byte, prefix_size> prefix)
{
  command_payload ret{};
  ret.type = command_payload_type::none;

  // cannot use receive buffer directly because:
  // 1. can only access receive buffer in hal v5
  // 2. receive buffer is a circular buffer, which does not play nice with the
  // standard lib's string parsing functions

  // read a command in the format '{prefix}:{number}\n'

  auto res = read_segment<prefix_size>(console, prefix, ':');
  if (res < 0) {
    hal::print(*console, "\nError: exceeded maximum prefix length\n");
    return ret;
  }

  std::array<hal::byte, payload_size + 1> payload{};
  res = read_segment<payload_size>(console, payload, '\n');
  if (res < 0) {
    hal::print<40>(*console, "Error: exceeded maximum payload length\n");
    return ret;
  }

  auto pstart = reinterpret_cast<char*>(payload.begin() + 1);
  // set string null-terminated, res \in [0, payload_size]
  payload[res] = 0;
  auto pend = reinterpret_cast<char*>(payload.begin() + res);

  char* parse_end;
  switch (payload[0]) {
    case 'i': {
      int val = std::strtol(pstart, &parse_end, 10);
      if (parse_end != pend) {
        hal::print<40>(*console, "Error: failed to parse integer\n");
        return ret;
      }
      ret.type = command_payload_type::int32;
      ret.i32 = val;
      return ret;
    }
    case 'f': {
      float val = std::strtod(pstart, &parse_end);
      if (parse_end != pend) {
        hal::print<40>(*console, "Error: failed to parse float\n");
        return ret;
      }
      ret.type = command_payload_type::float32;
      ret.f32 = val;
      return ret;
    }
  }

  hal::print<48>(*console, "Error: invalid payload type '%c'\n", payload[0]);
  return ret;
}

namespace sjsu::drive {
void application()
{
  using namespace std::chrono_literals;

  hal_serial console = resources::console();

  while (true) {
    std::array<hal::byte, 3> prefix{};
    prefix[0] = 0;
    auto payload = read_command<3, 32>(console, prefix);
    if (payload.type == command_payload_type::none) {
      continue;
    }
    handle_command(console, prefix, payload);
  }
}
}  // namespace sjsu::drive
