#include <libhal-exceptions/control.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/error.hpp>
#include <libhal/pointers.hpp>
#include <libhal/steady_clock.hpp>
#include <libhal/timeout.hpp>
#include <optional>
#include <resource_list.hpp>

// just a convenience type
using hal_serial = hal::v5::strong_ptr<hal::serial>;

struct command_def
{
  std::span<hal::byte> prefix;
  void (*callback)(std::span<std::span<hal::byte>> parameters);
};

std::optional<int> parse_int(std::span<hal::byte> s)
{
  char* end;
  int val = std::strtol(reinterpret_cast<char*>(s.begin()), &end, 10);
  if (end != reinterpret_cast<char*>(s.end())) {
    return std::nullopt;
  }
  return val;
}

std::optional<float> parse_float(std::span<hal::byte> s)
{
  char* end;
  float val = std::strtod(reinterpret_cast<char*>(s.begin()), &end, 10);
  if (end != reinterpret_cast<char*>(s.end())) {
    return std::nullopt;
  }
  return val;
}

class command_handler
{
  std::array<hal::byte, 256> line;
  size_t cursor;

  // readline() returns true if reached end of line
  bool readline(hal_serial& console)
  {
    if (cursor >= line.size()) {
      hal::print(*console,
                 "\nError: exceeded max command length 256 characters\n");
      cursor = 0;
      return false;
    }

    std::span<hal::byte> view{ line.begin() + cursor, 1 };
    auto n = console->read(view).data.size();
    if (n != 1) {
      return false;
    }
    // echo received key back to client
    console->write(view);

    if (line[cursor] == '\n') {
      cursor = 0;
      return true;
    }
    cursor++;
    return false;
  }

  void parse_command(std::span<std::span<hal::byte>>& segments)
  {
    size_t segment_cursor = 0;
    size_t start = 0;

    for (size_t i = 0; i < line.size(); i++) {
      char c = line[i];

      if (c != '\n' && c != ' ') {
        continue;
      }

      // if some non-space chars have been found in-between last separator and
      // current
      if (i - start > 0) {
        std::span<hal::byte> view{ line.begin() + start, i - start };
        segments[segment_cursor] = view;
        segment_cursor++;
      }

      // set the "beginning" of a non-separator character to the next for
      // current == separator
      if (c == ' ') {
        start = i + 1;
      }

      if (c == '\n') {
        break;
      }
    }

    segments = segments.subspan(0, segment_cursor);
  }

public:
  void handle(hal_serial& console, std::span<command_def> commands)
  {
    bool run = readline(console);
    if (!run) {
      return;
    }

    // size = maximum # of segments
    //
    // supposing the prefix is 1 char and each param is 1 char -> 1/2 of the
    // line must be spaces.
    //
    // thus, max # of segments = 256/2 = 128
    std::array<std::span<hal::byte>, 128> segments;
    std::span<std::span<hal::byte>> segview{ segments };
    parse_command(segview);
    if (segview.size() == 0) {
      return;
    }

    std::span<hal::byte> prefix = segments[0];
    std::span<std::span<hal::byte>> params{ segments.begin() + 1,
                                            segview.size() - 1 };

    for (size_t i = 0; i < commands.size(); i++) {
      command_def cmd = commands[i];
      if (cmd.prefix != prefix) {
        continue;
      }
      cmd.callback(params);
      break;
    }
  }
};

namespace sjsu::drive {
void application()
{
  using namespace std::chrono_literals;

  auto led = resources::status_led();
  auto clock = resources::clock();
  auto console = resources::console();

#define FLOAT(params, idx, var)                                                \
  auto _res##idx = parse_float(params[idx]);                                   \
  if (!_res##idx.has_value()) {                                                \
    return;                                                                    \
  }                                                                            \
  float var = _res##idx.value();

#define INT(params, idx, var)                                                  \
  auto _res##idx = parse_int(params[idx]);                                     \
  if (!_res##idx.has_value()) {                                                \
    return;                                                                    \
  }                                                                            \
  int var = _res##idx.value();

  std::array cmd_defs = {
    // not going to include all variations of {F/B/D}{L/R}{S/P}, they work in
    // the same fashion
    command_def{
      "fls",
      [console](params) {
        FLOAT(params, 0, speed)
        hal::print<32>(*console, "FLS: %f\n", speed);
      },
    },
    command_def{
      "brp",
      [console](params) {
        FLOAT(params, 0, speed)
        hal::print<32>(*console, "BRP: %f\n", speed);
      },
    },
    command_def{
      "drs",
      [console](params) {
        FLOAT(params, 0, speed)
        hal::print<32>(*console, "DRS: %f\n", speed);
      },
    },
    command_def{
      "noparam",
      [console](params) { hal::print(*console, "No parameter command\n"); },
    },
    command_def{
      "multiparam",
      [console](params) {
        FLOAT(params, 0, p1)
        INT(params, 1, p2)
        INT(params, 2, p3)
        hal::print<64>(
          *console, "Multi-parameter command: %f %d %d\n", p1, p2, p3);
      },
    },
  };  // namespace sjsu::drive

#undef FLOAT

  command_handler cmd{};

  while (true) {
    cmd.handle(console, cmd_defs);

    // Toggle LED
    led->level(true);
    hal::delay(*clock, 500ms);

    led->level(false);
    hal::delay(*clock, 500ms);
  }
}
}  // namespace sjsu::drive
