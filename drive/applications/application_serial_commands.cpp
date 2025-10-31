#include <libhal-exceptions/control.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/error.hpp>
#include <libhal/pointers.hpp>
#include <libhal/steady_clock.hpp>
#include <libhal/timeout.hpp>
#include <resource_list.hpp>
#include <stdexcept>
#include <string.h>

// just a convenience type
using hal_serial = hal::v5::strong_ptr<hal::serial>;

struct command_def
{
  char const* prefix;
  std::function<void(std::span<std::span<hal::byte>>)> callback;
};

int parse_int(std::span<hal::byte> s)
{
  char* end;
  int val = std::strtol(reinterpret_cast<char*>(s.begin().base()), &end, 10);
  if (end != reinterpret_cast<char*>(s.end().base())) {
    throw std::runtime_error("integer parse failure");
  }
  return val;
}

float parse_float(std::span<hal::byte> s)
{
  char* end;
  float val = std::strtod(reinterpret_cast<char*>(s.begin().base()), &end);
  if (end != reinterpret_cast<char*>(s.end().base())) {
    throw std::runtime_error("float parse failure");
  }
  return val;
}

std::span<hal::byte> get_param(std::span<std::span<hal::byte>> params,
                               size_t idx)
{
  if (idx >= params.size()) {
    throw std::runtime_error("not enough parameters");
  }
  return params[idx];
}

class command_handler
{
  std::array<hal::byte, 256> line;
  size_t cursor;

  // readline() reads one byte to the current line and returns:
  // 0) read one char successfully
  // 1) reached EOL
  // 2) no bytes to read & not reached EOL
  // 3) buffer overflow
  hal::byte readline(hal_serial& console)
  {
    if (cursor >= line.size()) {
      hal::print(*console,
                 "\nError: exceeded max command length 256 characters\n");
      cursor = 0;
      return 3;
    }

    std::span<hal::byte> view{ line.begin() + cursor, 1 };
    auto n = console->read(view).data.size();
    if (n < 1) {
      return 2;
    }
    // echo received key back to client
    console->write(view);

    switch (line[cursor]) {
      // Ctrl-C (end-of-text)
      case 0x03:
        cursor = 0;
        return 0;
      // backspace
      case '\b':
        cursor--;
        return 0;
      case '\n':
        cursor = 0;
        return 1;
    }

    cursor++;
    return 0;
  }

  // parse() parses the current line
  void parse(std::span<std::span<hal::byte>>& segments)
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
  // handle() reads the currently available bytes from the serial console and
  // executes a command if it is complete.
  void handle(hal_serial& console, std::span<command_def> commands)
  {
    while (true) {
      switch (readline(console)) {
        case 0:
          break;
        case 1:
          goto stop_reading;
        case 2:
        case 3:
          return;
      }
    }

  stop_reading:

    // size = maximum # of segments
    //
    // supposing the prefix is 1 char and each param is 1 char -> 1/2 of the
    // line must be spaces.
    //
    // thus, max # of segments = 256/2 = 128
    std::array<std::span<hal::byte>, 128> segments;
    std::span<std::span<hal::byte>> segview{ segments };
    parse(segview);
    if (segview.size() == 0) {
      return;
    }

    std::span<hal::byte> prefix = segments[0];
    std::span<std::span<hal::byte>> params{ segments.begin() + 1,
                                            segview.size() - 1 };

    for (size_t i = 0; i < commands.size(); i++) {
      command_def cmd = commands[i];
      bool equal = false;
      for (size_t pi = 0;; pi++) {
        char target_char = cmd.prefix[pi];
        bool given_end = pi >= prefix.size();
        bool target_end = target_char == '\0';
        if (given_end || target_end) {
          equal = given_end && target_end;
          break;
        }
        if (prefix[pi] != target_char) {
          equal = false;
          break;
        }
      }
      if (equal) {
        cmd.callback(params);
        break;
      }
    }
  }
};

// TODO:
// 2. implement backspace and <Ctrl-C>
// 3. improve get_param, error handling, and command def ergonomics

namespace sjsu::drive {
void application()
{
  using namespace std::chrono_literals;

  auto led = resources::status_led();
  auto clock = resources::clock();
  auto console = resources::console();

  std::array cmd_defs = {
    // not going to include all variations of {F/B/D}{L/R}{S/P}, they work in
    // the same fashion
    command_def{
      "fls",
      [console](auto params) {
        float speed = parse_float(get_param(params, 0));
        hal::print<32>(*console, "FLS: %f\n", speed);
      },
    },
    command_def{
      "brp",
      [console](auto params) {
        float speed = parse_float(get_param(params, 0));
        hal::print<32>(*console, "BRP: %f\n", speed);
      },
    },
    command_def{
      "drs",
      [console](auto params) {
        float speed = parse_float(get_param(params, 0));
        hal::print<32>(*console, "DRS: %f\n", speed);
      },
    },
    command_def{
      "noparam",
      [console](auto) { hal::print(*console, "No parameter command\n"); },
    },
    command_def{
      "multiparam",
      [console](auto params) {
        float p1 = parse_float(get_param(params, 0));
        int p2 = parse_int(get_param(params, 1));
        int p3 = parse_int(get_param(params, 2));
        hal::print<64>(
          *console, "Multi-parameter command: %f %d %d\n", p1, p2, p3);
      },
    },
  };  // namespace sjsu::drive

  command_handler cmd{};

  while (true) {
    // we tradeoff responsiveness for performance, effectively batching a bunch
    // of reads in a single 500ms interval into single read at the end of a
    // 500ms time block
    //
    // to ensure that the client doesn't overflow the receive buffer in this
    // time, we echo every character the client sends so the client can wait
    // until we "ack" their command.
    cmd.handle(console, cmd_defs);

    // Toggle LED
    led->level(true);
    hal::delay(*clock, 500ms);

    cmd.handle(console, cmd_defs);

    led->level(false);
    hal::delay(*clock, 500ms);
  }
}
}  // namespace sjsu::drive
