#include <libhal-exceptions/control.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/error.hpp>
#include <libhal/pointers.hpp>
#include <libhal/steady_clock.hpp>
#include <libhal/timeout.hpp>
#include <string.h>
#include <system_error>

namespace sjsu::drive {
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
    throw hal::argument_out_of_domain(nullptr);
  }
  return val;
}

float parse_float(std::span<hal::byte> s)
{
  char* end;
  float val = std::strtod(reinterpret_cast<char*>(s.begin().base()), &end);
  if (end != reinterpret_cast<char*>(s.end().base())) {
    throw hal::argument_out_of_domain(nullptr);
  }
  return val;
}

class command_handler
{
  std::array<hal::byte, 256> line;
  size_t cursor;

  enum class read_byte_stat : hal::byte
  {
    success,
    eol,
    noread,
    overflow
  };

  read_byte_stat read_byte(hal::serial& console)
  {
    if (cursor >= line.size()) {
      hal::print(console,
                 "\nError: exceeded max command length 256 characters\n");
      cursor = 0;
      return read_byte_stat::overflow;
    }

    std::span<hal::byte> view{ line.begin() + cursor, 1 };
    auto n = console.read(view).data.size();
    if (n < 1) {
      return read_byte_stat::noread;
    }
    // echo received key back to client console->write(view);

    switch (line[cursor]) {
      // Ctrl-C (end-of-text)
      case 0x03:
        view[0] = '\n';
        console.write(view);

        cursor = 0;
        return read_byte_stat::success;
      // backspace
      case '\b':
        view[0] = ' ';
        console.write(view);
        view[0] = '\b';
        console.write(view);

        if (cursor > 0) {
          cursor--;
        }
        return read_byte_stat::success;
      case '\n':
        cursor = 0;
        return read_byte_stat::eol;
    }

    cursor++;
    return read_byte_stat::success;
  }

  // parse() parses the current line
  void parse(std::span<std::span<hal::byte>> segments)
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

  bool prefix_match(char const* prefix, std::span<hal::byte> str)
  {
    for (size_t pi = 0;; pi++) {
      char target_char = prefix[pi];
      bool given_end = pi >= str.size();
      bool target_end = target_char == '\0';
      if (given_end || target_end) {
        return given_end && target_end;
      }
      if (str[pi] != target_char) {
        return false;
      }
    }
    return false;
  }

public:
  // handle() reads the currently available bytes from the serial console and
  // executes a command if it is complete.
  void handle(hal::serial& console,
              std::span<command_def> commands)
  {
    while (true) {
      switch (read_byte(console)) {
        case read_byte_stat::success:
          continue;
        case read_byte_stat::eol:
          break;
        case read_byte_stat::noread:
        case read_byte_stat::overflow:
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
      parse(segview);
      if (segview.size() == 0) {
        return;
      }

      std::span<hal::byte> prefix = segments[0];
      std::span<std::span<hal::byte>> params{ segments.begin() + 1,
                                              segview.size() - 1 };

      for (size_t i = 0; i < commands.size(); i++) {
        command_def cmd = commands[i];
        if (!prefix_match(cmd.prefix, prefix)) {
          continue;
        }
        cmd.callback(params);
        return;
      }
      hal::print(*console, "Unknown command.\n");
    }
  }
};

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
      [&console](auto params) {
        if (params.size() != 1) {
          throw hal::argument_out_of_domain(nullptr);
        }
        float speed = parse_float(params[0]);
        hal::print<32>(*console, "FLS: %f\n", speed);
      },
    },
    command_def{
      "brp",
      [&console](auto params) {
        if (params.size() != 1) {
          throw hal::argument_out_of_domain(nullptr);
        }
        float speed = parse_float(params[0]);
        hal::print<32>(*console, "BRP: %f\n", speed);
      },
    },
    command_def{
      "drs",
      [&console](auto params) {
        if (params.size() != 1) {
          throw hal::argument_out_of_domain(nullptr);
        }
        float speed = parse_float(params[0]);
        hal::print<32>(*console, "DRS: %f\n", speed);
      },
    },
    command_def{
      "noparam",
      [&console](auto params) {
        if (params.size() != 0) {
          throw hal::argument_out_of_domain(nullptr);
        }
        hal::print(*console, "Executed no parameter command\n");
      },
    },
    command_def{
      "multiparam",
      [&console](auto params) {
        if (params.size() != 3) {
          throw hal::argument_out_of_domain(nullptr);
        }
        float p1 = parse_float(params[0]);
        int p2 = parse_int(params[1]);
        int p3 = parse_int(params[2]);
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
    //
    // this makes it awkward if your TTY client sends one character at a time
    // instead of an entire line at a time, in these cases it makes more sense
    // to simply reconfigure your TTY client
    try {
      cmd.handle(console, cmd_defs);
    } catch (hal::exception e) {
      switch (e.error_code()) {
        case std::errc::argument_out_of_domain:
          hal::print(*console, "Error: invalid argument length or type\n");
          break;
        default:
          hal::print<32>(*console, "Error code: %d\n", e.error_code());
          break;
      }
    }

    // Toggle LED
    led->level(true);
    hal::delay(*clock, 500ms);

    cmd.handle(console, cmd_defs);

    led->level(false);
    hal::delay(*clock, 500ms);
  }
}
}  // namespace sjsu::drive
