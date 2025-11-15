#include "../include/serial_commands.hpp"

namespace sjsu::drivers::serial_commands {

int parse_int(std::span<hal::byte> p_str)
{
  char* end;
  int val =
    std::strtol(reinterpret_cast<char*>(p_str.begin().base()), &end, 10);
  if (end != reinterpret_cast<char*>(p_str.end().base())) {
    throw hal::argument_out_of_domain(nullptr);
  }
  return val;
}

float parse_float(std::span<hal::byte> p_str)
{
  char* end;
  float val = std::strtod(reinterpret_cast<char*>(p_str.begin().base()), &end);
  if (end != reinterpret_cast<char*>(p_str.end().base())) {
    throw hal::argument_out_of_domain(nullptr);
  }
  return val;
}

handler::read_stat handler::read()
{
  while (true) {
    if (m_cursor >= m_line.size()) {
      hal::print(*m_console,
                 "\nError: exceeded max command length 256 characters\n");
      m_cursor = 0;
      return read_stat::overflow;
    }

    std::span<hal::byte> view{ m_line.begin() + m_cursor, 1 };
    auto n = m_console->read(view).data.size();
    if (n < 1) {
      return read_stat::incomplete;
    }

    // echo received key back to client
    m_console->write(view);

    switch (view[0]) {
      // Ctrl-C (end-of-text)
      case 0x03:
        view[0] = '\n';
        m_console->write(view);
        m_cursor = 0;
        continue;
      case 127:   // delete
      case '\b':  // backspace
        view[0] = ' ';
        m_console->write(view);
        view[0] = '\b';
        m_console->write(view);
        if (m_cursor > 0) {
          m_cursor--;
        }
        continue;
      case '\r':
		view[0] = '\n';
        m_console->write(view);
		[[fallthrough]];
      case '\n':
        m_cursor = 0;
        return read_stat::complete;
    }
    m_cursor++;
  }
}

// parse() parses the current line
void handler::parse(std::span<std::span<hal::byte>>& p_segments)
{
  size_t segment_cursor = 0;
  size_t start = 0;

  for (size_t i = 0; i < m_line.size(); i++) {
    char c = m_line[i];

    if (c != '\n' && c != ' ') {
      continue;
    }

    // if found parameter
    if (i - start > 0) {
      std::span<hal::byte> view{ m_line.begin() + start, i - start };
      p_segments[segment_cursor] = view;
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

  p_segments = p_segments.subspan(0, segment_cursor);
}

bool handler::prefix_match(char const* p_prefix, std::span<hal::byte> p_str)
{
  for (size_t pi = 0;; pi++) {
    char target_char = p_prefix[pi];
    bool given_end = pi >= p_str.size();
    bool target_end = target_char == '\0';
    if (given_end || target_end) {
      return given_end && target_end;
    }
    if (p_str[pi] != target_char) {
      return false;
    }
  }
  return false;
}

handler::handler(hal::v5::strong_ptr<hal::serial> p_console)
  : m_cursor{ 0 }
  , m_console{ p_console }
{
}

// handle() reads the currently available bytes from the serial console and
// executes a command if it is complete.
void handler::handle(std::span<def> p_commands)
{
  switch (read()) {
    case read_stat::complete:
      break;
    case read_stat::incomplete:
    case read_stat::overflow:
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
  if (prefix.size() == 0) {
	  return;
  }

  std::span<std::span<hal::byte>> params{ segments.begin() + 1,
                                          segview.size() - 1 };

  for (size_t i = 0; i < p_commands.size(); i++) {
    def cmd = p_commands[i];
    if (!prefix_match(cmd.m_prefix, prefix)) {
      continue;
    }
    cmd.m_callback(params);
    return;
  }
  hal::print(*m_console, "Unknown command.\n");
}

}  // namespace sjsu::drivers::serial_commands
