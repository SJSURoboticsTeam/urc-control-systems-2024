#include <array>
#include <iterator>
#include <libhal-util/i2c.hpp>
#include <libhal/error.hpp>
#include <libhal/timeout.hpp>
#include <libhal/units.hpp>
#include <nhd0420d3z.hpp>

namespace sjsu::drivers {

namespace {
enum commands : hal::byte
{
  prefix = 0xFE,
  display_on = 0x41,
  display_off = 0x42,
  set_cursor = 0x45,
  cursor_home = 0x46,
  underline_cursor_on = 0x47,
  underline_cursor_off = 0x48,
  move_cursor_left_one_place = 0x49,
  move_cursor_right_one_place = 0x4A,
  blinking_cursor_on = 0x4B,
  blinking_cursor_off = 0x4C,
  backspace = 0x4E,
  clear_screen = 0x51,
  set_contrast = 0x52,
  set_backlight_brightness = 0x53,
  load_custom_character = 0x54,
  shift_display_left = 0x55,
  shift_display_right = 0x56,
};
};

nhd0420d3z::nhd0420d3z(hal::i2c& p_i2c, hal::byte p_i2c_address)
  : m_i2c_bus(p_i2c)
  , m_i2c_address(p_i2c_address)
{
}

void nhd0420d3z::send_data(hal::byte p_data)
{
  std::array<hal::byte, 1> command = { p_data };
  hal::write(m_i2c_bus, m_i2c_address, command, hal::never_timeout());
}

void nhd0420d3z::send_prefix()
{
  send_data(commands::prefix);
}

void nhd0420d3z::virtual_cursor_right()
{
  m_cursor_column++;
  if (m_cursor_column >= display_columns) {
    m_cursor_line++;
    m_cursor_column = 0;
  }
  if (m_cursor_line >= display_lines) {
    m_cursor_line = 0;
  }
}

void nhd0420d3z::virtual_cursor_left()
{
  if (m_cursor_column == 0) {
    if (m_cursor_line == 0) {
      return;
    } else {
      m_cursor_line--;
      m_cursor_column = display_columns;
    }
  } else {
    m_cursor_column--;
  }
}

void nhd0420d3z::start_new_line()
{
  m_cursor_line++;
  if (m_cursor_line >= display_lines) {
    m_cursor_line = 0;
  }
  m_cursor_column = 0;
  set_cursor_position(m_cursor_line, m_cursor_column);
}

void nhd0420d3z::power(bool p_on)
{
  send_prefix();
  if (p_on) {
    send_data(commands::display_on);
  } else {
    send_data(commands::display_off);
  }
}

void nhd0420d3z::set_display_contrast(hal::byte p_contrast)
{
  if (p_contrast < 1 || p_contrast > 50) {
    throw hal::argument_out_of_domain(this);
  }
  send_prefix();
  send_data(commands::set_contrast);
  send_data(p_contrast);
}

void nhd0420d3z::set_backlight_brightness(hal::byte p_brightness)
{
  if (p_brightness < 1 || p_brightness > 8) {
    throw hal::argument_out_of_domain(this);
  }
  send_prefix();
  send_data(commands::set_backlight_brightness);
  send_data(p_brightness);
}

void nhd0420d3z::write_char(char p_c)
{
  bool is_custom_char = p_c < 0x07;
  if (is_custom_char) {
    // display custom_character behavior, unused
  }

  bool is_ascii = p_c >= 0x20 && p_c <= 0x7F;
  if (is_ascii) {
    // display standard ASCII characters
    send_data(p_c);
    virtual_cursor_right();
    // explicitly set cursor position for same reason as backspace()
    set_cursor_position(m_cursor_line, m_cursor_column);
  }

  bool is_japanese_char = p_c >= 0xA0 && p_c <= 0xFD;
  if (is_japanese_char) {
    // display japanese characters, factory-masked on the SPLC780D controller,
    // unused
  }
}

void nhd0420d3z::backspace()
{
  send_prefix();
  send_data(commands::backspace);
  virtual_cursor_left();
  // we explicitly set cursor position because while the display does move the
  // cursor back, it has undesirable behavior when deleting from the start of
  // the line.
  //
  // if you are on line 2 for instance, you will end up jumping to the end of
  // line 3.
  //
  // the reason is because of how coordinates are represented internally by the
  // display, see coordinates_to_position for more details
  set_cursor_position(m_cursor_line, m_cursor_column);
}

hal::byte nhd0420d3z::coordinates_to_position(hal::byte p_line,
                                              hal::byte p_column)
{
  hal::byte pos = p_column;
  if (p_line % 2 == 1) {
    pos += 0x40;
  }
  if (p_line >= 2) {
    pos += 0x14;
  }
  return pos;
}

void nhd0420d3z::set_cursor_position(hal::byte p_line, hal::byte p_column)
{
  send_prefix();
  send_data(commands::set_cursor);
  send_data(coordinates_to_position(p_line, p_column));
  m_cursor_line = p_line;
  m_cursor_column = p_column;
}

void nhd0420d3z::home_cursor()
{
  send_prefix();
  send_data(commands::cursor_home);
  m_cursor_column = 0;
  m_cursor_line = 0;
}

void nhd0420d3z::clear_screen()
{
  send_prefix();
  send_data(commands::clear_screen);
}

void nhd0420d3z::display_message(std::string_view p_str)
{
  clear_screen();
  set_cursor_position(0, 0);
  auto strIt = p_str.begin();
  while (m_cursor_line < display_lines && strIt != p_str.end()) {
    if (*strIt == '\n') {
      start_new_line();
    } else {
      write_char(*strIt);
    }
    strIt++;
  }
}

void nhd0420d3z::underline_cursor(bool p_on)
{
  send_prefix();
  if (p_on) {
    send_data(commands::underline_cursor_on);
  } else {
    send_data(commands::underline_cursor_off);
  }
}

void nhd0420d3z::blinking_cursor(bool p_on)
{
  send_prefix();
  if (p_on) {
    send_data(commands::blinking_cursor_on);
  } else {
    send_data(commands::blinking_cursor_off);
  }
}

void nhd0420d3z::rotate_left() {
  send_prefix();
  send_data(commands::shift_display_left);
}

void nhd0420d3z::rotate_right() {
  send_prefix();
  send_data(commands::shift_display_right);
}

hal::byte nhd0420d3z::current_line() {
  return m_cursor_line;
}

hal::byte nhd0420d3z::current_column() {
  return m_cursor_column;
}
}  // namespace sjsu::drivers
