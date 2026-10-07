#include <array>
#include <libhal-util/i2c.hpp>
#include <libhal/error.hpp>
#include <libhal/units.hpp>
#include <nhd0420d3z.hpp>

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
  move_display_one_place_to_the_left = 0x55,
  move_display_one_place_to_the_right = 0x56,
};
}

namespace sjsu::drivers {
void nhd0420d3z::send_data(hal::byte p_data)
{
  std::array<hal::byte, 1> command = { p_data };
  std::span<hal::byte> data_out {};
  m_i2c_bus.transaction(m_i2c_address, data_out, command);
}
void nhd0420d3z::send_prefix()
{
  send_data(commands::prefix);
}

void nhd0420d3z::advance_cursor_right()
{
  m_cursor_column++;
  if (m_cursor_column >= display_columns - 1) {
    m_cursor_line++;
    m_cursor_column = 0;
  }
  if (m_cursor_line >= display_lines - 1) {
    m_cursor_line = 0;
  }
}
void nhd0420d3z::write_char(char p_c)
{
  bool is_custom_char = p_c >= 0x00 && p_c < 0x07;
  if (is_custom_char) {
    // display custom_character behavior, unused
  }

  bool is_ascii = p_c >= 0x20 && p_c <= 0x7F;
  if (is_ascii) {
    // display standard ASCII characters
    send_data(p_c);
    advance_cursor_right();
  }

  bool is_japanese_char = p_c >= 0xA0 && p_c <= 0xFD;
  if (is_japanese_char) {
    // display japanese characters, factory-masked on the SPLC780D controller,
    // unused
  }
}

hal::byte nhd0420d3z::coordinates_to_position(hal::byte p_line,
                                              hal::byte p_column)
{
  hal::byte pos = p_line;
  pos += (p_column / 2) * display_lines;
  if (p_column % 2) {
    pos += 0x40;
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

void nhd0420d3z::move_cursor_right()
{
  m_cursor_line++;
  if (m_cursor_line >= display_columns - 1) {
    return;
  }
  send_prefix();
  send_data(commands::move_cursor_right_one_place);
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

nhd0420d3z::nhd0420d3z(hal::i2c& p_i2c, hal::byte p_i2c_address)
  : m_i2c_bus(p_i2c)
  , m_i2c_address(p_i2c_address)
{
}
void nhd0420d3z::display_message(std::string_view p_str)
{
  clear_screen();
  set_cursor_position(0, 0);
  auto strIt = p_str.begin();
  while (m_cursor_line < display_lines && strIt != p_str.end()) {
    if (*strIt == '\n') {
      m_cursor_line++;
      set_cursor_position(m_cursor_line, 0);
    } else {
      write_char(*strIt);
    }
    strIt++;
  }
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
}  // namespace sjsu::drivers
