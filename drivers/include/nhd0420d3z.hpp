
#include <libhal-util/serial.hpp>
#include <libhal/i2c.hpp>
#include <libhal/units.hpp>
namespace sjsu::drivers {

class nhd0420d3z
{
private:
  constexpr static hal::byte default_i2c_address = 0x28;
  constexpr static hal::byte display_lines = 4;
  constexpr static hal::byte display_columns = 20;

  hal::byte coordinates_to_position(hal::byte line, hal::byte column);

public:
  void send_data(hal::byte data);
  void send_prefix();
  void write_char(char c);
  void set_cursor_position(hal::byte line, hal::byte column);
  void move_cursor_right();
  void home_cursor();
  void clear_screen();

private:
  hal::i2c& m_i2c_bus;
  hal::byte m_i2c_address;
  hal::byte m_cursor_line = 0;
  hal::byte m_cursor_column = 0;

public:
  nhd0420d3z(hal::i2c& p_i2c, hal::byte p_i2c_address = default_i2c_address);

  void display_message(std::string_view str);
  void power(bool on);
};

}  // namespace sjsu::drivers