#include <libhal/adc.hpp>
#include <libhal/input_pin.hpp>
#include <libhal/output_pin.hpp>
#include <tla2528.hpp>

namespace sjsu::drivers {

class tla2528_output_pin : public hal::output_pin
{
private:
  tla2528_output_pin(tla2528& p_tla2528,
                     hal::byte p_channel,
                     hal::output_pin::settings const& p_settings);
  void driver_configure(settings const& p_settings);
  void driver_level(bool p_high);
  bool driver_level();
  friend tla2528_output_pin make_output_pin(
    tla2528& p_tla2528,
    hal::byte p_channel,
    hal::output_pin::settings const& p_settings);
  tla2528* m_tla2528 = nullptr;
  hal::byte m_channel;

public:
  ~tla2528_output_pin();
};
tla2528_output_pin make_output_pin(tla2528& p_tla2528,
                                   hal::byte p_channel,
                                   hal::output_pin::settings const& p_settings);

class tla2528_input_pin : public hal::input_pin
{
private:
  tla2528_input_pin(tla2528& p_tla2528,
                    hal::byte p_channel,
                    hal::input_pin::settings const& p_settings);
  void driver_configure(settings const& p_settings);
  bool driver_level();
  friend tla2528_input_pin make_input_pin(
    tla2528& p_tla2528,
    hal::byte p_channel,
    hal::input_pin::settings const& p_settings);
  tla2528* m_tla2528 = nullptr;
  hal::byte m_channel;

public:
  ~tla2528_input_pin();
};
tla2528_input_pin make_input_pin(tla2528& p_tla2528,
                                 hal::byte p_channel,
                                 hal::input_pin::settings const& p_settings);

class tla2528_adc : public hal::adc
{
private:
  tla2528_adc(tla2528& p_tla2528, hal::byte p_channel);
  friend tla2528_adc make_adc(tla2528& p_tla2528, hal::byte p_channel);
  float driver_read();
  tla2528* m_tla2528 = nullptr;
  hal::byte m_channel;

public:
  ~tla2528_adc();
};
tla2528_adc make_adc(tla2528& p_tla2528, hal::byte p_channel);

}  // namespace sjsu::drivers