#pragma once

#include <libhal-util/i2c.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>

namespace sjsu::drivers {
// functions we ened to make
class opt4048
{
// private:
 

  // constructor

public:
  enum register_addresses : hal::byte {
        channel0_msb = 0x00,
        channel0_lsb = 0x01,
        channel1_msb = 0x02,
        channel1_lsb = 0x03,
        channel2_msb = 0x04,
        channel2_lsb = 0x05,
        channel3_msb = 0x06,
        channel3_lsb = 0x07,
        threshold_l = 0x08, //optional 
        threshold_h = 0x09, //optional
        settings = 0x0A,
        threshold_channel_select = 0x0B,
        flags = 0x0C,
        // device_id = 0x11
    };
    enum modes: hal::byte{
        power_down =0b00,
        forced_auto_range = 0b01,
        one_shot =0b10,
        continuous = 0b11, 
    };

    enum conversion_times: int{
        micro600 = 0b0000, 
        ms1 = 0b0001,
        ms1_8 = 0b0010, 
        ms3_4 = 0b0011,
        ms6_5 = 0b0100,
        ms12_7 = 0b0101,
        ms25 = 0b0110,
        ms50 = 0b0111,
        ms100 = 0b1000,
        ms200 = 0b1001,
        ms400 = 0b1010,
        ms800 = 0b1011
    };

    struct adc_codes
    {
        uint32_t ch0 = 0;
        uint32_t ch1 = 0;
        uint32_t ch2 = 0;
        uint32_t ch3 = 0;
    };

  struct xyz_values
  {
    float x = 0;
    float y = 0;
    float z = 0;
    float lux = 0;
  };

  struct rgb_values
  {
    float r = 0;
    float g = 0;
    float b = 0;
  };
  float XYZ_to_RGB[3][3] = { {  3.2404542, -1.5371385, -0.4985314 },
												{ -0.9692660,  1.8760108,  0.0415560 },
												{  0.0556434, -0.2040259,  1.0572252 }};
  int device_address = 0x44;  // not sure check this
  
  opt4048(hal::i2c& p_i2c,hal::steady_clock& p_clock,hal::serial& p_terminal);

  // select light range
  // select conversiokn time
  // measure light and color
  // choose mode of operation
  // set automatic full scale range setting, continuous mode, conversion time
  // somewhere in the middle

  // set register range on 0xA to 0xC
  void adjust_settings();  // keep it editable to see which performs optimally in the future

  /**
   * @brief Takes in channel address and returns 2 bytes that were read
   * @param channel_address
   * @param buffer
   *
   */
  void read_channel(hal::byte address, std::span<hal::byte> output);

  adc_codes get_adc_codes();

  xyz_values adc_codes_to_xyz(adc_codes adc);
  
  float sRGBCompandingFunction(float val);

  rgb_values xyz_to_rgb(xyz_values xyz);

  rgb_values get_data();  // will do all the conversions and return required data

  void get_msb_exp(hal::byte register_A, std::span<uint16_t> data);
  // set to conversion mode
    // M^-1 for sRGB @ D65 from www.brucelindbloom.com
	

  hal::i2c& m_i2c;
  hal::steady_clock& m_clock;
  hal::serial& m_terminal;
};
}  // namespace drivers