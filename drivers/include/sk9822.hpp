#pragma once
#include <array>
#include <libhal/units.hpp>
#include <libhal/steady_clock.hpp>
#include <libhal/output_pin.hpp>
#include <libhal-util/steady_clock.hpp>


using namespace hal::literals;
using namespace std::chrono_literals;

namespace hal {

/**
 * @brief Stores r, g, b, and brightness values.
 * 
 */
struct rgb_brightness {
    hal::byte r=0, g=0, b=0;
    hal::byte brightness=0;

    rgb_brightness() {};
    rgb_brightness(hal::byte p_r, hal::byte p_g, hal::byte p_b, hal::byte p_brightness) {
        r = p_r;
        g = p_g;
        b = p_b;
        brightness = p_brightness;
    }

    /**
     * @brief Set the r, g, b, and brightness values.
     * 
     * @param p_r 
     * @param p_g 
     * @param p_b 
     * @param p_brightness 
     */
    void set(hal::byte p_r, hal::byte p_g, hal::byte p_b, hal::byte p_brightness) {
        r = p_r;
        g = p_g;
        b = p_b;
        brightness = p_brightness;
    }
};

namespace colors {
    const rgb_brightness RED = rgb_brightness(0xff, 0x00, 0x00, 0b11111);
    const rgb_brightness GREEN = rgb_brightness(0x00, 0xff, 0x00, 0b11111);
    const rgb_brightness BLUE = rgb_brightness(0x00, 0x00, 0xff, 0b11111);
    const rgb_brightness WHITE = rgb_brightness(0xff, 0xff, 0xff, 0b11111);
    const rgb_brightness BLACK = rgb_brightness(0x00, 0x00, 0x00, 0x00);
}

template<std::size_t n_leds>
using light_strip = std::array<hal::rgb_brightness, n_leds>;
using light_strip_view = std::span<hal::rgb_brightness>;

namespace light_strip_util {
    void set_all(hal::light_strip_view lights, const hal::byte r, const hal::byte g, const hal::byte b, const hal::byte brightness) {
        rgb_brightness setting;
        setting.r = r;
        setting.g = g;
        setting.b = b;
        setting.brightness = brightness;
        for(auto i = lights.begin(); i != lights.end(); i ++) {
            *i = setting;
        }
    }

    void set_all(hal::light_strip_view lights, const rgb_brightness value) {
        for(auto i = lights.begin(); i != lights.end(); i ++) {
            *i = value;
        }
    }
};

struct sk9822 {
    public:
        constexpr static auto period = 8ns;
        constexpr static auto half_period = period/2;
        /**
         * @brief Construct a new sk9822 object.
         * 
         * @param clock_pin 
         * @param data_pin 
         * @param clock 
         */
        sk9822(hal::output_pin& clock_pin, hal::output_pin& data_pin, hal::steady_clock& clock);

        /**
         * @brief Send the updated rgb_brightness values to the light strips.
         * Changes to led brightness are only reflected when this is called.
         * 
         * @return hal::status 
         */
        hal::status update(hal::light_strip_view lights);
    private:
        hal::output_pin* clock_pin, *data_pin;
        hal::steady_clock* clock;

        /**
         * @brief Send a byte through the clock pin and data pins
         * 
         * @param value 
         * @return hal::status 
         */
        hal::status send_byte(hal::byte value);
};
};

hal::sk9822::sk9822(hal::output_pin& p_clock_pin, hal::output_pin& p_data_pin, hal::steady_clock& p_clock) {
    clock_pin = &p_clock_pin;
    data_pin = &p_data_pin;
    clock = &p_clock;
}

hal::status hal::sk9822::update(hal::light_strip_view lights) {
    // Start Frame
    HAL_CHECK(send_byte(0x00));
    HAL_CHECK(send_byte(0x00));
    HAL_CHECK(send_byte(0x00));
    HAL_CHECK(send_byte(0x00));

    for(auto i = lights.begin(); i != lights.end(); i ++) {
        HAL_CHECK(send_byte((*i).brightness | 0b11100000));
        // HAL_CHECK(send_byte((*i).brightness & 0x00011111));
        HAL_CHECK(send_byte((*i).b));
        HAL_CHECK(send_byte((*i).g));
        HAL_CHECK(send_byte((*i).r));
    }

    // End Frame
    HAL_CHECK(send_byte(0xff));
    HAL_CHECK(send_byte(0xff));
    HAL_CHECK(send_byte(0xff));
    HAL_CHECK(send_byte(0xff));

    return hal::success();
}

hal::status hal::sk9822::send_byte(hal::byte data) {
    for(int i = 0; i < 8; i ++) {
    if(data & (1 << i)) {
      HAL_CHECK((*data_pin).level(true));
    }else {
      HAL_CHECK((*data_pin).level(false));
    }
    hal::delay(*clock, half_period);
    HAL_CHECK((*clock_pin).level(true));
    hal::delay(*clock, period);
    HAL_CHECK((*clock_pin).level(false));    
    hal::delay(*clock, half_period);
  }

  return hal::success();
}
