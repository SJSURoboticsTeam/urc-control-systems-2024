#pragma once
#include <array>
#include <span>
#include <libhal/units.hpp>
#include <libhal/steady_clock.hpp>
#include <libhal/output_pin.hpp>
#include <libhal-util/steady_clock.hpp>


using namespace hal::literals;
using namespace std::chrono_literals;

namespace sjsu::drivers {

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
using light_strip = std::array<rgb_brightness, n_leds>;
using light_strip_view = std::span<rgb_brightness>;

namespace light_strip_util {
    void set_all(light_strip_view lights, const hal::byte r, const hal::byte g, const hal::byte b, const hal::byte brightness);
    void set_all(light_strip_view lights, const rgb_brightness value);
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
         */
        void update(light_strip_view lights);
    private:
        hal::output_pin* clock_pin, *data_pin;
        hal::steady_clock* clock;

        /**
         * @brief Send a byte through the clock pin and data pins
         * 
         * @param value 
         */
        void send_byte(hal::byte value);
};
};