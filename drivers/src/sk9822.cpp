#pragma once
#include <array>
#include <libhal/units.hpp>
#include <libhal/steady_clock.hpp>
#include <libhal/output_pin.hpp>
#include <libhal-util/steady_clock.hpp>
#include "../include/sk9822.hpp"

using namespace hal::literals;
using namespace std::chrono_literals;

namespace sjsu::drivers {

namespace light_strip_util {
    void set_all(light_strip_view lights, const hal::byte r, const hal::byte g, const hal::byte b, const hal::byte brightness) {
        rgb_brightness setting;
        setting.r = r;
        setting.g = g;
        setting.b = b;
        setting.brightness = brightness;
        for(auto i = lights.begin(); i != lights.end(); i ++) {
            *i = setting;
        }
    }

    void set_all(light_strip_view lights, const rgb_brightness value) {
        for(auto i = lights.begin(); i != lights.end(); i ++) {
            *i = value;
        }
    }
};

sk9822::sk9822(hal::output_pin& p_clock_pin, hal::output_pin& p_data_pin, hal::steady_clock& p_clock) {
    clock_pin = &p_clock_pin;
    data_pin = &p_data_pin;
    clock = &p_clock;
}

void sk9822::update(light_strip_view lights) {
    // Start Frame
    send_byte(0x00);
    send_byte(0x00);
    send_byte(0x00);
    send_byte(0x00);

    for(auto i = lights.begin(); i != lights.end(); i ++) {
        send_byte((*i).brightness | 0b11100000);
        // send_byte((*i).brightness & 0x00011111);
        send_byte((*i).b);
        send_byte((*i).g);
        send_byte((*i).r);
    }

    // End Frame
    send_byte(0xff);
    send_byte(0xff);
    send_byte(0xff);
    send_byte(0xff);
}

void sk9822::send_byte(hal::byte data) {
    for(int i = 0; i < 8; i ++) {
    if(data & (1 << i)) {
      (*data_pin).level(true);
    }else {
      (*data_pin).level(false);
    }
    hal::delay(*clock, half_period);
    (*clock_pin).level(true);
    hal::delay(*clock, period);
    (*clock_pin).level(false);    
    hal::delay(*clock, half_period);
  }

}

};