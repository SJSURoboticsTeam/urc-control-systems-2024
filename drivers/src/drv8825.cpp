#include "../include/drv8825.hpp"
#include <libhal/output_pin.hpp>

using namespace std::chrono_literals;


namespace sjsu::drivers {
drv8825::drv8825(
    hal::output_pin& p_direction_pin, 
    hal::output_pin& p_step_pin, 
    hal::steady_clock& p_steady_clock, 
    step_factor p_step_factor, 
    int p_steps_per_rotation, 
    hal::time_duration p_step_period,
    std::array<hal::output_pin*,3> p_mode_pins) 
    
    : m_direction_pin(p_direction_pin), 
    m_step_pin(p_step_pin), 
    m_clock(p_steady_clock) {
    
    m_mode_pins = p_mode_pins;
    set_step_factor(p_step_factor);
    m_partial_steps_to_deg = 360/(p_steps_per_rotation/32);
    m_step_period = p_step_period;
}

void drv8825::step(long p_steps) {
    m_partial_steps += p_steps * (32 >> static_cast<int>(m_step_factor));
    if (p_steps < 0) {
        p_steps *= -1;
        m_direction_pin.level(false);
    } else {
        m_direction_pin.level(true);
    }
    hal::delay(m_clock, 700ns);
    for (; p_steps > 0; p_steps--) {
        m_step_pin.level(true);
        hal::delay(m_clock, m_step_period);
        m_step_pin.level(false);
        hal::delay(m_clock, m_step_period);
    }
}

void drv8825::set_step_factor(step_factor p_step_factor){
    m_step_factor=p_step_factor;
    hal::byte mode = static_cast<hal::byte>(m_step_factor);
    if (mode & 0b001) {
        m_mode_pins[0]->level(true);
    } else {
        m_mode_pins[0]->level(false);
    }
    if (mode & 0b010) {
        m_mode_pins[1]->level(true);
    } else {
        m_mode_pins[1]->level(false);
    }
    if (mode & 0b100) {
        m_mode_pins[2]->level(true);
    } else {
        m_mode_pins[2]->level(false);
    }
}

long drv8825::get_partial_steps() {
    return m_partial_steps;
}

hal::degrees drv8825::get_position(){return m_partial_steps*m_partial_steps_to_deg;}

//can lose resolution
void drv8825::set_position(double position){m_partial_steps = position/m_partial_steps_to_deg;}

void drv8825::driver_position(hal::degrees p_position){
    long partial_step_difference = p_position/m_partial_steps_to_deg - m_partial_steps;
    step(partial_step_difference * static_cast<int>(m_step_factor));
}
};//name space drivers