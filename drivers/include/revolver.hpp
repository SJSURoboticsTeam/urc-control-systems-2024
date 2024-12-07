#pragma once
#include <libhal-util/steady_clock.hpp>
#include <libhal-lpc40/clock.hpp>
#include <libhal-soft/rc_servo.hpp>
#include <libhal-lpc40/pwm.hpp>
// #include <libhal-pca/pca9685.hpp>
#include <libhal-lpc40/input_pin.hpp>
#include <libhal-util/units.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-lpc40/uart.hpp>


namespace sjsu::science {
   
    class revolver 
    {
        private:
            int m_numVials = 12;
            hal::degrees m_clockwise = hal::degrees(0.0);
            hal::degrees m_counterclockwise = hal::degrees(180.0);
            hal::degrees m_stop = hal::degrees(90.0);
            std::chrono::milliseconds m_delay = std::chrono::milliseconds(5);

            hal::servo& revolver_servo_my;
            hal::input_pin& input_pin_my;
            hal::steady_clock& steady_clock_my;
            hal::serial& terminal_my;

            void revolverState(hal::degrees rotationState);
            

        public:
            revolver(hal::servo& p_servo, hal::input_pin& p_input_pin, hal::steady_clock& p_steady_clock, hal::serial& p_terminal);
            void revolverMoveVials(int vial);
            
    };
}