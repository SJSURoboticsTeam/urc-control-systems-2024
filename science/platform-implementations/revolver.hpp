#pragma once
#include <libhal-util/steady_clock.hpp>
#include <libhal-lpc40/clock.hpp>
#include <libhal-soft/rc_servo.hpp>
#include <libhal-lpc40/pwm.hpp>
#include <libhal-pca/pca9685.hpp>
#include <libhal-lpc40/input_pin.hpp>
#include <libhal-util/units.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-lpc40/uart.hpp>


namespace sjsu::science {
    // Class:
    //      private:
    //              member variables
    //              reference types of objects required for class
    //              member functions aka methods
    //              empty dummy constructor, but the reference types of objects will be assigned to the inputs in .cpp 
    //      public:
    //              static create method with hal::result wrapper
    //              member functions aka methods

    // Encapsulation:
    //      1. Header File
    //              a. Private = Implementation Details
    //              b. Public = Interface
    //      2. CPP File - Implementation, Actual
    //      3. Dummy Constructor + Create to Separate:
    //              a. Object Creation into Memory
    //              b. Initialization Logic + Error Handling
    //                  i. Error Handling is standardized via HAL
    //      4. application_framework - struct that acts as interface for application.cpp to access lpc4078 hardware
    //      5. application.cpp - Main application logic that interfaces with microcontrollers / higher-level modules
    //              - The implementation of more basic sensors / actuators managed by the microcontroller 
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

            hal::status revolverState(hal::degrees rotationState);
            revolver(hal::servo& p_servo, hal::input_pin& p_input_pin, hal::steady_clock& p_steady_clock, hal::serial& p_terminal);

        public:
            static hal::result<revolver> create(hal::servo& p_servo, hal::input_pin& p_input_pin, hal::steady_clock& p_steady_clock, hal::serial& p_terminal);                             
            hal::status revolverMoveVials(int vial);
    };
}