#pragma once
#include <libhal/output_pin.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/pointers.hpp>


namespace sjsu::science {

    class pump_manager {
        private:
            hal::v5::strong_ptr<hal::steady_clock>  m_clock;
            std::array<hal::v5::strong_ptr<hal::output_pin>,4> m_pumps;
            
           
        public:
            enum class pumps{
                DEIONIZED_WATER=0,
                BENEDICT_REAGENT,
                BIURET_REAGENT,
                KALLING_REAGENT
            };
            pump_manager(
                hal::v5::strong_ptr<hal::steady_clock> p_clock,
                hal::v5::strong_ptr<hal::output_pin> p_deionized_water_pump, 
                hal::v5::strong_ptr<hal::output_pin> p_benedict_reagent_pump, 
                hal::v5::strong_ptr<hal::output_pin> p_biuret_reagent,
                hal::v5::strong_ptr<hal::output_pin> p_kalling_reagent_pump 
            );
            void pump(pumps pump, hal::time_duration duration);
    };

}