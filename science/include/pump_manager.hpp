#pragma once
#include <libhal/output_pin.hpp>
#include <libhal-util/steady_clock.hpp>

namespace sjsu::science {

    class pump_manager {
        private:
            std::array<hal::output_pin*,5> m_pumps;
            hal::steady_clock& m_clock;
           
        public:
            enum class pumps{
                DEIONIZED_WATER=0,
                SAMPLE,
                MOLISCH_REAGENT,
                SULFURIC_ACID,
                BIURET_REAGENT
            };
            pump_manager(
                hal::steady_clock& p_clock,
                hal::output_pin& p_deionized_water_pump, 
                hal::output_pin& p_sample_pump, 
                hal::output_pin& p_molisch_reagent_pump, 
                hal::output_pin& p_sulfuric_acid_pump, 
                hal::output_pin& p_biuret_reagent
            );
            void pump(pumps pump, hal::time_duration duration);
    };

}