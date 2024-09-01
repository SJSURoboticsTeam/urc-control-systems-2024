#include "../include/pump_manager.hpp"

namespace sjsu::science {

    pump_manager::pump_manager(
        hal::steady_clock& p_clock,
        hal::output_pin& p_deionized_water_pump, 
        hal::output_pin& p_sample_pump, 
        hal::output_pin& p_molisch_reagent_pump, 
        hal::output_pin& p_sulfuric_acid_pump, 
        hal::output_pin& p_biuret_reagent
    ) : m_clock(p_clock) {
        m_pumps[0] = &p_deionized_water_pump;
        m_pumps[1] = &p_sample_pump;
        m_pumps[2] = &p_molisch_reagent_pump;
        m_pumps[3] = &p_sulfuric_acid_pump;
        m_pumps[4] = &p_biuret_reagent;
    };


    void pump_manager::pump(pumps pump, hal::time_duration duration) {
        (*(m_pumps[static_cast<int>(pump)])).level(true);
        hal::delay(m_clock, duration);
        (*(m_pumps[static_cast<int>(pump)])).level(false);
    }

}