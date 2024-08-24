#include "../include/science_state_machine.hpp"
#include <applications/application.hpp>

using namespace std::chrono_literals;

namespace sjsu::science{

    science_state_machine::science_state_machine(hardware_map_t& application )  : hardware(application){
        m_count = 0;
        vials_used = 0;
    }
   
    void science_state_machine::run_state_machine(science_state_machine::science_states state){
        switch(state){
            case science_state_machine::science_states::GET_SAMPLES:
                // sm_m_status.is_sample_finished=0;
                mix_solution();
                turn_on_pump(pump_manager::pumps::DEIONIZED_WATER, 5000ms);
                turn_on_pump(pump_manager::pumps::SAMPLE, 5000ms);
                move_sample(1);
                vials_used++;
                // sm_m_status.num_vials_used++;
                m_count++;
                turn_on_pump(pump_manager::pumps::SAMPLE, 5000ms);
                break; 
            case science_state_machine::science_states::MOLISCH_TEST:
                turn_on_pump(pump_manager::pumps::MOLISCH_REAGENT, 5000ms);
                move_sample(1);
                m_count++;
                turn_on_pump(pump_manager::pumps::SULFURIC_ACID, 5000ms);
                move_sample(2);
                m_count = m_count+2;
                break;
            case science_state_machine::science_states::BIURET_TEST:
                turn_on_pump(pump_manager::pumps::BIURET_REAGENT, 5000ms);
                m_count++;
                break;
            case science_state_machine::science_states::RESET:
                containment_reset();
                // sm_m_status.is_sample_finished=1;
                break; 
        }        
    }

    void science_state_machine::turn_on_pump( auto pump, hal::time_duration time){
        auto pump_controller = *hardware.pump_controller;
        pump_controller.pump(pump, time);
    }

    void science_state_machine::mix_solution(){
        // hardware.mixing_servo.velocity_control(10.0 rpm);
        // hal::delay(hardware.steady_clock, 5000ms);
    }

    void science_state_machine::containment_reset(){
        auto revolver_controller = *hardware.revolver_controller;
        revolver_controller.revolverMoveVials(m_count - 2);
        m_count = 0;
    }

    void science_state_machine::move_sample(int position){
        auto revolver_controller = *hardware.revolver_controller; 
        revolver_controller.revolverMoveVials(position);
    }

    // mission_control::status science_state_machine::get_status(){
    //     return sm_m_status;
    // }

    int science_state_machine::get_num_vials_left (){
        return 12-vials_used; 
    }
  
}


