#include "science_state_machine.hpp"
#include "../applications/application.hpp"

using namespace std::chrono_literals;

namespace sjsu::science{

    science_state_machine::science_state_machine(sjsu::science::application_framework& application, sjsu::science::mission_control::status& status) : hardware(application), m_count(0), sm_m_status(status) {}
// pass in mission control status intialize m_status
    hal::result<science_state_machine> science_state_machine::create(sjsu::science::application_framework& p_application, sjsu::science::mission_control::status& p_status){
        science_state_machine science_state_machine(p_application, p_status);
        return science_state_machine;
     }

    hal::status science_state_machine::run_state_machine(science_state_machine::science_states state){
        switch(state){
            case science_state_machine::science_states::GET_SAMPLES:
                sm_m_status.is_sample_finished=0;
                mix_solution();
                turn_on_pump(pump_manager::pumps::DEIONIZED_WATER, 5000ms);
                turn_on_pump(pump_manager::pumps::SAMPLE, 5000ms);
                move_sample(1);
                sm_m_status.num_vials_used++;
                m_count++;
                turn_on_pump(pump_manager::pumps::SAMPLE, 5000ms);
                sm_m_status.num_vials_used++;
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
                sm_m_status.is_sample_finished=1;
                break; 
        }
        return hal::success();
        
    }

    hal::status science_state_machine::turn_on_pump( auto pump, hal::time_duration time){
        auto pump_controller = *hardware.pump_controller;
        pump_controller.pump(pump, time);
        return hal::success();
    }

    hal::status science_state_machine::mix_solution(){
        // hardware.mixing_servo.velocity_control(10.0 rpm);
        // hal::delay(hardware.steady_clock, 5000ms);
        return hal::success();
    }

    hal::status science_state_machine::containment_reset(){
        auto revolver_controller = *hardware.revolver_controller;
        revolver_controller.revolverMoveVials(m_count - 2);
        m_count = 0;
        return hal::success();
    }

    hal::status science_state_machine::move_sample(int position){
        auto revolver_controller = *hardware.revolver_controller; 
        revolver_controller.revolverMoveVials(position);
        return hal::success(); 
    }

    mission_control::status science_state_machine::get_status(){
        return sm_m_status;
    }

    int science_state_machine::get_num_vials_left (){
        return 12-sm_m_status.num_vials_used; 
    }
  
}


