#include "../include/carousel.hpp"

namespace sjsu::science{
    carousel::carousel(hal::v5::strong_ptr<hal::actuator::rc_servo> p_servo) : m_servo(p_servo){
    }  
    void carousel::home(){
        current_pos = 0;
        (*m_servo).position(vial_pos[current_pos]);
    }
    void carousel::step_move(){
        current_pos++;
        if(current_pos>9) {
            current_pos=9;
            throw "Range Exceeded";
        }
        (*m_servo).position(vial_pos[current_pos]);
    }
    void carousel::step_move(int turns){
        current_pos+=turns;
        if(current_pos>9) {
            current_pos=9;
            throw "Range Exceeded";
        }
        (*m_servo).position(vial_pos[current_pos]);
    }
    void carousel::step_backward(){
        current_pos--;
        if(current_pos<0) {
            current_pos = 0;
            throw "Range Exceeded";
        }
        (*m_servo).position(vial_pos[current_pos]);
    }
    void carousel::step_backward(int turns){
        current_pos-=turns;
        if(current_pos<0) {
            current_pos = 0;
            throw "Range Exceeded";
        }
        (*m_servo).position(vial_pos[current_pos]);
    }
}