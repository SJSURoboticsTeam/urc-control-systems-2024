#include "../include/carousel.hpp"


namespace sjsu::science {

    carousel::carousel(hal::v5::strong_ptr<hal::actuator::rc_servo> p_servo) : m_carousel_servo(p_servo)
    {

    }

    void carousel::home()
    {
        degrees = 0;
        m_carousel_servo->position(degrees);
    }

    void carousel::step()
    {
        degrees += 20;
        m_carousel_servo->position(degrees);
    }

    void carousel::step(int n)
    {
        degrees += 20 * n;
        m_carousel_servo->position(degrees);
    }

    void carousel::step_backwards()
    {
        degrees -= 20;
        m_carousel_servo->position(degrees);
    }

    void carousel::step_backwards(int n)
    {
        degrees -= 20 * n;
        m_carousel_servo->position(degrees);
    }

}
