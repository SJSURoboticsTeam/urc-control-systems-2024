#include "revolver.hpp"


namespace sjsu::science {

    revolver::revolver(hal::servo& p_servo, hal::input_pin& p_input_pin, hal::steady_clock& p_steady_clock, hal::serial& p_terminal)
        : revolver_servo_my(p_servo), input_pin_my(p_input_pin), steady_clock_my(p_steady_clock), terminal_my(p_terminal)
    {
    }

    hal::result<revolver> revolver::create(hal::servo& p_servo, hal::input_pin& p_input_pin, hal::steady_clock& p_steady_clock, hal::serial& p_terminal) 
    {
        revolver revolver(p_servo, p_input_pin, p_steady_clock, p_terminal);
        return revolver;
    }

    hal::status revolver::revolverState(hal::degrees rotationState) 
    {
        HAL_CHECK(revolver_servo_my.position(rotationState));
        return hal::success();
    }

    hal::status revolver::revolverMoveVials(int vial) 
    {   
        if (vial != 0 && ((vial > 0 && vial <= m_numVials) || (vial < 0 && -vial <= m_numVials)))
        {
            int count = -1; // account for initial state change
            bool hallState;
            bool hallStateDelayed;
            
            revolverState((vial > 0) ? m_clockwise : m_counterclockwise);
            vial = (vial > 0) ? vial : -vial;

            while (count < vial) 
            {
                hallState = input_pin_my.level().value().state;
                hal::delay(steady_clock_my, m_delay);
                hallStateDelayed = input_pin_my.level().value().state;

                if (hallState != hallStateDelayed)
                {
                    ++count;
                }
            }

            revolverState(m_stop);
        }
        else
        {
            hal::print<1024>(terminal_my, "Vials can only be moved from %d to %d", -m_numVials, m_numVials);
        }
        
        return hal::success();
    }
}     