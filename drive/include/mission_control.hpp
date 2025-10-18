#pragma once

namespace sjsu::drive{
    class mission_control{
        public:
            struct mc_commands{
                int homed = 0;
                int interpolate = 1;
                int interpolate_close = 1;
                int auto_fix = 1;
                int throw_then_stop = 1;
            };  

            hal::result<mc_commands> get_command(
                hal::function_ref<hal::timeout_function> p_timeout)
              {
                return impl_get_command(p_timeout);
              }
            
              virtual ~mission_control() = default;
        private:
            virtual hal::result<mc_commands> impl_get_command(
            hal::function_ref<hal::timeout_function> p_timeout) = 0;
        
    };
    
}

//homing first, then drive
//should not drive before home
//mission control send homing command and move command explicitly
//chasis vloecity, x velocity, y velocity, rotational velocity
//settings: (could be preset, constants)
//interpolate?:
//interpolate to closest? (or else throw)
//auto fix the position
//auto throw -> stop? 
