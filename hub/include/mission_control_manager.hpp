#pragma once 

#include <array>
#include <cstdint>
#include <libhal-util/can.hpp>
#include <libhal/can.hpp>
#include <libhal/pointers.hpp>
#include <optional>

namespace sjsu::hub{
    struct int16_axis{
        int16_t x;
        int16_t y;
        int16_t z;
    };

    enum class gimbal_direction: uint8_t{
        left = 0x00,
        right = 0x01, 
        up = 0x02,
        down = 0x03,
    };

    struct gimbal_move_request{
        gimbal_direction direction;
        uint8_t offset; 
    };

    class mission_control_manager{
        public: 
            mission_control_manager(hal::v5::strong_ptr<hal::can_transceiver> p_can_transceiver);
            
            std::array<hal::byte, 2> int16_to_byte_array(int16_t p_num);

            std::optional<gimbal_move_request> read_gimbal_move_request();

            void reply_imu_accel_request(int16_axis accel);
            void reply_imu_gyro_request(int16_axis gyro);
            void reply_imu_mag_request(int16_axis mag);
 
            void clear_heartbeat_requests(); 
            bool reply_heartbeat(uint8_t imu_status, uint8_t lcd_status);

        private:
            hal::v5::strong_ptr<hal::can_transceiver> m_can_transceiver;
            hal::can_message_finder m_heartbeat_message_finder;
            hal::can_message_finder m_gimbal_message_finder;
    };
} //namespace sjsu::hub