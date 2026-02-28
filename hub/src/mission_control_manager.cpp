#include <resource_list.hpp>
#include <array>
#include <cstdint>
#include <libhal-util/can.hpp>
#include <libhal-util/serial.hpp>
#include <libhal/can.hpp>
#include <libhal/units.hpp> 
#include <bit>
#include <mission_control_manager.hpp>
#include <optional>
 
namespace{
    enum class can_message_id : uint32_t{
        heartbeat = 0x0E,
        heartbeat_reply = 0x0F,
        set_gimbal_offset = 0x300,
        imu_accel_reply = 0x301,
        imu_gyro_reply = 0x302,
        imu_mag_reply = 0x303, 
    };
}

namespace sjsu::hub{
    mission_control_manager::mission_control_manager(hal::v5::strong_ptr<hal::can_transceiver> p_can_transceiver):
    m_can_transceiver(p_can_transceiver),
    m_heartbeat_message_finder(hal::can_message_finder(*m_can_transceiver, static_cast<uint32_t> (can_message_id::heartbeat))),
    m_gimbal_message_finder(hal::can_message_finder(*m_can_transceiver, static_cast<uint32_t> (can_message_id::set_gimbal_offset)))
    {
    }

    std::array<hal::byte, 2> mission_control_manager::int16_to_byte_array(int16_t p_num){
        uint16_t unum = std::bit_cast<uint16_t> (p_num);
        std::array<hal::byte, 2> byte_array{static_cast<uint8_t> (unum >> 8), static_cast<uint8_t> (unum)}; 
        return byte_array;
    }

    std::optional<gimbal_move_request> mission_control_manager::read_gimbal_move_request(){
        auto console = resources::console();
        std::optional<hal::can_message> gimbal_request_message;
        while(true){
            std::optional<hal::can_message> check_gimbal_request_message =
            m_gimbal_message_finder.find();
            if(check_gimbal_request_message){
                hal::print(*console, "message passed");

                if(check_gimbal_request_message-> length == 2){
                    gimbal_request_message = check_gimbal_request_message;
                }
            }
            else{
                break;
            }
        }
        if(not gimbal_request_message){
            return std::nullopt;
        }

        auto& payload = gimbal_request_message->payload;
        
        gimbal_move_request gmr = {
            .direction = static_cast<gimbal_direction>(payload[0]),
            .offset = payload[1],
        };

        hal::print<64> (*console, "direction =0x%X, offset =0x%X", payload[0], payload[1]);

        return gmr;
    }

    void mission_control_manager::reply_imu_accel_request(int16_axis accel){
        std::array<hal::byte, 8> accel_can_payload{
            static_cast<uint8_t>(accel.x & 0xFF), 
            static_cast<uint8_t>((static_cast<uint16_t>(accel.x) >> 8) & 0xFF),
            static_cast<uint8_t>(accel.y & 0xFF),
            static_cast<uint8_t>((static_cast<uint16_t>(accel.y) >> 8) & 0xFF),
            static_cast<uint8_t>(accel.z & 0xFF),
            static_cast<uint8_t>((static_cast<uint16_t>(accel.z) >> 8) & 0xFF)
        };
        hal::can_message reply{
            .id = static_cast<uint32_t>(can_message_id::imu_accel_reply),
            .length = 6,
            .payload = accel_can_payload};
        m_can_transceiver->send(reply);
    }

    void mission_control_manager::reply_imu_gyro_request(int16_axis gyro){
        std::array<hal::byte, 8> gyro_can_payload{
            static_cast<uint8_t>(gyro.x & 0xFF), 
            static_cast<uint8_t>((static_cast<uint16_t>(gyro.x) >> 8) & 0xFF),
            static_cast<uint8_t>(gyro.y & 0xFF),
            static_cast<uint8_t>((static_cast<uint16_t>(gyro.y) >> 8) & 0xFF),
            static_cast<uint8_t>(gyro.z & 0xFF),
            static_cast<uint8_t>((static_cast<uint16_t>(gyro.z) >> 8) & 0xFF)
        };
        hal::can_message reply{
            .id = static_cast<uint32_t>(can_message_id::imu_gyro_reply),
            .length = 6,
            .payload = gyro_can_payload};
        m_can_transceiver->send(reply);
    }

    void mission_control_manager::reply_imu_mag_request(int16_axis mag){
        std::array<hal::byte, 8> mag_can_payload{
            static_cast<uint8_t>(mag.x & 0xFF), 
            static_cast<uint8_t>((static_cast<uint16_t>(mag.x) >> 8) & 0xFF),
            static_cast<uint8_t>(mag.y & 0xFF),
            static_cast<uint8_t>((static_cast<uint16_t>(mag.y) >> 8) & 0xFF),
            static_cast<uint8_t>(mag.z & 0xFF),
            static_cast<uint8_t>((static_cast<uint16_t>(mag.z) >> 8) & 0xFF)
        };
        hal::can_message reply{
            .id = static_cast<uint32_t>(can_message_id::imu_mag_reply),
            .length = 6,
            .payload = mag_can_payload};
        m_can_transceiver->send(reply);
    }

    void mission_control_manager::clear_heartbeat_requests(){
        while(m_heartbeat_message_finder.find())
        ;
    }

    bool mission_control_manager::reply_heartbeat(uint8_t imu_status, uint8_t lcd_status){
        if(m_heartbeat_message_finder.find()){
            m_can_transceiver->send(
                {.id = static_cast<uint32_t> (can_message_id::heartbeat_reply),
                .length = 2,
                .payload = {imu_status, lcd_status}}
            );
            return true;
        }
        else{
            return false;
        }
    }
}


