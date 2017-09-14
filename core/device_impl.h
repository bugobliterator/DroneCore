#pragma once

#include "global_include.h"
#include "mavlink_include.h"
#include "mavlink_parameters.h"
#include "mavlink_commands.h"
#include <cstdint>
#include <functional>
#include <atomic>
#include <vector>
#include <map>
#include <thread>
#include <mutex>

namespace dronecore {

class DroneCoreImpl;


class DeviceImpl
{
public:
    DeviceImpl() {}
    virtual ~DeviceImpl() {}
    virtual void process_mavlink_message(const mavlink_message_t &) {}

    typedef std::function<void(const mavlink_message_t &)> mavlink_message_handler_t;

    typedef std::function<void()> timeout_handler_t;

    virtual void register_mavlink_message_handler(uint16_t , mavlink_message_handler_t ,
                                          const void *) {}

    virtual void unregister_all_mavlink_message_handlers(const void *) {}

    virtual void register_timeout_handler(timeout_handler_t ,
                                  double ,
                                  const void *) {}

    virtual void refresh_timeout_handler(const void *) {}

    virtual void unregister_timeout_handler(const void *) {}

    virtual bool send_message(const mavlink_message_t &) { return false; }

    virtual MavlinkCommands::Result send_command_with_ack(uint16_t ,
                                                  const MavlinkCommands::Params &,
                                                  uint8_t component_id = 0)
    {   
        (void)component_id;
        return MavlinkCommands::Result::NO_DEVICE;
    }

    virtual MavlinkCommands::Result set_mode_with_ack(uint32_t , uint8_t ,
                                                      uint8_t component_id = 0)
    {
        (void)component_id;
        return MavlinkCommands::Result::NO_DEVICE;
    }

    typedef std::function<void(MavlinkCommands::Result, float)> command_result_callback_t;
    virtual void send_command_with_ack_async(uint16_t , const MavlinkCommands::Params &,
                                     command_result_callback_t ,
                                     uint8_t component_id = 0) { (void)component_id; }


    virtual void set_mode_with_ack_async(uint32_t , uint8_t ,
                                 command_result_callback_t ,
                                 uint8_t component_id = 0) { (void)component_id; }

    virtual MavlinkCommands::Result set_msg_rate(uint16_t , double ) 
    { return MavlinkCommands::Result::NO_DEVICE; }

    virtual void set_msg_rate_async(uint16_t , double ,
                            command_result_callback_t ) {}

    virtual void request_autopilot_version() {}

    virtual uint64_t get_target_uuid() const { return 0; }
    virtual uint8_t get_target_system_id() const { return 128; }
    virtual uint8_t get_target_component_id() const { return 128; }

    virtual void set_target_system_id(uint8_t ) {}

    virtual bool target_supports_mission_int() const { return false; }

    virtual bool is_armed() const { return false; }

    typedef std::function <void(bool success)> success_t;
    virtual void set_param_float_async(const std::string &, float , success_t ) {}
    virtual void set_param_int_async(const std::string &, int32_t , success_t ) {}
    virtual void set_param_ext_float_async(const std::string &, float , success_t ) {}
    virtual void set_param_ext_int_async(const std::string &, int32_t , success_t ) {}

    typedef std::function <void(bool success, float value)> get_param_float_callback_t;
    typedef std::function <void(bool success, int32_t value)> get_param_int_callback_t;

    virtual void get_param_float_async(const std::string &, get_param_float_callback_t ) {}
    virtual void get_param_int_async(const std::string &, get_param_int_callback_t ) {}
    virtual void get_param_ext_float_async(const std::string &, get_param_float_callback_t ) {}
    virtual void get_param_ext_int_async(const std::string &, get_param_int_callback_t ) {}

    static uint8_t get_own_system_id() { return _own_system_id; }
    static uint8_t get_own_component_id() { return _own_component_id; }

    // Non-copyable
    DeviceImpl(const DeviceImpl &) = delete;
    const DeviceImpl &operator=(const DeviceImpl &) = delete;
    virtual uint8_t get_device_type() const { return _device_type; }

protected:

    // TODO: should our own system ID have some value?
    static constexpr uint8_t _own_system_id = 0;
    static constexpr uint8_t _own_component_id = MAV_COMP_ID_SYSTEM_CONTROL;
    uint8_t _device_type;
};


} // namespace dronecore
