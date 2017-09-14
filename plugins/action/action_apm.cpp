#include "global_include.h"
#include "action_apm.h"
#include "device_apm.h"
#include "dronecore_impl.h"
#include "telemetry.h"
#include "apm_custom_mode.h"

namespace dronecore {

using namespace std::placeholders; // for `_1`

APMActionImpl::APMActionImpl() :
    _in_air_state_known(false),
    _in_air(false)
{
}

APMActionImpl::~APMActionImpl()
{

}

void APMActionImpl::init()
{
    // We need the system state.
    ((APMDeviceImpl*)_parent)->register_mavlink_message_handler(
        MAVLINK_MSG_ID_HEARTBEAT,
        std::bind(&APMActionImpl::process_sys_state, this, _1), (void *)this);

    // And we need to make sure the system state is actually sent.
    // We use the async call here because we should not block in the init call because
    // we won't receive an answer anyway in init because the receive loop is not
    // called while we are being created here.
    //((APMDeviceImpl*)_parent)->set_msg_rate_async(MAVLINK_MSG_ID_EXTENDED_SYS_STATE, 1.0, nullptr);
}

void APMActionImpl::deinit()
{
    ((APMDeviceImpl*)_parent)->unregister_all_mavlink_message_handlers((void *)this);
}

Action::Result APMActionImpl::arm() const
{
    Action::Result ret = arming_allowed();
    if (ret != Action::Result::SUCCESS) {
        return ret;
    }

    // Go to LOITER mode first.
    uint8_t flag_safety_armed = ((APMDeviceImpl*)_parent)->is_armed() ? MAV_MODE_FLAG_SAFETY_ARMED : 0;

    uint8_t mode = VEHICLE_MODE_FLAG_CUSTOM_MODE_ENABLED | flag_safety_armed;
    uint8_t custom_mode = apm::GUIDED;

    ret = action_result_from_command_result(
              ((APMDeviceImpl*)_parent)->set_mode_with_ack(
                  mode, custom_mode,
                  MavlinkCommands::DEFAULT_COMPONENT_ID_AUTOPILOT));

    if (ret != Action::Result::SUCCESS) {
        return ret;
    }
    return action_result_from_command_result(
               ((APMDeviceImpl*)_parent)->send_command_with_ack(
                   MAV_CMD_COMPONENT_ARM_DISARM,
                   MavlinkCommands::Params {1.0f, NAN, NAN, NAN, NAN, NAN, NAN},
                   MavlinkCommands::DEFAULT_COMPONENT_ID_AUTOPILOT));
}

Action::Result APMActionImpl::disarm() const
{
    Action::Result ret = disarming_allowed();
    if (ret != Action::Result::SUCCESS) {
        return ret;
    }

    return action_result_from_command_result(
               ((APMDeviceImpl*)_parent)->send_command_with_ack(
                   MAV_CMD_COMPONENT_ARM_DISARM,
                   MavlinkCommands::Params {0.0f, NAN, NAN, NAN, NAN, NAN, NAN},
                   MavlinkCommands::DEFAULT_COMPONENT_ID_AUTOPILOT));
}

Action::Result APMActionImpl::kill() const
{
    return action_result_from_command_result(
               ((APMDeviceImpl*)_parent)->send_command_with_ack(
                   MAV_CMD_COMPONENT_ARM_DISARM,
                   MavlinkCommands::Params {0.0f, NAN, NAN, NAN, NAN, NAN, NAN},
                   MavlinkCommands::DEFAULT_COMPONENT_ID_AUTOPILOT));
}

Action::Result APMActionImpl::takeoff() const
{
    Action::Result ret = taking_off_allowed();
    if (ret != Action::Result::SUCCESS) {
        return ret;
    }

    // Go to LOITER mode first.
    uint8_t flag_safety_armed = ((APMDeviceImpl*)_parent)->is_armed() ? MAV_MODE_FLAG_SAFETY_ARMED : 0;

    uint8_t mode = VEHICLE_MODE_FLAG_CUSTOM_MODE_ENABLED | flag_safety_armed;
    uint8_t custom_mode = apm::GUIDED;

    ret = action_result_from_command_result(
              ((APMDeviceImpl*)_parent)->set_mode_with_ack(
                  mode, custom_mode,
                  MavlinkCommands::DEFAULT_COMPONENT_ID_AUTOPILOT));


    return action_result_from_command_result(
               ((APMDeviceImpl*)_parent)->send_command_with_ack(
                   MAV_CMD_NAV_TAKEOFF,
                   MavlinkCommands::Params {NAN, NAN, NAN, NAN, NAN, NAN,
                                            _relative_takeoff_altitude_m},
                   MavlinkCommands::DEFAULT_COMPONENT_ID_AUTOPILOT));
}

Action::Result APMActionImpl::land() const
{
    return action_result_from_command_result(
               ((APMDeviceImpl*)_parent)->send_command_with_ack(
                   MAV_CMD_NAV_LAND,
                   MavlinkCommands::Params {NAN, NAN, NAN, NAN, NAN, NAN, NAN},
                   MavlinkCommands::DEFAULT_COMPONENT_ID_AUTOPILOT));
}

Action::Result APMActionImpl::return_to_launch() const
{
    // Note: the safety flag is not needed in future versions of the APM Firmware
    //       but want to be rather safe than sorry.
    uint8_t flag_safety_armed = ((APMDeviceImpl*)_parent)->is_armed() ? MAV_MODE_FLAG_SAFETY_ARMED : 0;

    uint8_t mode = VEHICLE_MODE_FLAG_CUSTOM_MODE_ENABLED |
                   flag_safety_armed;
    uint8_t custom_mode = apm::RTL;
    
    return action_result_from_command_result(
              ((APMDeviceImpl*)_parent)->set_mode_with_ack(
                  mode, custom_mode,
                  MavlinkCommands::DEFAULT_COMPONENT_ID_AUTOPILOT));
}

void APMActionImpl::arm_async(const Action::result_callback_t &callback)
{
    Action::Result ret = arming_allowed();
    if (ret != Action::Result::SUCCESS) {
        if (callback) {
            callback(ret);
        }
        return;
    }

    guided_before_arm_async(callback);

}

void APMActionImpl::arm_async_continued(MavlinkCommands::Result previous_result,
                                     const Action::result_callback_t &callback)
{
    if (previous_result != MavlinkCommands::Result::SUCCESS) {
        command_result_callback(previous_result, callback);
        return;
    }

    ((APMDeviceImpl*)_parent)->send_command_with_ack_async(
        MAV_CMD_COMPONENT_ARM_DISARM,
        MavlinkCommands::Params {1.0f, NAN, NAN, NAN, NAN, NAN, NAN},
        std::bind(&APMActionImpl::command_result_callback,
                  _1,
                  callback),
        MavlinkCommands::DEFAULT_COMPONENT_ID_AUTOPILOT);
}

void APMActionImpl::disarm_async(const Action::result_callback_t &callback)
{
    Action::Result ret = disarming_allowed();
    if (ret != Action::Result::SUCCESS) {
        if (callback) {
            callback(ret);
        }
        return;
    }

    ((APMDeviceImpl*)_parent)->send_command_with_ack_async(
        MAV_CMD_COMPONENT_ARM_DISARM,
        MavlinkCommands::Params {0.0f, NAN, NAN, NAN, NAN, NAN, NAN},
        std::bind(&APMActionImpl::command_result_callback,
                  _1,
                  callback),
        MavlinkCommands::DEFAULT_COMPONENT_ID_AUTOPILOT);
}

void APMActionImpl::kill_async(const Action::result_callback_t &callback)
{
    ((APMDeviceImpl*)_parent)->send_command_with_ack_async(
        MAV_CMD_COMPONENT_ARM_DISARM,
        MavlinkCommands::Params {0.0f, NAN, NAN, NAN, NAN, NAN, NAN},
        std::bind(&APMActionImpl::command_result_callback,
                  _1,
                  callback),
        MavlinkCommands::DEFAULT_COMPONENT_ID_AUTOPILOT);
}

void APMActionImpl::takeoff_async(const Action::result_callback_t &callback)
{
    Action::Result ret = taking_off_allowed();
    if (ret != Action::Result::SUCCESS) {
        if (callback) {
            callback(ret);
        }
        return;
    }

    guided_before_takeoff_async(callback);
}


void APMActionImpl::takeoff_async_continued(MavlinkCommands::Result previous_result,
                                         const Action::result_callback_t &callback)
{
    if (previous_result != MavlinkCommands::Result::SUCCESS) {
        command_result_callback(previous_result, callback);
        return;
    }

    ((APMDeviceImpl*)_parent)->send_command_with_ack_async(
        MAV_CMD_NAV_TAKEOFF,
        MavlinkCommands::Params {NAN, NAN, NAN, NAN, NAN, NAN,
                                 _relative_takeoff_altitude_m},
        std::bind(&APMActionImpl::command_result_callback,
                  _1,
                  callback),
        MavlinkCommands::DEFAULT_COMPONENT_ID_AUTOPILOT);
}

void APMActionImpl::land_async(const Action::result_callback_t &callback)
{
    ((APMDeviceImpl*)_parent)->send_command_with_ack_async(
        MAV_CMD_NAV_LAND,
        MavlinkCommands::Params {NAN, NAN, NAN, NAN, NAN, NAN, NAN},
        std::bind(&APMActionImpl::command_result_callback,
                  _1,
                  callback),
        MavlinkCommands::DEFAULT_COMPONENT_ID_AUTOPILOT);
}

void APMActionImpl::return_to_launch_async(const Action::result_callback_t &callback)
{
    // Note: the safety flag is not needed in future versions of the APM Firmware
    //       but want to be rather safe than sorry.
    uint8_t flag_safety_armed = ((APMDeviceImpl*)_parent)->is_armed() ? MAV_MODE_FLAG_SAFETY_ARMED : 0;

    uint8_t mode = VEHICLE_MODE_FLAG_CUSTOM_MODE_ENABLED |
                   flag_safety_armed;
    uint8_t custom_mode = apm::RTL;

      ((APMDeviceImpl*)_parent)->set_mode_with_ack_async(
      mode, custom_mode,
      std::bind(&APMActionImpl::command_result_callback,
                _1,
                callback),
      MavlinkCommands::DEFAULT_COMPONENT_ID_AUTOPILOT);
}

Action::Result APMActionImpl::arming_allowed() const
{
    if (!_in_air_state_known) {
        return Action::Result::COMMAND_DENIED_LANDED_STATE_UNKNOWN;
    }

    if (_in_air) {
        return Action::Result::COMMAND_DENIED_NOT_LANDED;
    }

    return Action::Result::SUCCESS;
}

Action::Result APMActionImpl::taking_off_allowed() const
{
    if (!_in_air_state_known) {
        return Action::Result::COMMAND_DENIED_LANDED_STATE_UNKNOWN;
    }

    if (_in_air) {
        return Action::Result::COMMAND_DENIED_NOT_LANDED;
    }

    return Action::Result::SUCCESS;
}

Action::Result APMActionImpl::disarming_allowed() const
{
    if (!_in_air_state_known) {
        return Action::Result::COMMAND_DENIED_LANDED_STATE_UNKNOWN;
    }

    if (_in_air) {
        return Action::Result::COMMAND_DENIED_NOT_LANDED;
    }

    return Action::Result::SUCCESS;
}

void APMActionImpl::process_sys_state(const mavlink_message_t &message)
{
    mavlink_heartbeat_t heartbeat;
    mavlink_msg_heartbeat_decode(&message, &heartbeat);

    if (heartbeat.system_status == MAV_STATE_ACTIVE) {
        _in_air = true;
        _in_air_state_known = true;
    } else if (heartbeat.system_status == MAV_STATE_STANDBY) {
        _in_air = false;
        _in_air_state_known = true;
    }
    //something critical has happened, we might even have crashed or going there
    if (heartbeat.system_status == MAV_STATE_CRITICAL) {
        _in_air_state_known = false;
    }
}

void APMActionImpl::guided_before_takeoff_async(const Action::result_callback_t &callback)
{
    uint8_t flag_safety_armed = ((APMDeviceImpl*)_parent)->is_armed() ? MAV_MODE_FLAG_SAFETY_ARMED : 0;

    uint8_t mode = VEHICLE_MODE_FLAG_CUSTOM_MODE_ENABLED | flag_safety_armed;
    uint8_t custom_mode = apm::GUIDED;
    ((APMDeviceImpl*)_parent)->set_mode_with_ack_async(
        mode, custom_mode,
        std::bind(&APMActionImpl::takeoff_async_continued, this, _1,
                  callback),
        MavlinkCommands::DEFAULT_COMPONENT_ID_AUTOPILOT);
}

void APMActionImpl::guided_before_arm_async(const Action::result_callback_t &callback)
{
    uint8_t flag_safety_armed = ((APMDeviceImpl*)_parent)->is_armed() ? MAV_MODE_FLAG_SAFETY_ARMED : 0;

    uint8_t mode = VEHICLE_MODE_FLAG_CUSTOM_MODE_ENABLED | flag_safety_armed;
    uint8_t custom_mode = apm::GUIDED;

    ((APMDeviceImpl*)_parent)->set_mode_with_ack_async(
        mode, custom_mode,
        std::bind(&APMActionImpl::arm_async_continued, this, _1,
                  callback),
        MavlinkCommands::DEFAULT_COMPONENT_ID_AUTOPILOT);
}


void APMActionImpl::set_takeoff_altitude(float relative_altitude_m)
{
    ((APMDeviceImpl*)_parent)->set_param_float_async("MIS_TAKEOFF_ALT", relative_altitude_m,
                                   std::bind(&APMActionImpl::receive_takeoff_alt_param,
                                             this, _1, relative_altitude_m));

}

void APMActionImpl::receive_takeoff_alt_param(bool success, float new_relative_altitude_m)
{
    if (success) {
        // TODO: This should not be buffered like this, the param system
        //       needs some refactoring.
        _relative_takeoff_altitude_m = new_relative_altitude_m;
    }
}

float APMActionImpl::get_takeoff_altitude_m() const
{
    return _relative_takeoff_altitude_m;
}

void APMActionImpl::set_max_speed(float speed_m_s)
{
    // TODO: add retries
    //const int retries = 3;
    ((APMDeviceImpl*)_parent)->set_param_float_async("MPC_XY_CRUISE", speed_m_s,
                                   std::bind(&APMActionImpl::receive_max_speed_result, this, _1,
                                             speed_m_s));
}

void APMActionImpl::receive_max_speed_result(bool success, float new_speed_m_s)
{
    if (!success) {
        Debug() << "setting max speed param failed";
        return;
    }
    _max_speed_m_s = new_speed_m_s;
}

float APMActionImpl::get_max_speed_m_s() const
{
    return _max_speed_m_s;
}

Action::Result
APMActionImpl::action_result_from_command_result(MavlinkCommands::Result result)
{
    switch (result) {
        case MavlinkCommands::Result::SUCCESS:
            return Action::Result::SUCCESS;
        case MavlinkCommands::Result::NO_DEVICE:
            return Action::Result::NO_DEVICE;
        case MavlinkCommands::Result::CONNECTION_ERROR:
            return Action::Result::CONNECTION_ERROR;
        case MavlinkCommands::Result::BUSY:
            return Action::Result::BUSY;
        case MavlinkCommands::Result::COMMAND_DENIED:
            return Action::Result::COMMAND_DENIED;
        case MavlinkCommands::Result::TIMEOUT:
            return Action::Result::TIMEOUT;
        default:
            return Action::Result::UNKNOWN;
    }
}

void APMActionImpl::command_result_callback(MavlinkCommands::Result command_result,
                                         const Action::result_callback_t &callback)
{
    Action::Result action_result = action_result_from_command_result(command_result);

    if (callback) {
        callback(action_result);
    }
}


} // namespace dronecore
