#include "action.h"
#include "action_impl.h"
#include "action_px4.h"
#include "action_apm.h"

namespace dronecore {

Action::Action(DeviceImpl *device) :
    _device(device)
{
    switch(device->get_device_type()) {
        case MAV_AUTOPILOT_PX4:
            _impl = new PX4ActionImpl();
            break;
        case MAV_AUTOPILOT_ARDUPILOTMEGA:
            _impl = new APMActionImpl();
            break;
        case MAV_AUTOPILOT_GENERIC:
            _impl = new PX4ActionImpl();
            break;
    }
}

Action::~Action()
{
    if(_impl != nullptr) {
        delete _impl;
    }
}

Action::Result Action::arm() const
{
    return _impl->arm();
}

Action::Result Action::disarm() const
{
    return _impl->disarm();
}

Action::Result Action::kill() const
{
    return _impl->kill();
}

Action::Result Action::takeoff() const
{
    return _impl->takeoff();
}

Action::Result Action::land() const
{
    return _impl->land();
}

Action::Result Action::return_to_launch() const
{
    return _impl->return_to_launch();
}

void Action::arm_async(result_callback_t callback)
{
    _impl->arm_async(callback);
}

void Action::disarm_async(result_callback_t callback)
{
    _impl->disarm_async(callback);
}

void Action::kill_async(result_callback_t callback)
{
    _impl->kill_async(callback);
}

void Action::takeoff_async(result_callback_t callback)
{
    _impl->takeoff_async(callback);
}

void Action::land_async(result_callback_t callback)
{
    _impl->land_async(callback);
}

void Action::return_to_launch_async(result_callback_t callback)
{
    _impl->return_to_launch_async(callback);
}

void Action::set_takeoff_altitude(float relative_altitude_m)
{
    _impl->set_takeoff_altitude(relative_altitude_m);
}

float Action::get_takeoff_altitude_m() const
{
    return _impl->get_takeoff_altitude_m();
}

void Action::set_max_speed(float speed_m_s)
{
    _impl->set_max_speed(speed_m_s);
}

float Action::get_max_speed_m_s() const
{
    return _impl->get_max_speed_m_s();
}

const char *Action::result_str(Result result)
{
    switch (result) {
        case Result::SUCCESS:
            return "Success";
        case Result::NO_DEVICE:
            return "No device";
        case Result::CONNECTION_ERROR:
            return "Connection error";
        case Result::BUSY:
            return "Busy";
        case Result::COMMAND_DENIED:
            return "Command denied";
        case Result::COMMAND_DENIED_LANDED_STATE_UNKNOWN:
            return "Command denied, landed state is unknown";
        case Result::COMMAND_DENIED_NOT_LANDED:
            return "Command denied, not landed";
        case Result::TIMEOUT:
            return "Timeout";
        case Result::UNKNOWN:
        default:
            return "Unknown";
    }
}


} // namespace dronecore
