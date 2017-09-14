#pragma once

#include "plugin_impl_base.h"
#include "mavlink_include.h"
#include "device_impl.h"
#include "action.h"
#include <cstdint>

namespace dronecore {

class ActionImpl : public PluginImplBase
{
public:
    ActionImpl() {}
    ~ActionImpl() {}

    virtual Action::Result arm() const = 0;
    virtual Action::Result disarm() const = 0;
    virtual Action::Result kill() const = 0;
    virtual Action::Result takeoff() const = 0;
    virtual Action::Result land() const = 0;
    virtual Action::Result return_to_launch() const = 0;

    virtual void arm_async(const Action::result_callback_t &callback) = 0;
    virtual void disarm_async(const Action::result_callback_t &callback) = 0;
    virtual void kill_async(const Action::result_callback_t &callback) = 0;
    virtual void takeoff_async(const Action::result_callback_t &callback) = 0;
    virtual void land_async(const Action::result_callback_t &callback) = 0;
    virtual void return_to_launch_async(const Action::result_callback_t &callback) = 0;

    virtual void set_takeoff_altitude(float relative_altitude_m) = 0;
    virtual float get_takeoff_altitude_m() const = 0;

    virtual void set_max_speed(float speed_m_s) = 0;
    virtual float get_max_speed_m_s() const = 0;

};

} // namespace dronecore
