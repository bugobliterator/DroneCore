#pragma once

#include "telemetry.h"
#include "plugin_impl_base.h"
#include "device_px4.h"
#include "mavlink_include.h"
#include <atomic>
#include <mutex>

// Since not all vehicles support/require level calibration, this
// is disabled for now.
//#define LEVEL_CALIBRATION

namespace dronecore {

class DeviceImpl;

class TelemetryImpl : public PluginImplBase
{
public:
    TelemetryImpl() {}
    virtual ~TelemetryImpl() {}

    virtual Telemetry::Result set_rate_position(double rate_hz) = 0;
    virtual Telemetry::Result set_rate_home_position(double rate_hz) = 0;
    virtual Telemetry::Result set_rate_in_air(double rate_hz) = 0;
    virtual Telemetry::Result set_rate_attitude(double rate_hz) = 0;
    virtual Telemetry::Result set_rate_camera_attitude(double rate_hz) = 0;
    virtual Telemetry::Result set_rate_ground_speed_ned(double rate_hz) = 0;
    virtual Telemetry::Result set_rate_gps_info(double rate_hz) = 0;
    virtual Telemetry::Result set_rate_battery(double rate_hz) = 0;
    virtual Telemetry::Result set_rate_rc_status(double rate_hz) = 0;

    virtual void set_rate_position_async(double rate_hz, Telemetry::result_callback_t callback) = 0;
    virtual void set_rate_home_position_async(double rate_hz, Telemetry::result_callback_t callback) = 0;
    virtual void set_rate_in_air_async(double rate_hz, Telemetry::result_callback_t callback) = 0;
    virtual void set_rate_attitude_async(double rate_hz, Telemetry::result_callback_t callback) = 0;
    virtual void set_rate_camera_attitude_async(double rate_hz, Telemetry::result_callback_t callback) = 0;
    virtual void set_rate_ground_speed_ned_async(double rate_hz, Telemetry::result_callback_t callback) = 0;
    virtual void set_rate_gps_info_async(double rate_hz, Telemetry::result_callback_t callback) = 0;
    virtual void set_rate_battery_async(double rate_hz, Telemetry::result_callback_t callback) = 0;
    virtual void set_rate_rc_status_async(double rate_hz, Telemetry::result_callback_t callback) = 0;

    virtual Telemetry::Position get_position() const = 0;
    virtual Telemetry::Position get_home_position() const = 0;
    virtual bool in_air() const = 0;
    virtual bool armed() const = 0;
    virtual Telemetry::EulerAngle get_attitude_euler_angle() const = 0;
    virtual Telemetry::Quaternion get_attitude_quaternion() const = 0;
    virtual Telemetry::EulerAngle get_camera_attitude_euler_angle() const = 0;
    virtual Telemetry::Quaternion get_camera_attitude_quaternion() const = 0;
    virtual Telemetry::GroundSpeedNED get_ground_speed_ned() const = 0;
    virtual Telemetry::GPSInfo get_gps_info() const = 0;
    virtual Telemetry::Battery get_battery() const = 0;
    virtual Telemetry::FlightMode get_flight_mode() const = 0;
    virtual Telemetry::Health get_health() const = 0;
    virtual bool get_health_all_ok() const = 0;
    virtual Telemetry::RCStatus get_rc_status() const = 0;
 
    virtual void position_async(Telemetry::position_callback_t &callback) = 0;
    virtual void home_position_async(Telemetry::position_callback_t &callback) = 0;
    virtual void in_air_async(Telemetry::in_air_callback_t &callback) = 0;
    virtual void armed_async(Telemetry::armed_callback_t &callback) = 0;
    virtual void attitude_quaternion_async(Telemetry::attitude_quaternion_callback_t &callback) = 0;
    virtual void attitude_euler_angle_async(Telemetry::attitude_euler_angle_callback_t &callback) = 0;
    virtual void camera_attitude_quaternion_async(Telemetry::attitude_quaternion_callback_t &callback) = 0;
    virtual void camera_attitude_euler_angle_async(Telemetry::attitude_euler_angle_callback_t &callback) = 0;
    virtual void ground_speed_ned_async(Telemetry::ground_speed_ned_callback_t &callback) = 0;
    virtual void gps_info_async(Telemetry::gps_info_callback_t &callback) = 0;
    virtual void battery_async(Telemetry::battery_callback_t &callback) = 0;
    virtual void flight_mode_async(Telemetry::flight_mode_callback_t &callback) = 0;
    virtual void health_async(Telemetry::health_callback_t &callback) = 0;
    virtual void health_all_ok_async(Telemetry::health_all_ok_callback_t &callback) = 0;
    virtual void rc_status_async(Telemetry::rc_status_callback_t &callback) = 0;

};

} // namespace dronecore
