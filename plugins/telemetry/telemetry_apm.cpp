#include "telemetry_apm.h"
#include "device_apm.h"
#include "math_conversions.h"
#include "global_include.h"
#include "px4_custom_mode.h"
#include <cmath>
#include <functional>

namespace dronecore {

APMTelemetryImpl::APMTelemetryImpl() :
    _position_mutex(),
    _position(Telemetry::Position {NAN, NAN, NAN, NAN}),
    _home_position_mutex(),
    _home_position(Telemetry::Position {NAN, NAN, NAN, NAN}),
    _in_air(false),
    _armed(false),
    _attitude_quaternion_mutex(),
    _attitude_quaternion(Telemetry::Quaternion {NAN, NAN, NAN, NAN}),
    _camera_attitude_euler_angle_mutex(),
    _camera_attitude_euler_angle(Telemetry::EulerAngle {NAN, NAN, NAN}),
    _ground_speed_ned_mutex(),
    _ground_speed_ned(Telemetry::GroundSpeedNED {NAN, NAN, NAN}),
    _gps_info_mutex(),
    _gps_info(Telemetry::GPSInfo {0, 0}),
    _battery_mutex(),
    _battery(Telemetry::Battery {NAN, NAN}),
    _flight_mode_mutex(),
    _flight_mode(Telemetry::FlightMode::UNKNOWN),
    _health_mutex(),
    _health(Telemetry::Health {false, false, false, false, false, false, false}),
    _rc_status_mutex(),
    _rc_status(Telemetry::RCStatus {false, false, 0.0f}),
    _position_subscription(nullptr),
    _home_position_subscription(nullptr),
    _in_air_subscription(nullptr),
    _armed_subscription(nullptr),
    _attitude_quaternion_subscription(nullptr),
    _attitude_euler_angle_subscription(nullptr),
    _camera_attitude_quaternion_subscription(nullptr),
    _camera_attitude_euler_angle_subscription(nullptr),
    _ground_speed_ned_subscription(nullptr),
    _gps_info_subscription(nullptr),
    _battery_subscription(nullptr),
    _flight_mode_subscription(nullptr),
    _health_subscription(nullptr),
    _health_all_ok_subscription(nullptr),
    _rc_status_subscription(nullptr),
    _ground_speed_ned_rate_hz(0.0),
    _position_rate_hz(0.0)
{
}

APMTelemetryImpl::~APMTelemetryImpl()
{
}


void APMTelemetryImpl::init()
{
    using namespace std::placeholders; // for `_1`

    ((APMDeviceImpl*)_parent)->register_mavlink_message_handler(
        MAVLINK_MSG_ID_GLOBAL_POSITION_INT,
        std::bind(&APMTelemetryImpl::process_global_position_int, this, _1), (void *)this);

    ((APMDeviceImpl*)_parent)->register_mavlink_message_handler(
        MAVLINK_MSG_ID_HOME_POSITION,
        std::bind(&APMTelemetryImpl::process_home_position, this, _1), (void *)this);

    ((APMDeviceImpl*)_parent)->register_mavlink_message_handler(
        MAVLINK_MSG_ID_ATTITUDE,
        std::bind(&APMTelemetryImpl::process_attitude, this, _1), (void *)this);

    ((APMDeviceImpl*)_parent)->register_mavlink_message_handler(
        MAVLINK_MSG_ID_MOUNT_ORIENTATION,
        std::bind(&APMTelemetryImpl::process_mount_orientation, this, _1), (void *)this);

    ((APMDeviceImpl*)_parent)->register_mavlink_message_handler(
        MAVLINK_MSG_ID_GPS_RAW_INT,
        std::bind(&APMTelemetryImpl::process_gps_raw_int, this, _1), (void *)this);

    ((APMDeviceImpl*)_parent)->register_mavlink_message_handler(
        MAVLINK_MSG_ID_EXTENDED_SYS_STATE,
        std::bind(&APMTelemetryImpl::process_extended_sys_state, this, _1), (void *)this);

    ((APMDeviceImpl*)_parent)->register_mavlink_message_handler(
        MAVLINK_MSG_ID_SYS_STATUS,
        std::bind(&APMTelemetryImpl::process_sys_status, this, _1), (void *)this);

    ((APMDeviceImpl*)_parent)->register_mavlink_message_handler(
        MAVLINK_MSG_ID_HEARTBEAT,
        std::bind(&APMTelemetryImpl::process_heartbeat, this, _1), (void *)this);

    ((APMDeviceImpl*)_parent)->register_mavlink_message_handler(
        MAVLINK_MSG_ID_RC_CHANNELS,
        std::bind(&APMTelemetryImpl::process_rc_channels, this, _1), (void *)this);

    ((APMDeviceImpl*)_parent)->register_timeout_handler(
        std::bind(&APMTelemetryImpl::receive_rc_channels_timeout, this), 1.0, (const void *)this);

    // If not available, just hardcode it to true.
    set_health_level_calibration(true);

}

void APMTelemetryImpl::deinit()
{
    ((APMDeviceImpl*)_parent)->unregister_all_mavlink_message_handlers((void *)this);
}

Telemetry::Result APMTelemetryImpl::set_rate_position(double rate_hz)
{
    _position_rate_hz = rate_hz;
    double max_rate_hz = std::max(_position_rate_hz, _ground_speed_ned_rate_hz);
    return telemetry_result_from_command_result(
               ((APMDeviceImpl*)_parent)->set_msg_rate(MAV_DATA_STREAM_POSITION, max_rate_hz));
}

Telemetry::Result APMTelemetryImpl::set_rate_home_position(double rate_hz)
{
    (void)rate_hz;
    return  Telemetry::Result::COMMAND_DENIED;
}

Telemetry::Result APMTelemetryImpl::set_rate_in_air(double rate_hz)
{
    (void)rate_hz;
    return  Telemetry::Result::COMMAND_DENIED;
}

Telemetry::Result APMTelemetryImpl::set_rate_attitude(double rate_hz)
{
    return telemetry_result_from_command_result(
               ((APMDeviceImpl*)_parent)->set_msg_rate(MAV_DATA_STREAM_EXTRA1, rate_hz));
}

Telemetry::Result APMTelemetryImpl::set_rate_camera_attitude(double rate_hz)
{
    return telemetry_result_from_command_result(
               ((APMDeviceImpl*)_parent)->set_msg_rate(MAV_DATA_STREAM_EXTRA3, rate_hz));
}

Telemetry::Result APMTelemetryImpl::set_rate_ground_speed_ned(double rate_hz)
{
    _ground_speed_ned_rate_hz = rate_hz;
    double max_rate_hz = std::max(_position_rate_hz, _ground_speed_ned_rate_hz);

    return telemetry_result_from_command_result(
               ((APMDeviceImpl*)_parent)->set_msg_rate(MAV_DATA_STREAM_EXTRA2, max_rate_hz));
}

Telemetry::Result APMTelemetryImpl::set_rate_gps_info(double rate_hz)
{
    return telemetry_result_from_command_result(
               ((APMDeviceImpl*)_parent)->set_msg_rate(MAV_DATA_STREAM_EXTENDED_STATUS, rate_hz));
}

Telemetry::Result APMTelemetryImpl::set_rate_battery(double rate_hz)
{
    return telemetry_result_from_command_result(
               ((APMDeviceImpl*)_parent)->set_msg_rate(MAV_DATA_STREAM_EXTENDED_STATUS, rate_hz));
}

Telemetry::Result APMTelemetryImpl::set_rate_rc_status(double rate_hz)
{
    return telemetry_result_from_command_result(
               ((APMDeviceImpl*)_parent)->set_msg_rate(MAV_DATA_STREAM_RC_CHANNELS, rate_hz));
}

void APMTelemetryImpl::set_rate_position_async(double rate_hz, Telemetry::result_callback_t callback)
{
    _position_rate_hz = rate_hz;
    double max_rate_hz = std::max(_position_rate_hz, _ground_speed_ned_rate_hz);

    ((APMDeviceImpl*)_parent)->set_msg_rate_async(
        MAVLINK_MSG_ID_GLOBAL_POSITION_INT,
        max_rate_hz,
        std::bind(&APMTelemetryImpl::command_result_callback, std::placeholders::_1, callback));

}

void APMTelemetryImpl::set_rate_home_position_async(double rate_hz,
                                                 Telemetry::result_callback_t callback)
{
    (void)rate_hz;
    (void)callback;
}

void APMTelemetryImpl::set_rate_in_air_async(double rate_hz, Telemetry::result_callback_t callback)
{
    (void)rate_hz;
    (void)callback;
}

void APMTelemetryImpl::set_rate_attitude_async(double rate_hz, Telemetry::result_callback_t callback)
{
    ((APMDeviceImpl*)_parent)->set_msg_rate_async(
        MAV_DATA_STREAM_EXTRA1,
        rate_hz,
        std::bind(&APMTelemetryImpl::command_result_callback, std::placeholders::_1, callback));
}

void APMTelemetryImpl::set_rate_camera_attitude_async(double rate_hz,
                                                   Telemetry::result_callback_t callback)
{
    ((APMDeviceImpl*)_parent)->set_msg_rate_async(
        MAV_DATA_STREAM_EXTRA3,
        rate_hz,
        std::bind(&APMTelemetryImpl::command_result_callback, std::placeholders::_1, callback));
}

void APMTelemetryImpl::set_rate_ground_speed_ned_async(double rate_hz,
                                                    Telemetry::result_callback_t callback)
{
    _ground_speed_ned_rate_hz = rate_hz;
    double max_rate_hz = std::max(_position_rate_hz, _ground_speed_ned_rate_hz);

    ((APMDeviceImpl*)_parent)->set_msg_rate_async(
        MAV_DATA_STREAM_EXTRA2,
        max_rate_hz,
        std::bind(&APMTelemetryImpl::command_result_callback, std::placeholders::_1, callback));
}

void APMTelemetryImpl::set_rate_gps_info_async(double rate_hz, Telemetry::result_callback_t callback)
{
    ((APMDeviceImpl*)_parent)->set_msg_rate_async(
        MAV_DATA_STREAM_EXTENDED_STATUS,
        rate_hz,
        std::bind(&APMTelemetryImpl::command_result_callback, std::placeholders::_1, callback));
}

void APMTelemetryImpl::set_rate_battery_async(double rate_hz, Telemetry::result_callback_t callback)
{
    ((APMDeviceImpl*)_parent)->set_msg_rate_async(
        MAV_DATA_STREAM_EXTENDED_STATUS,
        rate_hz,
        std::bind(&APMTelemetryImpl::command_result_callback, std::placeholders::_1, callback));
}

void APMTelemetryImpl::set_rate_rc_status_async(double rate_hz, Telemetry::result_callback_t callback)
{
    ((APMDeviceImpl*)_parent)->set_msg_rate_async(
        MAV_DATA_STREAM_RC_CHANNELS,
        rate_hz,
        std::bind(&APMTelemetryImpl::command_result_callback, std::placeholders::_1, callback));
}

Telemetry::Result APMTelemetryImpl::telemetry_result_from_command_result(
    MavlinkCommands::Result command_result)
{
    switch (command_result) {
        case MavlinkCommands::Result::SUCCESS:
            return Telemetry::Result::SUCCESS;
        case MavlinkCommands::Result::NO_DEVICE:
            return Telemetry::Result::NO_DEVICE;
        case MavlinkCommands::Result::CONNECTION_ERROR:
            return Telemetry::Result::CONNECTION_ERROR;
        case MavlinkCommands::Result::BUSY:
            return Telemetry::Result::BUSY;
        case MavlinkCommands::Result::COMMAND_DENIED:
            return Telemetry::Result::COMMAND_DENIED;
        case MavlinkCommands::Result::TIMEOUT:
            return Telemetry::Result::TIMEOUT;
        default:
            return Telemetry::Result::UNKNOWN;
    }
}

void APMTelemetryImpl::command_result_callback(MavlinkCommands::Result command_result,
                                            const Telemetry::result_callback_t &callback)
{
    Telemetry::Result action_result = telemetry_result_from_command_result(command_result);

    callback(action_result);
}

void APMTelemetryImpl::process_global_position_int(const mavlink_message_t &message)
{
    mavlink_global_position_int_t global_position_int;
    mavlink_msg_global_position_int_decode(&message, &global_position_int);
    set_position(Telemetry::Position({global_position_int.lat * 1e-7,
                                      global_position_int.lon * 1e-7,
                                      global_position_int.alt * 1e-3f,
                                      global_position_int.relative_alt * 1e-3f
                                     }));
    set_ground_speed_ned({global_position_int.vx * 1e-2f,
                          global_position_int.vy * 1e-2f,
                          global_position_int.vz * 1e-2f
                         });

    if (_position_subscription) {
        _position_subscription(get_position());
    }

    if (_ground_speed_ned_subscription) {
        _ground_speed_ned_subscription(get_ground_speed_ned());
    }
}

void APMTelemetryImpl::process_home_position(const mavlink_message_t &message)
{
    mavlink_home_position_t home_position;
    mavlink_msg_home_position_decode(&message, &home_position);
    set_home_position(Telemetry::Position({home_position.latitude * 1e-7,
                                           home_position.longitude * 1e-7,
                                           home_position.altitude * 1e-3f,
                                           // the relative altitude of home is 0 by definition.
                                           0.0f
                                          }));

    set_health_home_position(true);

    if (_home_position_subscription) {
        _home_position_subscription(get_home_position());
    }
}

void APMTelemetryImpl::process_attitude(const mavlink_message_t &message)
{
    mavlink_attitude_t attitude;
    mavlink_msg_attitude_decode(&message, &attitude);

    Telemetry::EulerAngle euler_angle {
        attitude.roll,
        attitude.pitch,
        attitude.yaw
    };

    set_attitude_euler(euler_angle);

    if (_attitude_quaternion_subscription) {
        _attitude_quaternion_subscription(get_attitude_quaternion());
    }

    if (_attitude_euler_angle_subscription) {
        _attitude_euler_angle_subscription(get_attitude_euler_angle());
    }
}

void APMTelemetryImpl::process_mount_orientation(const mavlink_message_t &message)
{
    mavlink_mount_orientation_t mount_orientation;
    mavlink_msg_mount_orientation_decode(&message, &mount_orientation);

    Telemetry::EulerAngle euler_angle {
        mount_orientation.roll,
        mount_orientation.pitch,
        mount_orientation.yaw
    };

    set_camera_attitude_euler_angle(euler_angle);

    if (_camera_attitude_quaternion_subscription) {
        _camera_attitude_quaternion_subscription(get_camera_attitude_quaternion());
    }

    if (_camera_attitude_euler_angle_subscription) {
        _camera_attitude_euler_angle_subscription(get_camera_attitude_euler_angle());
    }
}

void APMTelemetryImpl::process_gps_raw_int(const mavlink_message_t &message)
{
    mavlink_gps_raw_int_t gps_raw_int;
    mavlink_msg_gps_raw_int_decode(&message, &gps_raw_int);
    set_gps_info({gps_raw_int.satellites_visible,
                  gps_raw_int.fix_type
                 });

    // TODO: This is just an interim hack, we will have to look at
    //       estimator flags in order to decide if the position
    //       estimate is good enough.
    const bool gps_ok = ((gps_raw_int.fix_type >= 3) && (gps_raw_int.satellites_visible >= 8));

    set_health_global_position(gps_ok);
    // Local is not different from global for now until things like flow are in place.
    set_health_local_position(gps_ok);

    if (_gps_info_subscription) {
        _gps_info_subscription(get_gps_info());
    }
}

void APMTelemetryImpl::process_extended_sys_state(const mavlink_message_t &message)
{
    mavlink_extended_sys_state_t extended_sys_state;
    mavlink_msg_extended_sys_state_decode(&message, &extended_sys_state);

    if (extended_sys_state.landed_state == MAV_LANDED_STATE_IN_AIR) {
        set_in_air(true);
    } else if (extended_sys_state.landed_state == MAV_LANDED_STATE_ON_GROUND) {
        set_in_air(false);
    }
    // If landed_state is undefined, we use what we have received last.

    if (_in_air_subscription) {
        _in_air_subscription(in_air());
    }

}

void APMTelemetryImpl::process_sys_status(const mavlink_message_t &message)
{
    mavlink_sys_status_t sys_status;
    mavlink_msg_sys_status_decode(&message, &sys_status);
    set_battery(Telemetry::Battery({sys_status.voltage_battery * 1e-3f,
                                    // FIXME: it is strange calling it percent when the range goes from 0 to 1.
                                    sys_status.battery_remaining * 1e-2f
                                   }));

    if (_battery_subscription) {
        _battery_subscription(get_battery());
    }
}

void APMTelemetryImpl::process_heartbeat(const mavlink_message_t &message)
{
    mavlink_heartbeat_t heartbeat;
    mavlink_msg_heartbeat_decode(&message, &heartbeat);

    set_armed(((heartbeat.base_mode & MAV_MODE_FLAG_SAFETY_ARMED) ? true : false));

    if (_armed_subscription) {
        _armed_subscription(armed());
    }

    if (heartbeat.base_mode & MAV_MODE_FLAG_CUSTOM_MODE_ENABLED) {

        Telemetry::FlightMode flight_mode = to_flight_mode_from_custom_mode(heartbeat.custom_mode);
        set_flight_mode(flight_mode);

        if (_flight_mode_subscription) {
            _flight_mode_subscription(get_flight_mode());
        }
    }

    if (heartbeat.system_status == MAV_STATE_ACTIVE) {
        set_in_air(true);
    } else if (heartbeat.system_status == MAV_STATE_STANDBY) {
        set_in_air(false);
    }
    // If state is CRITICAL, we use what we have received last.

    if (_in_air_subscription) {
        _in_air_subscription(in_air());
    }

    if (_health_subscription) {
        _health_subscription(get_health());
    }
    if (_health_all_ok_subscription) {
        _health_all_ok_subscription(get_health_all_ok());
    }
}

void APMTelemetryImpl::process_rc_channels(const mavlink_message_t &message)
{
    mavlink_rc_channels_t rc_channels;
    mavlink_msg_rc_channels_decode(&message, &rc_channels);

    bool rc_ok = (rc_channels.chancount > 0);
    set_rc_status(rc_ok, rc_channels.rssi);

    ((APMDeviceImpl*)_parent)->refresh_timeout_handler(this);
}

Telemetry::FlightMode APMTelemetryImpl::to_flight_mode_from_custom_mode(uint32_t custom_mode)
{
    px4::px4_custom_mode px4_custom_mode;
    px4_custom_mode.data = custom_mode;

    switch (px4_custom_mode.main_mode) {
        case px4::PX4_CUSTOM_MAIN_MODE_OFFBOARD:
            return Telemetry::FlightMode::OFFBOARD;
        case px4::PX4_CUSTOM_MAIN_MODE_AUTO:
            switch (px4_custom_mode.sub_mode) {
                case px4::PX4_CUSTOM_SUB_MODE_AUTO_READY:
                    return Telemetry::FlightMode::READY;
                case px4::PX4_CUSTOM_SUB_MODE_AUTO_TAKEOFF:
                    return Telemetry::FlightMode::TAKEOFF;
                case px4::PX4_CUSTOM_SUB_MODE_AUTO_LOITER:
                    return Telemetry::FlightMode::HOLD;
                case px4::PX4_CUSTOM_SUB_MODE_AUTO_MISSION:
                    return Telemetry::FlightMode::MISSION;
                case px4::PX4_CUSTOM_SUB_MODE_AUTO_RTL:
                    return Telemetry::FlightMode::RETURN_TO_LAUNCH;
                case px4::PX4_CUSTOM_SUB_MODE_AUTO_LAND:
                    return Telemetry::FlightMode::LAND;
                default:
                    return Telemetry::FlightMode::UNKNOWN;
            }
        default:
            return Telemetry::FlightMode::UNKNOWN;
    }
}

void APMTelemetryImpl::receive_param_cal_gyro(bool success, int value)
{
    if (!success) {
        Debug() << "Error: Param for gyro cal failed.";
        return;
    }

    bool ok = (value != 0);
    set_health_gyrometer_calibration(ok);
}

void APMTelemetryImpl::receive_param_cal_accel(bool success, int value)
{
    if (!success) {
        Debug() << "Error: Param for accel cal failed.";
        return;
    }

    bool ok = (value != 0);
    set_health_accelerometer_calibration(ok);
}

void APMTelemetryImpl::receive_param_cal_mag(bool success, int value)
{
    if (!success) {
        Debug() << "Error: Param for mag cal failed.";
        return;
    }

    bool ok = (value != 0);
    set_health_magnetometer_calibration(ok);
}

#ifdef LEVEL_CALIBRATION
void APMTelemetryImpl::receive_param_cal_level(bool success, float value)
{
    if (!success) {
        Debug() << "Error: Param for level cal failed.";
        return;
    }

    bool ok = (value != 0);
    set_health_level_calibration(ok);
}
#endif

void APMTelemetryImpl::receive_rc_channels_timeout()
{
    const bool rc_ok = false;
    set_rc_status(rc_ok, 0.0f);
}

Telemetry::Position APMTelemetryImpl::get_position() const
{
    std::lock_guard<std::mutex> lock(_position_mutex);
    return _position;
}

void APMTelemetryImpl::set_position(Telemetry::Position position)
{
    std::lock_guard<std::mutex> lock(_position_mutex);
    _position = position;
}

Telemetry::Position APMTelemetryImpl::get_home_position() const
{
    std::lock_guard<std::mutex> lock(_home_position_mutex);
    return _home_position;
}

void APMTelemetryImpl::set_home_position(Telemetry::Position home_position)
{
    std::lock_guard<std::mutex> lock(_home_position_mutex);
    _home_position = home_position;
}

bool APMTelemetryImpl::in_air() const
{
    return _in_air;
}

bool APMTelemetryImpl::armed() const
{
    return _armed;
}

void APMTelemetryImpl::set_in_air(bool in_air)
{
    _in_air = in_air;
}

void APMTelemetryImpl::set_armed(bool armed)
{
    _armed = armed;
}

Telemetry::Quaternion APMTelemetryImpl::get_attitude_quaternion() const
{
    std::lock_guard<std::mutex> lock(_attitude_quaternion_mutex);
    return _attitude_quaternion;
}

Telemetry::EulerAngle APMTelemetryImpl::get_attitude_euler_angle() const
{
    std::lock_guard<std::mutex> lock(_attitude_quaternion_mutex);
    Telemetry::EulerAngle euler = to_euler_angle_from_quaternion(_attitude_quaternion);

    return euler;
}

void APMTelemetryImpl::set_attitude_quaternion(Telemetry::Quaternion quaternion)
{
    std::lock_guard<std::mutex> lock(_attitude_quaternion_mutex);
    _attitude_quaternion = quaternion;
}

void APMTelemetryImpl::set_attitude_euler(Telemetry::EulerAngle euler_angle)
{
    std::lock_guard<std::mutex> alock(_attitude_euler_angle_mutex);
    std::lock_guard<std::mutex> qlock(_attitude_quaternion_mutex);
    _attitude_euler = euler_angle;
    _attitude_quaternion = to_quaternion_from_euler_angle(euler_angle);
}

Telemetry::Quaternion APMTelemetryImpl::get_camera_attitude_quaternion() const
{
    std::lock_guard<std::mutex> lock(_camera_attitude_euler_angle_mutex);
    Telemetry::Quaternion quaternion
        = to_quaternion_from_euler_angle(_camera_attitude_euler_angle);

    return quaternion;
}

Telemetry::EulerAngle APMTelemetryImpl::get_camera_attitude_euler_angle() const
{
    std::lock_guard<std::mutex> lock(_camera_attitude_euler_angle_mutex);

    return _camera_attitude_euler_angle;
}

void APMTelemetryImpl::set_camera_attitude_euler_angle(Telemetry::EulerAngle euler_angle)
{
    std::lock_guard<std::mutex> lock(_camera_attitude_euler_angle_mutex);
    _camera_attitude_euler_angle = euler_angle;
}

Telemetry::GroundSpeedNED APMTelemetryImpl::get_ground_speed_ned() const
{
    std::lock_guard<std::mutex> lock(_ground_speed_ned_mutex);
    return _ground_speed_ned;
}

void APMTelemetryImpl::set_ground_speed_ned(Telemetry::GroundSpeedNED ground_speed_ned)
{
    std::lock_guard<std::mutex> lock(_ground_speed_ned_mutex);
    _ground_speed_ned = ground_speed_ned;
}

Telemetry::GPSInfo APMTelemetryImpl::get_gps_info() const
{
    std::lock_guard<std::mutex> lock(_gps_info_mutex);
    return _gps_info;
}

void APMTelemetryImpl::set_gps_info(Telemetry::GPSInfo gps_info)
{
    std::lock_guard<std::mutex> lock(_gps_info_mutex);
    _gps_info = gps_info;
}

Telemetry::Battery APMTelemetryImpl::get_battery() const
{
    std::lock_guard<std::mutex> lock(_battery_mutex);
    return _battery;
}

void APMTelemetryImpl::set_battery(Telemetry::Battery battery)
{
    std::lock_guard<std::mutex> lock(_battery_mutex);
    _battery = battery;
}

Telemetry::FlightMode APMTelemetryImpl::get_flight_mode() const
{
    std::lock_guard<std::mutex> lock(_flight_mode_mutex);
    return _flight_mode;
}

void APMTelemetryImpl::set_flight_mode(Telemetry::FlightMode flight_mode)
{
    std::lock_guard<std::mutex> lock(_flight_mode_mutex);
    _flight_mode = flight_mode;
}

Telemetry::Health APMTelemetryImpl::get_health() const
{
    std::lock_guard<std::mutex> lock(_health_mutex);
    return _health;
}

bool APMTelemetryImpl::get_health_all_ok() const
{
    std::lock_guard<std::mutex> lock(_health_mutex);
    if (_health.gyrometer_calibration_ok &&
        _health.accelerometer_calibration_ok &&
        _health.magnetometer_calibration_ok &&
        _health.level_calibration_ok &&
        _health.local_position_ok &&
        _health.global_position_ok &&
        _health.home_position_ok) {
        return true;
    } else {
        return false;
    }
}

Telemetry::RCStatus APMTelemetryImpl::get_rc_status() const
{
    std::lock_guard<std::mutex> lock(_rc_status_mutex);
    return _rc_status;
}

void APMTelemetryImpl::set_health_local_position(bool ok)
{
    std::lock_guard<std::mutex> lock(_health_mutex);
    _health.local_position_ok = ok;
}

void APMTelemetryImpl::set_health_global_position(bool ok)
{
    std::lock_guard<std::mutex> lock(_health_mutex);
    _health.global_position_ok = ok;
}

void APMTelemetryImpl::set_health_home_position(bool ok)
{
    std::lock_guard<std::mutex> lock(_health_mutex);
    _health.home_position_ok = ok;
}

void APMTelemetryImpl::set_health_gyrometer_calibration(bool ok)
{
    std::lock_guard<std::mutex> lock(_health_mutex);
    _health.gyrometer_calibration_ok = ok;
}

void APMTelemetryImpl::set_health_accelerometer_calibration(bool ok)
{
    std::lock_guard<std::mutex> lock(_health_mutex);
    _health.accelerometer_calibration_ok = ok;
}

void APMTelemetryImpl::set_health_magnetometer_calibration(bool ok)
{
    std::lock_guard<std::mutex> lock(_health_mutex);
    _health.magnetometer_calibration_ok = ok;
}

void APMTelemetryImpl::set_health_level_calibration(bool ok)
{
    std::lock_guard<std::mutex> lock(_health_mutex);
    _health.level_calibration_ok = ok;
}

void APMTelemetryImpl::set_rc_status(bool available, float signal_strength_percent)
{
    std::lock_guard<std::mutex> lock(_rc_status_mutex);

    if (available) {
        _rc_status.available_once = true;
        _rc_status.signal_strength_percent = signal_strength_percent;
    } else {
        _rc_status.signal_strength_percent = 0.0f;
    }

    _rc_status.lost = !available;

}

void APMTelemetryImpl::position_async(Telemetry::position_callback_t &callback)
{
    _position_subscription = callback;
}

void APMTelemetryImpl::home_position_async(Telemetry::position_callback_t &callback)
{
    _home_position_subscription = callback;
}

void APMTelemetryImpl::in_air_async(Telemetry::in_air_callback_t &callback)
{
    _in_air_subscription = callback;
}

void APMTelemetryImpl::armed_async(Telemetry::armed_callback_t &callback)
{
    _armed_subscription = callback;
}

void APMTelemetryImpl::attitude_quaternion_async(Telemetry::attitude_quaternion_callback_t &callback)
{
    _attitude_quaternion_subscription = callback;
}

void APMTelemetryImpl::attitude_euler_angle_async(Telemetry::attitude_euler_angle_callback_t
                                               &callback)
{
    _attitude_euler_angle_subscription = callback;
}

void APMTelemetryImpl::camera_attitude_quaternion_async(Telemetry::attitude_quaternion_callback_t
                                                     &callback)
{
    _camera_attitude_quaternion_subscription = callback;
}

void APMTelemetryImpl::camera_attitude_euler_angle_async(Telemetry::attitude_euler_angle_callback_t
                                                      &callback)
{
    _camera_attitude_euler_angle_subscription = callback;
}

void APMTelemetryImpl::ground_speed_ned_async(Telemetry::ground_speed_ned_callback_t &callback)
{
    _ground_speed_ned_subscription = callback;
}

void APMTelemetryImpl::gps_info_async(Telemetry::gps_info_callback_t &callback)
{
    _gps_info_subscription = callback;
}

void APMTelemetryImpl::battery_async(Telemetry::battery_callback_t &callback)
{
    _battery_subscription = callback;
}

void APMTelemetryImpl::flight_mode_async(Telemetry::flight_mode_callback_t &callback)
{
    _flight_mode_subscription = callback;
}

void APMTelemetryImpl::health_async(Telemetry::health_callback_t &callback)
{
    _health_subscription = callback;
}

void APMTelemetryImpl::health_all_ok_async(Telemetry::health_all_ok_callback_t &callback)
{
    _health_all_ok_subscription = callback;
}

void APMTelemetryImpl::rc_status_async(Telemetry::rc_status_callback_t &callback)
{
    _rc_status_subscription = callback;
}

} // namespace dronecore