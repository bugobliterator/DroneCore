#include "dronecore_impl.h"
#include "global_include.h"
#include <mutex>
#include "device_px4.h"
#include "device_apm.h"

namespace dronecore {

DroneCoreImpl::DroneCoreImpl() :
    _connections_mutex(),
    _connections(),
    _devices_mutex(),
    _devices(),
    _device_impls(),
    _on_discover_callback(nullptr),
    _on_timeout_callback(nullptr)
{}

DroneCoreImpl::~DroneCoreImpl()
{
    {
        std::lock_guard<std::recursive_mutex> lock(_devices_mutex);
        _should_exit = true;

        for (auto it = _devices.begin(); it != _devices.end(); ++it) {
            delete it->second;
        }

        for (auto it = _device_impls.begin(); it != _device_impls.end(); ++it) {
            delete it->second;
        }
        _devices.clear();
        _device_impls.clear();
    }

    std::vector<Connection *> tmp_connections;
    {
        std::lock_guard<std::mutex> lock(_connections_mutex);

        // We need to copy the connections to a temporary vector. This way they won't
        // get used anymore while they are cleaned up.
        tmp_connections = _connections;
        _connections.clear();
    }

    for (auto connection : tmp_connections) {
        delete connection;
    }

}

void DroneCoreImpl::receive_message(const mavlink_message_t &message)
{
    // Don't ever create a device with sysid 0.
    if (message.sysid == 0) {
        return;
    }

    // FIXME: Ignore messages from QGroundControl for now. Usually QGC identifies
    //        itself with sysid 255.
    //        A better way would probably be to parse the heartbeat message and
    //        look at type and check if it is MAV_TYPE_GCS.
    if (message.sysid == 255) {
        return;
    }

    std::lock_guard<std::recursive_mutex> lock(_devices_mutex);

    // Change system id of null device
    if (_devices.find(0) != _devices.end()) {
        auto null_device = _devices[0];
        _devices.erase(0);
        _devices.insert(std::pair<uint8_t, Device *>(message.sysid, null_device));
    }

    if (_device_impls.find(0) != _device_impls.end()) {
        auto null_device_impl = _device_impls[0];
        _device_impls.erase(0);
        null_device_impl->set_target_system_id(message.sysid);
        _device_impls.insert(std::pair<uint8_t, DeviceImpl *>(message.sysid, null_device_impl));
    }

    if (message.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
        mavlink_heartbeat_t hbeat;
        mavlink_msg_heartbeat_decode(&message, &hbeat);

        create_device_if_not_existing(message.sysid, hbeat.autopilot);
    }


    if (_should_exit) {
        // Don't try to call at() if devices have already been destroyed
        // in descructor.
        return;
    }

    if (message.sysid != 1) {
        Debug() << "sysid: " << int(message.sysid);
    }

    if (_device_impls.find(message.sysid) != _device_impls.end()) {
        _device_impls.at(message.sysid)->process_mavlink_message(message);
    }
}

bool DroneCoreImpl::send_message(const mavlink_message_t &message)
{
    std::lock_guard<std::mutex> lock(_connections_mutex);

    for (auto it = _connections.begin(); it != _connections.end(); ++it) {
        if (!(**it).send_message(message)) {
            Debug() << "send fail";
            return false;
        }
    }

    return true;
}

void DroneCoreImpl::add_connection(Connection *new_connection)
{
    std::lock_guard<std::mutex> lock(_connections_mutex);
    _connections.push_back(new_connection);
}

const std::vector<uint64_t> &DroneCoreImpl::get_device_uuids() const
{
    // This needs to survive the scope but we need to clean it up.
    static std::vector<uint64_t> uuids = {};
    uuids.clear();

    for (auto it = _device_impls.begin(); it != _device_impls.end(); ++it) {
        uint64_t uuid = it->second->get_target_uuid();
        if (uuid != 0) {
            uuids.push_back(uuid);
        }
    }

    return uuids;
}

Device &DroneCoreImpl::get_device()
{
    {
        std::lock_guard<std::recursive_mutex> lock(_devices_mutex);
        // In get_device withoiut uuid, we expect to have only
        // one device conneted.
        if (_device_impls.size() == 1) {
            return *(_devices.at(_device_impls.begin()->first));
        }

        if (_device_impls.size() > 1) {
            Debug() << "Error: more than one device found:";

            int i = 0;
            for (auto it = _device_impls.begin(); it != _device_impls.end(); ++it) {
                Debug() << "strange: " << i << ": " << int(it->first) << ", " << it->second;
                ++i;
            }

            // Just return first device instead of failing.
            return *_devices.begin()->second;
        } else {
            Debug() << "Error: no device found.";

            uint8_t system_id = 0;
            uint8_t dev_type = MAV_AUTOPILOT_PX4;

            create_device_if_not_existing(system_id, dev_type);
            return *_devices[system_id];
        }
    }
}

Device &DroneCoreImpl::get_device(uint64_t uuid)
{
    {
        std::lock_guard<std::recursive_mutex> lock(_devices_mutex);
        // TODO: make a cache map for this.
        for (auto it = _device_impls.begin(); it != _device_impls.end(); ++it) {
            if (it->second->get_target_uuid() == uuid) {
                return *(_devices.at(it->first));
            }
        }
    }

    // We have not found a device with this UUID.
    // TODO: this is an error condition that we ought to handle properly.
    Debug() << "device with UUID: " << uuid << " not found";

    // Create a dummy
    uint8_t system_id = 0;
    uint8_t dev_type = MAV_AUTOPILOT_PX4;

    create_device_if_not_existing(system_id, dev_type);

    return *_devices[system_id];
}

void DroneCoreImpl::create_device_if_not_existing(uint8_t system_id, uint8_t dev_type)
{
    std::lock_guard<std::recursive_mutex> lock(_devices_mutex);

    if (_should_exit) {
        // When the device_impl got destroyed in the destructor, we have to give up.
        return;
    }

    // existing already.
    if (_devices.find(system_id) != _devices.end()) {
        //Debug() << "ID: " << int(system_id) << " exists already.";
        return;
    }

    // Create both lists in parallel
    switch (dev_type) {
    case MAV_AUTOPILOT_PX4: {
        DeviceImpl *px4_device_impl = new PX4DeviceImpl(this, system_id);
        _device_impls.insert(std::pair<uint8_t, DeviceImpl *>(system_id, px4_device_impl));

        Device *px4_device = new Device(px4_device_impl);
        _devices.insert(std::pair<uint8_t, Device *>(system_id, px4_device));
        break; 
    }
    case MAV_AUTOPILOT_ARDUPILOTMEGA: {
        DeviceImpl *apm_device_impl = new APMDeviceImpl(this, system_id);
        _device_impls.insert(std::pair<uint8_t, DeviceImpl *>(system_id, apm_device_impl));

        Device *apm_device = new Device(apm_device_impl);
        _devices.insert(std::pair<uint8_t, Device *>(system_id, apm_device));
        break;
    }
    default : {
        DeviceImpl *default_device_impl = new PX4DeviceImpl(this, system_id);
        _device_impls.insert(std::pair<uint8_t, DeviceImpl *>(system_id, default_device_impl));

        Device *default_device = new Device(default_device_impl);
        _devices.insert(std::pair<uint8_t, Device *>(system_id, default_device));
        break;
    }
    };

}

void DroneCoreImpl::notify_on_discover(uint64_t uuid)
{
    if (_on_discover_callback != nullptr) {
        _on_discover_callback(uuid);
    }
}

void DroneCoreImpl::notify_on_timeout(uint64_t uuid)
{
    if (_on_timeout_callback != nullptr) {
        _on_timeout_callback(uuid);
    }
}

void DroneCoreImpl::register_on_discover(DroneCore::event_callback_t callback)
{
    _on_discover_callback = callback;
}

void DroneCoreImpl::register_on_timeout(DroneCore::event_callback_t callback)
{
    _on_timeout_callback = callback;
}

} // namespace dronecore
