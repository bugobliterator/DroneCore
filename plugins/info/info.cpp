#include "info.h"
#include "info_impl.h"
#include "device_impl.h"
namespace dronecore {

Info::Info(DeviceImpl *device) :
    _device(device)
{
    _impl = new InfoImpl();
}

Info::~Info()
{
    if(_impl != nullptr) {
        delete _impl;
    }
}
uint64_t Info::uuid() const
{
    return _impl->get_uuid();
}

bool Info::is_complete() const
{
    return _impl->is_complete();
}

Info::Version Info::get_version() const
{
    return _impl->get_version();
}

} // namespace dronecore
