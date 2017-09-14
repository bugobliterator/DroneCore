#include "logging.h"
#include "logging_impl.h"
#include "device_impl.h"
namespace dronecore {

Logging::Logging(DeviceImpl *device) :
    _device(device)
{
    _impl = new LoggingImpl();
}

Logging::~Logging()
{
    if(_impl != nullptr) {
        delete _impl;
    }
}
Logging::Result Logging::start_logging() const
{
    return _impl->start_logging();
}

Logging::Result Logging::stop_logging() const
{
    return _impl->stop_logging();
}

void Logging::start_logging_async(result_callback_t callback)
{
    _impl->start_logging_async(callback);
}

void Logging::stop_logging_async(result_callback_t callback)
{
    _impl->stop_logging_async(callback);
}

const char *Logging::result_str(Result result)
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
        case Result::TIMEOUT:
            return "Timeout";
        case Result::UNKNOWN:
        default:
            return "Unknown";
    }
}


} // namespace dronecore
