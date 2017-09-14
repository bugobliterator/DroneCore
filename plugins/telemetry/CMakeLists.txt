set(class_name
    Telemetry
    PARENT_SCOPE
)

set(source_files
    telemetry.cpp
    telemetry_px4.cpp
    telemetry_apm.cpp
    math_conversions.cpp
    PARENT_SCOPE
)

set(header_files
    telemetry.h
    PARENT_SCOPE
)


set(impl_header_files
    telemetry_impl.h
    telemetry_px4.h
    telemetry_apm.h
    PARENT_SCOPE
)
