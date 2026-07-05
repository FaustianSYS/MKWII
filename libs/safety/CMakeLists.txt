add_library(flightsim_safety STATIC
    src/safe_mode.cpp
)

target_include_directories(flightsim_safety
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(flightsim_safety
    PUBLIC flightsim_core
)

flightsim_apply_safety_profile(flightsim_safety)
flightsim_enable_coverage(flightsim_safety)
flightsim_register_static_analysis(flightsim_safety)

install(TARGETS flightsim_safety EXPORT FlightSimTargets)
