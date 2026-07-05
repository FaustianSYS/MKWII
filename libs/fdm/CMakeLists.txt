add_library(flightsim_fdm STATIC
    src/init.cpp
    src/environment.cpp
    src/aero.cpp
    src/propulsion.cpp
    src/eom.cpp
)

target_include_directories(flightsim_fdm
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(flightsim_fdm
    PUBLIC flightsim_core flightsim_safety
)

flightsim_apply_safety_profile(flightsim_fdm)
flightsim_enable_coverage(flightsim_fdm)
flightsim_register_static_analysis(flightsim_fdm)

install(TARGETS flightsim_fdm EXPORT FlightSimTargets)
