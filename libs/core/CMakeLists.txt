add_library(flightsim_core STATIC
    src/atmosphere.cpp
)

target_include_directories(flightsim_core
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

flightsim_apply_safety_profile(flightsim_core)
flightsim_enable_coverage(flightsim_core)
flightsim_register_static_analysis(flightsim_core)

install(TARGETS flightsim_core EXPORT FlightSimTargets)
