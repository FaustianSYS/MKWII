add_library(flightsim_sim STATIC
    src/scheduler.cpp
)

target_include_directories(flightsim_sim
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(flightsim_sim
    PUBLIC flightsim_fdm flightsim_safety flightsim_core
)

flightsim_apply_safety_profile(flightsim_sim)
flightsim_enable_coverage(flightsim_sim)
flightsim_register_static_analysis(flightsim_sim)

install(TARGETS flightsim_sim EXPORT FlightSimTargets)
