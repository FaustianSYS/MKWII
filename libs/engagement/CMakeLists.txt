add_library(flightsim_engagement STATIC
    src/missile/missile_object.cpp
    src/missile/missile_eom.cpp
    src/missile/missile_control_allocation.cpp
    src/missile/missile_mass_properties.cpp
    src/missile/missile.cpp
    src/missile/geometric_seeker_source.cpp
    src/target/target.cpp
    src/scenario/scenario.cpp
    src/scenario/scene_entity.cpp
)

target_include_directories(flightsim_engagement
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(flightsim_engagement
    PUBLIC flightsim_core
)

install(TARGETS flightsim_engagement EXPORT FlightSimTargets)
