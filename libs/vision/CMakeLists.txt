add_library(flightsim_vision STATIC
    src/camera_model.cpp
    src/seeker_image_processor.cpp
)

target_include_directories(flightsim_vision
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(flightsim_vision
    PUBLIC flightsim_core
)

install(TARGETS flightsim_vision EXPORT FlightSimTargets)
