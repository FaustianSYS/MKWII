cmake_minimum_required(VERSION 3.24)

project(FlightSim
    VERSION 0.1.0
    DESCRIPTION "MIL-STD safety-aligned 6-DOF flight dynamics simulator"
    LANGUAGES CXX
)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/modules")

include(SafetyProfile)
include(Coverage)
include(Traceability)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(FLIGHTSIM_SAFETY_BUILD "Enable restricted C++ safety profile" ON)
option(FLIGHTSIM_ENABLE_COVERAGE "Enable coverage instrumentation for verification builds" OFF)
option(FLIGHTSIM_ENABLE_EXCEPTIONS "Allow exceptions in non-safety targets" OFF)
option(FLIGHTSIM_BUILD_VISION "Build C++ vision library for UE5 seeker pipeline" ON)
option(BUILD_TESTING "Build unit and integration tests" ON)
set(FLIGHTSIM_BUILD_PROFILE "debug" CACHE STRING "Active build profile: debug, flight, or safety-verify")

if(FLIGHTSIM_SAFETY_BUILD)
    flightsim_apply_global_safety_flags()
endif()

add_subdirectory(libs)
add_subdirectory(apps)

if(BUILD_TESTING)
    enable_testing()
    add_subdirectory(tests)
endif()

install(DIRECTORY include/ DESTINATION include)

install(EXPORT FlightSimTargets
    FILE FlightSimTargets.cmake
    NAMESPACE flightsim::
    DESTINATION lib/cmake/FlightSim
)
