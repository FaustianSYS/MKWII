add_executable(flightsim_unit_tests
    main.cpp
    test_core.cpp
    test_safety.cpp
    test_fdm.cpp
    test_engagement.cpp
)

if(FLIGHTSIM_BUILD_VISION)
    target_sources(flightsim_unit_tests PRIVATE test_vision.cpp)
    target_compile_definitions(flightsim_unit_tests PRIVATE FLIGHTSIM_HAS_VISION=1)
    target_link_libraries(flightsim_unit_tests PRIVATE flightsim_vision)
endif()

target_include_directories(flightsim_unit_tests PRIVATE ${CMAKE_SOURCE_DIR}/tests/support)

target_link_libraries(flightsim_unit_tests
    PRIVATE
        flightsim_sim
        flightsim_fdm
        flightsim_safety
        flightsim_core
        flightsim_engagement
)

target_compile_options(flightsim_unit_tests PRIVATE -fno-rtti)

flightsim_enable_coverage(flightsim_unit_tests)

add_test(NAME unit COMMAND flightsim_unit_tests)
