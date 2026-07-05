add_executable(flightsim_integration_tests
    main.cpp
    test_scenarios.cpp
)

target_include_directories(flightsim_integration_tests PRIVATE ${CMAKE_SOURCE_DIR}/tests/support)

target_link_libraries(flightsim_integration_tests
    PRIVATE flightsim_sim
)

target_compile_options(flightsim_integration_tests PRIVATE -fno-rtti)

flightsim_enable_coverage(flightsim_integration_tests)

add_test(NAME integration COMMAND flightsim_integration_tests)
