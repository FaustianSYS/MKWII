# Coverage.cmake — gcov/lcov instrumentation for verification builds

function(flightsim_enable_coverage target)
    if(NOT FLIGHTSIM_ENABLE_COVERAGE)
        return()
    endif()

    if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
        target_compile_options(${target} PRIVATE --coverage -O0 -g)
        target_link_options(${target} PRIVATE --coverage)
    else()
        message(WARNING "Coverage instrumentation not supported for ${CMAKE_CXX_COMPILER_ID}")
    endif()
endfunction()

function(flightsim_add_coverage_report)
    if(NOT FLIGHTSIM_ENABLE_COVERAGE)
        return()
    endif()

    find_program(LCOV_EXECUTABLE lcov)
    find_program(GENHTML_EXECUTABLE genhtml)

    if(LCOV_EXECUTABLE AND GENHTML_EXECUTABLE)
        add_custom_target(coverage
            COMMAND ${LCOV_EXECUTABLE} --capture --directory . --output-file coverage.info
            COMMAND ${LCOV_EXECUTABLE} --remove coverage.info '/usr/*' '*/tests/*' --output-file coverage.filtered.info
            COMMAND ${GENHTML_EXECUTABLE} coverage.filtered.info --output-directory coverage_html
            WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
            COMMENT "Generating coverage report"
        )
    else()
        add_custom_target(coverage
            COMMENT "Install lcov and genhtml to generate HTML coverage reports"
        )
    endif()
endfunction()
