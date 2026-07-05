# SafetyProfile.cmake — DO-178C / DO-332 restricted C++ compile profile

function(flightsim_apply_global_safety_flags)
    add_compile_options(
        -Wall -Wextra -Werror
        -fno-associative-math
    )
endfunction()

add_library(flightsim_safety_policy INTERFACE)
add_library(flightsim::safety_policy ALIAS flightsim_safety_policy)

target_compile_features(flightsim_safety_policy INTERFACE cxx_std_17)

target_compile_options(flightsim_safety_policy INTERFACE
    -Wall -Wextra -Werror
    -fno-rtti
    -ffunction-sections
    -fdata-sections
    -fno-associative-math
)

target_compile_definitions(flightsim_safety_policy INTERFACE
    FLIGHTSIM_SAFETY_CRITICAL=1
)

if(NOT FLIGHTSIM_ENABLE_EXCEPTIONS)
    target_compile_options(flightsim_safety_policy INTERFACE -fno-exceptions)
endif()

add_library(flightsim_deterministic INTERFACE)
add_library(flightsim::deterministic ALIAS flightsim_deterministic)

target_link_libraries(flightsim_deterministic INTERFACE flightsim::safety_policy)

target_compile_definitions(flightsim_deterministic INTERFACE
    FLIGHTSIM_DETERMINISTIC=1
)

function(flightsim_apply_safety_profile target)
    target_link_libraries(${target} PUBLIC flightsim::deterministic)
endfunction()

function(flightsim_register_static_analysis target)
    if(NOT TARGET flightsim_static_analysis)
        add_custom_target(flightsim_static_analysis
            COMMENT "Static analysis placeholder — run via CI"
        )
    endif()
    add_dependencies(flightsim_static_analysis ${target})
endfunction()

install(TARGETS flightsim_safety_policy flightsim_deterministic
    EXPORT FlightSimTargets
)
