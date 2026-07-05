add_executable(fdm_runner main.cpp)

target_link_libraries(fdm_runner
    PRIVATE flightsim_sim
)

flightsim_apply_safety_profile(fdm_runner)
flightsim_enable_coverage(fdm_runner)

install(TARGETS fdm_runner RUNTIME DESTINATION bin)
