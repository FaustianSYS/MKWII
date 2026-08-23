add_subdirectory(core)
add_subdirectory(safety)
add_subdirectory(fdm)
add_subdirectory(sim)
add_subdirectory(engagement)

if(FLIGHTSIM_BUILD_VISION)
    add_subdirectory(vision)
endif()

flightsim_add_coverage_report()
flightsim_add_traceability(flightsim_core)
