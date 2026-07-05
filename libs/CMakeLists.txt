add_subdirectory(core)
add_subdirectory(safety)
add_subdirectory(fdm)
add_subdirectory(sim)
add_subdirectory(engagement)

if(JUMPINGSPIDER_BUILD_VISION)
    add_subdirectory(vision)
endif()

jumpingspider_add_coverage_report()
jumpingspider_add_traceability(jumpingspider_core)
