add_executable(engagement_runner main.cpp)

target_link_libraries(engagement_runner
    PRIVATE flightsim_engagement
)

install(TARGETS engagement_runner RUNTIME DESTINATION bin)
