add_executable(air_defense_runner main.cpp)

target_link_libraries(air_defense_runner
    PRIVATE flightsim_engagement
)

install(TARGETS air_defense_runner RUNTIME DESTINATION bin)
