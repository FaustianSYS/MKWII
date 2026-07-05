# Traceability.cmake — RTM generation from @req tags in source

function(flightsim_add_traceability target)
    set(TRACE_GEN_SCRIPT "${CMAKE_SOURCE_DIR}/tools/trace_gen/trace_gen.py")
    if(EXISTS "${TRACE_GEN_SCRIPT}")
        add_custom_target(traceability
            COMMAND ${CMAKE_COMMAND} -E make_directory "${CMAKE_BINARY_DIR}/traceability"
            COMMAND python3 "${TRACE_GEN_SCRIPT}"
                --source-dir "${CMAKE_SOURCE_DIR}"
                --output "${CMAKE_BINARY_DIR}/traceability/RTM.csv"
            WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
            COMMENT "Generating Requirements Traceability Matrix"
        )
        add_dependencies(flightsim_static_analysis traceability)
    endif()
endfunction()
