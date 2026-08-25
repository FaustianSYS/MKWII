cmake_minimum_required(VERSION 3.24)
project(flightsim_ros)

if(NOT FLIGHTSIM_BUILD_DIR)
  if(DEFINED FLIGHTSIM_BUILD_PROFILE AND NOT FLIGHTSIM_BUILD_PROFILE STREQUAL "")
    set(FLIGHTSIM_BUILD_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../build/${FLIGHTSIM_BUILD_PROFILE}" CACHE PATH
        "Path to FlightSim core library build directory")
  else()
    set(FLIGHTSIM_BUILD_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../build/debug" CACHE PATH
        "Path to FlightSim core library build directory")
  endif()
endif()

if(NOT DEFINED FLIGHTSIM_BUILD_VISION)
  set(FLIGHTSIM_BUILD_VISION ON)
endif()

set(FLIGHTSIM_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../..")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(flightsim_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# ── FDM node ─────────────────────────────────────────────────────────────────
add_executable(flightsim_fdm_node src/fdm_node.cpp)

target_include_directories(flightsim_fdm_node PRIVATE
  "${FLIGHTSIM_ROOT}/include"
)

target_link_directories(flightsim_fdm_node PRIVATE
  "${FLIGHTSIM_BUILD_DIR}/libs/sim"
  "${FLIGHTSIM_BUILD_DIR}/libs/fdm"
  "${FLIGHTSIM_BUILD_DIR}/libs/safety"
  "${FLIGHTSIM_BUILD_DIR}/libs/core"
)

ament_target_dependencies(flightsim_fdm_node rclcpp flightsim_msgs)

target_link_libraries(flightsim_fdm_node
  flightsim_sim
  flightsim_fdm
  flightsim_safety
  flightsim_core
)

# ── Engagement node ───────────────────────────────────────────────────────────
add_executable(flightsim_engagement_node
  src/engagement_node.cpp
  src/ros_seeker_track_source.cpp
)

target_include_directories(flightsim_engagement_node PRIVATE
  "${FLIGHTSIM_ROOT}/include"
  "${CMAKE_CURRENT_SOURCE_DIR}/src"
)

target_link_directories(flightsim_engagement_node PRIVATE
  "${FLIGHTSIM_BUILD_DIR}/libs/engagement"
  "${FLIGHTSIM_BUILD_DIR}/libs/core"
)

ament_target_dependencies(flightsim_engagement_node rclcpp flightsim_msgs std_msgs)

target_link_libraries(flightsim_engagement_node
  flightsim_engagement
  flightsim_core
)

set(_install_targets flightsim_fdm_node flightsim_engagement_node)

# ── Vision + UE5 software bridge ──────────────────────────────────────────────
if(FLIGHTSIM_BUILD_VISION)
  add_executable(flightsim_vision_node src/vision_node.cpp)

  target_include_directories(flightsim_vision_node PRIVATE
    "${FLIGHTSIM_ROOT}/include"
  )

  target_link_directories(flightsim_vision_node PRIVATE
    "${FLIGHTSIM_BUILD_DIR}/libs/vision"
    "${FLIGHTSIM_BUILD_DIR}/libs/core"
  )

  ament_target_dependencies(flightsim_vision_node rclcpp flightsim_msgs sensor_msgs)

  target_link_libraries(flightsim_vision_node
    flightsim_vision
    flightsim_core
  )

  # Software stand-in for the real UE5 plugin (used when UE5 is not running).
  add_executable(flightsim_ue5_bridge_node src/ue5_bridge_node.cpp)

  target_include_directories(flightsim_ue5_bridge_node PRIVATE
    "${FLIGHTSIM_ROOT}/include"
  )

  target_link_directories(flightsim_ue5_bridge_node PRIVATE
    "${FLIGHTSIM_BUILD_DIR}/libs/vision"
    "${FLIGHTSIM_BUILD_DIR}/libs/core"
  )

  ament_target_dependencies(flightsim_ue5_bridge_node rclcpp flightsim_msgs sensor_msgs)

  target_link_libraries(flightsim_ue5_bridge_node
    flightsim_vision
    flightsim_core
  )

  list(APPEND _install_targets flightsim_vision_node flightsim_ue5_bridge_node)
endif()

install(TARGETS ${_install_targets}
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
