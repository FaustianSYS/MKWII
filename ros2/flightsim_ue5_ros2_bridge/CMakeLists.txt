cmake_minimum_required(VERSION 3.15)
project(flightsim_ue5_ros2_bridge)

# Build with the system C++ toolchain so rclcpp gets its own libc++/libstdc++.
# UE5 will dlopen() this library; it must NOT be linked against UE5's libc++.
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(flightsim_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(${PROJECT_NAME} SHARED
    src/fsros2_bridge.cpp
)

# Make the wrapper self-contained when Unreal loads it directly from the
# plugin, without relying on the user's shell LD_LIBRARY_PATH.  ROS 2 ament
# normally handles system package paths, but flightsim_msgs lives in a sibling
# overlay package, so add that relative path explicitly.
set_target_properties(${PROJECT_NAME} PROPERTIES
    BUILD_RPATH "$ORIGIN;$ORIGIN/../../flightsim_msgs/lib;/opt/ros/jazzy/lib"
    INSTALL_RPATH "$ORIGIN;$ORIGIN/../../flightsim_msgs/lib;/opt/ros/jazzy/lib"
)

set(FLIGHTSIM_MSGS_LIB_DIR "${CMAKE_INSTALL_PREFIX}/../flightsim_msgs/lib")
target_link_options(${PROJECT_NAME} PRIVATE "LINKER:--no-as-needed")
target_link_libraries(${PROJECT_NAME}
    "${FLIGHTSIM_MSGS_LIB_DIR}/libflightsim_msgs__rosidl_generator_c.so"
    "${FLIGHTSIM_MSGS_LIB_DIR}/libflightsim_msgs__rosidl_typesupport_c.so"
    "${FLIGHTSIM_MSGS_LIB_DIR}/libflightsim_msgs__rosidl_typesupport_cpp.so"
)

target_include_directories(${PROJECT_NAME}
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
    rclcpp
    flightsim_msgs
    sensor_msgs
)

# Install the library and the C header.
install(TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}Targets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
)

install(FILES include/fsros2_bridge.h
    DESTINATION include
)

ament_package()
