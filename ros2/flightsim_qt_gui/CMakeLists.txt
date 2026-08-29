cmake_minimum_required(VERSION 3.24)
project(flightsim_qt_gui)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(flightsim_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Gui Widgets OpenGL)
find_package(OpenGL REQUIRED)

add_executable(flightsim_qt_tactical_gui
  src/main.cpp
  src/main_window.cpp
  src/main_window.hpp
  src/camera_view_widget.cpp
  src/camera_view_widget.hpp
  src/obj_mesh.cpp
  src/obj_mesh.hpp
  src/shahed_model.cpp
  src/shahed_model.hpp
  src/track_view_3d.cpp
  src/track_view_3d.hpp
  src/telemetry_bridge.cpp
  src/telemetry_bridge.hpp
)

target_include_directories(flightsim_qt_tactical_gui PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/src
)

ament_target_dependencies(flightsim_qt_tactical_gui
  ament_index_cpp
  rclcpp
  flightsim_msgs
  std_msgs
)

target_link_libraries(flightsim_qt_tactical_gui
  Qt5::Core
  Qt5::Gui
  Qt5::Widgets
  Qt5::OpenGL
  OpenGL::GL
)

install(TARGETS flightsim_qt_tactical_gui
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

set(FLIGHTSIM_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../..")
install(FILES
  "${FLIGHTSIM_ROOT}/assets/ue5/models/shahed_136/source/shahed_136.obj"
  DESTINATION share/${PROJECT_NAME}/models
)

ament_package()
