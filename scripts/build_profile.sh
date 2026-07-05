#!/usr/bin/env bash
# Shared build profile resolution for FlightSim scripts.
# Source this file, then call flightsim_resolve_build_profile.

flightsim_resolve_build_profile() {
  FLIGHTSIM_ROOT="${FLIGHTSIM_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
  FLIGHTSIM_BUILD_PROFILE="${FLIGHTSIM_BUILD_PROFILE:-debug}"

  case "${FLIGHTSIM_BUILD_PROFILE}" in
    debug)
      FLIGHTSIM_CMAKE_PRESET="debug"
      ;;
    flight)
      FLIGHTSIM_CMAKE_PRESET="flight"
      ;;
    safety-verify)
      FLIGHTSIM_CMAKE_PRESET="safety-verify"
      ;;
    *)
      echo "Unsupported FLIGHTSIM_BUILD_PROFILE='${FLIGHTSIM_BUILD_PROFILE}' (use debug, flight, or safety-verify)" >&2
      return 1
      ;;
  esac

  FLIGHTSIM_BUILD_DIR="${FLIGHTSIM_BUILD_DIR:-${FLIGHTSIM_ROOT}/build/${FLIGHTSIM_BUILD_PROFILE}}"
  FLIGHTSIM_ROS2_BUILD_DIR="${FLIGHTSIM_ROS2_BUILD_DIR:-${FLIGHTSIM_ROOT}/ros2/build/${FLIGHTSIM_BUILD_PROFILE}}"
  FLIGHTSIM_ROS2_INSTALL_DIR="${FLIGHTSIM_ROS2_INSTALL_DIR:-${FLIGHTSIM_ROOT}/ros2/install/${FLIGHTSIM_BUILD_PROFILE}}"

  export FLIGHTSIM_ROOT
  export FLIGHTSIM_BUILD_PROFILE
  export FLIGHTSIM_CMAKE_PRESET
  export FLIGHTSIM_BUILD_DIR
  export FLIGHTSIM_ROS2_BUILD_DIR
  export FLIGHTSIM_ROS2_INSTALL_DIR
}
