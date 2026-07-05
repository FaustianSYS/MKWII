#!/usr/bin/env bash
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck disable=SC1091
source "${ROOT}/scripts/build_profile.sh"

export FLIGHTSIM_BUILD_PROFILE=flight
flightsim_resolve_build_profile

EXTRA_CMAKE_ARGS=()
if [[ "${FLIGHTSIM_BUILD_GUI:-ON}" == "ON" ]]; then
  EXTRA_CMAKE_ARGS+=("-DFLIGHTSIM_BUILD_GUI=ON")
fi

echo "==> Configuring flight build (${FLIGHTSIM_BUILD_DIR})"
cmake --preset "${FLIGHTSIM_CMAKE_PRESET}" -B "${FLIGHTSIM_BUILD_DIR}" "${EXTRA_CMAKE_ARGS[@]}"

echo "==> Building flight (release)"
cmake --build "${FLIGHTSIM_BUILD_DIR}"

echo ""
echo "Flight build ready:"
echo "  ${FLIGHTSIM_BUILD_DIR}/apps/engagement_runner/engagement_runner"
echo "  ${FLIGHTSIM_BUILD_DIR}/apps/tactical_gui/flightsim_tactical_gui"
