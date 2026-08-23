#!/usr/bin/env bash
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck disable=SC1091
source "${ROOT}/scripts/build_profile.sh"

export FLIGHTSIM_BUILD_PROFILE=debug
flightsim_resolve_build_profile

echo "==> Configuring debug build (${FLIGHTSIM_BUILD_DIR})"
cmake --preset "${FLIGHTSIM_CMAKE_PRESET}" -B "${FLIGHTSIM_BUILD_DIR}"

echo "==> Building debug"
cmake --build "${FLIGHTSIM_BUILD_DIR}"

if [[ "${BUILD_TESTING:-ON}" == "ON" ]]; then
  echo "==> Running debug tests"
  ctest --preset debug --output-on-failure
fi

echo ""
echo "Debug build ready:"
echo "  ${FLIGHTSIM_BUILD_DIR}/apps/engagement_runner/engagement_runner"
