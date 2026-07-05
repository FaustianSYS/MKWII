# Cert-oriented GCC toolchain flags for cross-compilation or dedicated safety builds

set(CMAKE_CXX_FLAGS_INIT "-Wall -Wextra -Werror -fno-rtti -fno-exceptions -fno-associative-math")
set(CMAKE_CXX_FLAGS_RELEASE_INIT "-O2 -DNDEBUG")
set(CMAKE_CXX_FLAGS_DEBUG_INIT "-O0 -g")

set(FLIGHTSIM_SAFETY_BUILD ON CACHE BOOL "" FORCE)
