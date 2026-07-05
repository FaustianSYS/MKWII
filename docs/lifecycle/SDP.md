# Software Development Plan (SDP)

## 1. Development Environment

- Language: C++17 (restricted subset)
- Build system: CMake 3.24+
- Target: host GCC/Clang with safety profile (`-fno-rtti`, `-fno-exceptions`)

## 2. Coding Standards

- MISRA C++:2023 baseline
- No dynamic allocation in real-time path after initialization
- All safety-critical APIs are `noexcept` with explicit error returns

## 3. Design Method

- Fixed-size data structures
- Static libraries with explicit dependency graph
- Requirement tags embedded in source (`@req LLR-xxx`)

## 4. Configuration Management

See [SCMP.md](SCMP.md).

## 5. Module Development Order

1. `libs/core` — math and atmosphere
2. `libs/safety` — fault register, safe mode, monitors
3. `libs/fdm` — physics models
4. `libs/sim` — scheduler
5. `apps/fdm_runner` — headless runner
