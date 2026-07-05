# Tool Qualification Plan (DO-330)

## 1. Tools Requiring Qualification

| Tool | Version | Use | TQL |
|------|---------|-----|-----|
| GCC / G++ | 13+ | Compiler | 3 |
| CMake | 3.24+ | Build generator | 5 |
| clang-tidy | 17+ | Static analysis | 2 |
| cppcheck | 2.x | Static analysis | 2 |
| gcov/lcov | system | Coverage | 3 |

## 2. Qualification Approach

- **TQL 2:** Verify tool detects known violation corpus in `tools/static_analysis/test_samples/`
- **TQL 3:** Compare compiler output against golden object behavior on reference snippets
- **TQL 5:** Document CMake reproducible build on reference platform

## 3. In-House Test Harness

The minimal unit test harness in `tests/support/minimal_test.hpp` is developed in-house and verified by review; no third-party test framework dependency.

## 4. Records

- Tool version captured in CI logs
- Qualification evidence stored under `docs/lifecycle/tool_qualification/`

## 5. Re-qualification Triggers

Tool major version upgrade, compiler switch, or changed verification criteria.
