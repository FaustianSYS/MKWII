# Software Configuration Management Plan (SCMP)

## 1. Baseline Items

- Source code under `libs/`, `include/`, `apps/`
- CMake build definitions and presets
- Lifecycle and safety documentation
- Verification tests and golden data

## 2. Version Control

- Git repository with tagged releases
- Protected main branch; changes via reviewed merge

## 3. Change Control

Each change shall:

1. Reference affected requirements (HLR/LLR/SSR)
2. Include updated verification evidence
3. Regenerate RTM when requirements or trace tags change

## 4. Build Identification

- `project(FlightSim VERSION x.y.z)` in root `CMakeLists.txt`
- Build artifacts stored with commit hash in CI

## 5. Problem Reporting

Track anomalies in project issue tracker with hazard/safety cross-reference when applicable.
