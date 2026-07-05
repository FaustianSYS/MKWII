# Plan for Software Aspects of Certification (PSAC)

## 1. Scope

This PSAC covers the FlightSim safety-critical 6-DOF flight dynamics model (FDM) and deterministic simulation loop.

| Module | DAL | Rationale |
|--------|-----|-----------|
| `libs/core` | B | Foundational math; anomaly affects downstream state |
| `libs/safety` | A | Fault detection and safe-mode transitions |
| `libs/fdm` | A | Equations of motion and force models |
| `libs/sim` | A | Real-time scheduler and integration orchestration |
| `apps/fdm_runner` | B | Verification harness / host executable |

## 2. Standards Compliance

- MIL-STD-882E — system safety process
- DO-178C — software lifecycle objectives
- DO-332 — C++ object-oriented technology supplement
- MISRA C++:2023 — coding standard subset

## 3. Software Life Cycle

Planning, development, verification, configuration management, quality assurance, and certification liaison per DO-178C Sections 4–11.

## 4. Tool Qualification

See [tool_qualification_plan.md](tool_qualification_plan.md).

## 5. Deliverables

- Software requirements (HLR/LLR)
- Design data (headers, module dependency graph)
- Source code and trace tags
- Verification results (unit, integration, coverage, static analysis)
- Requirements Traceability Matrix (RTM)
