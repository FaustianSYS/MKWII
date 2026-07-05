# High-Level Requirements (HLR)

| ID | Text | SSR |
|----|------|-----|
| HLR-CORE-001 | The FDM shall provide bounded, deterministic vector and quaternion math operations. | SSR-001 |
| HLR-CORE-002 | The FDM shall compute ISA atmosphere properties for altitude inputs. | SSR-002 |
| HLR-SAF-001 | The system shall detect and register software faults in a latched fault register. | SSR-003 |
| HLR-SAF-002 | The system shall enter safe mode on critical faults and hold last valid state. | SSR-004 |
| HLR-SAF-003 | Control inputs shall be validated and saturated to envelope limits. | SSR-005 |
| HLR-FDM-001 | The FDM shall maintain a fixed-size aircraft state vector. | SSR-001 |
| HLR-FDM-002 | The FDM shall integrate 6-DOF rigid-body equations of motion with fixed timestep. | SSR-001 |
| HLR-FDM-003 | The FDM shall compute aerodynamic and propulsive forces from tabulated models. | SSR-002 |
| HLR-SIM-001 | The simulation scheduler shall execute a deterministic, wall-clock-independent loop. | SSR-001 |
| HLR-SIM-002 | The scheduler shall monitor state validity each integration step. | SSR-003 |

# Low-Level Requirements (LLR)

| ID | Text | HLR |
|----|------|-----|
| LLR-CORE-001 | `is_finite` shall return false for NaN and Inf. | HLR-CORE-001 |
| LLR-CORE-002 | `clamp` shall bound values to [min, max]. | HLR-CORE-001 |
| LLR-CORE-003..016 | Vector, matrix, quaternion operations per `types.hpp`. | HLR-CORE-001 |
| LLR-CORE-017..024 | ISA constants and atmosphere functions. | HLR-CORE-002 |
| LLR-SAF-001 | FaultRegister set/clear/query operations. | HLR-SAF-001 |
| LLR-SAF-002 | SafeModeFsm latched fault on NaN/divergence/watchdog. | HLR-SAF-002 |
| LLR-SAF-003 | SafeModeFsm hold on invalid input. | HLR-SAF-002 |
| LLR-SAF-004..005 | InputGuard saturation and rate limiting. | HLR-SAF-003 |
| LLR-SAF-006..007 | StateMonitor NaN, speed, rate, altitude checks. | HLR-SIM-002 |
| LLR-FDM-001 | AircraftState struct definition. | HLR-FDM-001 |
| LLR-FDM-002 | Deterministic state initialization. | HLR-FDM-001 |
| LLR-FDM-003 | Aero force computation from coefficients. | HLR-FDM-003 |
| LLR-FDM-004 | Propulsion force computation. | HLR-FDM-003 |
| LLR-FDM-005 | EOM integration (semi-implicit Euler / RK4). | HLR-FDM-002 |
| LLR-FDM-006 | Environment density update from altitude. | HLR-CORE-002 |
| LLR-SIM-001..004 | Scheduler init, step, run, watchdog. | HLR-SIM-001 |
