# Independent Review Checklist

## Requirements Review

- [ ] All HLRs trace to system safety requirements (SSR)
- [ ] All LLRs trace to HLRs
- [ ] No orphan requirements or untagged safety-critical functions

## Design Review

- [ ] Module dependency graph matches implementation (no upward deps)
- [ ] No dynamic allocation in RT path after init
- [ ] Safe-mode FSM transitions reviewed for completeness

## Code Review

- [ ] MISRA / coding standard violations resolved or formally deviated
- [ ] `@req` tags present on all safety-critical functions
- [ ] `@mcdc` functions identified for DAL A logic

## Verification Review

- [ ] Unit and integration tests pass in CI
- [ ] RTM generated and complete
- [ ] Coverage meets DAL thresholds
- [ ] Static analysis clean

## Safety Review

- [ ] Hazard log hazards mitigated by requirements
- [ ] Fault injection scenarios tested
- [ ] Safe-mode behavior verified (Hold, LatchedFault)

## Sign-off

| Role | Name | Date |
|------|------|------|
| Software Lead | | |
| Verification Lead | | |
| Safety Lead | | |
