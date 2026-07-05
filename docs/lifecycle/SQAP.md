# Software Quality Assurance Plan (SQAP)

## 1. QA Activities

- Review lifecycle plans before development baseline
- Audit requirement traceability completeness
- Witness verification execution in CI
- Confirm static analysis and coverage gates pass

## 2. Independence

Verification test design reviewed independently from implementation author where practicable.

## 3. Conformity Checks

| Check | Frequency |
|-------|-----------|
| RTM generation | Every CI run |
| Coding standard compliance | Every CI run |
| Coverage thresholds | Verification builds |
| MC/DC on tagged functions | Release baseline |

## 4. Records

Retain CI logs, coverage HTML, RTM CSV, and test result artifacts per SCMP retention policy.

## 5. Non-conformance

Failed gates block merge until resolved or waived through formal safety review.
