#!/usr/bin/env python3
"""Generate Requirements Traceability Matrix from @req tags in source."""

from __future__ import annotations

import argparse
import csv
import re
from pathlib import Path

REQ_PATTERN = re.compile(r"@req\s+(LLR-[A-Z]+-\d+)")
MCDC_PATTERN = re.compile(r"@mcdc\s+(LLR-[A-Z]+-\d+)")


def scan_sources(source_dir: Path) -> list[dict[str, str]]:
    rows: list[dict[str, str]] = []
    extensions = {".cpp", ".hpp", ".h", ".cxx", ".cc"}

    for path in sorted(source_dir.rglob("*")):
        if path.suffix not in extensions:
            continue
        if "build" in path.parts or ".git" in path.parts:
            continue

        try:
            text = path.read_text(encoding="utf-8")
        except OSError:
            continue

        lines = text.splitlines()
        for index, line in enumerate(lines, start=1):
            for match in REQ_PATTERN.finditer(line):
                req_id = match.group(1)
                mcdc = "yes" if any(
                    MCDC_PATTERN.search(lines[i])
                    for i in range(max(0, index - 3), min(len(lines), index + 2))
                ) else "no"
                rows.append(
                    {
                        "requirement_id": req_id,
                        "file": str(path.relative_to(source_dir)),
                        "line": str(index),
                        "mcdc": mcdc,
                    }
                )
    return rows


def write_rtm(rows: list[dict[str, str]], output: Path) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(
            handle,
            fieldnames=["requirement_id", "file", "line", "mcdc"],
        )
        writer.writeheader()
        writer.writerows(rows)


def main() -> int:
    parser = argparse.ArgumentParser(description="Generate RTM from @req tags")
    parser.add_argument("--source-dir", type=Path, default=Path("."))
    parser.add_argument("--output", type=Path, default=Path("RTM.csv"))
    args = parser.parse_args()

    rows = scan_sources(args.source_dir.resolve())
    write_rtm(rows, args.output.resolve())
    print(f"Wrote {len(rows)} trace entries to {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
