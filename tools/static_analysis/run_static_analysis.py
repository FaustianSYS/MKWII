#!/usr/bin/env python3
"""Run static analysis tools and fail on findings."""

from __future__ import annotations

import argparse
import shutil
import subprocess
import sys
from pathlib import Path


def run_command(name: str, cmd: list[str], cwd: Path) -> int:
    print(f"=== {name} ===")
    result = subprocess.run(cmd, cwd=cwd, check=False)
    return result.returncode


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source-dir", type=Path, default=Path("."))
    parser.add_argument("--build-dir", type=Path, default=Path("build/safety-verify"))
    args = parser.parse_args()

    source_dir = args.source_dir.resolve()
    build_dir = args.build_dir.resolve()
    rc = 0

    compile_db = build_dir / "compile_commands.json"
    if compile_db.exists() and shutil.which("clang-tidy"):
        rc |= run_command(
            "clang-tidy",
            [
                "run-clang-tidy",
                "-p",
                str(build_dir),
                "-header-filter=include/flightsim/.*",
            ],
            source_dir,
        )
    else:
        print("clang-tidy skipped (compile_commands.json or tool missing)")

    if shutil.which("cppcheck"):
        rc |= run_command(
            "cppcheck",
            [
                "cppcheck",
                "--enable=warning,style,performance,portability",
                "--error-exitcode=1",
                "--inline-suppr",
                "-I",
                str(source_dir / "include"),
                str(source_dir / "libs"),
                str(source_dir / "apps"),
            ],
            source_dir,
        )
    else:
        print("cppcheck skipped (tool missing)")

    return rc


if __name__ == "__main__":
    raise SystemExit(main())
