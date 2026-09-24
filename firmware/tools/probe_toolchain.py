#!/usr/bin/env python3
"""Probe build tools without installing or mutating the host."""

from __future__ import annotations

import argparse
import json
import os
import shutil
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Iterable


_TOOL_COMMANDS = {
    "target_compiler": ("arm-none-eabi-gcc", "--version"),
    "make": ("make", "--version"),
    "cubemx": ("STM32CubeMX", "--version"),
}
_HOST_COMPILER_NAMES = ("cc", "gcc", "clang", "cl")


def _resolve(name: str, path_override: Iterable[str] | None) -> str | None:
    if path_override is not None and not tuple(path_override):
        return None
    if path_override is None:
        return shutil.which(name)
    return shutil.which(name, path=os.pathsep.join(path_override))


def _version(executable: str, argument: str) -> str | None:
    try:
        result = subprocess.run(
            [executable, argument],
            check=False,
            capture_output=True,
            text=True,
            timeout=3,
        )
    except (OSError, subprocess.SubprocessError):
        return None
    output = (result.stdout or result.stderr).strip()
    return output.splitlines()[0] if output else None


def probe_toolchain(path_override: list[str] | None = None) -> dict[str, object]:
    """Return a factual tool-presence report for the current build host."""
    tools: dict[str, dict[str, object]] = {}
    for key, (name, argument) in _TOOL_COMMANDS.items():
        executable = _resolve(name, path_override)
        tools[key] = {
            "name": name,
            "path": executable,
            "available": executable is not None,
            "version": _version(executable, argument) if executable else None,
        }

    host_path = next(
        (_resolve(name, path_override) for name in _HOST_COMPILER_NAMES if _resolve(name, path_override)),
        None,
    )
    tools["host_compiler"] = {
        "name": Path(host_path).name if host_path else None,
        "path": host_path,
        "available": host_path is not None,
        "version": _version(host_path, "--version") if host_path else None,
    }

    python_version = _version(sys.executable, "--version")
    tools["python"] = {
        "name": Path(sys.executable).name,
        "path": sys.executable,
        "available": python_version is not None,
        "version": python_version,
    }

    target_compiler = bool(tools["target_compiler"]["available"])
    make_available = bool(tools["make"]["available"])
    return {
        "schema_version": 1,
        "probed_at_utc": datetime.now(timezone.utc).isoformat(),
        "path_override_used": path_override is not None,
        "target_compiler": target_compiler,
        "host_compiler": bool(tools["host_compiler"]["available"]),
        "make_available": make_available,
        "cubemx_available": bool(tools["cubemx"]["available"]),
        "can_target_build": target_compiler and make_available,
        "can_generate_cube": bool(tools["cubemx"]["available"]),
        "tools": tools,
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, help="write JSON report to this path")
    args = parser.parse_args(argv)
    report = probe_toolchain()
    payload = json.dumps(report, indent=2, sort_keys=True) + "\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(payload, encoding="utf-8")
    else:
        print(payload, end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
