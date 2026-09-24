#!/usr/bin/env python3
"""Focused contract test for probe_toolchain."""

from __future__ import annotations

import importlib.util
from pathlib import Path


_MODULE_PATH = Path(__file__).with_name("probe_toolchain.py")
_SPEC = importlib.util.spec_from_file_location("gofly_probe_toolchain", _MODULE_PATH)
if _SPEC is None or _SPEC.loader is None:
    raise RuntimeError("unable to load probe_toolchain.py")
_probe_module = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(_probe_module)


def main() -> int:
    report = _probe_module.probe_toolchain(path_override=[])
    assert report["target_compiler"] is False
    assert report["make_available"] is False
    assert report["cubemx_available"] is False
    assert report["can_target_build"] is False
    assert isinstance(report["host_compiler"], bool)
    print("probe_toolchain contract: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
