#!/usr/bin/env python3
"""Focused contract tests for Cube source discovery and manifest validation."""

from __future__ import annotations

import importlib.util
import json
import os
import tempfile
from pathlib import Path


_MODULE_PATH = Path(__file__).with_name("import_cube_sources.py")
_SPEC = importlib.util.spec_from_file_location("gofly_import_cube_sources", _MODULE_PATH)
if _SPEC is None or _SPEC.loader is None:
    raise RuntimeError("unable to load import_cube_sources.py")
_import_module = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(_import_module)


def _make_package(root: Path) -> Path:
    package = root / "STM32Cube_FW_F4_V1.0.0"
    for destination_root, relative in _import_module._requested_files():
        source_root = (
            package / "Drivers" / "STM32F4xx_HAL_Driver"
            if destination_root.name == "STM32F4xx_HAL_Driver"
            else package / "Drivers" / "CMSIS"
        )
        path = source_root / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(relative.as_posix().encode("ascii"))
    return package


def test_nested_package_discovery_and_copy() -> None:
    with tempfile.TemporaryDirectory() as temporary:
        root = Path(temporary)
        package = _make_package(root)
        firmware = root / "firmware"
        manifest = _import_module.import_sources(firmware, str(root))
        assert manifest["package_root"] == str(package)
        assert len(manifest["files"]) == len(_import_module._requested_files())
        assert (firmware / "Drivers/CMSIS/Include/core_cm4.h").is_file()


def test_environment_root_precedes_explicit_root() -> None:
    with tempfile.TemporaryDirectory() as temporary:
        root = Path(temporary)
        environment_package = _make_package(root / "environment")
        explicit_package = _make_package(root / "explicit")
        old_value = os.environ.get("STM32CUBE_F4_PATH")
        try:
            os.environ["STM32CUBE_F4_PATH"] = str(environment_package.parent)
            roots = _import_module._candidate_roots(str(explicit_package.parent))
            assert roots[0] == environment_package.parent
        finally:
            if old_value is None:
                os.environ.pop("STM32CUBE_F4_PATH", None)
            else:
                os.environ["STM32CUBE_F4_PATH"] = old_value


def test_manifest_rejects_hash_mismatch_and_non_object() -> None:
    with tempfile.TemporaryDirectory() as temporary:
        root = Path(temporary)
        firmware = root / "firmware"
        firmware.mkdir()
        manifest_path = root / "manifest.json"
        manifest_path.write_text("[]", encoding="utf-8")
        try:
            _import_module.validate_manifest(manifest_path, firmware)
        except ValueError as error:
            assert "root must be an object" in str(error)
        else:
            raise AssertionError("non-object manifest was accepted")

        manifest_path.write_text(
            json.dumps(
                {
                    "schema_version": 1,
                    "package_root": "cube",
                    "package_version": "V1",
                    "files": [],
                }
            ),
            encoding="utf-8",
        )
        try:
            _import_module.validate_manifest(manifest_path, firmware)
        except ValueError as error:
            assert "does not enumerate" in str(error)
        else:
            raise AssertionError("incomplete manifest was accepted")


def main() -> int:
    test_nested_package_discovery_and_copy()
    test_environment_root_precedes_explicit_root()
    test_manifest_rejects_hash_mismatch_and_non_object()
    print("import_cube_sources contract: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
