#!/usr/bin/env python3
"""Import exact STM32CubeF4 sources; never synthesize vendor code."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import shutil
import sys
from datetime import datetime, timezone
from pathlib import Path


_HAL_FILES = (
    "Inc/stm32f4xx_hal.h",
    "Inc/stm32f4xx_hal_conf_template.h",
    "Inc/stm32f4xx_hal_cortex.h",
    "Inc/stm32f4xx_hal_gpio.h",
    "Inc/stm32f4xx_hal_dma.h",
    "Inc/stm32f4xx_hal_rcc.h",
    "Inc/stm32f4xx_hal_rcc_ex.h",
    "Inc/stm32f4xx_hal_pwr.h",
    "Inc/stm32f4xx_hal_pwr_ex.h",
    "Inc/stm32f4xx_hal_spi.h",
    "Inc/stm32f4xx_hal_i2c.h",
    "Inc/stm32f4xx_hal_uart.h",
    "Inc/stm32f4xx_hal_usart.h",
    "Inc/stm32f4xx_hal_adc.h",
    "Inc/stm32f4xx_hal_tim.h",
    "Inc/stm32f4xx_hal_tim_ex.h",
    "Inc/stm32f4xx_hal_pcd.h",
    "Inc/stm32f4xx_hal_pcd_ex.h",
    "Inc/stm32f4xx_ll_usb.h",
    "Src/stm32f4xx_hal.c",
    "Src/stm32f4xx_hal_cortex.c",
    "Src/stm32f4xx_hal_gpio.c",
    "Src/stm32f4xx_hal_dma.c",
    "Src/stm32f4xx_hal_rcc.c",
    "Src/stm32f4xx_hal_rcc_ex.c",
    "Src/stm32f4xx_hal_pwr.c",
    "Src/stm32f4xx_hal_pwr_ex.c",
    "Src/stm32f4xx_hal_spi.c",
    "Src/stm32f4xx_hal_i2c.c",
    "Src/stm32f4xx_hal_uart.c",
    "Src/stm32f4xx_hal_usart.c",
    "Src/stm32f4xx_hal_adc.c",
    "Src/stm32f4xx_hal_tim.c",
    "Src/stm32f4xx_hal_tim_ex.c",
    "Src/stm32f4xx_hal_pcd.c",
    "Src/stm32f4xx_hal_pcd_ex.c",
    "Src/stm32f4xx_ll_usb.c",
)
_CMSIS_FILES = (
    "Include/cmsis_compiler.h",
    "Include/cmsis_gcc.h",
    "Include/core_cm4.h",
    "Include/core_cmFunc.h",
    "Include/core_cmInstr.h",
    "Include/core_cmSimd.h",
    "Include/mpu_armv7.h",
    "Include/tz_context.h",
    "Device/ST/STM32F4xx/Include/stm32f405xx.h",
    "Device/ST/STM32F4xx/Include/system_stm32f4xx.h",
    "Device/ST/STM32F4xx/Source/Templates/system_stm32f4xx.c",
)


def _candidate_roots(explicit: str | None) -> list[Path]:
    roots: list[Path] = []
    env_root = os.environ.get("STM32CUBE_F4_PATH")
    if env_root:
        roots.append(Path(env_root))
    user_profile = Path(os.environ.get("USERPROFILE", Path.home()))
    roots.extend(
        [
            user_profile / "STM32Cube" / "Repository",
            Path("C:/ST"),
            Path("C:/Program Files/STMicroelectronics"),
            Path("C:/Program Files (x86)/STMicroelectronics"),
        ]
    )
    if explicit:
        roots.append(Path(explicit))
    result: list[Path] = []
    seen: set[str] = set()
    for root in roots:
        key = os.path.normcase(os.path.abspath(str(root)))
        if key not in seen:
            seen.add(key)
            result.append(root)
    return result


def _find_package(roots: list[Path]) -> tuple[Path | None, list[str]]:
    searched: list[str] = []
    candidates: list[Path] = []
    for root in roots:
        searched.append(str(root))
        if not root.exists():
            continue
        if (root / "Drivers" / "STM32F4xx_HAL_Driver").is_dir():
            candidates.append(root)
        candidates.extend(sorted(root.glob("STM32Cube_FW_F4_V*")))
        candidates.extend(sorted(root.glob("*/Drivers/STM32F4xx_HAL_Driver")))
    for candidate in candidates:
        if candidate.name == "STM32F4xx_HAL_Driver":
            package_root = candidate.parent.parent
        else:
            package_root = candidate
        if (package_root / "Drivers" / "STM32F4xx_HAL_Driver").is_dir():
            return package_root, searched
    return None, searched


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _requested_files() -> list[tuple[Path, Path]]:
    return [
        (Path("Drivers/STM32F4xx_HAL_Driver"), Path(relative))
        for relative in _HAL_FILES
    ] + [
        (Path("Drivers/CMSIS"), Path(relative))
        for relative in _CMSIS_FILES
    ]


def _copy_files(package_root: Path, firmware_root: Path) -> list[dict[str, str]]:
    requested = [
        (package_root / source_root, destination_root, relative)
        for destination_root, relative in _requested_files()
        for source_root in (
            Path("Drivers/STM32F4xx_HAL_Driver")
            if destination_root.name == "STM32F4xx_HAL_Driver"
            else Path("Drivers/CMSIS"),
        )
    ]
    missing = [str(root / relative) for root, _, relative in requested if not (root / relative).is_file()]
    if missing:
        raise FileNotFoundError("Cube package is missing required files:\n" + "\n".join(missing))

    manifest_files: list[dict[str, str]] = []
    for source_root, destination_root, relative in requested:
        source = source_root / relative
        destination = firmware_root / destination_root / relative
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, destination)
        manifest_files.append(
            {
                "source": str(source),
                "destination": str(destination.relative_to(firmware_root)).replace(os.sep, "/"),
                "sha256": _sha256(source),
            }
        )
    return manifest_files


def validate_manifest(manifest_path: Path, firmware_root: Path) -> dict[str, object]:
    try:
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise ValueError(f"unable to read manifest {manifest_path}: {error}") from error
    if not isinstance(manifest, dict):
        raise ValueError("Cube source manifest root must be an object")
    if manifest.get("schema_version") != 1:
        raise ValueError("unsupported Cube source manifest schema")
    if not manifest.get("package_root") or not manifest.get("package_version"):
        raise ValueError("Cube source manifest lacks package identity")

    expected = {
        f"{destination_root.as_posix()}/{relative.as_posix()}"
        for destination_root, relative in _requested_files()
    }
    entries = manifest.get("files")
    if not isinstance(entries, list):
        raise ValueError("Cube source manifest files must be a list")
    actual = {entry.get("destination") for entry in entries if isinstance(entry, dict)}
    if actual != expected or len(entries) != len(expected):
        raise ValueError("Cube source manifest does not enumerate exactly the required files")

    root = firmware_root.resolve()
    for entry in entries:
        if not isinstance(entry, dict):
            raise ValueError("Cube source manifest contains a non-object file entry")
        destination_text = entry.get("destination")
        expected_hash = entry.get("sha256")
        if not isinstance(destination_text, str) or not isinstance(expected_hash, str):
            raise ValueError("Cube source manifest file entry lacks destination or sha256")
        destination = (firmware_root / Path(destination_text)).resolve()
        try:
            destination.relative_to(root)
        except ValueError as error:
            raise ValueError(f"manifest destination escapes firmware root: {destination_text}") from error
        if not destination.is_file():
            raise FileNotFoundError(f"manifest destination is missing: {destination}")
        if _sha256(destination).lower() != expected_hash.lower():
            raise ValueError(f"manifest hash mismatch: {destination_text}")
    return manifest


def import_sources(firmware_root: Path, cube_root: str | None = None, dry_run: bool = False) -> dict[str, object]:
    roots = _candidate_roots(cube_root)
    package_root, searched = _find_package(roots)
    if package_root is None:
        raise FileNotFoundError(
            "No STM32CubeF4 package found. Searched:\n" + "\n".join(f"  {path}" for path in searched)
        )
    package_version = package_root.name
    manifest_files: list[dict[str, str]] = []
    if not dry_run:
        manifest_files = _copy_files(package_root, firmware_root)
    return {
        "schema_version": 1,
        "package_root": str(package_root),
        "package_version": package_version,
        "searched_paths": searched,
        "imported_at_utc": datetime.now(timezone.utc).isoformat(),
        "files": manifest_files,
        "dry_run": dry_run,
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--firmware-root", type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument("--cube-root")
    parser.add_argument("--manifest", type=Path)
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--validate-only", action="store_true")
    args = parser.parse_args(argv)

    firmware_root = args.firmware_root.resolve()
    if args.validate_only:
        if args.manifest is None:
            parser.error("--validate-only requires --manifest")
        try:
            manifest = validate_manifest(args.manifest.resolve(), firmware_root)
        except (FileNotFoundError, OSError, ValueError) as error:
            print(f"Cube source manifest invalid: {error}", file=sys.stderr)
            return 2
        print(json.dumps(manifest, indent=2, sort_keys=True))
        return 0

    try:
        manifest = import_sources(firmware_root, args.cube_root, args.dry_run)
    except (FileNotFoundError, OSError, ValueError) as error:
        print(f"Cube source import failed: {error}", file=sys.stderr)
        return 2
    payload = json.dumps(manifest, indent=2, sort_keys=True) + "\n"
    if args.manifest and not args.dry_run:
        args.manifest.parent.mkdir(parents=True, exist_ok=True)
        temporary = args.manifest.with_suffix(args.manifest.suffix + ".tmp")
        temporary.write_text(payload, encoding="utf-8")
        temporary.replace(args.manifest)
    else:
        print(payload, end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
