"""Install a previously built ASRPRO completion-callback SDK archive.

The installed Tianwen app consumes the SDK's prebuilt ``core.a``.  This helper
copies only the compatible player source/header and archive into that SDK; it
does not clean the vendor build tree or replace the current firmware image.
Run with ``--check`` first.  ``--apply`` is required for installation and
creates a timestamped backup beside the installed SDK.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import shutil
import sys
import tempfile
from datetime import datetime
from pathlib import Path

from patch_sdk import PATCH_MARKER, _patch_header, _patch_source, _sha256, _validate_source


ROOT = Path(__file__).resolve().parents[1]
INSTALL_ROOT = Path(os.environ.get("TIANWEN_HOME", "D:/app/twenblock")).resolve()
DEFAULT_SOURCE_PARENT = ROOT / "records/build"
RELATIVE_FILES = (
    Path("components/cmd_info/prompt_player.c"),
    Path("components/cmd_info/prompt_player.h"),
    Path("projects/offline_asr_sample/project_file/core.a"),
)
SOURCE_DIRS = ("components", "driver", "system", "utils", "libs", "projects/offline_asr_sample/src")
IGNORED_SUFFIXES = {".o", ".d", ".elf", ".lst", ".map"}


def _is_patch_file(relative: Path) -> bool:
    return relative.as_posix() in {
        "components/cmd_info/prompt_player.c",
        "components/cmd_info/prompt_player.h",
    }


def _fingerprint_files(sdk: Path) -> dict[str, str]:
    files: dict[str, str] = {}
    for directory in SOURCE_DIRS:
        base = sdk / directory
        if not base.is_dir():
            raise RuntimeError(f"SDK source directory is missing: {base}")
        for path in base.rglob("*"):
            if not path.is_file():
                continue
            relative = path.relative_to(sdk)
            if _is_patch_file(relative) or path.suffix.lower() in IGNORED_SUFFIXES:
                continue
            if any(part.endswith(".bak.asrpro-yy-v1") for part in relative.parts):
                continue
            if any(part.lower() in {"build", ".git"} for part in relative.parts):
                continue
            files[relative.as_posix()] = _sha256(path)
    project = sdk / "projects/offline_asr_sample/project_file"
    for relative in ("makefile", "source_file.prj", "build/source_file.mk"):
        path = project / relative
        if not path.is_file():
            raise RuntimeError(f"SDK project input is missing: {path}")
        files[(Path("projects/offline_asr_sample/project_file") / relative).as_posix()] = _sha256(path)
    return files


def _expected_patched_sources(target: Path) -> tuple[bytes, bytes]:
    source = target / RELATIVE_FILES[0]
    header = target / RELATIVE_FILES[1]
    with tempfile.TemporaryDirectory(prefix="asrpro-sdk-check-") as temporary:
        sdk = Path(temporary) / "asr_pro_sdk"
        source_copy = sdk / RELATIVE_FILES[0]
        header_copy = sdk / RELATIVE_FILES[1]
        source_copy.parent.mkdir(parents=True, exist_ok=True)
        header_copy.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, source_copy)
        shutil.copy2(header, header_copy)
        if PATCH_MARKER in source_copy.read_text(encoding="utf-8"):
            pass
        elif "yy_prompt_play(" in source_copy.read_text(encoding="utf-8"):
            from patch_sdk import _upgrade_legacy_patch

            _upgrade_legacy_patch(sdk)
        else:
            _patch_header(header_copy)
            _patch_source(source_copy)
        return source_copy.read_bytes(), header_copy.read_bytes()


def _validate_compatible_sdk(source_sdk: Path, target_sdk: Path) -> None:
    _validate_source(source_sdk)
    if target_sdk.name.lower() != "asr_pro_sdk":
        raise RuntimeError(f"target is not an ASRPRO SDK directory: {target_sdk}")
    source_files = _fingerprint_files(source_sdk)
    target_files = _fingerprint_files(target_sdk)
    if source_files != target_files:
        source_only = sorted(set(source_files) - set(target_files))
        target_only = sorted(set(target_files) - set(source_files))
        changed = sorted(k for k in set(source_files) & set(target_files) if source_files[k] != target_files[k])
        details = []
        if source_only:
            details.append("staging-only=" + ", ".join(source_only[:4]))
        if target_only:
            details.append("installed-only=" + ", ".join(target_only[:4]))
        if changed:
            details.append("changed=" + ", ".join(changed[:8]))
        raise RuntimeError("SDK source versions differ; refusing to install (" + "; ".join(details) + ")")

    expected_source, expected_header = _expected_patched_sources(target_sdk)
    if (source_sdk / RELATIVE_FILES[0]).read_bytes() != expected_source:
        raise RuntimeError("staging prompt_player.c is not the patch for the installed SDK source")
    if (source_sdk / RELATIVE_FILES[1]).read_bytes() != expected_header:
        raise RuntimeError("staging prompt_player.h is not the patch for the installed SDK header")
    archive = source_sdk / RELATIVE_FILES[2]
    if archive.stat().st_size < 1_000_000:
        raise RuntimeError(f"staging core.a is unexpectedly small: {archive.stat().st_size} bytes")
    if not (source_sdk / "projects/offline_asr_sample/project_file/build/program.elf").is_file():
        raise RuntimeError("staging SDK has no successful program.elf build output")
    project = source_sdk / "projects/offline_asr_sample/project_file"
    if not (project / "../../../../user_code/[0]code.bin").resolve().is_file():
        raise RuntimeError("staging SDK has no generated code image")


def _backup_path(target: Path) -> Path:
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    backup = target.parent / f".codex_asrpro_yy_prompt_play_backup_{stamp}"
    backup.mkdir()
    for relative in RELATIVE_FILES:
        target_file = target / relative
        if not target_file.is_file():
            shutil.rmtree(backup)
            raise RuntimeError(f"installed SDK file is missing: {target_file}")
        destination = backup / relative
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(target_file, destination)
    manifest = {
        "created_local": datetime.now().isoformat(timespec="seconds"),
        "target_sdk": str(target),
        "sha256": {relative.as_posix(): _sha256(target / relative) for relative in RELATIVE_FILES},
    }
    (backup / "manifest.json").write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    return backup


def _latest_staging_sdk() -> Path:
    candidates = []
    for path in DEFAULT_SOURCE_PARENT.glob("asrpro_ui_compat*/asr_pro_sdk"):
        project = path / "projects/offline_asr_sample/project_file"
        elf = project / "build/program.elf"
        archive = project / "core.a"
        code = project / "../../../../user_code/[0]code.bin"
        if elf.is_file() and archive.is_file() and code.resolve().is_file():
            candidates.append(path)
    if not candidates:
        raise RuntimeError("no complete asrpro_ui_compat staging SDK found; run tools/build_sdk_compat.py first")
    return max(
        candidates,
        key=lambda path: (path / "projects/offline_asr_sample/project_file/core.a").stat().st_mtime_ns,
    )


def _atomic_copy(source: Path, destination: Path) -> None:
    with tempfile.NamedTemporaryFile(prefix=destination.name + ".", suffix=".tmp", dir=destination.parent, delete=False) as stream:
        temporary = Path(stream.name)
    try:
        shutil.copy2(source, temporary)
        os.replace(temporary, destination)
    finally:
        temporary.unlink(missing_ok=True)


def install(source_sdk: Path, target_sdk: Path, apply: bool) -> None:
    source_sdk = source_sdk.resolve()
    target_sdk = target_sdk.resolve()
    expected_target = INSTALL_ROOT / "asrpro/asr_pro_sdk"
    if target_sdk != expected_target:
        raise RuntimeError(f"target must be Tianwen's ASRPRO SDK: {expected_target}")
    _validate_compatible_sdk(source_sdk, target_sdk)
    identical = all(_sha256(source_sdk / relative) == _sha256(target_sdk / relative) for relative in RELATIVE_FILES)
    if identical:
        print(f"already installed and validated: {target_sdk}")
        return
    if not apply:
        print("compatibility patch is ready; no installed files changed")
        print("run again with --apply to install the player source/header and core.a")
        return

    backup = _backup_path(target_sdk)
    try:
        for relative in RELATIVE_FILES:
            _atomic_copy(source_sdk / relative, target_sdk / relative)
        _validate_compatible_sdk(source_sdk, target_sdk)
    except Exception:
        for relative in RELATIVE_FILES:
            saved = backup / relative
            if saved.is_file():
                _atomic_copy(saved, target_sdk / relative)
        raise
    print(f"installed and verified ASRPRO compatibility library: {target_sdk}")
    print(f"original files backed up at: {backup}")
    print(f"core.a sha256: {_sha256(target_sdk / RELATIVE_FILES[2])}")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source-sdk", type=Path, default=None,
                        help="staging SDK from build_sdk_compat.py; defaults to the newest complete build")
    parser.add_argument("--target-sdk", type=Path, default=INSTALL_ROOT / "asrpro/asr_pro_sdk")
    action = parser.add_mutually_exclusive_group(required=True)
    action.add_argument("--check", action="store_true")
    action.add_argument("--apply", action="store_true")
    args = parser.parse_args()
    install(args.source_sdk or _latest_staging_sdk(), args.target_sdk, apply=args.apply)
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:
        print(f"install_sdk_compat: ERROR: {exc}", file=sys.stderr)
        raise SystemExit(2)
