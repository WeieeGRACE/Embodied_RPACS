"""Build a disposable ASRPRO SDK archive for the Tianwen UI compiler."""

from __future__ import annotations

import argparse
import os
import shutil
from datetime import datetime
from pathlib import Path

from patch_sdk import patch_sdk, rebuild_sdk


ROOT = Path(__file__).resolve().parents[1]
INSTALL = Path(os.environ.get("TIANWEN_HOME", "D:/app/twenblock"))
DEFAULT_WORK = ROOT / "records/build/asrpro_ui_compat"


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--work", type=Path, default=None,
                        help="isolated build directory; by default a unique directory is chosen")
    args = parser.parse_args()
    work = (args.work or DEFAULT_WORK).resolve()
    sdk_source = INSTALL / "asrpro/asr_pro_sdk"
    sdk_target = work / "asr_pro_sdk"
    if work.exists():
        if args.work is not None:
            raise SystemExit(f"refusing to overwrite requested build workspace: {work}")
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        work = work.with_name(work.name + "_" + stamp)
        if work.exists():
            raise SystemExit(f"timestamped build workspace already exists: {work}")
        sdk_target = work / "asr_pro_sdk"
    if not sdk_source.is_dir():
        raise SystemExit(f"Tianwen ASRPRO SDK not found: {sdk_source}")
    if not (ROOT / "records/generated.cpp").is_file():
        raise SystemExit("records/generated.cpp is required; export current Blockly code first")

    work.mkdir(parents=True)
    shutil.copytree(
        sdk_source,
        sdk_target,
        ignore=shutil.ignore_patterns("build", "core.a", "*.elf", "*.lst", "*.map"),
    )
    project = sdk_target / "projects/offline_asr_sample/project_file"
    build = project / "build"
    build.mkdir(parents=True)
    shutil.copy2(sdk_source / "projects/offline_asr_sample/project_file/build/source_file.mk", build)
    for filename in ("asr.cpp", "_asr.cpp"):
        shutil.copy2(ROOT / "records/generated.cpp", work / filename)
    (work / "user_code").mkdir()

    patch_sdk(sdk_target)
    rebuild_sdk(sdk_target)
    print(f"Tianwen UI-compatible archive ready: {project / 'core.a'}")
    print(f"staged firmware compile log: {project / 'yy_prompt_rebuild.log'}")
    print(f"staging directory: {work}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
