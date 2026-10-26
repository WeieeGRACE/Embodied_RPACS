"""Patch and validate the ASRPRO prompt-player SDK compatibility interface.

The generated ASRPRO program uses ``yy_prompt_play`` so it can distinguish a
normal prompt completion from an interruption or a failed prompt.  Tianwen's
stock SDK exposes only the legacy callback, which is why linking stops with an
undefined reference.  This script patches one SDK tree, rebuilds its project
archive, and leaves a byte-for-byte backup before changing installed files.

The script is deliberately strict: it refuses an unknown source layout,
repeatedly applying a patch, or a missing SDK build tool.  Use ``--check`` to
validate an already patched SDK without modifying it.
"""

from __future__ import annotations

import argparse
import hashlib
import os
import re
import shutil
import subprocess
import sys
from pathlib import Path


PATCH_MARKER = "ASRPRO_YY_PROMPT_PLAY_V1"
BACKUP_SUFFIX = ".bak.asrpro-yy-v1"


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _backup(path: Path) -> Path:
    backup = path.with_name(path.name + BACKUP_SUFFIX)
    if backup.exists():
        if _sha256(backup) != _sha256(path):
            raise RuntimeError(f"existing backup does not match {path}: {backup}")
    else:
        shutil.copy2(path, backup)
    return backup


def _replace_once(source: str, old: str, new: str, label: str) -> str:
    count = source.count(old)
    if count != 1:
        raise RuntimeError(f"expected one {label}, found {count}")
    return source.replace(old, new, 1)


def _replace_exact_count(source: str, old: str, new: str, expected: int, label: str) -> str:
    count = source.count(old)
    if count != expected:
        raise RuntimeError(f"expected {expected} {label}, found {count}")
    return source.replace(old, new)


def _initialize_multi_play_callback(source: str) -> str:
    duplicate = (
        "    voice_play_info.play_done_callback = play_done_callback;    \n"
        "    voice_play_info.play_done_callback = play_done_callback;    \n"
    )
    initialized = (
        "    voice_play_info.play_done_callback = play_done_callback;    \n"
        "    voice_play_info.done_ex = NULL;\n"
        "    voice_play_info.cookie = 0;\n"
        "    voice_play_info.play_done_callback = play_done_callback;    \n"
    )
    return _replace_once(source, duplicate, initialized, "multi-command callback initialization")


def _patch_header(path: Path) -> None:
    source = path.read_text(encoding="utf-8")
    if PATCH_MARKER in source:
        return
    anchor = "extern uint32_t prompt_play_by_cmd_handle("
    if source.count(anchor) != 1:
        raise RuntimeError(f"unexpected prompt_player.h layout: {path}")
    if "yy_prompt_play(" in source:
        raise RuntimeError("unmarked yy_prompt_play declaration already exists")
    declaration = "\n/* " + PATCH_MARKER + " */\n"
    declaration += "#ifdef __cplusplus\nextern \"C\" {\n#endif\n"
    declaration += "typedef void (*yy_play_done_ex_t)(uint32_t cookie, int32_t reason);\n"
    declaration += "extern uint32_t yy_prompt_play(uint16_t voice_id, yy_play_done_ex_t callback, uint32_t cookie);\n"
    declaration += "#ifdef __cplusplus\n}\n#endif\n"
    source = source.replace(anchor, declaration + "\n" + anchor, 1)
    path.write_text(source, encoding="utf-8", newline="\n")


def _patch_source(path: Path) -> None:
    source = path.read_text(encoding="utf-8")
    if PATCH_MARKER in source:
        return

    source = _replace_once(
        source,
        "typedef \nstruct voice_play_info_st\n{\n",
        "/* " + PATCH_MARKER + " */\n"
        "typedef \nstruct voice_play_info_st\n{\n",
        "voice_play_info_t marker",
    )
    source = _replace_once(
        source,
        "    play_done_callback_t play_done_callback;    //!播放结束的回调函数;\n",
        "    play_done_callback_t play_done_callback;    //!播放结束的回调函数;\n"
        "    yy_play_done_ex_t done_ex;                 //! reason-bearing callback;\n"
        "    uint32_t cookie;                           //! caller-owned request cookie;\n",
        "completion fields",
    )
    helper = """
static void yy_notify_done(const voice_play_info_t *info, int32_t reason)
{
    if (info->done_ex)
    {
        info->done_ex(info->cookie, reason);
    }
    if (info->play_done_callback)
    {
        info->play_done_callback(info->cmd_handle);
    }
}

"""
    source = _replace_once(
        source,
        "static void combination_callback(int32_t play_cb_state);\n\n",
        "static void combination_callback(int32_t play_cb_state);\n\n" + helper,
        "completion helper anchor",
    )
    source = _replace_exact_count(
        source,
        "        if (voice_play_info.play_done_callback)\n        {\n            voice_play_info.play_done_callback(voice_play_info.cmd_handle);\n        }",
        "        yy_notify_done(&voice_play_info, -99);",
        2,
        "pop callback",
    )
    source = _replace_once(
        source,
        "            if (voice_play_info.play_done_callback)     \n            {                \n                voice_play_info.play_done_callback(voice_play_info.cmd_handle);\n            }",
        "            yy_notify_done(&voice_play_info, play_cb_state);",
        "completion callback",
    )

    old_signature = """uint32_t prompt_play_by_cmd_handle(
            cmd_handle_t cmd_handle, 
            int select_index, 
            play_done_callback_t play_done_callback,
            bool preemptive)"""
    new_signature = """static uint32_t yy_prompt_play_by_cmd_handle_ex(
            cmd_handle_t cmd_handle,
            int select_index,
            play_done_callback_t play_done_callback,
            bool preemptive,
            yy_play_done_ex_t done_ex,
            uint32_t cookie)"""
    source = _replace_once(source, old_signature, new_signature, "legacy player signature")
    source = _replace_once(
        source,
        "    #if !AUDIO_PLAYER_ENABLE\n        if(play_done_callback)",
        "    #if !AUDIO_PLAYER_ENABLE\n        if (done_ex) done_ex(cookie, -99);\n        if(play_done_callback)",
        "disabled-player failure",
    )
    source = _replace_once(
        source,
        "    voice_play_info.cmd_handle = cmd_handle;\n    voice_play_info.play_done_callback = play_done_callback;",
        "    voice_play_info.cmd_handle = cmd_handle;\n"
        "    voice_play_info.play_done_callback = play_done_callback;\n"
        "    voice_play_info.done_ex = done_ex;\n"
        "    voice_play_info.cookie = cookie;",
        "completion state assignment",
    )
    source = _replace_once(
        source,
        "    if ((!is_valid_cmd_handle(cmd_handle) && (select_index != -2)) || !prompt_player.play_queue)\n    {\n        if (play_done_callback)\n        {\n            play_done_callback(cmd_handle);\n        }",
        "    if ((!is_valid_cmd_handle(cmd_handle) && (select_index != -2)) || !prompt_player.play_queue)\n    {\n"
        "        if (done_ex) done_ex(cookie, -99);\n"
        "        if (play_done_callback)\n        {\n            play_done_callback(cmd_handle);\n        }",
        "invalid-player failure",
    )
    source = _replace_once(
        source,
        "        if (voice_play_info.play_done_callback)\n        {\n            cmd_handle = (select_index == -2) ? NULL : cmd_handle;\n            voice_play_info.play_done_callback(cmd_handle);\n        }",
        "        if (done_ex) done_ex(cookie, -99);\n"
        "        if (voice_play_info.play_done_callback)\n        {\n"
        "            cmd_handle = (select_index == -2) ? NULL : cmd_handle;\n"
        "            voice_play_info.play_done_callback(cmd_handle);\n        }",
        "queue-full failure",
    )

    wrapper = """
uint32_t prompt_play_by_cmd_handle(
            cmd_handle_t cmd_handle,
            int select_index,
            play_done_callback_t play_done_callback,
            bool preemptive)
{
    return yy_prompt_play_by_cmd_handle_ex(cmd_handle, select_index,
                                           play_done_callback, preemptive,
                                           NULL, 0);
}

uint32_t yy_prompt_play(uint16_t voice_id, yy_play_done_ex_t callback, uint32_t cookie)
{
    return yy_prompt_play_by_cmd_handle_ex(cmd_info_find_command_by_id(voice_id),
                                           -1, NULL, true, callback, cookie);
}

"""
    source = _replace_once(
        source,
        "uint32_t prompt_play_by_voice_id(uint16_t voice_id, play_done_callback_t play_done_callback, bool preemptive)\n",
        wrapper + "uint32_t prompt_play_by_voice_id(uint16_t voice_id, play_done_callback_t play_done_callback, bool preemptive)\n",
        "public wrapper anchor",
    )
    source = _initialize_multi_play_callback(source)
    path.write_text(source, encoding="utf-8", newline="\n")


def _upgrade_legacy_patch(sdk: Path) -> None:
    """Mark and validate an SDK built by the earlier workspace patcher."""
    source_path = sdk / "components/cmd_info/prompt_player.c"
    header_path = sdk / "components/cmd_info/prompt_player.h"
    source = source_path.read_text(encoding="utf-8")
    expected = {
        "public implementation": "uint32_t yy_prompt_play(uint16_t id,void (*cb)(uint32_t,int32_t),uint32_t cookie)",
        "private implementation": "static uint32_t yy_play_inner_ex(",
        "reason storage": "void (*done_ex)(uint32_t,int32_t);",
        "cookie storage": "uint32_t cookie;",
        "legacy callback adapter": "return yy_play_inner_ex(h,index,cb,preempt,NULL,0);",
        "reason-bearing completion": "voice_play_info.done_ex(voice_play_info.cookie,play_cb_state)",
    }
    missing = [label for label, token in expected.items() if token not in source]
    if missing:
        raise RuntimeError("unrecognized legacy SDK patch; missing: " + ", ".join(missing))
    source = _replace_once(
        source,
        "typedef \nstruct voice_play_info_st\n{\n",
        "/* " + PATCH_MARKER + " (upgraded legacy patch) */\n"
        "typedef \nstruct voice_play_info_st\n{\n",
        "legacy source marker",
    )
    source = _initialize_multi_play_callback(source)
    source_path.write_text(source, encoding="utf-8", newline="\n")
    _patch_header(header_path)


def _validate_source(sdk: Path, *, require_archive: bool = True) -> None:
    source_path = sdk / "components/cmd_info/prompt_player.c"
    header_path = sdk / "components/cmd_info/prompt_player.h"
    project = sdk / "projects/offline_asr_sample/project_file"
    archive = project / "core.a"
    config = sdk / "projects/offline_asr_sample/src/user_config.h"
    make_sources = project / "build/source_file.mk"
    for path in (source_path, header_path, project / "makefile", config, make_sources):
        if not path.is_file():
            raise RuntimeError(f"missing ASRPRO SDK file: {path}")
    source = source_path.read_text(encoding="utf-8")
    header = header_path.read_text(encoding="utf-8")
    build = make_sources.read_text(encoding="utf-8")
    config_text = config.read_text(encoding="utf-8")
    if sdk.name.lower() != "asr_pro_sdk":
        raise RuntimeError(f"refusing to patch a non-ASRPRO SDK directory: {sdk}")
    if PATCH_MARKER not in source or PATCH_MARKER not in header:
        raise RuntimeError("ASRPRO completion patch marker is missing")
    if source.count("uint32_t yy_prompt_play(") != 1 or "yy_prompt_play" not in header:
        raise RuntimeError("yy_prompt_play declaration/definition is missing")
    has_typed_cookie = "yy_play_done_ex_t done_ex" in source and "uint32_t cookie" in source
    has_legacy_cookie = "void (*done_ex)(uint32_t,int32_t)" in source and "uint32_t cookie" in source
    if not (has_typed_cookie or has_legacy_cookie):
        raise RuntimeError("reason/cookie completion state is missing")
    if "voice_play_info.done_ex = NULL;" not in source or "voice_play_info.cookie = 0;" not in source:
        raise RuntimeError("multi-command callback state is not safely initialized")
    if "AUDIO_PLAY_CB_STATE_DONE                      = 0" not in (
        sdk / "components/player/audio_play/audio_play_api.h"
    ).read_text(encoding="utf-8"):
        raise RuntimeError("SDK completion status is not the expected zero value")
    if not re.search(r"#define\s+USE_AEC_MODULE\s+1\b", config_text):
        raise RuntimeError("SDK must keep USE_AEC_MODULE=1 for the current project")
    if not re.search(r"#define\s+PAUSE_VOICE_IN_WITH_PLAYING\s+0\b", config_text):
        raise RuntimeError("SDK must keep PAUSE_VOICE_IN_WITH_PLAYING=0")
    if "OBJS += build/objs/prompt_player.o" not in build or "components/cmd_info/prompt_player.c" not in build:
        raise RuntimeError("project build does not compile the patched prompt_player.c")
    if require_archive and not archive.is_file():
        raise RuntimeError(f"patched SDK archive is missing: {archive}")


def patch_sdk(sdk: Path, *, check_only: bool = False) -> None:
    sdk = sdk.resolve()
    source_path = sdk / "components/cmd_info/prompt_player.c"
    header_path = sdk / "components/cmd_info/prompt_player.h"
    archive = sdk / "projects/offline_asr_sample/project_file/core.a"
    for path in (source_path, header_path):
        if not path.exists():
            raise RuntimeError(f"missing ASRPRO SDK file: {path}")
    if archive.exists() and not archive.is_file():
        raise RuntimeError(f"ASRPRO SDK archive path is not a file: {archive}")
    source = source_path.read_text(encoding="utf-8")
    if PATCH_MARKER in source:
        _validate_source(sdk, require_archive=False)
        print(f"already patched: {sdk}")
        return
    if check_only:
        raise RuntimeError(f"SDK is not patched: {sdk}")

    source_backup = _backup(source_path)
    header_backup = _backup(header_path)
    archive_backup = _backup(archive) if archive.is_file() else None
    try:
        if "yy_prompt_play(" in source:
            _upgrade_legacy_patch(sdk)
        else:
            _patch_header(header_path)
            _patch_source(source_path)
        _validate_source(sdk, require_archive=False)
    except Exception:
        shutil.copy2(source_backup, source_path)
        shutil.copy2(header_backup, header_path)
        raise
    print(f"patched source: {source_path}")
    backup_paths = [str(source_backup), str(header_backup)]
    if archive_backup is not None:
        backup_paths.append(str(archive_backup))
    print("backups: " + ", ".join(backup_paths))


def rebuild_sdk(sdk: Path) -> None:
    sdk = sdk.resolve()
    _validate_source(sdk, require_archive=False)
    project = sdk / "projects/offline_asr_sample/project_file"
    make = sdk / "tools/build-tools/bin/make.exe"
    if not make.is_file():
        raise RuntimeError(f"SDK make tool is missing: {make}")
    install = Path(os.environ.get("TIANWEN_HOME", "D:/app/twenblock"))
    env = os.environ.copy()
    env["PATH"] = os.pathsep.join(
        [
            str(install / "asrpro/gcc/bin"),
            str(install / "toolchains/build-tools/bin"),
            str(sdk / "tools/build-tools/bin"),
            str(Path(os.environ.get("SystemRoot", "C:/Windows")) / "System32"),
            env.get("PATH", ""),
        ]
    )
    backup = project / ("core.a" + BACKUP_SUFFIX)
    source = sdk / "components/cmd_info/prompt_player.c"
    header = sdk / "components/cmd_info/prompt_player.h"
    source_backup = source.with_name(source.name + BACKUP_SUFFIX)
    header_backup = header.with_name(header.name + BACKUP_SUFFIX)
    log = project / "yy_prompt_rebuild.log"
    try:
        with log.open("wb") as output:
            clean = subprocess.run([str(make), "clean"], cwd=project, env=env, stdout=output,
                                   stderr=subprocess.STDOUT)
            if clean.returncode:
                raise subprocess.CalledProcessError(clean.returncode, clean.args)
            result = subprocess.run([str(make), "-j4"], cwd=project, env=env, stdout=output,
                                    stderr=subprocess.STDOUT)
            if result.returncode:
                raise subprocess.CalledProcessError(result.returncode, result.args)
        _validate_source(sdk)
        if not (project / "build/program.elf").is_file():
            raise RuntimeError("SDK build reported success but build/program.elf is missing")
        if not (project / "../../../../user_code/[0]code.bin").resolve().is_file():
            raise RuntimeError("SDK build reported success but user code image is missing")
    except Exception:
        if source_backup.is_file():
            shutil.copy2(source_backup, source)
        if header_backup.is_file():
            shutil.copy2(header_backup, header)
        if backup.is_file():
            shutil.copy2(backup, project / "core.a")
        else:
            (project / "core.a").unlink(missing_ok=True)
        try:
            subprocess.run([str(make), "clean"], cwd=project, env=env,
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except OSError:
            pass
        raise
    print(f"build log: {log}")
    print(f"rebuilt patched SDK archive: {project / 'core.a'}")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--sdk", type=Path, required=True)
    parser.add_argument("--check", action="store_true", help="validate without modifying or rebuilding")
    parser.add_argument("--no-build", action="store_true", help="patch only; do not rebuild core.a")
    args = parser.parse_args()
    patch_sdk(args.sdk, check_only=args.check)
    if not args.check and not args.no_build:
        rebuild_sdk(args.sdk)
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:
        print(f"patch_sdk: ERROR: {exc}", file=sys.stderr)
        raise SystemExit(2)
