"""Collect records, checksum every release file, and make the user handoff ZIP."""
from pathlib import Path
import hashlib, json, shutil, zipfile
ROOT=Path(__file__).resolve().parents[1]
release=ROOT/'release/yaoyao_asrpro_2m_20261025_obstacle'
manifest=json.loads((release/'manifest.json').read_text(encoding='utf-8'))
digest=manifest['firmwareSHA256']
assert hashlib.sha256((release/'fw.bin').read_bytes()).hexdigest()==digest
shutil.copyfile(ROOT.parent/'OBSTACLE_BOARD_TEST.md',release/'上板说明.md')
shutil.copyfile(ROOT/'README.md',release/'README.md')
shutil.copyfile(ROOT/'voice_runtime.cpp',release/'voice_runtime.cpp')
validation=release/'validation'
validation.mkdir(exist_ok=True)
for name in ['full_sdk_build.log','package.log','package_commands.json','host_tests.log','blockly_validation.json','cross_compile.log','link.log','generated.cpp','current_verification.json','release_validation.json','model_start_obstacle.json','model_events_obstacle.json','model_poll_obstacle.json']:
    shutil.copyfile(ROOT/'records'/name,validation/name)
shutil.copyfile(ROOT/'records/OBSTACLE_VALIDATION.md',validation/'VALIDATION.md')
for name in ['patch_sdk.py','build_project.py','prepare_firmware.py','build_firmware.py','check_project.py','package_firmware.py','verify_release.py','finalize_release.py']:
    target=release/'tools'/name
    target.parent.mkdir(exist_ok=True)
    shutil.copyfile(ROOT/'tools'/name,target)
entries=[]
for p in sorted(release.rglob('*')):
    if p.is_file() and p.name!='SHA256SUMS.txt':
        entries.append(hashlib.sha256(p.read_bytes()).hexdigest()+'  '+p.relative_to(release).as_posix())
(release/'SHA256SUMS.txt').write_text('\n'.join(entries)+'\n',encoding='utf-8')
archive=release.parent/(release.name+'.zip')
with zipfile.ZipFile(archive,'w',zipfile.ZIP_DEFLATED) as z:
    for p in sorted(release.rglob('*')):
        if p.is_file():z.write(p,p.relative_to(release.parent))
with zipfile.ZipFile(archive) as z:assert z.testzip() is None
print('ZIP:',archive,'bytes:',archive.stat().st_size)
print('Firmware SHA256:',digest)
