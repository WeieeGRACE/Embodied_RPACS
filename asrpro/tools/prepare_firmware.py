"""Prepare an isolated current-version SDK and freshly generated model resources."""
from pathlib import Path
import json, os, re, shutil, sys, zipfile

ROOT=Path(__file__).resolve().parents[1]
INSTALL=Path(os.environ.get('TIANWEN_HOME','D:/app/twenblock'))
WORK=ROOT/'records/build/firmware_short_20261023'
WORK.mkdir(parents=True,exist_ok=True)
sdk=WORK/'asr_pro_sdk'
if not sdk.exists():
    shutil.copytree(INSTALL/'asrpro/asr_pro_sdk',sdk,
                    ignore=shutil.ignore_patterns('build','core.a','*.elf','*.lst','*.map'))
project=sdk/'projects/offline_asr_sample'
build=project/'project_file/build'
(build/'objs').mkdir(parents=True,exist_ok=True)
shutil.copyfile(INSTALL/'asrpro/asr_pro_sdk/projects/offline_asr_sample/project_file/build/source_file.mk',build/'source_file.mk')
config=project/'src/user_config.h'
s=config.read_text(encoding='utf-8')
s,n=re.subn(r'(#define\s+USE_AEC_MODULE\s+)0\b',r'\g<1>1',s)
assert n==1 or re.search(r'#define\s+USE_AEC_MODULE\s+1',s)
config.write_text(s,encoding='utf-8')
shutil.copyfile(ROOT/'records/generated.cpp',WORK/'asr.cpp')
shutil.copyfile(ROOT/'records/generated.cpp',WORK/'_asr.cpp')
if '--code-only' in sys.argv:
    (WORK/'user_code').mkdir(exist_ok=True)
    print('Prepared current code and AEC SDK; model import deferred',flush=True)
    raise SystemExit(0)

# Always use a new resource directory; never mix with installed example resources.
resource=WORK/'resources'
if resource.exists():raise SystemExit('Resources already prepared; inspect before reusing this build directory')
resource.mkdir()
with zipfile.ZipFile(ROOT/'records/current_model.zip',metadata_encoding='gbk') as z:
    for info in z.infolist():
        target=(resource/info.filename).resolve()
        if not target.is_relative_to(resource.resolve()):raise ValueError('Unsafe zip path')
        if info.is_dir():target.mkdir(parents=True,exist_ok=True)
        else:
            target.parent.mkdir(parents=True,exist_ok=True)
            target.write_bytes(z.read(info))
for name in ['dnn']:
    (resource/name).mkdir()
    for p in (INSTALL/'asrpro'/name).iterdir():
        if p.is_file() and p.name.startswith('['):shutil.copyfile(p,resource/name/p.name)
(resource/'data').mkdir()
shutil.copyfile(INSTALL/'asrpro/data/boot.bin',resource/'data/boot.bin')
# Tianwen normally resets empty replies after download. Reproduce that step
# when recovering its download manually; the server's "无" placeholder is voiced.
empty_reply=list((resource/'voice/mp3').glob('[[]23]*'))
assert len(empty_reply)==1
shutil.copyfile(INSTALL/'asrpro/demo_audio/空回复.mp3',empty_reply[0])
(WORK/'user_code').mkdir(exist_ok=True)
print('Prepared fresh SDK; USE_AEC_MODULE=1; current model resources only',flush=True)
