"""Collect records, checksum every release file, and make the user handoff ZIP."""
from pathlib import Path
import hashlib, json, shutil, zipfile
ROOT=Path(__file__).resolve().parents[1]
release=ROOT/'release/yaoyao_asrpro_2m_20261023_short'
manifest=json.loads((release/'manifest.json').read_text(encoding='utf-8'))
digest=manifest['firmwareSHA256']
for p in [release/'上板说明.md']:
    s=p.read_text(encoding='utf-8')
    # Keep the documented hash synchronized when packaging source assets changes.
    import re
    s=re.sub(r'(?<=`)[0-9a-f]{64}(?=`)',digest,s)
    p.write_text(s,encoding='utf-8')
validation=release/'validation'
validation.mkdir(exist_ok=True)
for name in ['full_sdk_build.log','package.log','package_commands.json','host_tests.log','blockly_validation.json','cross_compile.log','link.log','generated.cpp']:
    shutil.copyfile(ROOT/'records'/name,validation/name)
gui=json.loads((ROOT/'records/gui_final.json').read_text(encoding='utf-8'))
(validation/'gui_compile.json').write_text(json.dumps({'events':gui['build']},ensure_ascii=False,indent=2),encoding='utf-8')
(validation/'model_generation.json').write_text(json.dumps({
    'logs':[x for x in gui['logs'] if '模型' in x['text'] or 'TTS' in x['text']],
    'downloadEvents':[x for x in gui['events'] if x['time']>=gui['generationStartedAt']],
    'downloadRecovery':'Downloaded same generated resource URL directly, verified word/audio IDs, imported through Tianwen.'
},ensure_ascii=False,indent=2),encoding='utf-8')
shutil.copyfile(ROOT/'records/VALIDATION.md',validation/'VALIDATION.md')
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
