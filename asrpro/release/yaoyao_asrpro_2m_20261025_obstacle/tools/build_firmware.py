"""Full SDK build in the workspace; no serial/download operations."""
from pathlib import Path
import os, subprocess, sys
ROOT=Path(__file__).resolve().parents[1]
INSTALL=Path(os.environ.get('TIANWEN_HOME','D:/app/twenblock'))
WORK=ROOT/'records/build/firmware_obstacle_20261025'
project=WORK/'asr_pro_sdk/projects/offline_asr_sample/project_file'
env=os.environ.copy()
env['PATH']=os.pathsep.join([str(INSTALL/'asrpro/gcc/bin'),str(INSTALL/'toolchains/build-tools/bin'),str(INSTALL/'asrpro/asr_pro_sdk/tools/build-tools/bin'),str(Path(os.environ.get('SystemRoot','C:/Windows'))/'System32'),env.get('PATH','')])
log=ROOT/'records/full_sdk_build.log'
with log.open('wb') as f:
    result=subprocess.run([str(INSTALL/'toolchains/build-tools/bin/make.exe'),'-j4'],cwd=project,env=env,stdout=f,stderr=subprocess.STDOUT)
print('Full SDK build exit:',result.returncode,'log:',log,flush=True)
if result.returncode:
    sys.stdout.reconfigure(encoding='utf-8',errors='replace')
    print(log.read_text(encoding='utf-8',errors='replace')[-10000:])
sys.exit(result.returncode)
