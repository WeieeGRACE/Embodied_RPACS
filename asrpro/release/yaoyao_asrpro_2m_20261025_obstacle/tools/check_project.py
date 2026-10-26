"""Cross-compile generated code with Tianwen's own SDK; run host behavioral tests."""
from pathlib import Path
import os, re, subprocess, sys

ROOT=Path(__file__).resolve().parents[1]
sys.stdout.reconfigure(encoding='utf-8',errors='replace')
INSTALL=Path(os.environ.get('TIANWEN_HOME','D:/app/twenblock'))
SDK=INSTALL/'asrpro/asr_pro_sdk'
OUT=ROOT/'records/build'
OUT.mkdir(parents=True,exist_ok=True)
def run(args, name, **kwargs):
    result=subprocess.run(args,capture_output=True,**kwargs)
    output=(result.stdout+result.stderr).decode('utf-8',errors='replace').replace('\r\r\n','\n').replace('\r\n','\n')
    (ROOT/'records'/name).write_text(output,encoding='utf-8')
    print(output or f'{name}: PASS')
    if result.returncode: sys.exit(result.returncode)

mk=(SDK/'projects/offline_asr_sample/project_file/build/source_file.mk').read_text()
includes=re.findall(r'^CPP_FLAGS \+= -I(.+)$',mk,re.M)
flags=['-march=rv32imafc','-mabi=ilp32f','-mcmodel=medlow','-msmall-data-limit=8',
       '-msave-restore','-mfdiv','-Os','-fsigned-char','-ffunction-sections','-fdata-sections',
       '-fno-common','-fno-exceptions','-flto','-fpermissive','-g',
       '-DCI_CONFIG_FILE="user_config.h"','-fshort-enums']
flags += ['-I'+s.strip().replace('$(ROOT_DIR)',SDK.as_posix()) for s in includes]
run([str(INSTALL/'asrpro/gcc/bin/riscv-nuclei-elf-g++.exe'),*flags,'-c',
     str(ROOT/'records/generated.cpp'),'-o',str(OUT/'yaoyao.o')],'cross_compile.log')

# Link against the installed vendor core; no model generation or device access.
project=SDK/'projects/offline_asr_sample'
makefile=(project/'project_file/makefile').read_text()
wraps=re.findall(r'^LD_FLAGS \+= (-Wl,--wrap,\w+)$',makefile,re.M)
libs=re.findall(r'^LIBS \+= (.+)$',mk,re.M)
run([str(INSTALL/'asrpro/gcc/bin/riscv-nuclei-elf-g++.exe'),
     '-march=rv32imafc','-mabi=ilp32f','-mcmodel=medlow','-msave-restore','-Os','-flto',
     '-T',str(project/'src/asrpro.lds'),'-nostartfiles','-Wl,--gc-sections','--specs=nano.specs',
     '-Wl,-Map,'+str(OUT/'yaoyao.map'),'-L'+str(SDK/'utils'),'-L'+str(SDK/'libs'),
     *wraps,'-o',str(OUT/'yaoyao.elf'),str(OUT/'yaoyao.o'),
     '-Wl,--start-group',str(ROOT/'records/build/firmware_obstacle_20261025/asr_pro_sdk/projects/offline_asr_sample/project_file/core.a'),'-lm',*libs,'-lcikd_asrBc',
     '-Wl,--end-group'],'link.log')

for header in ['asr.h','setup.h','HardwareSerial.h','queue.h','semphr.h']:
    (OUT/header).write_text('#include "../../tests/host_sdk.h"\n',encoding='utf-8')
vc=Path('C:/Program Files (x86)/Microsoft Visual Studio/18/BuildTools/VC/Auxiliary/Build/vcvars64.bat')
if not vc.exists(): raise SystemExit('MSVC Build Tools required for host behavioral checks')
bat=OUT/'run_host_tests.cmd'
bat.write_text('@echo off\nset "PATH=%SystemRoot%\\System32;%PATH%"\ncall "'+str(vc)+'" >nul\n'
    'cl /nologo /EHsc /std:c++14 /utf-8 /I"'+str(OUT)+'" "'+str(ROOT/'tests/test_runtime.cpp')+
    '" /Fo"'+str(OUT/'test_runtime.obj')+'" /Fe"'+str(OUT/'test_runtime.exe')+'"\n'
    'if errorlevel 1 exit /b 1\n"'+str(OUT/'test_runtime.exe')+'"\n',encoding='utf-8')
run(['cmd.exe','/d','/c',str(bat)],'host_tests.log',cwd=OUT)
