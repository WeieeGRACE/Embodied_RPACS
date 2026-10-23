from pathlib import Path
import subprocess,json,hashlib,sys
out=Path(__file__).resolve().parent
sdk=Path('D:/app/twenblock/asrpro/asr_pro_sdk')
project=sdk/'projects/offline_asr_sample/project_file'
compiler=sdk.parent/'gcc/bin/riscv-nuclei-elf-g++.exe'
prj=(project/'source_file.prj').read_text(errors='replace')
includes=['-I'+str(sdk/line.split(':',1)[1].strip()) for line in prj.splitlines() if line.startswith('include-path:')]
common=['-march=rv32imafc','-mabi=ilp32f','-mcmodel=medlow','-msmall-data-limit=8','-msave-restore','-mfdiv','-Os','-fmessage-length=0','-fsigned-char','-ffunction-sections','-fdata-sections','-fno-common','-flto','-g','-fshort-enums']
commands=[]
def run(stage,args):
    cmd=[str(compiler),*args];commands.append(cmd);(out/"commands.json").write_text(json.dumps(commands,ensure_ascii=False,indent=2),encoding="utf8")
    p=subprocess.run(cmd,cwd=project,stdout=subprocess.PIPE,stderr=subprocess.STDOUT)
    (out/(stage+'.log')).write_bytes(p.stdout)
    print(stage+': exit '+str(p.returncode),flush=True)
    if p.returncode: print(p.stdout.decode(errors='replace').encode('ascii',errors='backslashreplace').decode());sys.exit(p.returncode)
run('cross_compile',common+['-fno-exceptions','-fpermissive','-DCI_CONFIG_FILE="user_config.h"',*includes,'-c',str(out/'generated.cpp'),'-o',str(out/'asrpro.o')])
libs=[str(sdk/line.split(':',1)[1].strip().replace('$(LIBS_PATH)','libs')) for line in prj.splitlines() if line.startswith('library-file:')]
wraps=['abs','isnan','isinf','cosf','sinf','tanf','expf','sqrtf','fabsf','logf','log10f']
run('link',common+['-T',str(sdk/'projects/offline_asr_sample/src/asrpro.lds'),'-nostartfiles','-Wl,--gc-sections','-Wl,-Map,'+str(out/'asrpro.map'),'--specs=nano.specs',*['-Wl,--wrap,'+s for s in wraps],'-o',str(out/'asrpro.elf'),str(out/'asrpro.o'),'-Wl,--start-group',str(project/'core.a'),*libs,str(sdk/'libs/libcikd_asrBc.a'),'-lm','-Wl,--end-group'])
(out/'commands.json').write_text(json.dumps(commands,ensure_ascii=False,indent=2),encoding='utf8')
print('PASS current generated C++ cross-compilation and SDK core linking; no model/resources package produced')

