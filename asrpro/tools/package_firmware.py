"""Package with the installed official SDK CLI; never invoke a serial burner."""
from pathlib import Path
import binascii, json, os, shutil, struct, subprocess, sys
ROOT=Path(__file__).resolve().parents[1]
INSTALL=Path(os.environ.get('TIANWEN_HOME','D:/app/twenblock'))
WORK=ROOT/'records/build/firmware_obstacle_20261025'
TOOL=INSTALL/'asrpro/asr_pro_sdk/tools/ci-tool-kit.exe'
commands=[]
sys.stdout.reconfigure(encoding='utf-8',errors='replace')
log=(ROOT/'records/package.log').open('wb')
def run(*args):
    cmd=[str(TOOL),*map(str,args)];commands.append(cmd)
    log.write(('COMMAND '+json.dumps(cmd)+'\n').encode())
    result=subprocess.run(cmd,cwd=WORK,stdout=subprocess.PIPE,stderr=subprocess.STDOUT)
    log.write(result.stdout);log.flush()
    print(result.stdout.decode('utf-8',errors='replace')[-1200:],flush=True)
    if result.returncode:raise RuntimeError('SDK CLI failed: '+str(result.returncode))

for name in ['asr','voice','user_file','dnn','data']:
    if not (WORK/name).exists():shutil.copytree(WORK/'resources'/name,WORK/name)
shutil.copyfile(INSTALL/'asrpro/asr_pro_sdk/libs/libfbin.a',WORK/'user_code/[1]code.bin')
for parent in [WORK/'resources/voice/mp3',WORK/'voice/mp3']:
    empty_reply=list(parent.glob('[[]25]*'))
    assert len(empty_reply)==1
    shutil.copyfile(INSTALL/'asrpro/demo_audio/空回复.mp3',empty_reply[0])
run('ID3-editor','--input-dir',WORK/'voice/mp3')
run('merge','user-file','-i',WORK/'voice/mp3')
shutil.copyfile(WORK/'voice/mp3/mp3.bin',WORK/'voice/voice.bin')
run('merge','user-file','-i',WORK/'user_code')
run('merge','user-file','-i',WORK/'user_file')
run('merge','asr-file','-i',WORK/'asr')
run('merge','nn-file','-i',WORK/'dnn','-a',WORK/'asr')
out=WORK/'packed'
out.mkdir(exist_ok=True)
run('make-firmware','-f','v2','--factory-id','100','--brand-id','100',
    '--board-name','haohaodada','--hardware-version','2.0.0',
    '--firmware-name','Yaoyao_ASRPRO_2M','--firmware-version','2.0.0',
    '--user-code',WORK/'user_code/user_code.bin','--asr-file',WORK/'asr/asr.bin',
    '--nn-file',WORK/'dnn/dnn.bin','--voice-file',WORK/'voice/voice.bin',
    '--user-file',WORK/'user_file/user_file.bin','--boot-file',WORK/'data/boot.bin',
    '--rom-size','0x200000','--nvdata-size','0x10000','--chip-name','CI130*','--output-path',out)
(ROOT/'records/package_commands.json').write_text(json.dumps(commands,indent=2),encoding='utf-8')
log.close()
print('Packaging outputs:',[(p.name,p.stat().st_size)for p in out.iterdir()])

# The installed SDK CLI v2 packer writes each alignment gap one byte short.
# Keep its bootloader/partition metadata, place exact payloads at the offsets
# consumed by ci_flash_data_info.c, then verify every size and CRC independently.
raw=(out/'Yaoyao_ASRPRO_2M_2.0.0.bin').read_bytes()
names={0:'user_code/user_code.bin',2:'asr/asr.bin',3:'dnn/dnn.bin',4:'voice/voice.bin',5:'user_file/user_file.bin'}
parts=[]
for index,name in names.items():
    _,offset,size,crc,_=struct.unpack_from('<IIIIB',raw,0x2000+166+index*17)
    payload=(WORK/name).read_bytes()
    assert len(payload)==size and binascii.crc_hqx(payload,0)==crc
    parts.append((offset,payload))
fixed=bytearray(b'\xff'*max(offset+len(data) for offset,data in parts))
fixed[:min(offset for offset,data in parts)]=raw[:min(offset for offset,data in parts)]
for offset,data in parts:fixed[offset:offset+len(data)]=data
(out/'fw.bin').write_bytes(fixed)
print('Verified partition placement fix:',len(raw),'->',len(fixed),'bytes')
