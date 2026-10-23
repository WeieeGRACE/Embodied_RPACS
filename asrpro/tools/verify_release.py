"""Verify packaged bytes, SDK checksums, actual model words/IDs, then export."""
from pathlib import Path
import binascii, hashlib, json, os, re, shutil, struct, zipfile
import xml.etree.ElementTree as E
ROOT=Path(__file__).resolve().parents[1]
WORK=ROOT/'records/build/firmware_short_20261023'
image=WORK/'packed/fw.bin'
fw=image.read_bytes()
assert len(fw)<0x1F0000
table=fw[0x2000:0x2000+278]
assert sum(table[:276])&65535==struct.unpack_from('<H',table,276)[0]
assert fw[:8192]==(WORK/'data/boot.bin').read_bytes()
partitions={}
paths={'code':'user_code/user_code.bin','asr':'asr/asr.bin','dnn':'dnn/dnn.bin',
       'voice':'voice/voice.bin','user_file':'user_file/user_file.bin'}
for name,index in [('code',0),('asr',2),('dnn',3),('voice',4),('user_file',5)]:
    version,offset,size,crc,status=struct.unpack_from('<IIIIB',table,166+index*17)
    payload=(WORK/paths[name]).read_bytes()
    assert len(payload)==size and fw[offset:offset+size]==payload,name
    assert binascii.crc_hqx(payload,0)==crc,(name,crc)
    partitions[name]={'offset':offset,'size':size,'crc16':crc,'sha256':hashlib.sha256(payload).hexdigest()}

def file_table(data):
    n=struct.unpack_from('<H',data)[0];result={}
    assert n<100
    for i in range(n):
        ident,offset,size=struct.unpack_from('<HII',data,2+i*10)
        assert ident not in result and offset+size<=len(data)
        result[ident]=data[offset:offset+size]
    return result
code=file_table((WORK/'user_code/user_code.bin').read_bytes())
assert code[0]==(WORK/'user_code/[0]code.bin').read_bytes()
voice=file_table((WORK/'voice/voice.bin').read_bytes())
def mp3_frames(data):
    # SDK CI/PRIV headers use a 30-byte custom prefix (not a standard ID3 size).
    if data.startswith(b'ID3') and data[10:14]==b'PRIV' and data[20:22]==b'CI':
        return data[30:]
    if data.startswith(b'ID3'):
        n=0
        for x in data[6:10]:n=(n<<7)|(x&127)
        return data[10+n:]
    return data
silent=Path(os.environ.get('TIANWEN_HOME','D:/app/twenblock'))/'asrpro/demo_audio/空回复.mp3'
assert mp3_frames(voice[23])==mp3_frames(silent.read_bytes())
assert set(voice)=={0,23,9000,9001,9002,9100,9101,9103,9104,9105,9106,9108,10001,10002,9201}
for p in (WORK/'resources/voice/mp3').glob('*.mp3'):
    ident=int(re.match(r'\[(\d+)\]',p.name)[1])
    assert mp3_frames(voice[ident])==mp3_frames(p.read_bytes())
previous=ROOT/'release/yaoyao_asrpro_2m_20261023_receipts/model/voice/mp3'
for ident in [0,10001,10002]:
    original=list(previous.glob(f'[[]{ident}]*.mp3'))
    assert len(original)==1
    assert mp3_frames(voice[ident])==mp3_frames(original[0].read_bytes()),ident
asr=file_table((WORK/'asr/asr.bin').read_bytes())
for ident in [0,1]:
    original=(WORK/f'resources/asr/[{ident}]asr_chinese_V00487_normal.dat').read_bytes()
    assert asr[ident][:len(original)]==original
    # Official merge nn-file adds its 20-byte DNN pruning metadata to each model.
    assert len(asr[ident])==len(original)+20
    assert asr[ident][len(original):].startswith(b'<prun_table>')
user=file_table((WORK/'user_file/user_file.bin').read_bytes())
assert user[60000]==(WORK/'resources/user_file/[60000]{cmd_info}.xlsx.bin').read_bytes()

with zipfile.ZipFile(WORK/'resources/user_file/cmd_info/[60000]{cmd_info}.xlsx') as z:
    sheet=E.fromstring(z.read('xl/worksheets/sheet1.xml'))
rows=[{re.sub(r'\d','',c.get('r')):''.join(c.itertext()) for c in r} for r in sheet.findall('.//{*}row')]
report=json.loads((ROOT/'records/blockly_validation.json').read_text(encoding='utf-8'))
expected_audio={9000:'已接受',9001:'忙碌中',9002:'无法执行',9100:'空闲',
                9101:'执行中',9103:'已到达',9104:'已停止',9105:'失败',
                9106:'故障',9108:'已完成',9201:'收到'}
assert {int(x['id']):x['text'] for x in report['audio']}==expected_audio
assert next(x['reply'] for x in report['recognition'] if x['id']=='0')=='幺幺在，请说指令。'
words={r['C']:r for r in rows[1:] if r.get('C') not in ['P1','','<welcome>','<inactivate>','<beep>']}
assert set(words)=={x['phrase'] for x in report['recognition']}
for rec in report['recognition']:
    r=words[rec['phrase']]
    assert r['D']==rec['id']
    assert r['G']==('YES' if rec['type']=='唤醒词' else 'NO')
    assert int(r['M']) in voice
for clip in report['audio']:
    ident=int(clip['id']);assert ident in voice
    assert any(r.get('C')=='P1' and r.get('D')==clip['id'] and r.get('M')==clip['id'] for r in rows)
config=(WORK/'asr_pro_sdk/projects/offline_asr_sample/src/user_config.h').read_text(encoding='utf-8')
assert re.search(r'#define\s+USE_AEC_MODULE\s+1',config)
assert re.search(r'#define\s+PAUSE_VOICE_IN_WITH_PLAYING\s+0',config)
generated=(ROOT/'records/generated.cpp').read_text(encoding='utf-8')
assert generated==(WORK/'asr.cpp').read_text(encoding='utf-8')
assert 'set_state_enter_wakeup(8589934);' in generated
assert '//{playid:10001,voice:幺幺准备好了，请说幺幺耍起唤醒我。}' in generated
assert '//{playid:10002,voice:幺幺休息了，请说幺幺耍起唤醒我。}' in generated
assert 'Serial1.begin(115200);' in generated and 'Serial1.begin();' not in generated

release=ROOT/'release/yaoyao_asrpro_2m_20261023_short'
release.mkdir(parents=True,exist_ok=True)
shutil.copyfile(image,release/'fw.bin')
shutil.copyfile(ROOT/'asrpro.hd',release/'asrpro.hd')
shutil.copytree(WORK/'resources',release/'model',dirs_exist_ok=True)
with zipfile.ZipFile(release/'model_resources.zip','w',zipfile.ZIP_DEFLATED) as z:
    for p in sorted((WORK/'resources').rglob('*')):
        if p.is_file():z.write(p,p.relative_to(WORK/'resources'))
result={'date':'2026-10-23','software':'Tianwen Block AI 2026 autumn 0.2.8',
        'romBytes':0x200000,'nvOffset':0x1F0000,'firmwareBytes':len(fw),
        'firmwareSHA256':hashlib.sha256(fw).hexdigest(),'aecEnabled':True,'wakeTimeoutMs':8589934,'receiptAfterTransmit':True,
        'recognition':report['recognition'],'audio':report['audio'],
        'originalWelcomeWakeSleepPreserved':True,'executionStatusSpeechDeduplicated':True,
        'packagedVoiceIDs':sorted(voice),'partitions':partitions,
        'checks':['host behavioral checks including shared receipts, execution-state dedup, and timeout overflow','save/reopen baud 115200','full SDK build with AEC',
                  'empty command replies use vendor silent MP3',
                  'partition-table checksum','five partition CRC16s and exact bytes',
                  'bootloader exact bytes','current code file exact bytes',
                  'two current model payloads exact bytes plus SDK DNN pruning metadata',
                  'command info exact bytes','21 word/ID/wake-class matches',
                  '11 feedback audio IDs and silence/wake audio references',
                  'all packaged MP3 frames match current generated resources'],
        'hardwareTested':False,'deviceFlashed':False}
(ROOT/'records/release_validation.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
shutil.copyfile(ROOT/'records/release_validation.json',release/'manifest.json')
print('Verified firmware:',len(fw),'bytes; SHA256',result['firmwareSHA256'])
print('Release:',release)
