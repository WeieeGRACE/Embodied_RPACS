"""Validate current evidence, then collect a matched FPGA/ASRPRO handoff."""
from pathlib import Path
import datetime, hashlib, html, json, re, shutil, sys, xml.etree.ElementTree as ET, zipfile

ROOT=Path(__file__).resolve().parents[1]
REC=ROOT/'records/obstacle_20261025'
EXPORT=ROOT/'release/Embodied_RPACS_main_20261026'
ASR=ROOT/'asrpro/release/yaoyao_asrpro_2m_20261025_obstacle'
DEST=ROOT/'release/Embodied_RPACS_obstacle_20261026'
def sha(p): return hashlib.sha256(p.read_bytes()).hexdigest()
def read(p): return p.read_text(encoding='utf-8',errors='replace')
def dump(p,value): p.write_text(json.dumps(value,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')

def validate():
    expected={
        'system.log':'PASS IMU/speed/timeout/still/overflow faults and root reset defaults',
        'system_oled.log':'PASS obstacle OLED entry, voice-off prompt, waiting, cruise, paused, ranges and keys',
        'system_50mhz.log':'PASS main project menu + 50MHz',
        'dispatch.log':'PASS dispatch 177 checks',
        'live_speed.log':'PASS live_speed 9 checks',
        'obstacle.log':'PASS main obstacle audio gate, menu navigation, takeover, busy, ToF bypass, live manual speed, exit',
        'attitude_pipeline.log':'PASS attitude display pipeline rounding, sign and limits',
    }
    suites={}
    for name,marker in expected.items():
        p=REC/name;s=read(p)
        assert marker in s and s.rstrip().endswith('Errors: 0, Warnings: 0'),name
        assert not re.search(r'\*\* (Fatal|Error)',s),name
        suites[name]={'sha256':sha(p),'passLines':re.findall(r'^# (PASS .*?)\r?$',s,re.M)}
    assert 'PASS main obstacle full  left route' in read(REC/'obstacle.log')
    assert 'PASS main obstacle full right route' in read(REC/'obstacle.log')
    for name in ['build.log','saved_project_build.log']:
        assert 'Bitstream generation completed' in read(REC/name),name
    assert 'PASS 44 unique constrained pins' in read(REC/'report_checks.log')
    timing=read(ROOT/'impl/pnr/Embodied_RPACS_tr_content.html')
    rows=[[html.unescape(re.sub('<[^>]+>','',c)).strip() for c in re.findall(r'<t[dh]\b[^>]*>(.*?)</t[dh]>',r,re.S|re.I)] for r in re.findall(r'<tr\b[^>]*>(.*?)</tr>',timing,re.S|re.I)]
    for label in ['Numbers of Setup Violated Endpoints','Numbers of Hold Violated Endpoints']:
        assert next(r[1] for r in rows if r and r[0]==label)=='0',label
    freq=next(r for r in rows if len(r)>=4 and r[1]=='clk_50m' and r[2]=='50.000(MHz)')
    fs=ROOT/'impl/pnr/Embodied_RPACS.fs'
    paths=[f.attrib['path'] for f in ET.parse(ROOT/'Embodied_RPACS.gprj').findall('./FileList/File')]
    assert paths,'empty project manifest'
    assert all((ROOT/p).stat().st_mtime<=fs.stat().st_mtime for p in paths),'stale bitstream'
    sources={p:sha(ROOT/p) for p in paths}
    host=ROOT/'asrpro/records/host_tests.log'
    assert 'PASS 190 checks' in read(host)
    manifest=json.loads(read(ASR/'manifest.json'))
    assert manifest['firmwareSHA256']==sha(ASR/'fw.bin')
    assert (ASR/'asrpro.hd').read_bytes()==(ROOT/'asrpro/asrpro.hd').read_bytes()
    assert manifest['hardwareTested'] is False and manifest['deviceFlashed'] is False
    result={'version':'2026-10-26 OA09 integrated','validatedAt':datetime.datetime.now().astimezone().isoformat(),
        'fpga':{'device':'GW5A-LV25UG324C2/I1','clockMHz':50,'actualFmax':freq[3],
                'setupViolations':0,'holdViolations':0,'bytes':fs.stat().st_size,'sha256':sha(fs)},
        'asrpro':{'bytes':manifest['firmwareBytes'],'sha256':manifest['firmwareSHA256'],
                  'hdSHA256':sha(ASR/'asrpro.hd'),'hostChecks':190,'words':23,'feedbackClips':15},
        'simulations':suites,'projectSources':sources,
        'hardwareTested':False,'deviceFlashed':False}
    dump(REC/'validation.json',result)
    md=f'''# 主工程与ASRPRO避障整合验证

版本：2026-10-26 OA09 integrated。验证时间：{result['validatedAt']}。
本记录仅包含数字仿真、构建及资源检查。未烧录、未执行实车测试。

## 构建与管脚

- Gowin 1.9.12，GW5A-LV25UG324C2/I1，50MHz；build.tcl与保存工程verify_project.tcl均成功。
- 实际Fmax：{freq[3]}；setup/hold违规均0，TNS为0；时序裕量较小，具体路径见随包timing报告。
- 44个唯一约束管脚、电气属性、10个开漏输出、CPU/SSPI复用GPIO、单一IMU/编码器/运动控制/PI/驱动实例检查通过。
- 工程及导出不引用历史tests目录；位流新于全部工程输入。
- FPGA位流SHA256：`{sha(fs)}`。
- ASRPRO完整固件SHA256：`{manifest['firmwareSHA256']}`；1,622,062字节。

## 仿真验证（6组整机/接口 + 1组流水专测）

| 日志 | 验证范围 | 结果 |
|---|---|---|
| system.log | 全菜单、4项定量动作、8项8秒电机测试、K0、停止/导航/语音开关、测距恢复、IMU/速度/超时/溢出故障、事件输出 | PASS |
| system_oled.log | 完整OLED位图、设置、数据单位和有效性、新避障面板各状态及提示 | PASS |
| system_50mhz.log | 真实50MHz参数、115200、1MHz SPI、10ms测速、20kHz PWM、菜单启用语音和停止 | PASS |
| dispatch.log | 177项；播报门控、错误/过期/重置确认、超时、各路线阶段接管、无路暂停、优先级、原子缓冲取消后不补发 | PASS |
| live_speed.log | 9项；实时升降、斜坡步长、限速优先/解除、目标与里程不重置、暂停不运动 | PASS |
| obstacle.log | 菜单与语音启动门控、左右两次连续绕行、两段300mm、实际里程回线、事件次数、实时档位、语音接管/忙碌/后退/退出/K0 | PASS |
| attitude_pipeline.log | 姿态显示流水格式化、正负号、四舍五入与180度上限 | PASS |

全部最终日志以Errors: 0, Warnings: 0结束。整机长路线使用加速参数及独立PWM驱动电机/IMU模型、真实I2C/UART位级通信；50MHz独立套件验证生产时钟与接口参数。
障碍位置由测试脚本切换三路测距输入，未建立完整光学和真实场地模型。

## ASRPRO与实测边界

天问实际积木加载/保存/重开/代码生成通过；23条词、15段反馈音频、隔离SDK完整构建、190项运行测试及固件逐分区/资源验证通过。
SDK厂商警告及模型生成下载恢复记录见asrpro/validation/VALIDATION.md和相关日志；未宣称ASRPRO零警告。
实际识别、回声抑制、播报打断、电机标定、20cm/90°精度、制动余量和ToF物理表现均待用户上板。
逐项实测步骤见OBSTACLE_BOARD_TEST.md；所有实测栏保持“待测”。
'''
    (REC/'VALIDATION.md').write_text(md,encoding='utf-8')
    if EXPORT.is_dir():
        exported_records=EXPORT/'records/obstacle_20261025'
        exported_records.mkdir(parents=True,exist_ok=True)
        shutil.copyfile(REC/'VALIDATION.md',exported_records/'VALIDATION.md')
    return result

result=validate()
if '--validate-only' in sys.argv:
    print('PASS final digital evidence; validation.json and VALIDATION.md written')
    raise SystemExit(0)
assert EXPORT.is_dir(),'run tools/export_main.ps1 first'
for rel,digest in result['projectSources'].items():assert sha(EXPORT/rel)==digest,rel
if DEST.exists():
    old_manifest=DEST/'manifest.json'
    assert old_manifest.is_file(), 'refusing to overwrite an unrecognized handoff directory'
    old_version=json.loads(read(old_manifest)).get('version')
    assert old_version in ['2026-10-25 OA09 integrated','2026-10-26 OA09 integrated'], 'refusing to overwrite an unrecognized handoff version'
DEST.mkdir(parents=True,exist_ok=True)
for folder in ['fpga','asrpro','validation','source']:(DEST/folder).mkdir(exist_ok=True)
shutil.copyfile(ROOT/'impl/pnr/Embodied_RPACS.fs',DEST/'fpga/Embodied_RPACS.fs')
shutil.copytree(ASR,DEST/'asrpro',dirs_exist_ok=True)
shutil.copytree(EXPORT,DEST/'source/Embodied_RPACS_main',dirs_exist_ok=True)
for name in ['README.md','OBSTACLE_BOARD_TEST.md','PIN_ASSIGNMENT.md']:
    shutil.copyfile(ROOT/name,DEST/name)
for p in REC.iterdir():
    if p.is_file() and p.suffix in ['.md','.json','.log']:shutil.copyfile(p,DEST/'validation'/p.name)
for name in ['Embodied_RPACS_tr_content.html','Embodied_RPACS.pin.html','Embodied_RPACS.rpt.txt','Embodied_RPACS.timing_paths']:
    shutil.copyfile(ROOT/'impl/pnr'/name,DEST/'validation'/name)
# Include ASR source/tests/generation tools and the exact downloaded model.
asrc=DEST/'source/asrpro';asrc.mkdir(parents=True,exist_ok=True)
for name in ['README.md','asrpro.hd','voice_runtime.cpp']:shutil.copyfile(ROOT/'asrpro'/name,asrc/name)
for name in ['tools','tests']:shutil.copytree(ROOT/'asrpro'/name,asrc/name,ignore=shutil.ignore_patterns('__pycache__'),dirs_exist_ok=True)
shutil.copytree(ASR/'validation',asrc/'records',dirs_exist_ok=True)
shutil.copyfile(ROOT/'asrpro/records/current_model.zip',asrc/'records/current_model.zip')
shutil.copyfile(ROOT/'asrpro/records/OBSTACLE_VALIDATION.md',asrc/'records/OBSTACLE_VALIDATION.md')
# verify_release compares retained startup audio with this original baseline.
baseline=ROOT/'asrpro/release/yaoyao_asrpro_2m_20261023_receipts/model/voice/mp3'
target=asrc/'release/yaoyao_asrpro_2m_20261023_receipts/model/voice/mp3';target.mkdir(parents=True,exist_ok=True)
for p in baseline.glob('*.mp3'):
    if p.name.startswith(('[0]','[10001]','[10002]')):shutil.copyfile(p,target/p.name)
dump(DEST/'manifest.json',result)
(DEST/'SHA256SUMS.txt').write_text(''.join(f'{sha(p)}  {p.relative_to(DEST).as_posix()}\n' for p in sorted(DEST.rglob('*')) if p.is_file() and p.name!='SHA256SUMS.txt'),encoding='utf-8')
archive=DEST.with_suffix('.zip')
with zipfile.ZipFile(archive,'w',zipfile.ZIP_DEFLATED) as z:
    for p in sorted(DEST.rglob('*')):
        if p.is_file():z.write(p,p.relative_to(DEST.parent))
with zipfile.ZipFile(archive) as z:assert z.testzip() is None
archive.with_suffix('.zip.sha256').write_text(sha(archive)+'  '+archive.name+'\n',encoding='utf-8')
print('PASS matched handoff:',archive,'bytes:',archive.stat().st_size)
print('SHA256:',sha(archive))
