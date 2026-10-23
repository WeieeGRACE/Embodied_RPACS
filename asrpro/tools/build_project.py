"""Build the editable Tianwen ASRPRO XML project (standard library only)."""
from pathlib import Path
import xml.etree.ElementTree as E

ROOT = Path(__file__).resolve().parents[1]
counter = 0
def block(kind, fields=None):
    global counter
    counter += 1
    b = E.Element('block', type=kind, id=f'yaoyao_{counter}')
    for name, value in (fields or {}).items():
        E.SubElement(b, 'field', name=name).text = str(value)
    return b
def chain(blocks):
    for a, b in zip(blocks, blocks[1:]): E.SubElement(a, 'next').append(b)
    return blocks[0]
def statement(parent, name, blocks):
    E.SubElement(parent, 'statement', name=name).append(chain(blocks))
def number(parent, name, value, kind='math_number', field='NUM'):
    E.SubElement(parent, 'value', name=name).append(block(kind, {field:value}))
def text(code): return block('block_with_textarea', {'input':code})

commands = [
    (1, '寻找红色水瓶', 0x01),
    (2, '停止任务', 0x02), (3, '停车', 0x02), (4, '停止', 0x02),
    (5, '查询状态', 0x03), (6, '现在什么状态', 0x03),
    (10, '前进', 0x10), (11, '向前走', 0x10),
    (12, '后退', 0x11), (13, '向后走', 0x11),
    (14, '左转', 0x12), (15, '向左转', 0x12), (16, '向左原地旋转', 0x12),
    (17, '右转', 0x13), (18, '向右转', 0x13), (19, '向右原地旋转', 0x13),
    (20, '加速', 0x14), (21, '快一点', 0x14),
    (22, '减速', 0x15), (23, '慢一点', 0x15),
]
audio = [(9000,'已接受'), (9001,'忙碌中'), (9002,'无法执行'),
         (9100,'空闲'), (9101,'执行中'), (9103,'已到达'),
         (9104,'已停止'), (9105,'失败'), (9106,'故障'),
         (9108,'已完成'), (9201,'收到')]

def build():
    root = E.Element('xml', xmlns='http://www.w3.org/1999/xhtml')
    source = text((ROOT/'voice_runtime.cpp').read_text(encoding='utf-8'))
    source.set('x','0'); source.set('y','0'); root.append(source)
    setup = block('setup'); setup.set('x','0'); setup.set('y','260'); root.append(setup)
    init = [block('asr_speak', {'VOICE_SET':'小蝶-清新女声','vol':10,'speed':10}),
            block('asr_welcom', {'NAME':'幺幺准备好了，请说幺幺耍起唤醒我。'}),
            block('asr_sleep', {'NAME':'幺幺休息了，请说幺幺耍起唤醒我。'}),
            block('asr_id', {'ASR':'幺幺耍起','leibie':'唤醒词','huifu':'幺幺在，请说指令。','id':0})]
    for ident, phrase, code in commands:
        init.append(block('asr_id', {'ASR':phrase, 'leibie':'唤醒词' if code==2 else '命令词',
                                     'huifu':'', 'id':ident}))
    init += [block('asr_add_id', {'NAME':phrase,'id':ident}) for ident, phrase in audio]
    statement(setup,'DO',init)
    hw = block('hardware_init'); hw.set('x','750'); hw.set('y','260'); root.append(hw)
    serial = block('serial_begin', {'serial_select':'Serial1','TX':'PA_2','RX':'PA_3'})
    number(serial,'CONTENT',115200)
    statement(hw,'DO',[serial,text('vol_set(7);\nyy_init();')])
    proc = block('procedures_defnoreturn', {'NAME':'ASR_CODE'})
    proc.set('x','750'); proc.set('y','520'); root.append(proc)
    # Largest milliseconds value whose multiplication by the SDK's 500 Hz
    # tick rate fits uint32_t. The standard block truncates fractional seconds.
    wake = text('set_state_enter_wakeup(8589934); // 8589.934秒，SDK安全上限')
    switch = block('controls_switch_case'); E.SubElement(switch,'mutation',elseif=str(len(commands)))
    E.SubElement(switch,'value',name='IF0').append(block('asr_snid'))
    for i, (ident, phrase, code) in enumerate(commands,1):
        number(switch,f'IF{i}',ident)
        statement(switch,f'DO{i}',[text(f'yy_send_command(0x{code:02X}); // {phrase}')])
    statement(proc,'STACK',[wake,switch])
    for name, priority, y, code in [('yaoyao_uart_rx',5,900,'yy_rx_poll();'),
                                    ('yaoyao_audio',4,1150,'yy_audio_poll();')]:
        thread = block('thread_app', {'app_name':name,'uxPriority':priority,'mem':512})
        thread.set('x','750'); thread.set('y',str(y)); root.append(thread)
        loop = block('controls_repeat_forever'); statement(loop,'DO',[text(code)])
        statement(thread,'DO',[loop])
    E.indent(root)
    output = E.tostring(root,encoding='unicode')
    output += '\n<!-- edittype="asr_pro" -->\n<!-- officialLib="[]" -->\n<!-- personalLib="[]" -->\n'
    (ROOT/'asrpro.hd').write_text(output,encoding='utf-8')
    print(f'Built {ROOT / "asrpro.hd"}: {len(commands)+1} recognition phrases, {len(audio)} feedback clips')

if __name__ == '__main__': build()
