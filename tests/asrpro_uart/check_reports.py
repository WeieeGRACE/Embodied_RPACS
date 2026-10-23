"""Verify source constraints and final Gowin pin/timing reports; save concise evidence."""
from pathlib import Path
import hashlib
import json
import re
import xml.etree.ElementTree as ET

# Approved P6 mapping is independent of CST and generated reports.
TOF_P6 = {
    'tof_front_scl': 'G16', 'tof_front_sda': 'G18',
    'tof_front_gpio1': 'F17', 'tof_front_xshut_n': 'F18',
    'tof_left_scl': 'H13', 'tof_left_sda': 'H14',
    'tof_left_gpio1': 'K15', 'tof_left_xshut_n': 'K16',
    'tof_right_scl': 'J13', 'tof_right_sda': 'K14',
    'tof_right_gpio1': 'K12', 'tof_right_xshut_n': 'K13',
}


root = Path(__file__).resolve().parent
project = root.parent.parent
expected = json.loads((root / 'pins.json').read_text())
cst = (root / 'src/asrpro_uart_top.cst').read_text()
locations = re.findall(r'IO_LOC\s+"([^"]+)"\s+(\w+);', cst)
pins = dict(locations)
assert len(locations) == len(pins) == len(set(pins.values())) == len(expected) == 38
assert pins == {name: pin for name, pin, _, _ in expected}
assert {name: pin for name, pin in pins.items() if name.startswith('tof_')} == TOF_P6
rtl = (root / 'src/asrpro_uart_top.v').read_text()
ports_text = rtl.split(')(input wire')[0] if ')(input wire' in rtl else rtl
header = rtl[rtl.index(' input wire'):rtl.index(');')]
declared = set()
for chunk in re.split(r'\b(?:input|output|inout)\s+wire\s+', header)[1:]:
    declared.update(re.findall(r'\b\w+\b', chunk))
assert declared == set(pins), (declared - set(pins), set(pins) - declared)
assert '7\'b0' in rtl and "=9'bz" in rtl
entries = ET.parse(root / 'asrpro_uart.gprj').findall('./FileList/File')
for entry in entries:
    assert (root / entry.attrib['path']).is_file(), entry.attrib
assert sum(e.attrib.get('top') == '1' for e in entries) == 1
checked = 0
partner = project / 'tests/vl53l0x_three/src/vl53l0x_three_top.cst'
motion_uart = project / 'tests/asr_motion/src/asr_motion_gpio.cst'
sensor_pins = set(TOF_P6.values())
for other in [*(project / 'src').rglob('*.cst'), *(project / 'tests').rglob('*.cst')]:
    if other.resolve() == (root / 'src/asrpro_uart_top.cst').resolve():
        continue
    other_entries = re.findall(r'IO_LOC\s+"([^"]+)"\s+(\w+)\s*;', other.read_text())
    if other.resolve() == partner.resolve():
        assert {name: pin for name, pin in other_entries if name.startswith('tof_')} == TOF_P6
        assert all(pin not in sensor_pins or TOF_P6.get(name) == pin for name, pin in other_entries)
    else:
        assert not sensor_pins & {pin for _, pin in other_entries}, other
    # Existing standalone ASR motion GPIO variant intentionally uses the same UART.
    uart_entries = [(name, pin) for name, pin in other_entries if pin in {'D17', 'D18'}]
    if other.resolve() == motion_uart.resolve():
        assert dict(uart_entries) == {'uart_rx': 'D17', 'uart_tx': 'D18'}
    else:
        assert not uart_entries, (other, uart_entries)
    checked += 1
assert partner.is_file()
report = (root / 'impl/pnr/asrpro_uart.rpt.txt').read_text()
table = report.split('7. Pinout by Port Name')[1].split('8. All Package Pins')[0]
evidence = []
for name, pin, direction, pull in expected:
    line = next(line for line in table.splitlines() if line.split('|')[0].strip() == name)
    fields = [part.strip() for part in line.split('|')]
    actual_direction = {'IN': 'in', 'OUT': 'out', 'INOUT': 'io'}[direction]
    assert fields[2].split('/')[0] == pin, (name, 'position', fields)
    assert fields[4] == actual_direction, (name, 'direction', fields)
    assert fields[7] == 'LVCMOS33' and fields[16] == '3.3', (name, 'voltage', fields)
    assert fields[9] == pull, (name, 'pull', fields)
    assert fields[8] == ('OFF' if direction == 'IN' else '8'), (name, 'drive', fields)
    evidence.append(f'{name:22} {pin:4} {actual_direction:3} DRIVE={fields[8]:3} PULL={pull:4} LVCMOS33 3.3V')
for config in ['project_process_config.json', 'asrpro_uart_process_config.json']:
    data = json.loads((root / 'impl' / config).read_text())
    assert data['CPU'] and data['SSPI']
    assert data['TopModule'] == 'asrpro_uart_top' and data['OUTPUT_BASE_NAME'] == 'asrpro_uart'
    assert float(data['Global_Freq']) == 50
timing = (root / 'impl/pnr/asrpro_uart_tr_content.html').read_text()
freq = timing.split('Max Frequency Summary:')[1].split('</table>')[0]
values = re.findall(r'<td>([^<]+)', freq)
fmax = float(values[3].split('(')[0])
assert fmax >= 50, fmax
tns = timing.split('Total Negative Slack Summary:')[1].split('</table>')[0]
values = re.findall(r'<td>([^<]+)', tns)
assert values[2:4] == ['0.000', '0'] and values[6:8] == ['0.000', '0'], values
for suite in ['protocol', 'control_fifo', 'top']:
    log = (root / f'records/{suite}.log').read_text()
    assert f'PASS {suite}' in log and '** Fatal:' not in log and '** Error:' not in log
fs = root / 'impl/pnr/asrpro_uart.fs'
assert fs.stat().st_size > 100000
assert all(fs.stat().st_mtime_ns >= (root / item.attrib['path']).stat().st_mtime_ns for item in entries), 'Stale bitstream'
device = (root / 'impl/pnr/device.cfg').read_text()
assert 'set CPU regular_io = true' in device and 'set SSPI regular_io = true' in device
evidence += [f'PASS 38 unique ports; all source ports constrained; saved GUI configs match Tcl',
             f'PASS fixed P6 mapping in both projects; checked {checked} other CST files; only ToF partner and ASR motion UART reuse allowed',
             f'PASS 50 MHz clock; Fmax={fmax} MHz; setup/hold TNS=0',
             'PASS protocol, control_fifo, top simulation logs',
             f'FS bytes={fs.stat().st_size}; SHA256={hashlib.sha256(fs.read_bytes()).hexdigest()}']
output = '\n'.join(evidence) + '\n'
(root / 'records/report_check.txt').write_text(output)
print(output, end='')
