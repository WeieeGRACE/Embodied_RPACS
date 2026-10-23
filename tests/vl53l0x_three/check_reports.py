"""Check final Gowin reports against CST, without modifying generated reports."""
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
cst = (root / 'src/vl53l0x_three_top.cst').read_text()
report = (root / 'impl/pnr/vl53l0x_three.rpt.txt').read_text()
locations = re.findall(r'IO_LOC\s+"([^"]+)"\s+(\w+);', cst)
pins = dict(locations)
assert len(locations) == len(pins)
assert len(pins) == len(set(pins.values())) == 25
assert {name: pin for name, pin in pins.items() if name.startswith('tof_')} == TOF_P6
sensor_pins = set(TOF_P6.values())
project = root.parent.parent
other_constraints = list((project / 'src').rglob('*.cst')) + list((project / 'tests').rglob('*.cst'))
checked = 0
partner = project / 'tests/asrpro_uart/src/asrpro_uart_top.cst'
for other in other_constraints:
    if other.resolve() == (root / 'src/vl53l0x_three_top.cst').resolve():
        continue
    entries = re.findall(r'IO_LOC\s+"([^"]+)"\s+(\w+)\s*;', other.read_text())
    if other.resolve() == partner.resolve():
        assert {name: pin for name, pin in entries if name.startswith('tof_')} == TOF_P6
        assert all(pin not in sensor_pins or TOF_P6.get(name) == pin for name, pin in entries)
    else:
        assert not sensor_pins & {pin for _, pin in entries}, other
    checked += 1
assert partner.is_file()

table = report.split('7. Pinout by Port Name')[1].split('8. All Package Pins')[0]
evidence = []
for name, pin in pins.items():
    line = next(line for line in table.splitlines() if line.split('|')[0].strip() == name)
    fields = [part.strip() for part in line.split('|')]
    assert fields[2].split('/')[0] == pin, (name, 'pin')
    assert fields[7] == 'LVCMOS33' and fields[9] == 'NONE', (name, 'electrical')
    inp = name in ('clk_50m', 'reset_n', 'imu_gnd_reserved') or name.endswith('gpio1')
    direction = 'in' if inp else ('io' if name.startswith('tof_') or name == 'oled_sda' else 'out')
    assert fields[4] == direction, (name, 'direction')
    assert fields[8] == ('OFF' if inp else '8'), (name, 'drive')
    assert fields[16] == '3.3', (name, 'VCCIO')
    evidence.append(f'{name:20} {pin:4} {direction:3} DRIVE={fields[8]:3} PULL=NONE LVCMOS33 3.3V')
for config in ('project_process_config.json', 'vl53l0x_three_process_config.json'):
    data = json.loads((root / 'impl' / config).read_text())
    # Gowin process options may be nested; inspect exact option keys recursively.
    def options(node):
        if isinstance(node, dict):
            for key, value in node.items():
                if key in ('CPU', 'SSPI'):
                    yield key, value
                yield from options(value)
        elif isinstance(node, list):
            for item in node:
                yield from options(item)
    found = dict(options(data))
    assert found.get('CPU') is True and found.get('SSPI') is True, config
timing = (root / 'impl/pnr/vl53l0x_three_tr_content.html').read_text()
freq = timing.split('Max Frequency Summary:')[1].split('</table>')[0]
values = re.findall(r'<td>([^<]+)', freq)
fmax = float(values[3].split('(')[0])
assert fmax >= 50, fmax
tns = timing.split('Total Negative Slack Summary:')[1].split('</table>')[0]
values = re.findall(r'<td>([^<]+)', tns)
assert values[2:4] == ['0.000', '0'] and values[6:8] == ['0.000', '0']
fs = root / 'impl/pnr/vl53l0x_three.fs'
assert fs.stat().st_size > 100000
sources = ET.parse(root / 'vl53l0x_three.gprj').findall('./FileList/File')
assert all(fs.stat().st_mtime_ns >= (root / item.attrib['path']).stat().st_mtime_ns for item in sources), 'Stale bitstream'
device = (root / 'impl/pnr/device.cfg').read_text()
assert 'set CPU regular_io = true' in device and 'set SSPI regular_io = true' in device
for suite in ('edges', 'bus', 'three', 'oled', 'timing'):
    log = (root / f'records/{suite}.log').read_text()
    assert 'PASS ' in log and '** Fatal:' not in log and '** Error:' not in log, suite
evidence += [f'PASS 25 pins; CPU/SSPI GPIO in both saved configs',
             f'PASS fixed P6 mapping in both projects; checked {checked} other CST files (only ASR protection may share ToF pins)',
             f'PASS 50 MHz constraint; Fmax={fmax} MHz; setup/hold TNS=0',
             f'FS bytes={fs.stat().st_size}; SHA256={hashlib.sha256(fs.read_bytes()).hexdigest()}']
output = '\n'.join(evidence) + '\n'
(root / 'records/report_check.txt').write_text(output)
print(output, end='')
