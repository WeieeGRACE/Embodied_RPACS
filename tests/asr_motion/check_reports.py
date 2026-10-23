"""Verify the board artifact against saved constraints and timing reports."""
from pathlib import Path
import argparse
import hashlib
import json
import re
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parent


def check(gpio=False):
    basename = 'asr_motion_gpio' if gpio else 'asr_motion_test'
    project_name = 'asr_motion_gpio' if gpio else 'asr_motion'
    project = ET.parse(ROOT / (project_name + '.gprj'))
    sources = [(ROOT / entry.attrib['path']).resolve() for entry in project.findall('.//File')]
    assert all(path.is_file() for path in sources), 'Missing project source'
    fs = ROOT / f'impl/pnr/{basename}.fs'
    assert fs.stat().st_size > 1_000_000, 'Missing/empty bitstream'
    assert all(fs.stat().st_mtime_ns >= path.stat().st_mtime_ns for path in sources), 'Bitstream predates source'
    timing = (ROOT / f'impl/pnr/{basename}_tr_content.html').read_text()
    for kind in ('Setup', 'Hold'):
        match = re.search(r'Numbers of ' + kind + r' Violated Endpoints</td>\s*<td>(\d+)</td>', timing)
        assert match and match[1] == '0', f'{kind} timing violations'
    frequencies = re.findall(r'<td[^>]*>([\d.]+)\(MHz\)</td>', timing)
    assert len(frequencies) == 2 and float(frequencies[0]) == 50 and float(frequencies[1]) >= 50, '50 MHz timing not met'
    device = (ROOT / 'impl/pnr/device.cfg').read_text()
    assert 'set CPU regular_io = true' in device and 'set SSPI regular_io = true' in device
    cmd = (ROOT / 'impl/pnr/cmd.do').read_text()
    assert '-global_freq 50.000' in cmd
    for name in ((project_name + '_process_config.json',) if gpio else ('asr_motion_process_config.json', 'project_process_config.json')):
        cfg = json.loads((ROOT / 'impl' / name).read_text())
        assert cfg['CPU'] and cfg['SSPI'] and float(cfg['Global_Freq']) == 50
        assert cfg['TopModule'] == 'asr_motion_test_top' and cfg['OUTPUT_BASE_NAME'] == basename
    report = (ROOT / f'impl/pnr/{basename}.rpt.txt').read_text()
    locations = dict(re.findall(r'IO_LOC\s+"([^"]+)"\s+(\w+)\s*;', (ROOT / ('src/asr_motion_gpio.cst' if gpio else 'src/asr_motion_test_top.cst')).read_text()))
    assert len(locations) == len(set(locations.values())) == 28
    pin_rows = {}
    for port, ball in locations.items():
        row = next((line for line in report.splitlines() if line.split('|')[0].strip() == port), None)
        assert row, f'Missing pin {port}'
        cells = [cell.strip() for cell in row.split('|')]
        assert cells[2].split('/')[0] == ball and cells[7] == 'LVCMOS33' and cells[16] == '3.3', row
        pin_rows[port] = cells
    assert pin_rows['uart_rx'][4] == 'in' and pin_rows['uart_tx'][4] == 'out'
    assert pin_rows['imu_gnd_reserved'][4] == 'in' and pin_rows['imu_gnd_reserved'][9] == 'NONE'
    resource_text = report.split('3. Resource Usage Summary')[1].split('4. I/O Bank Usage Summary')[0]
    resources = {}
    for name in ('Logic', 'Register', 'BSRAM', 'DSP', 'I/O Port'):
        match = re.search(r'^\s*' + re.escape(name) + r'\s*\|\s*([\d.]+)/(\d+)', resource_text, re.M)
        assert match, f'Missing resource {name}'
        resources[name] = f'{match[1]}/{match[2]}'
    evidence = dict(
        result='PASS', fmax_mhz=float(frequencies[1]), setup_violations=0, hold_violations=0,
        pins=28, cpu_gpio=True, sspi_gpio=True, resources=resources,
        bitstream=str(fs.relative_to(ROOT)), bytes=fs.stat().st_size,
        sha256=hashlib.sha256(fs.read_bytes()).hexdigest(),
        source_sha256={str(p.relative_to(ROOT)) if p.is_relative_to(ROOT) else str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in sources},
    )
    (ROOT / ('records/gpio_build_evidence.json' if gpio else 'records/build_evidence.json')).write_text(json.dumps(evidence, indent=2) + '\n')
    print(json.dumps({key: value for key, value in evidence.items() if key != 'source_sha256'}, indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--gpio', action='store_true')
    check(parser.parse_args().gpio)
