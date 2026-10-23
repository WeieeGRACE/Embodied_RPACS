"""Check actual UART values against the 12 captured sensor bytes.

This verifies digital read/parse integrity, not optical accuracy or calibration.
--require-all additionally requires every observed channel transaction to succeed.
"""
import argparse
from collections import Counter
from pathlib import Path
import re

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--log', required=True, type=Path)
parser.add_argument('--require-all', action='store_true')
args = parser.parse_args()
pattern = re.compile(
    r'(FRONT|LEFT|RIGHT)\s+seq=(\w+) mm=([\d-]+) id=([\w-]+) rev=([\w-]+) '
    r'pre=([\w-]+) final=([\w-]+) status=([\w-]+) ambient=([\d-]+) signal=([\d-]+) '
    r'gpio=([01]) valid=([01]) err=(\w+) dbg=(\w+) pad=(\w) rd=(\w+) raw=([\w-]+)')
counts, success, errors = Counter(), Counter(), Counter()
for match in pattern.finditer(args.log.read_text(encoding='utf-8-sig')):
    (side, seq, mm, model, rev, pre, final, status, ambient, signal,
     gpio, valid, err, debug, pads, mask, raw) = match.groups()
    counts[side] += 1
    mask = int(mask, 16)
    for field, bit in ((rev, 0), (model, 1), (pre, 2), (final, 3)):
        assert bool(mask & (1 << bit)) == (field != '--'), (side, seq, 'register read mask')
    if mask & 16:
        data = bytes.fromhex(raw)
        assert len(data) == 12, (side, seq, 'raw length')
        for field, offset in ((mm, 10), (ambient, 6), (signal, 8)):
            assert int(field) == int.from_bytes(data[offset:offset+2], 'big'), (side, seq, field)
        assert int(status, 16) == (data[0] >> 3) & 15, (side, seq, 'raw status')
    else:
        assert raw == '-' * 24 and all(set(x) == {'-'} for x in (mm, status, ambient, signal)), (side, seq, 'unread values')
    if valid == '1':
        assert mask == 31 and err == '00' and model == 'EE' and pads == '7' and debug == '0000', (side, seq, 'success consistency')
        success[side] += 1
    else:
        assert err != '00', (side, seq, 'invalid without error')
        errors[(side, err)] += 1
assert counts, 'No complete diagnostic lines found'
for side in ('FRONT', 'LEFT', 'RIGHT'):
    print(f'{side}: observed={counts[side]} communication_ok={success[side]} errors=' +
          ','.join(f'{code}:{count}' for (channel, code), count in sorted(errors.items()) if channel == side))
print('PASS raw-byte parsing and unread-value integrity for captured lines; optical accuracy unverified')
if args.require_all:
    assert all(counts[side] > 0 and success[side] == counts[side] for side in ('FRONT', 'LEFT', 'RIGHT')), 'Three-channel communication is not fully working'
    print('PASS all three channels communicate throughout this capture')
