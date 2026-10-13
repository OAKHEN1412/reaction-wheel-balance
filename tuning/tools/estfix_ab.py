"""Interleaved A/B analysis for the wheel-estimator zero-crossing fix.

    py -X utf8 tuning/tools/estfix_ab.py <bridge log section>

The bridge log must contain the `estfix=N` acks; every telemetry row after an ack
is attributed to that mode. (The A/B build had a temporary `estfix 0|1` serial
command; the committed firmware keeps only WheelSpeedEstimator::crossingFix.) For each mode the script reports balancing time,
falls, the outcome of each push (an excursion > 2.5 deg from the steady median),
how many lock-in episodes occurred, and push strength.

A lock-in episode is >= 0.2 s where the command and the estimate point opposite
ways while |estimate| grows.

Result on 2026-10-13: fix ON gave 0 falls over 34 pushes; fix OFF gave 5 falls
over 26 pushes.
"""
import sys, re, statistics as st
mode = None; rows = []
for ln in open(sys.argv[1], encoding='utf-8', errors='ignore'):
    ln = ln.strip()
    m = re.match(r'^estfix=(\d)$', ln)
    if m: mode = int(m.group(1)); continue
    p = ln.split(',')
    if len(p) == 7 and p[0].isdigit():
        try: rows.append((int(p[0]), p[1], float(p[2]), float(p[3]), float(p[4]), float(p[5]), float(p[6]), mode))
        except ValueError: pass
# balancing time and falls per mode
bal = {0: 0.0, 1: 0.0}; falls = {0: 0, 1: 0}; early = {0: 0, 1: 0}
prev = None; seg_start = None
for r in rows:
    if prev and prev[1] == 'BALANCING' and r[1] == 'BALANCING' and r[7] is not None:
        bal[r[7]] += (r[0] - prev[0]) / 1000
    if r[1] == 'BALANCING' and (prev is None or prev[1] != 'BALANCING'): seg_start = r[0]
    if prev and prev[1] == 'BALANCING' and r[1] == 'FALLEN' and r[7] is not None:
        falls[r[7]] += 1
        if r[0] - seg_start < 2000: early[r[7]] += 1
    prev = r
# lock-in signature: runs of >=10 samples (~0.2 s) with cmd and estimate opposite, |est|>20 rpm and |est| growing
sig = {0: 0, 1: 0}; run = []
def close(run):
    if len(run) >= 10 and abs(run[-1][4]) > abs(run[0][4]) + 30 and run[0][7] is not None: sig[run[0][7]] += 1
for r in rows:
    bad = r[1] == 'BALANCING' and abs(r[4]) > 20 and abs(r[5]) > 50 and (r[4] > 0) != (r[5] > 0)
    if bad and (not run or run[-1][7] == r[7]): run.append(r)
    else: close(run); run = [r] if bad else []
close(run)
# pushes per mode (excursions > 2.5 deg from the steady median), outcome
base = st.median([r[2] for r in rows if r[1] == 'BALANCING' and abs(r[3]) < 5 and abs(r[2]) < 3])
pushes = {0: [0, 0], 1: [0, 0]}   # [recovered, fell]
i = 0
while i < len(rows):
    r = rows[i]
    if r[1] != 'BALANCING' or abs(r[2] - base) <= 2.5 or r[7] is None: i += 1; continue
    md = r[7]; j = i; outcome = None
    while j < len(rows):
        if rows[j][1] != 'BALANCING': outcome = 1; break
        if abs(rows[j][2] - base) <= 1.0: outcome = 0; break
        j += 1
    if outcome is not None: pushes[md][outcome] += 1
    i = j + 1
print(f'baseline {base:+.2f} deg')
for md in (1, 0):
    name = 'FIX ON ' if md else 'FIX OFF'
    tot = sum(pushes[md])
    print(f'{name}: balancing {bal[md]:5.1f} s, falls {falls[md]} (within 2 s of start: {early[md]}), '
          f'falls/min {falls[md] / max(bal[md], 1e-9) * 60:4.1f}, pushes {tot} recovered {pushes[md][0]} fell {pushes[md][1]}, '
          f'lock-in episodes {sig[md]}')
# push strength per mode: peak |deviation| reached and peak |rate| within the first 0.3 s of each excursion
pk = {0: [], 1: []}
i = 0
while i < len(rows):
    r = rows[i]
    if r[1] != 'BALANCING' or abs(r[2] - base) <= 2.5 or r[7] is None: i += 1; continue
    md = r[7]; win = [x for x in rows[i:i+15] if x[1] == 'BALANCING']
    pk[md].append(max(abs(x[3]) for x in win))
    j = i
    while j < len(rows) and rows[j][1] == 'BALANCING' and abs(rows[j][2] - base) > 1.0: j += 1
    i = j + 1
for md in (1, 0):
    v = sorted(pk[md])
    print(('FIX ON ' if md else 'FIX OFF') + f': push peak rate median {v[len(v)//2]:.1f} dps, upper quartile {v[3*len(v)//4]:.1f}, n={len(v)}')
