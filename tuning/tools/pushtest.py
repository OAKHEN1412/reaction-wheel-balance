"""Hand-push test analysis for report table 4.12 (section 4.5.2).

    py -X utf8 tuning/tools/pushtest.py <bridge log or telemetry csv> <events.json>

Reads the telemetry CSV (ms,state,theta,theta_dot,wheel_rpm,cmd_rpm,duty) and
splits it into BALANCING segments. The reference angle is the median theta
over steady samples (|theta_dot| < 5 dps). A push is any excursion more than
THR = 2.5 deg from that reference.

For each push it reports the peak deviation, and the recovery time from
leaving the +-1 deg band until re-entering it and staying there for 200 ms.
A push that never re-enters the band before the segment ends is marked
fell=True. It also reports the max |wheel rpm| and max duty over that window.

The reference is the steady median, not 0 deg, because the frame balances at
about -1 deg with the wheel held near 110 rpm (see HANDOFF 2026-10-13).
"""
import sys, statistics as st, json
rows = []
for ln in open(sys.argv[1], encoding='utf-8', errors='ignore'):
    p = ln.strip().split(',')
    if len(p) != 7: continue
    try: rows.append((int(p[0]), p[1], float(p[2]), float(p[3]), float(p[4]), float(p[5]), float(p[6])))
    except ValueError: pass
segs, cur = [], []
for r in rows:
    if r[1] == 'BALANCING': cur.append(r)
    elif cur: segs.append(cur); cur = []
if cur: segs.append(cur)
bal = [r for s in segs for r in s]
BASE = st.median([r[2] for r in bal if abs(r[3]) < 5 and abs(r[2]) < 3])
THR, BAND, HOLD = 2.5, 1.0, 200
print(f'baseline (steady median) = {BASE:+.2f} deg')
events = []
for si, s in enumerate(segs, 1):
    if s[-1][0] - s[0][0] < 1000: continue
    i = 0
    while i < len(s):
        if abs(s[i][2] - BASE) <= THR: i += 1; continue
        j = i
        while j > 0 and abs(s[j-1][2] - BASE) > BAND: j -= 1
        k, rec = i, None
        while k < len(s):
            if abs(s[k][2] - BASE) <= BAND:
                m = k
                while m < len(s) and abs(s[m][2] - BASE) <= BAND and s[m][0] - s[k][0] < HOLD: m += 1
                if m < len(s) and s[m][0] - s[k][0] >= HOLD: rec = k; break
                k = max(m, k + 1)
            else: k += 1
        end = rec if rec is not None else len(s) - 1
        win = s[j:end + 1]
        peak = max(win, key=lambda r: abs(r[2] - BASE))
        events.append(dict(seg=si, t_push=s[j][0], t_seg=round((s[j][0] - s[0][0]) / 1000, 2),
                           dir='+' if s[i][2] > BASE else '-', peak_dev=round(peak[2] - BASE, 2),
                           rec_s=round((s[rec][0] - s[j][0]) / 1000, 2) if rec is not None else None,
                           wheel_max=round(max(abs(r[4]) for r in win), 1), duty_max=round(max(r[6] for r in win), 3),
                           fell=rec is None))
        i = end + 1
for e in events: print(e)
json.dump({'base': BASE, 'events': events}, open(sys.argv[2], 'w'))
