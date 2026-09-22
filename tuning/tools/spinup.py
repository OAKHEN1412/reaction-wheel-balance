"""Spin-up strength per launch: how fast the wheel comes up under full duty.

The spin-up is the one part of a launch that is open-loop and identical every
time -- full duty from a stopped wheel -- so the wheel speed it reaches by a
fixed time is a direct read of supply voltage and motor health. Use it to tell
"the machine got weaker" from "the parameters got worse": if rpm@300ms drops
across a session, no parameter comparison from that session can be trusted.

    py -X utf8 tuning/tools/spinup.py <log> [<log> ...]
"""
import statistics as st
import sys
import os

sys.path.insert(0, os.path.dirname(__file__))
from jumpruns import parse, runs  # noqa: E402

SAMPLE_MS = 300


def spinup_stats(path):
    rows, _ = parse(path)
    out = []
    for i, (s, j, after) in enumerate(runs(rows), 1):
        if len(j) < 10:
            continue
        side = 1 if j[0][2] > 0 else -1
        kick = next((k for k, r in enumerate(j) if r[6] >= 0.99 and r[5] * side > 0), None)
        if kick is None or kick < 3:
            continue
        spin = j[:kick]
        t0 = spin[0][0]
        at_t = next((abs(r[4]) for r in spin if r[0] - t0 >= SAMPLE_MS), None)
        peak = max(abs(r[4]) for r in spin)
        out.append((i, peak, at_t, j[kick][0] - t0))
    return out


def main():
    for path in sys.argv[1:]:
        d = spinup_stats(path)
        if not d:
            print(f"{os.path.basename(path)}: no launches")
            continue
        peaks = [x[1] for x in d]
        at_t = [x[2] for x in d if x[2] is not None]
        spins = [x[3] for x in d]
        print(f"{os.path.basename(path):24s} n={len(d):2d}  "
              f"peak FG {st.mean(peaks):5.0f} rpm (SD {st.pstdev(peaks):4.1f})   "
              f"rpm@{SAMPLE_MS}ms {st.mean(at_t):5.0f} (SD {st.pstdev(at_t):4.1f})   "
              f"spin-up {st.mean(spins):4.0f} ms")
        if "-v" in sys.argv:
            for i, peak, a, sp in d:
                print(f"   run {i:2d}: peak {peak:4.0f}  @{SAMPLE_MS}ms {a if a is None else round(a):>4}  spin {sp} ms")


if __name__ == "__main__":
    main()
