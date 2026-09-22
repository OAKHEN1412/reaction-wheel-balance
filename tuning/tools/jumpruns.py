"""Summarise EVERY jump in a telemetry log, one line per launch.

jumpstat.py reports a single launch, which meant re-running it with a line
offset for each attempt in a tuning session. This walks the whole file.

    py -X utf8 tuning/tools/jumpruns.py <log> [--side +|-] [--csv]

The column that has actually predicted the outcome on this hardware is
`sat%` -- the share of the first second after handover in which duty is
pinned at the rail. Successful catches ran at 0%; every failure sat at
21-44%. Angle, rate and wheel speed at handover overlap heavily between
the two outcomes, so treat them as description, not as a target.
"""
import sys

HELD_MS = 3000      # balancing this long after capture counts as a success
SAT_WINDOW_MS = 1000  # window after capture used for the duty-saturation share


def parse(path):
    rows, msgs = [], {}
    for ln in open(path, encoding="utf-8", errors="replace"):
        ln = ln.strip()
        p = ln.split(",")
        if len(p) >= 7 and p[0].isdigit():
            try:
                rows.append((int(p[0]), p[1], *map(float, p[2:7])))
                continue
            except ValueError:
                pass
        if ln.startswith("jump"):
            msgs[len(rows)] = ln
    return rows, msgs


def runs(rows):
    """Split into (start_index, jump_rows, aftermath_rows) per JUMP_UP episode.

    The aftermath stops at the next launch. Letting it run to the end of the
    file makes every run report the same overshoot -- the largest one in the
    whole session -- which is how this was found.
    """
    out, cur, start = [], None, 0
    for i, r in enumerate(rows):
        if r[1] == "JUMP_UP":
            if cur is None:
                cur, start = [], i
            cur.append(r)
        elif cur is not None:
            out.append([start, cur, i])
            cur = None
    if cur is not None:
        out.append([start, cur, len(rows)])
    # Each aftermath ends where the next jump begins.
    return [(s, j, rows[end:out[k + 1][0] if k + 1 < len(out) else len(rows)])
            for k, (s, j, end) in enumerate(out)]


def summarise(idx, jrows, after, msgs):
    t0, th0 = jrows[0][0], jrows[0][2]
    side = 1 if th0 > 0 else -1
    peak = max(abs(r[3]) for r in jrows)
    # First BALANCING row after the jump = handover.
    cap = next((r for r in after if r[1] == "BALANCING"), None)
    note = next((m for k, m in msgs.items() if k >= idx and m.startswith("jump: abort")), "")

    rec = dict(n=None, side="+" if side > 0 else "-", rest=th0, spin=None,
               peak=peak, cap_deg=None, cap_dps=None, cap_rpm=None,
               over=None, sat=None, held=None, result="no-capture", note=note)

    # Spin-up ends at the first full-duty row pushing in the kick direction.
    kick = next((r for r in jrows if r[6] >= 0.99 and r[5] * side > 0), None)
    if kick:
        rec["spin"] = kick[0] - t0

    if cap is None:
        rec["note"] = rec["note"] or f"min|theta|={min(abs(r[2]) for r in jrows):.1f}"
        return rec

    bal = [r for r in after if r[0] >= cap[0] and r[1] == "BALANCING"]
    fell = next((r for r in after if r[0] > cap[0] and r[1] == "FALLEN"), None)
    win = [r for r in bal if r[0] - cap[0] <= SAT_WINDOW_MS]
    held_ms = (fell[0] if fell else bal[-1][0]) - cap[0]

    rec.update(
        cap_deg=cap[2], cap_dps=cap[3], cap_rpm=cap[4],
        over=max((-side * r[2] for r in bal), default=0.0),
        sat=(sum(1 for r in win if abs(r[6]) >= 0.99) / len(win)) if win else None,
        held=held_ms,
        # A catch that stood for HELD_MS is a success even if it fell later:
        # what the jump has to deliver is a frame the balance loop can take
        # over. Runs that stood for 28-57 s before drifting over are the
        # successes, not failures.
        result=("held" if held_ms >= HELD_MS else "FELL"),
    )
    return rec


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    flags = {a for a in sys.argv[1:] if a.startswith("--")}
    want_side = next((a.split("=")[1] for a in flags if a.startswith("--side=")), None)

    rows, msgs = parse(args[0])
    recs = []
    for i, (idx, jrows, after) in enumerate(runs(rows), 1):
        r = summarise(idx, jrows, after, msgs)
        r["n"] = i
        if want_side and r["side"] != want_side:
            continue
        recs.append(r)

    def f(v, spec, dash="-"):
        return dash if v is None else format(v, spec)

    if "--csv" in flags:
        print("n,side,rest_deg,spinup_ms,peak_dps,cap_deg,cap_dps,cap_rpm,overshoot_deg,sat_pct,held_ms,result,note")
        for r in recs:
            print(f"{r['n']},{r['side']},{f(r['rest'],'.1f','')},{f(r['spin'],'d','')},"
                  f"{f(r['peak'],'.0f','')},{f(r['cap_deg'],'.1f','')},{f(r['cap_dps'],'.0f','')},"
                  f"{f(r['cap_rpm'],'.0f','')},{f(r['over'],'.1f','')},"
                  f"{'' if r['sat'] is None else round(r['sat']*100)},{f(r['held'],'d','')},{r['result']},"
                  f"\"{r['note']}\"")
    else:
        print(f"{'#':>3} {'sd':>2} {'rest':>6} {'spin':>5} {'peak':>5} "
              f"{'cap':>6} {'rate':>5} {'whl':>5} {'over':>5} {'sat':>4} {'held':>6}  result")
        for r in recs:
            print(f"{r['n']:>3} {r['side']:>2} {f(r['rest'],'6.1f')} {f(r['spin'],'5d')} "
                  f"{f(r['peak'],'5.0f')} {f(r['cap_deg'],'6.1f')} {f(r['cap_dps'],'5.0f')} "
                  f"{f(r['cap_rpm'],'5.0f')} {f(r['over'],'5.1f')} "
                  f"{'   -' if r['sat'] is None else format(round(r['sat']*100),'3d')+'%'} "
                  f"{f(r['held'],'6d')}  {r['result']}"
              + (f"  {r['note']}" if r["note"] else ""))

    held = [r for r in recs if r["result"] == "held"]
    if recs:
        print(f"\n{len(held)}/{len(recs)} held >= {HELD_MS/1000:.0f}s", end="")
        sats = [r["sat"] for r in recs if r["sat"] is not None]
        if sats:
            hs = [r["sat"] for r in held if r["sat"] is not None]
            fs = [r["sat"] for r in recs if r["sat"] is not None and r["result"] != "held"]
            print(f"  |  duty saturation: held {_avg(hs)}  not-held {_avg(fs)}")
        else:
            print()


def _avg(xs):
    return "-" if not xs else f"{sum(xs)/len(xs)*100:.0f}% (n={len(xs)})"


if __name__ == "__main__":
    main()
