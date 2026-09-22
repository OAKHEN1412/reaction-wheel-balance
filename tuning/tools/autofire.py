"""Fire a batch of jump attempts automatically, one side at a time.

A tuning batch is the same three steps over and over: wait for the frame to
settle on its stop, fire, watch what happened. The frame usually falls back
onto the side it started from, so a batch needs no hands at all -- and when a
catch succeeds this stops the machine so the frame lies down again by itself.

Runs against tuning/tools/bridge.ps1: it reads the bridge log and writes the
bridge command file.

    py -X utf8 tuning/tools/autofire.py --log tune.log --cmd cmd.txt \
        --side - --runs 6 --params "550 450 8 75"

--side picks which stop to launch from (+ or -); attempts from the other side
are skipped and waited out, so a batch stays on one side even if the frame
ends up on the wrong one. SETTLE_S of stillness is required before firing,
which is longer than the firmware's own 500 ms gate, to leave time to let go.
"""
import argparse
import sys
import time

SETTLE_S = 2.0        # stillness required before firing (firmware asks 0.5)
REST_DEG = (10.0, 22.0)
STILL_DPS = 3.0
STILL_RPM = 10.0
HOLD_S = 5.0          # a catch standing this long is a success; then stop it
OUTCOME_TIMEOUT_S = 20.0


def tail_rows(fh, msgs=None):
    """Yield parsed telemetry rows as they appear; None for non-CSV lines.

    Board messages are collected into `msgs` when given: an attempt that the
    firmware aborts (the FG glitch at spin-up trips the overspeed guard often
    enough to matter) never launched, so it must not be counted as a failure.
    """
    for ln in fh:
        p = ln.strip().split(",")
        if len(p) >= 7 and p[0].isdigit():
            try:
                yield (int(p[0]), p[1], *map(float, p[2:7]))
                continue
            except ValueError:
                pass
        if msgs is not None and ln.startswith("jump"):
            msgs.append(ln.strip())
        yield None


def echo_mismatch(params, ack):
    """Compare the parameters sent with the ones the board echoed in its ack.

    Returns '' when they agree (or the firmware predates the echo), otherwise a
    short description of the first disagreement.
    """
    import re
    got = dict(re.findall(r"(rpm|kick|cap|rate|hold)=([-\d.]+)", ack))
    if not got:
        return ""  # old firmware without the echo; nothing to check
    names = ["rpm", "kick", "cap", "rate", "hold"]
    sent = params.split()
    for name, val in zip(names, sent):
        if name in got and abs(float(got[name]) - float(val)) > 0.051:
            return f"{name} sent {val} got {got[name]}"
    return ""


def send(cmd_path, text):
    with open(cmd_path, "w", encoding="ascii") as fh:
        fh.write(text + "\n")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--log", required=True)
    ap.add_argument("--cmd", required=True)
    ap.add_argument("--side", choices=["+", "-"], required=True)
    ap.add_argument("--runs", type=int, default=6)
    ap.add_argument("--params", default="550 450 8 75",
                    help="spin_rpm kick_ms capture_deg handover_rate_dps [coast_hold]")
    ap.add_argument("--params-b", default=None,
                    help="second parameter set; launches alternate A,B,A,B... "
                         "Interleaving matters: this machine has drifted within a "
                         "batch before (handover rate fell with elapsed time, "
                         "r=-0.79), and back-to-back blocks charge that drift "
                         "entirely to whichever set ran second.")
    ap.add_argument("--max-minutes", type=float, default=15.0)
    args = ap.parse_args()

    want = 1 if args.side == "+" else -1
    deadline = time.time() + args.max_minutes * 60
    fh = open(args.log, encoding="utf-8", errors="replace")
    fh.seek(0, 2)  # only watch what happens from now on

    fired = 0
    still_since = None
    sets = [args.params] + ([args.params_b] if args.params_b else [])
    if len(sets) > 1:
        print(f"autofire: {args.runs} runs on side {args.side}, alternating", flush=True)
        for k, s in enumerate(sets):
            print(f"  {chr(65+k)}: {s}", flush=True)
    else:
        print(f"autofire: {args.runs} runs on side {args.side}, params '{args.params}'", flush=True)

    while fired < args.runs and time.time() < deadline:
        # --- wait for a settled frame on the side we are tuning -------------
        row = None
        for r in tail_rows(fh):
            if r is not None:
                row = r
        if row is None:
            time.sleep(0.1)
            continue

        _, state, theta, rate, wheel, _, _ = row
        settled = (state in ("WAIT_UPRIGHT", "FALLEN")
                   and (1 if theta > 0 else -1) == want
                   and REST_DEG[0] <= abs(theta) <= REST_DEG[1]
                   and abs(rate) <= STILL_DPS and abs(wheel) <= STILL_RPM)
        if not settled:
            still_since = None
            time.sleep(0.1)
            continue
        if still_since is None:
            still_since = time.time()
            print(f"  settled at {theta:.1f} deg, firing in {SETTLE_S:.0f}s...", flush=True)
        if time.time() - still_since < SETTLE_S:
            time.sleep(0.1)
            continue

        # --- fire -----------------------------------------------------------
        fired += 1
        still_since = None
        which = (fired - 1) % len(sets)
        params = sets[which]
        tag = f"{chr(65+which)} " if len(sets) > 1 else ""
        print(f"[{fired}/{args.runs}] {tag}jump {params}  (from {theta:.1f} deg)", flush=True)
        send(args.cmd, f"jump {params}")

        # --- watch for the outcome -------------------------------------------
        t_fire = time.time()
        balancing_since = None
        outcome = "timeout"
        msgs = []
        launched = False
        while time.time() - t_fire < OUTCOME_TIMEOUT_S:
            last = None
            for r in tail_rows(fh, msgs):
                if r is not None:
                    last = r
            # The board must acknowledge the launch. Without this check a frame
            # that is simply lying there reads as FALLEN and every refused
            # command is recorded as a failed launch -- three phantom "runs"
            # were logged that way on 2026-09-22 while the frame lay at 96 deg
            # and the board was refusing every command.
            if not launched:
                ack = next((m for m in msgs if m.startswith("jump: SPINUP")), None)
                if ack is not None:
                    bad = echo_mismatch(params, ack)
                    if bad:
                        # Launched under parameters other than the ones sent:
                        # a value garbled in transit but still within range.
                        fired -= 1
                        outcome = f"WRONG PARAMS ({bad}), stopping and retrying"
                        send(args.cmd, "stop")
                        time.sleep(1.5)
                        break
                    launched = True
                elif time.time() - t_fire > 2.0:
                    fired -= 1
                    outcome = ("NOT LAUNCHED, retrying -- "
                               + (msgs[-1] if msgs else "no reply from the board"))
                    break
            if not launched:
                # Nothing below may run until the board has said SPINUP. The
                # frame is lying in FALLEN before every launch, so evaluating
                # outcomes here recorded twelve refused commands as twelve
                # falls on 2026-09-23 -- the ack check above was being
                # bypassed, not failing.
                time.sleep(0.05)
                continue
            if any(m.startswith("jump: abort") for m in msgs):
                fired -= 1  # never got airborne; does not count against the batch
                outcome = f"ABORTED, retrying -- {[m for m in msgs if m.startswith('jump: abort')][-1]}"
                break
            if last is None:
                time.sleep(0.05)
                continue
            state = last[1]
            if state == "BALANCING":
                if balancing_since is None:
                    balancing_since = time.time()
                elif time.time() - balancing_since >= HOLD_S:
                    outcome = f"HELD {HOLD_S:.0f}s -> stopping"
                    send(args.cmd, "stop")
                    time.sleep(1.0)
                    break
            elif state == "FALLEN":
                if balancing_since:
                    outcome = f"fell after {time.time()-balancing_since:.1f}s"
                else:
                    outcome = "fell"
                break
            elif state == "WAIT_UPRIGHT" and balancing_since is None and time.time() - t_fire > 3:
                outcome = "aborted / never captured"
                break
            time.sleep(0.05)
        print(f"      -> {outcome}", flush=True)
        time.sleep(1.5)  # let the frame come to rest before looking again

    print(f"autofire: done, {fired} run(s) fired", flush=True)
    return 0 if fired == args.runs else 1


if __name__ == "__main__":
    sys.exit(main())
