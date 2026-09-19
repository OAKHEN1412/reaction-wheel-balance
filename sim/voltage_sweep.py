"""Reproducible 5-degree voltage-mode comparison; run with py -X utf8 voltage_sweep.py."""
import json
from pathlib import Path
import numpy as np
import params
import model
import design_gains as dg


def batch(p, gains, inertia=False):
    results = []
    duty = []
    rng = np.random.default_rng(20260919)
    for seed in range(20):
        sample = dict(p)
        if inertia:
            for key in ('m_w', 'k_w'):
                sample[key] = rng.uniform(*params.RANGES[key])
        h = model.simulate(params.derive(sample), gains, theta0=np.deg2rad(5), T=3.5, seed=seed)
        results.append(dg.evaluate_run(h))
        duty.append(float(np.mean(h['saturated'])))
    return dict(success=sum(int(r['success']) for r in results),
                saturated=sum(int(r['saturated']) for r in results),
                saturation_pct=100*np.mean(duty))


def main():
    nominal = params.nominal_derived()
    candidates = {'lqr': dg.lqr_gains_dict(nominal, **dg.DEFAULT_Q),
                  'soft': dg.pole_placement_gains(nominal, wn=4, Kw=1, Ki=0.0387)}
    rows = []
    for name, gains in candidates.items():
        for tau in (0.12, 0.18, 0.25):
            for coast in (1.0, 0.3):
                p = dict(params.NOMINAL, tau_m=tau, coast_decel_frac=coast)
                row = dict(gains=name, tau_m=tau, coast=coast,
                           nominal=batch(p, gains), inertia=batch(p, gains, True))
                rows.append(row)
                print(json.dumps(row), flush=True)
    old = batch(dict(params.NOMINAL, control_mode='speed'), candidates['lqr'])
    damping = []
    for factor in (1.2, 1.5):
        gains = dict(candidates['lqr'], Kd=candidates['lqr']['Kd']*factor)
        p = dict(params.NOMINAL, tau_m=0.25, coast_decel_frac=0.3)
        n, w = batch(p, gains), batch(p, gains, True)
        damping.append(dict(kd_factor=factor, nominal_success=n['success'],
                            inertia_success=w['success'], nominal_saturated=n['saturated'],
                            inertia_saturated=w['saturated']))
        print(json.dumps(damping[-1]), flush=True)
    report = dict(gains=candidates, rows=rows, old_speed=old,
                  damping_trials_worst_case=damping)
    Path(__file__).with_name('voltage_results.json').write_text(json.dumps(report, indent=2), encoding='utf8')
    print('old speed:', old, flush=True)

if __name__ == '__main__':
    main()
