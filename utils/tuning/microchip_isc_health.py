# SPDX-License-Identifier: GPL-2.0-or-later
#
# Copyright (C) 2026, Microchip Technology Inc.
#
# microchip_isc_health.py - end-of-run calibration health summary.
#
# Scores the generated tuning sections (keyed by each module's out_name) and
# prints a per-stage PASS/WARN/FAIL report with an overall 0-100 score, so the
# operator sees calibration quality at a glance.
#
# The score covers only the stages this tool emits: black level, AWB reference
# gains, CCM matrices, lux reference, and the presence of the authored Cbc/Gamma
# sections. Colour-temperature accuracy is expressed through the AWB
# ct_color_matrix (Robertson estimation), so there is no separate correction
# curve to score. Post-capture image white balance requires a live frame and is
# validated on the board rather than here.
#
# The summary is advisory: it never fails the run, it indicates which stage to
# improve next.

import logging

logger = logging.getLogger(__name__)

# Severity ordering for the overall verdict.
PASS = 'PASS'
WARN = 'WARN'
FAIL = 'FAIL'
_RANK = {PASS: 0, WARN: 1, FAIL: 2}


def _worst(a, b):
    return a if _RANK[a] >= _RANK[b] else b


def health_summary(sections: dict, measured=None):
    """
    Score the generated tuning sections and build a human-readable report.

    Args:
        sections: {out_name: section_dict} for the sections the tuner produced,
                  e.g. {'BlackLevelCorrection': {...}, 'Awb': {...}, ...}.
                  Missing sections score as not-calibrated rather than raising.
        measured: set of out_names that were calibrated from real captures this
                  run. Stages NOT in this set are authored/fallback values: they
                  are capped at WARN and scored at half so a no-capture
                  regeneration cannot read as fully calibrated. Pass None to
                  treat every present section as measured (structural check only).

    Returns:
        (score:int 0-100, report:list[str], verdict:str) where verdict is the
        worst per-stage severity (PASS/WARN/FAIL).
    """
    score = 0
    report = []
    verdict = PASS

    def add(pts, max_pts, severity, label, detail='', stage=None):
        nonlocal score, verdict
        # A stage that fell back to authored config values was not measured this
        # run: never let it read as a full PASS, and halve its contribution.
        if measured is not None and stage is not None and stage not in measured:
            if severity == PASS:
                severity = WARN
            pts = pts // 2
            detail = (detail + '  [authored fallback, not measured]').strip()
        score += pts
        verdict = _worst(verdict, severity)
        icon = {PASS: '[ OK ]', WARN: '[WARN]', FAIL: '[FAIL]'}[severity]
        report.append(f'  {icon}  {label:<26} {pts:2d}/{max_pts} pts  {detail}')

    blc = sections.get('BlackLevelCorrection') or {}
    awb = sections.get('Awb') or {}
    agc = sections.get('Agc') or {}
    ccm = sections.get('Ccm') or {}
    cbc = sections.get('Cbc') or {}
    gamma = sections.get('Gamma') or {}

    # -- BLC (20 pts): datasheet pedestal + per-channel consistency ------------
    bl = float(blc.get('black_level', 0))
    ch = [float(blc.get(k, bl)) for k in
          ('black_level_r', 'black_level_gr', 'black_level_gb', 'black_level_b')]
    spread = max(ch) - min(ch) if ch else 0.0
    if 55 <= bl <= 75:
        if spread <= 5:
            add(20, 20, PASS, 'BLC black_level',
                f'{bl:.0f} DN, spread={spread:.0f} DN')
        elif spread <= 15:
            add(14, 20, WARN, 'BLC black_level',
                f'{bl:.0f} DN, per-channel spread={spread:.0f} DN')
        else:
            add(7, 20, WARN, 'BLC black_level',
                f'{bl:.0f} DN, large spread={spread:.0f} DN (redo dark frame)')
    elif 40 <= bl <= 90:
        add(10, 20, WARN, 'BLC black_level',
            f'{bl:.0f} DN outside expected 55-75 (datasheet=64)')
    else:
        add(0, 20, FAIL, 'BLC black_level',
            f'{bl:.0f} DN wrong (IMX219 datasheet=64)')

    # -- AWB (30 pts): reference_gains_table coverage, trend, plausibility -----
    rgt = list(awb.get('reference_gains_table', []))
    n = len(rgt)
    if n >= 2:
        cts = [float(e.get('ct', 0)) for e in rgt]
        r_gains = [float(e.get('r', 0)) for e in rgt]
        b_gains = [float(e.get('b', 0)) for e in rgt]
        order = sorted(range(n), key=lambda i: cts[i])
        r_sorted = [r_gains[i] for i in order]
        b_sorted = [b_gains[i] for i in order]
        # Physical trend: rGain falls, bGain rises as CT climbs.
        r_inv = sum(1 for i in range(n - 1) if r_sorted[i + 1] > r_sorted[i] + 1e-6)
        b_inv = sum(1 for i in range(n - 1) if b_sorted[i + 1] < b_sorted[i] - 1e-6)
        max_b = max(b_gains) if b_gains else 0.0
        min_ct = min(cts) if cts else 9999
        likely_noir = max_b > 2.5 and min_ct < 3500

        mono_ok = (r_inv == 0 and b_inv == 0)
        b_ok = (max_b <= 4.0) if likely_noir else (max_b <= 2.5)

        if n >= 3 and mono_ok and b_ok:
            note = ' (NoIR: high bGain at warm CT expected)' if likely_noir else ''
            add(30, 30, PASS, f'AWB gains ({n} CT pts)',
                f'monotone, max bGain={max_b:.2f}{note}', stage='Awb')
        elif n >= 3 and b_ok:
            add(22, 30, WARN, f'AWB gains ({n} CT pts)',
                f'non-monotone trend (rGain inv={r_inv}, bGain inv={b_inv})',
                stage='Awb')
        elif n >= 3:
            add(16, 30, WARN, f'AWB gains ({n} CT pts)',
                f'max bGain={max_b:.2f} high (check BLC / NIR)', stage='Awb')
        else:
            add(18, 30, WARN, f'AWB gains ({n} CT pts)',
                'only 2 CT points (add a 3rd for range coverage)', stage='Awb')
    elif n == 1:
        add(6, 30, WARN, 'AWB gains (1 CT pt)',
            'single illuminant: off-CT rows are extrapolated', stage='Awb')
    else:
        add(0, 30, FAIL, 'AWB gains', 'NOT CALIBRATED', stage='Awb')

    # -- AWB inter-frame repeatability (advisory, no score weight) -------------
    # _awb_variance is the per-CT spread of the individual frames' gains (the
    # AWB module attaches it when a CT has >1 frame; stripped from the emitted
    # YAML). Like the Macbeth QA gate it carries no points -- it grades the
    # captures, not the output -- but a noisy operating point is folded into the
    # verdict so the operator resettles the sensor before trusting the gains.
    awb_var = awb.get('_awb_variance') or []
    multi = [v for v in awb_var if float(v.get('n', 0)) > 1]
    if multi:
        worst = max(max(float(v.get('r_rel_std', 0)),
                        float(v.get('b_rel_std', 0))) for v in multi)
        noisy = [v for v in multi
                 if max(float(v.get('r_rel_std', 0)),
                        float(v.get('b_rel_std', 0))) > 0.02]
        sev = WARN if noisy else PASS
        icon = {PASS: '[ OK ]', WARN: '[WARN]', FAIL: '[FAIL]'}[sev]
        detail = (f'worst rel std {worst * 100:.1f}% over {len(multi)} CT(s)'
                  if not noisy
                  else f'{len(noisy)}/{len(multi)} CT(s) >2% inter-frame std '
                       f'(worst {worst * 100:.1f}%)')
        report.append(f'  {icon}  {"AWB repeatability":<26} (advisory)  {detail}')
        verdict = _worst(verdict, sev)

    # -- CCM structure (15 pts): matrices present, row sums ~1.0, not identity -
    mats = ccm.get('matrices', {}) or {}
    nm = len(mats)
    if nm >= 1:
        identity = [1, 0, 0, 0, 1, 0, 0, 0, 1]
        is_identity = True
        rows_ok = True
        for entry in mats.values():
            flat = list(entry.get('ccm', [])) if isinstance(entry, dict) else []
            if len(flat) != 9:
                rows_ok = False
                continue
            flat = [float(v) for v in flat]
            if any(abs(flat[i] - identity[i]) > 0.01 for i in range(9)):
                is_identity = False
            rs = [flat[0] + flat[1] + flat[2],
                  flat[3] + flat[4] + flat[5],
                  flat[6] + flat[7] + flat[8]]
            if any(abs(s - 1.0) >= 0.20 for s in rs):
                rows_ok = False
        if is_identity:
            add(4, 15, WARN, f'CCM matrices ({nm})',
                'all identity (run ccm module with a Macbeth chart)', stage='Ccm')
        elif nm >= 3 and rows_ok:
            add(15, 15, PASS, f'CCM matrices ({nm})', 'row sums ~1.0', stage='Ccm')
        elif rows_ok:
            add(10, 15, WARN, f'CCM matrices ({nm})',
                'need >=3 CT buckets for full-range interpolation', stage='Ccm')
        else:
            add(6, 15, WARN, f'CCM matrices ({nm})',
                'row sums deviate >20% (recalibrate)', stage='Ccm')
    else:
        add(0, 15, FAIL, 'CCM matrices', 'NOT CALIBRATED', stage='Ccm')

    # -- CCM colour accuracy (10 pts): measured CIEDE2000 of the matrices ------
    # _delta_e_avg/_max are diagnostics the CCM module attaches when it solves
    # from a Macbeth chart (stripped from the emitted YAML). Absent means the
    # section fell back to authored values or no chart was present.
    de_avg = ccm.get('_delta_e_avg')
    de_max = ccm.get('_delta_e_max')
    if de_avg is not None:
        de_avg = float(de_avg)
        de_max = float(de_max) if de_max is not None else de_avg
        det = f'avg ΔE00={de_avg:.2f}, max={de_max:.2f}'
        if de_avg <= 2.0 and de_max <= 5.0:
            add(10, 10, PASS, 'CCM colour accuracy', det, stage='Ccm')
        elif de_avg <= 3.0:
            add(8, 10, PASS, 'CCM colour accuracy', det, stage='Ccm')
        elif de_avg <= 4.0:
            add(6, 10, WARN, 'CCM colour accuracy', f'{det} (fair)', stage='Ccm')
        elif de_avg <= 6.0:
            add(3, 10, WARN, 'CCM colour accuracy',
                f'{det} (high — recalibrate)', stage='Ccm')
        else:
            add(1, 10, FAIL, 'CCM colour accuracy',
                f'{det} (poor colour match)', stage='Ccm')
    else:
        add(0, 10, WARN, 'CCM colour accuracy',
            'no ΔE00 (authored fallback or no Macbeth chart)', stage='Ccm')

    # -- Lux (15 pts): reference_Y calibrated ---------------------------------
    lr = agc.get('lux_reference', {}) or {}
    ref_y = float(lr.get('reference_Y', 0) or 0)
    if ref_y > 5:
        if ref_y < 460:
            add(15, 15, PASS, 'Lux reference_Y', f'{ref_y:.1f} bins', stage='Agc')
        else:
            add(8, 15, WARN, 'Lux reference_Y',
                f'{ref_y:.1f} near saturation (use a lower-lux capture)',
                stage='Agc')
    else:
        add(0, 15, WARN, 'Lux reference_Y',
            'uncalibrated (AE falls back to a fixed lux estimate)', stage='Agc')

    # -- Authored Cbc (4 pts): section present --------------------------------
    if cbc:
        add(4, 4, PASS, 'Authored Cbc', 'section present')
    else:
        add(0, 4, FAIL, 'Authored Cbc', 'missing (passthrough config key absent?)')

    # -- Gamma tone curve (6 pts): OECF fit quality ---------------------------
    # _oecf_rms is the RMS error between the reconstructed 64-entry tone LUT (at
    # the fitted exponent) and the measured neutral-ramp OECF; the Gamma module
    # attaches it only when it fit gamma from a located chart. Absent means the
    # section fell back to the authored config (no chart), which the 'Gamma'
    # gate then caps at WARN and halves.
    if not gamma:
        add(0, 6, FAIL, 'Gamma tone curve', 'section missing', stage='Gamma')
    else:
        rms = gamma.get('_oecf_rms')
        if rms is None:
            add(6, 6, PASS, 'Gamma tone curve',
                'authored exponent (no OECF fit)', stage='Gamma')
        else:
            rms = float(rms)
            g_fit = gamma.get('_gamma_fit')
            det = f'gamma={float(g_fit):.2f}, OECF RMS={rms:.3f}' if g_fit \
                else f'OECF RMS={rms:.3f}'
            if rms <= 0.03:
                add(6, 6, PASS, 'Gamma tone curve', det, stage='Gamma')
            elif rms <= 0.06:
                add(5, 6, PASS, 'Gamma tone curve', det, stage='Gamma')
            elif rms <= 0.10:
                add(3, 6, WARN, 'Gamma tone curve',
                    f'{det} (fit loose — check ramp exposure)', stage='Gamma')
            else:
                add(1, 6, WARN, 'Gamma tone curve',
                    f'{det} (poor OECF fit)', stage='Gamma')

    # -- Macbeth capture QA (advisory, no score weight) -----------------------
    # Input-quality gate: clipping, tilt or uneven light on a chart biases every
    # solve derived from it. Reported as PASS/WARN/FAIL and folded into the
    # overall verdict, but it carries no points so the 0-100 remains a score of
    # the calibration output, not of the captures fed to it. Absent when no
    # chart was located this run.
    qa = sections.get('MacbethQA') or {}
    qa_imgs = qa.get('images') or []
    if qa_imgs:
        worst_qa = qa.get('worst', PASS)
        icon = {PASS: '[ OK ]', WARN: '[WARN]', FAIL: '[FAIL]'}[worst_qa]
        bad = [v for v in qa_imgs if v.get('severity') != PASS]
        n = len(qa_imgs)
        detail = (f'{n} chart(s) clean' if not bad
                  else f'{len(bad)}/{n} chart(s) flagged')
        report.append(f'  {icon}  {"Macbeth capture QA":<26} (advisory)  {detail}')
        for v in bad:
            report.append(f'           - {v.get("name")}: '
                          f'{"; ".join(v.get("issues", []))}')
        verdict = _worst(verdict, worst_qa)

    # -- Lens-shading / green imbalance (advisory, no score weight) -----------
    # LSC is a still-path software corrector, not an IPA algorithm, so it carries
    # no points (like Macbeth QA). Report per-CT map coverage plus the measured
    # green imbalance (_geq: per-CT GR/GB deviation, stripped from the emitted
    # YAML), and fold a WARN into the verdict when a map hit the corner-gain cap
    # or the greens diverge enough to leave a visible maze. Absent when no
    # flat-field capture was supplied.
    lsc = sections.get('LensShadingCorrection') or {}
    if lsc:
        per_ct = lsc.get('per_ct')
        n_ct = len(per_ct) if per_ct else (1 if lsc.get('gains') else 0)
        geq = lsc.get('_geq') or []
        worst_geq = max((float(e.get('max_dev', 0)) for e in geq), default=0.0)
        # 3% per-sector GR/GB deviation is the practical threshold for a visible
        # green maze after demosaic.
        sev = WARN if worst_geq > 0.03 else PASS
        icon = {PASS: '[ OK ]', WARN: '[WARN]', FAIL: '[FAIL]'}[sev]
        cov = f'{n_ct} CT map(s)' if n_ct else 'no map'
        detail = (f'{cov}, worst green imbalance {worst_geq * 100:.1f}%'
                  if geq else f'{cov}')
        report.append(f'  {icon}  {"Lens shading / GEQ":<26} (advisory)  {detail}')
        if sev == WARN:
            report.append('           - green imbalance >3%: consider a '
                          'green-equalisation pass before demosaic')
        verdict = _worst(verdict, sev)

    # -- Noise profile (advisory, no score weight) ----------------------------
    # The still-path denoiser noise model (read variance, shot slope, gain
    # exponent) measured from a flat-field gain sweep. Like LSC it is a still-
    # path corrector, not an IPA algorithm, so it carries no points. Report the
    # measured constants and fold a WARN into the verdict when the variance-vs-
    # signal fit is loose or the gain exponent is poorly determined -- either
    # means the denoiser would be sized from a noisy profile. Absent when no
    # flat-field gain sweep was supplied (the runtime keeps its built-in model).
    noise = sections.get('NoiseProfile') or {}
    if noise:
        per_gain = noise.get('_per_gain') or []
        worst_r2 = min((float(e.get('r2', 0)) for e in per_gain), default=0.0)
        exp_r2 = noise.get('_gain_exp_r2')
        n_g = len(per_gain)
        # A loose per-gain line (<0.9) or, with a real sweep, a poor gain-scaling
        # fit (<0.8) means the emitted constants are only approximate.
        weak = worst_r2 < 0.9 or (n_g >= 2 and exp_r2 is not None
                                  and float(exp_r2) < 0.8)
        sev = WARN if weak else PASS
        icon = {PASS: '[ OK ]', WARN: '[WARN]', FAIL: '[FAIL]'}[sev]
        detail = (f'{n_g} gain(s), read_var={float(noise.get("read_noise_var", 0)):.2f} '
                  f'shot={float(noise.get("shot_slope", 0)):.3f} '
                  f'gainExp={float(noise.get("gain_exponent", 0)):.2f}, '
                  f'worst fit R2={worst_r2:.2f}')
        report.append(f'  {icon}  {"Noise profile":<26} (advisory)  {detail}')
        if n_g < 2:
            report.append('           - single gain: gain exponent not '
                          'measured (sweep >=2 gains)')
        verdict = _worst(verdict, sev)

    return score, report, verdict


def log_health_summary(sections: dict, measured=None):
    """Compute the health summary and emit it at INFO."""
    score, report, verdict = health_summary(sections, measured)
    logger.info('=' * 60)
    logger.info(f'Calibration health: {score}/100  ({verdict})')
    for line in report:
        logger.info(line)
    if measured is not None:
        stages = {'Awb', 'Agc', 'Ccm', 'Gamma'}
        fell_back = sorted(stages - set(measured))
        if fell_back:
            logger.info(f'  note: {", ".join(fell_back)} used authored config '
                        f'values (no captures this run)')
    logger.info('=' * 60)
    return score, report, verdict
