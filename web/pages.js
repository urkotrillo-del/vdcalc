// The three read-only result pages.
const FIG = {
  top_speed_kmh: { label: 'Top speed', unit: 'km/h', d: 0, better: 'high' },
  peak_power_kw: { label: 'Peak power', unit: 'kW', d: 1, better: 'high' },
  peak_torque_nm: { label: 'Peak torque', unit: 'N·m', d: 0, better: 'high' },
  time_0_60_mph: { label: '0–60 mph', unit: 's', d: 2, better: 'low' },
  time_0_100_kmh: { label: '0–100 km/h', unit: 's', d: 2, better: 'low' },
  time_0_200_kmh: { label: '0–200 km/h', unit: 's', d: 2, better: 'low' },
  time_0_top: { label: c => `0–${Math.round(c.run_top_speed_kmh)} km/h`, unit: 's', d: 2, better: 'low' },
  distance_top_0: { label: c => `${Math.round(c.run_top_speed_kmh)}–0 km/h`, unit: 'm', d: 1, better: 'low' },
  time_0_100_0: { label: '0–100–0 km/h, time', unit: 's', d: 2, better: 'low' },
  distance_0_100_0: { label: '0–100–0 km/h, distance', unit: 'm', d: 0, better: 'low' },
  time_0_200_0: { label: '0–200–0 km/h, time', unit: 's', d: 2, better: 'low' },
  distance_0_200_0: { label: '0–200–0 km/h, distance', unit: 'm', d: 0, better: 'low' },
  time_0_top_0: { label: c => `0–${Math.round(c.run_top_speed_kmh)}–0 km/h, time`, unit: 's', d: 2, better: 'low' },
  distance_0_top_0: { label: c => `0–${Math.round(c.run_top_speed_kmh)}–0 km/h, distance`, unit: 'm', d: 0, better: 'low' },
  distance_100_0_kmh: { label: '100–0 km/h', unit: 'm', d: 1, better: 'low' },
  distance_200_0_kmh: { label: '200–0 km/h', unit: 'm', d: 1, better: 'low' },
  lap_time_s: { label: 'Lap time', unit: '', d: 2, better: 'low', lap: true },
  lap_speed_min_kmh: { label: 'Minimum speed', unit: 'km/h', d: 0, better: 'high' },
  lap_speed_max_kmh: { label: 'Maximum speed', unit: 'km/h', d: 0, better: 'high' },
  lap_speed_avg_kmh: { label: 'Average speed', unit: 'km/h', d: 1, better: 'high' },
  lap_max_lateral_g: { label: 'Peak lateral acceleration', unit: 'g', d: 2, better: 'high' },
  lap_max_braking_g: { label: 'Peak braking deceleration', unit: 'g', d: 2, better: 'high' },
};

function fmtLap(s) { const m = Math.floor(s / 60); return `${m}:${(s - 60 * m).toFixed(2).padStart(5, '0')}`; }
function fmtVal(f, v) { return v === null || v === undefined ? '–' : (f.lap ? fmtLap(v) : v.toFixed(f.d)); }

function figuresTable(keys, cur, ref, pre = {}) {
  const withRef = !!ref;
  let h = '<table class="fig"><tr><th></th><th>Value</th>' + (withRef ? '<th>Reference</th><th>Difference</th><th>%</th>' : '') + '</tr>';
  for (const k of keys) {
    const f = FIG[k], v = cur[k];
    h += `<tr><td>${typeof f.label === 'function' ? f.label(cur) : f.label}</td><td>${pre[k] ? `<span class="at">${pre[k]}</span> ` : ''}<b>${fmtVal(f, v)}</b> <span class="u">${f.unit}</span></td>`;
    if (withRef) {
      const r = ref[k];
      if (r === null || r === undefined || v === null || v === undefined) h += '<td>–</td><td>–</td><td>–</td>';
      else {
        const d = v - r, pct = r !== 0 ? 100 * d / r : null;
        const small = Math.abs(d) < 0.5 * Math.pow(10, -f.d);
        const cls = small ? '' : ((d < 0) === (f.better === 'low') ? 'good' : 'bad');
        h += `<td>${fmtVal(f, r)} ${f.unit}</td><td class="${cls}">${d >= 0 ? '+' : '−'}${Math.abs(d).toFixed(f.d + (f.lap ? 0 : 0))} ${f.lap ? 's' : f.unit}</td><td class="${cls}">${pct === null ? '–' : (pct >= 0 ? '+' : '−') + Math.abs(pct).toFixed(1)}</td>`;
      }
    }
    h += '</tr>';
  }
  return h + '</table>';
}

// ---------------------------------------------------------------- drawing of the car
function drawingSvg(d) {
  // The picture is fixed: the dimension lines and the centre of gravity are always in the same place, only the numbers change.
  // The drawing (1024 x 816) is placed with a scale and an offset; X() and Y() turn its coordinates into those of the page.
  const S = 1.1, OX = 110, OY = 10, X = x => OX + S * x, Y = y => OY + S * y;
  const m = (v, n = 3) => v.toFixed(n) + ' m', kg = v => Math.round(v) + ' kg';
  const xf = X(283), xr = X(750), xg = X(502), gy = Y(247), cgY = Y(150), yc = Y(580), ty = 125 * S, yd = 350, yd2 = 415;
  const dimH = (x1, x2, y, label) => `<line class="dim" x1="${x1}" y1="${y}" x2="${x2}" y2="${y}" marker-start="url(#da)" marker-end="url(#da)"/><text class="dtx" x="${(x1 + x2) / 2}" y="${y - 10}" text-anchor="middle">${label}</text>`;
  const box = (x, y, v) => `<rect class="dbox" x="${x - 86}" y="${y - 38}" width="172" height="76" rx="22"/><text class="dkg" x="${x}" y="${y + 9}" text-anchor="middle">${kg(v)}</text>`;
  // Centre of gravity: the usual symbol, a circle with two opposite quarters filled.
  const cg = (x, y, r = 15) => `<circle class="cgc" cx="${x}" cy="${y}" r="${r}"/><path class="cgf" d="M${x},${y}V${y - r}A${r},${r} 0 0 1 ${x + r},${y}ZM${x},${y}V${y + r}A${r},${r} 0 0 1 ${x - r},${y}Z"/><path class="cgx" d="M${x - r - 8},${y}H${x + r + 8}M${x},${y - r - 8}V${y + r + 8}"/>`;
  const fw = d.front_axle_mass_kg / 2, rw = d.rear_axle_mass_kg / 2;
  const XL = 160, XR = 1190, rx = xf - 42 * S, ry = Y(190) + 42 * S;
  return `<svg viewBox="100 0 1140 980" width="100%" role="img" aria-label="Side and top view of the car with its dimensions and static wheel loads" style="max-width:900px;display:block;margin:0 auto">
    <defs><marker id="da" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="7" markerHeight="7" orient="auto-start-reverse"><path d="M0,1 L10,5 L0,9 z" class="mk"/></marker></defs>
    <image href="car_drawing.png" x="${OX}" y="${OY}" width="${1024 * S}" height="${816 * S}"/>
    <path class="dext" d="M${xf},${Y(190)}V${yc - ty}M${xg},${cgY}V${yc}M${xr},${Y(190)}V${yc - ty}M${XL},${yc - ty}H${xf - 86}M${XL},${yc + ty}H${xf - 86}M${xr + 86},${yc - ty}H${XR}M${xr + 86},${yc + ty}H${XR}"/>
    ${cg(xg, cgY)}${cg(xg, yc)}
    <line class="dim" x1="${xg + 28}" y1="${cgY}" x2="${xg + 28}" y2="${gy}" marker-start="url(#da)" marker-end="url(#da)"/><text class="dtx" x="${xg + 42}" y="${(cgY + gy) / 2 + 10}">h = ${m(d.cg_height_m)}</text>
    <line class="dim" x1="${xf}" y1="${Y(190)}" x2="${rx}" y2="${ry}" marker-end="url(#da)"/><text class="dtx" x="${rx + 10}" y="${gy + 38}" text-anchor="end">r = ${m(d.wheel_radius_m)}</text>
    ${dimH(xf, xr, yd, 'L = ' + m(d.wheelbase_m))}
    ${dimH(xf, xg, yd2, 'a = ' + m(d.cg_from_front_axle_m))}
    ${dimH(xg, xr, yd2, 'b = ' + m(d.cg_from_rear_axle_m))}
    ${box(xf, yc - ty, fw)}${box(xf, yc + ty, fw)}${box(xr, yc - ty, rw)}${box(xr, yc + ty, rw)}
    <line class="dim" x1="${XL}" y1="${yc - ty}" x2="${XL}" y2="${yc + ty}" marker-start="url(#da)" marker-end="url(#da)"/>
    <text class="dtx" x="${XL - 14}" y="${yc}" text-anchor="middle" transform="rotate(-90 ${XL - 14} ${yc})"><tspan class="dl">FRONT TRACK</tspan> ${m(d.front_track_m)}</text>
    <line class="dim" x1="${XR}" y1="${yc - ty}" x2="${XR}" y2="${yc + ty}" marker-start="url(#da)" marker-end="url(#da)"/>
    <text class="dtx" x="${XR + 24}" y="${yc}" text-anchor="middle" transform="rotate(90 ${XR + 24} ${yc})"><tspan class="dl">REAR TRACK</tspan> ${m(d.rear_track_m)}</text>
    <text class="dsm" x="670" y="${OY + 816 * S + 40}" text-anchor="middle">TOTAL ${kg(d.mass_kg)} · FRONT AXLE ${kg(d.front_axle_mass_kg)} (${(100 * d.front_weight_fraction).toFixed(1)} %) · REAR AXLE ${kg(d.rear_axle_mass_kg)} · DRIVE ${d.drive}</text></svg>`;
}

// ---------------------------------------------------------------- pages
function showIssuesBox(el, issues, hint) {
  const errs = (issues || []).filter(i => i.level === 'error');
  el.innerHTML = errs.length ? `<b>This cannot be calculated yet.</b><ul>${errs.map(i => `<li>${escapeHtml(i.message)}</li>`).join('')}</ul>${hint || ''}` : '';
  return errs.length > 0;
}

function renderCarRes(res, ref) {
  const msg = document.getElementById('msg-car-res'), body = document.getElementById('body-car-res');
  const bad = !res.ok || !res.vehicle;
  showIssuesBox(msg, res.issues, '<div class="small">Correct it in <b>Data › Car</b>.</div>');
  body.hidden = bad;
  if (bad) return;
  const v = res.vehicle, r = ref && ref.vehicle ? ref.vehicle : null;
  const peakAt = ys => { let i = 0; ys.forEach((y, j) => { if (y > ys[i]) i = j; }); return 'AT ' + (Math.round(v.engine.rpm[i] / 10) * 10).toLocaleString('en') + ' rpm'; };
  document.getElementById('fig-engine').innerHTML = figuresTable(['peak_power_kw', 'peak_torque_nm'], res.key_figures, ref && ref.key_figures,
    { peak_power_kw: peakAt(v.engine.power_kw), peak_torque_nm: peakAt(v.engine.torque_nm) });
  document.getElementById('drawing').innerHTML = drawingSvg(v.drawing);
  {
    const e = tidy(engineChart(v, r), 'ENGINE SPEED (rpm)', { legend: false, right: 44 });
    e.yAxis[1].axisLabel = { ...e.yAxis[1].axisLabel, color: COLORS[1] };
    chart('ch-engine', e);
  }
  chart('ch-gears', tidy(gearsChart(v, r), 'VEHICLE SPEED (km/h)'));
  chart('ch-tyres', tidy(tyresChart(v), 'LOAD ON ONE TYRE (kN)'));
}

function renderPerfRes(res, ref) {
  const msg = document.getElementById('msg-perf-res'), body = document.getElementById('body-perf-res');
  const bad = !res.ok || !res.performance;
  showIssuesBox(msg, res.issues, '<div class="small">Correct it in <b>Data › Car</b>.</div>');
  body.hidden = bad;
  if (bad) return;
  const p = res.performance, rp = ref && ref.performance ? ref.performance : null;
  if (!p.gg.straight) {                               // the page is newer than the Python part it received
    msg.innerHTML = '<b>The Python part of the program is out of date.</b><div class="small">Reload the page. If you start the server with <code>python scripts/serve.py</code> it is rebuilt on every reload.</div>';
    body.hidden = true;
    return;
  }
  const kf = res.key_figures, rk = ref && ref.key_figures;
  const big = (id, k) => { const f = FIG[k]; document.getElementById(id).innerHTML = `${fmtVal(f, kf[k])} <span class="u">${f.unit}</span>`; };
  big('big-acc', 'time_0_100_kmh'); big('big-brk', 'distance_100_0_kmh'); big('big-run', 'time_0_100_0');
  document.getElementById('fig-acc').innerHTML = figuresTable(['time_0_60_mph', 'time_0_100_kmh', 'time_0_200_kmh', 'time_0_top'], kf, rk);
  document.getElementById('fig-brk').innerHTML = figuresTable(['distance_100_0_kmh', 'distance_200_0_kmh', 'distance_top_0'], kf, rk);
  document.getElementById('fig-run').innerHTML = figuresTable(['time_0_100_0', 'distance_0_100_0', 'time_0_200_0', 'distance_0_200_0', 'time_0_top_0', 'distance_0_top_0'], kf, rk);
  document.getElementById('perf-note').innerHTML = `Top speed <b>${Math.round(kf.top_speed_kmh)} km/h</b>, limited by ${p.key_figures.top_speed_limited_by.replace('_', ' ')}. The runs to the top speed stop at 98 % of it.`;
  chart('ch-profile', tidy(profileChart(p, rp), 'DISTANCE (m)', { legend: false }));
  chart('ch-forces', tidy(forcesChart(res.vehicle, ref && ref.vehicle ? ref.vehicle : null, kf.top_speed_kmh), 'VEHICLE SPEED (km/h)', { legend: false, right: 28 }));
  const slider = document.getElementById('gg-speed');
  slider.max = p.gg.diagrams.length - 1;
  if (S.ggIdx > +slider.max) S.ggIdx = +slider.max;
  slider.value = S.ggIdx;
  drawGG();
}

function drawGG() {
  const p = S.res && S.res.performance;
  if (!p) return;
  const idx = S.ggIdx, rp = S.reference && S.reference.performance ? S.reference.performance.gg : null;
  const d = p.gg.diagrams[idx];
  document.getElementById('gg-out').textContent = `${d.speed_kmh} km/h · gear ${d.gear} · ${Math.round(d.rpm).toLocaleString('en-US')} rpm`;
  chart('ch-gg', ggChart(p.gg, idx, rp));
}

function mapValues(c, mode) {
  if (mode === 'lat') { const v = c.lat_accel_g.map(Math.abs); return { v, unit: 'g', digits: 2 }; }
  if (mode === 'long') return { v: c.long_accel_g, unit: 'g', digits: 2 };
  if (mode === 'gear') return { v: c.gear.map(g => (g === null ? 0 : g)), unit: '', digits: 0 };   // 0: off the throttle, no gear
  return { v: c.speed_kmh, unit: 'km/h', digits: 0 };
}

function renderCircuitRes(res, ref) {
  const msg = document.getElementById('msg-circuit-res'), body = document.getElementById('body-circuit-res');
  const btn = document.getElementById('btn-whatif');
  let issues = res.ok ? res.circuit_error : res.issues;
  const bad = !res.ok || !res.circuit;
  showIssuesBox(msg, issues, '<div class="small">Correct it in <b>Data › Car</b> or <b>Data › Circuit</b>.</div>');
  body.hidden = bad; btn.disabled = bad;
  if (bad) return;
  const c = res.circuit, s = c.summary, rc = ref && ref.circuit ? ref.circuit : null;
  const same = rc && Math.abs(rc.summary.length_m - s.length_m) < 0.01 * s.length_m;
  const kf = res.key_figures, rk = same ? ref.key_figures : null;
  let dl = '';
  if (rk) { const d = kf.lap_time_s - rk.lap_time_s; dl = ` <span class="dlt ${Math.abs(d) < 0.005 ? '' : d < 0 ? 'good' : 'bad'}">${d >= 0 ? '+' : '−'}${Math.abs(d).toFixed(2)} s</span>`; }
  document.getElementById('fig-lap-time').innerHTML = s.lap_time_text + dl;
  document.getElementById('fig-lap-sub').textContent = `${(s.length_m / 1000).toFixed(3)} km · ${s.ds_m.toFixed(1)} m spacing · height ${s.height_min_m.toFixed(1)} to ${s.height_max_m.toFixed(1)} m`;
  document.getElementById('fig-lap').innerHTML =
    figuresTable(['lap_speed_min_kmh', 'lap_speed_max_kmh', 'lap_speed_avg_kmh', 'lap_max_lateral_g', 'lap_max_braking_g'], kf, rk) +
    (rc && !same ? '<p class="small muted">The reference is on a different circuit: it is not compared here.</p>' : '');
  const m = mapValues(c, document.getElementById('map-color').value);
  drawMap(document.getElementById('map-res'), c.x_m, c.y_m, { values: m.v, vmin: Math.min(...m.v), vmax: Math.max(...m.v), unit: m.unit, digits: m.digits, start: 0, dir: 1 });
  const nCh = 5;
  document.getElementById('circ-right').style.setProperty('--n', nCh);
  chart('ch-lap-speed', alignAlong(lapSpeedChart(c, same ? rc : null), c));
  chart('ch-lap-acc', alignAlong(lapAccChart(c), c));
  chart('ch-lap-gear', alignAlong(lapGearChart(c), c));
  chart('ch-lap-height', alignAlong(lapHeightChart(c), c));
  chart('ch-lap-delta', alignAlong(lapDeltaChart(c, same ? rc : null), c, true));
  if (S.whatif) renderWhatif(S.whatif);
}

function renderWhatif(w) {
  document.getElementById('card-whatif').hidden = false;
  const body = document.getElementById('wi-body');
  if (!w.ok) { body.innerHTML = `<div class="issue error">${escapeHtml(w.issues.map(i => i.message).join(' '))}</div>`; return; }
  const sgn = x => (x >= 0 ? '+' : '−') + Math.abs(x).toFixed(3);
  const cls = x => (Math.abs(x) < 0.0005 ? '' : x < 0 ? 'good' : 'bad');
  let h = `<p class="small muted">Baseline lap time ${fmtLap(w.baseline_lap_time_s)}. Sorted by the size of the effect. Negative (green) is faster.</p><div id="ch-wi" class="chart" style="height:${Math.max(220, 34 * w.rows.length + 90)}px"></div>`;
  h += '<table class="fig"><tr><th>Datum</th><th>Reduced (s)</th><th>Increased (s)</th><th>Per 1 % (s)</th></tr>';
  for (const r of w.rows) h += `<tr><td>${escapeHtml(r.label)}</td><td class="${cls(r.delta_minus_s)}">${sgn(r.delta_minus_s)}</td><td class="${cls(r.delta_plus_s)}">${sgn(r.delta_plus_s)}</td><td>${sgn(r.effect_per_percent_s)}</td></tr>`;
  h += '</table>';
  if (w.skipped.length) h += `<p class="small muted">Not studied (the changed data became invalid): ${w.skipped.map(s => escapeHtml(s.label)).join(', ')}.</p>`;
  body.innerHTML = h;
  chart('ch-wi', whatifChart(w.rows));
}
