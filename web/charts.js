// Charts (ECharts) and the circuit map (canvas). Reference curves are drawn faint and left out of the legends.
const ACCENT = getComputedStyle(document.documentElement).getPropertyValue('--accent').trim() || '#f03e3e';   // set in style.css only
// Colours are kept for what must draw the eye: red, orange, yellow, green, blue. Everything else is white or grey.
const COLORS = ['#f2f2f2', ACCENT, '#2fcf7a', '#f5d33f', '#4d9bff', '#ff8a2a', '#8a8a8a', '#b5b5b5'];
const FONT = 'Barlow, "Segoe UI", sans-serif';
echarts.registerTheme('vd', {
  color: COLORS, backgroundColor: 'transparent', textStyle: { fontFamily: FONT, color: '#8a8a8a', fontSize: 12 },
  categoryAxis: { axisLine: { lineStyle: { color: '#2a2a2a' } }, axisLabel: { color: '#8a8a8a' }, splitLine: { lineStyle: { color: '#161616' } } },
  valueAxis: { axisLine: { show: true, lineStyle: { color: '#3d3d3d' } }, axisLabel: { color: '#aaaaaa' }, nameTextStyle: { color: '#8a8a8a' }, splitLine: { lineStyle: { color: '#262626' } } },
  legend: { textStyle: { color: '#b5b5b5' } },
  tooltip: { backgroundColor: '#111111', borderColor: '#2e2e2e', textStyle: { color: '#f2f2f2' } },
});
const charts = {};

function chart(id, option) {
  const el = document.getElementById(id);
  if (!el || !el.clientWidth) return;
  let c = charts[id];
  if (!c || c.isDisposed() || c.getDom() !== el) { c = echarts.init(el, 'vd'); charts[id] = c; }
  c.setOption(option, true);
  c.resize();
}
window.addEventListener('resize', () => Object.values(charts).forEach(c => !c.isDisposed() && c.resize()));

const pairs = (xs, ys) => xs.map((x, i) => [x, ys[i]]);
const kmh = a => a.map(v => (v === null ? null : v * 3.6));

function ser(name, xs, ys, o = {}) {
  return {
    name, type: 'line', data: pairs(xs, ys), showSymbol: false, yAxisIndex: o.y || 0, step: o.step, z: o.faint ? 1 : 3,
    lineStyle: { width: (o.width || 2) * 0.62, type: o.dash ? 'dashed' : 'solid', color: o.color, opacity: o.faint ? 0.3 : 1 },
    itemStyle: { color: o.color }, emphasis: { disabled: true }, markLine: o.markLine,
  };
}

function base(xName, yName, legend, extra = {}) {
  return {
    animation: false,
    grid: { left: 20, right: 16, top: legend ? 74 : 36, bottom: 40, containLabel: true },
    tooltip: { trigger: 'axis', valueFormatter: v => (typeof v === 'number' ? +v.toPrecision(4) : v) },
    legend: legend ? { top: 0, type: 'scroll', data: legend } : { show: false },
    xAxis: { type: 'value', name: xName, nameLocation: 'middle', nameGap: 30, scale: true },
    yAxis: { type: 'value', name: yName, scale: true, nameTextStyle: { align: 'left' } },
    ...extra,
  };
}

function engineChart(v, r) {
  const e = v.engine;
  const s = [ser('Torque (N·m)', e.rpm, e.torque_nm, { color: COLORS[0] }), ser('Power (kW)', e.rpm, e.power_kw, { color: COLORS[1], y: 1 }),
    { name: 'Curve points', type: 'scatter', data: pairs(e.points_rpm, e.points_torque_nm), symbolSize: 3, itemStyle: { color: COLORS[0], opacity: 0.35 }, z: 2 }];
  const names = ['Torque (N·m)', 'Power (kW)', 'Curve points'];
  if (r) {
    s.push(ser('ref torque', r.engine.rpm, r.engine.torque_nm, { color: COLORS[0], faint: true }));
    s.push(ser('ref power', r.engine.rpm, r.engine.power_kw, { color: COLORS[1], y: 1, faint: true }));
  }
  const o = base('Engine speed (rpm)', 'Torque (N·m)', names);
  o.yAxis = [{ type: 'value', name: 'Torque (N·m)', scale: true }, { type: 'value', name: 'Power (kW)', scale: true, splitLine: { show: false } }];
  o.series = s;
  return o;
}

function gearsChart(v, r) {
  const g = v.gears;
  // Every gear is a straight line through the origin; it is drawn from zero to the rev cut (the last point).
  const lines = g.gears.map(x => {
    const sp = kmh(x.speed_ms), last = sp.length - 1;
    const s = ser('Gear ' + x.gear, [0, sp[last]], [0, x.rpm[last]], { color: '#f2f2f2', width: 1 });
    s.endLabel = { show: true, formatter: x.gear + '', color: '#8a8a8a', fontSize: 11, distance: 5 };
    return s;
  });
  // The run starts at the origin and follows the line of the first gear (the clutch slip at the start is not drawn).
  const fromZero = gg => {
    const sp = kmh(gg.sawtooth_speed_ms), rp = gg.sawtooth_rpm, f = gg.gears[0], fs = kmh(f.speed_ms), k = f.rpm[f.rpm.length - 1] / fs[fs.length - 1];
    let i = 0;
    while (i < sp.length && !(sp[i] > 0 && Math.abs(rp[i] - k * sp[i]) < 0.01 * rp[i])) i++;
    return i >= sp.length ? [sp, rp] : [[0, ...sp.slice(i)], [0, ...rp.slice(i)]];
  };
  const [es, er] = fromZero(g);
  const env = ser('Full-throttle run', es, er, { color: COLORS[4], width: 2.6 });
  const s = [...lines, env];
  if (r) { const [rs, rr] = fromZero(r.gears); s.push(ser('ref run', rs, rr, { color: COLORS[4], width: 2.6, faint: true })); }
  const o = base('Vehicle speed (km/h)', 'Engine speed (rpm)', ['Full-throttle run']);
  o.yAxis.min = 0; o.xAxis.min = 0; o.xAxis.scale = false;
  o.series = s;
  return o;
}

// Forces against speed, styled like the gears chart: thin white gear curves with their number at the start, one coloured envelope.
function forcesChart(v, r, top) {
  const f = v.forces, x = kmh(f.speed_ms);
  const clean = (xs, ys) => { const X = [], Y = []; ys.forEach((q, j) => { if (q !== null) { X.push(xs[j]); Y.push(q); } }); return [X, Y]; };
  const s = f.gear_force_n.map((y, i) => {
    const [xs, ys] = clean(x, y), q = ser('Gear ' + (i + 1), xs, ys, { color: '#f2f2f2', width: 1.2 });
    q.markPoint = { silent: true, data: [{ coord: q.data[0], symbol: 'circle', symbolSize: 1, itemStyle: { color: '#000', borderWidth: 0 }, label: { show: true, formatter: (i + 1) + '', color: '#8a8a8a', fontSize: 11, position: i === 0 ? 'bottom' : 'left', distance: 4 } }] };
    return q;
  });
  const env = ser('Engine (best gear)', x, f.engine_envelope_n, { color: COLORS[4], width: 2.6 });
  const tyre = ser('Traction limit', x, f.tyre_limit_n, { color: '#f2f2f2', dash: true, width: 1.8 });
  const drag = ser('Drag + rolling', x, f.resistance_n, { color: COLORS[6], width: 1.8 });
  // Names written on the curves (the invisible point only carries the text).
  const at = (xs, ys, xv, text, color, pos) => ({ coord: [xv, interp(xv, xs, ys)], symbol: 'circle', symbolSize: 1, itemStyle: { color: '#000', borderWidth: 0 }, label: { show: true, formatter: text, color, fontSize: 10, letterSpacing: 1, position: pos, distance: 7 } });
  const xmax = x[x.length - 1];
  env.markPoint = { silent: true, data: [at(x, f.engine_envelope_n, 0.07 * xmax, 'ENGINE', COLORS[4], 'bottom')] };
  tyre.markPoint = { silent: true, data: [at(x, f.tyre_limit_n, 0.85 * xmax, 'TRACTION LIMIT', '#f2f2f2', 'top')] };
  const dm = [at(x, f.resistance_n, 0.3 * xmax, 'DRAG + ROLLING', COLORS[6], 'top')];
  if (top) dm.push({ coord: [top, interp(top, x, f.resistance_n)], symbol: 'circle', symbolSize: 6, itemStyle: { color: COLORS[4] }, label: { show: true, formatter: 'TOP SPEED ' + Math.round(top) + ' km/h', color: '#f2f2f2', fontSize: 11, position: 'top', distance: 9 } });
  drag.markPoint = { silent: true, data: dm };
  s.push(env, tyre, drag);
  if (r) {
    const rx = kmh(r.forces.speed_ms);
    s.push(ser('ref engine', rx, r.forces.engine_envelope_n, { color: COLORS[4], faint: true, width: 2.6 }));
    s.push(ser('ref tyre', rx, r.forces.tyre_limit_n, { color: '#f2f2f2', faint: true, dash: true }));
    s.push(ser('ref resistance', rx, r.forces.resistance_n, { color: COLORS[6], faint: true }));
  }
  const o = base('Vehicle speed (km/h)', 'Force (N)', null);
  o.series = s;
  return o;
}

function tyresChart(v) {
  const t = v.tyres;
  const kn = a => a.map(z => z / 1000);
  const s = [ser('Front longitudinal', kn(t.front.load_n), t.front.mu_x, { color: COLORS[0] }), ser('Front lateral', kn(t.front.load_n), t.front.mu_y, { color: COLORS[0], dash: true }),
    ser('Rear longitudinal', kn(t.rear.load_n), t.rear.mu_x, { color: COLORS[2] }), ser('Rear lateral', kn(t.rear.load_n), t.rear.mu_y, { color: COLORS[2], dash: true })];
  s[0].markLine = { silent: true, symbol: 'none', lineStyle: { color: '#8f8f8f', type: 'dotted' },
    data: [{ xAxis: t.static_load_n.front / 1000, label: { formatter: 'static front', position: 'insideEndTop', color: '#8a8a8a', fontSize: 10, fontWeight: 400, textBorderWidth: 0 } },
      { xAxis: t.static_load_n.rear / 1000, label: { formatter: 'static rear', position: 'insideStartTop', color: '#8a8a8a', fontSize: 10, fontWeight: 400, textBorderWidth: 0 } }] };
  // The legend icon of the dashed curves is dashed too.
  const DASH = 'path://M0,0H5V3H0ZM8,0H13V3H8ZM16,0H21V3H16Z';
  const o = base('Load on one tyre (kN)', 'Friction coefficient', s.map(q => (q.lineStyle.type === 'dashed' ? { name: q.name, icon: DASH } : q.name)));
  o.yAxis.min = 0;
  o.series = s;
  return o;
}

function curveChart(curve) {
  const o = base('Engine speed (rpm)', 'Torque (N·m)', null);
  o.tooltip = { trigger: 'axis' };
  o.series = [ser('Torque', curve.rpm, curve.torque_nm, { color: COLORS[0] }),
    { type: 'scatter', data: pairs(curve.rpm, curve.torque_nm), symbolSize: 4, itemStyle: { color: COLORS[0] } }];
  return o;
}

// Charts of Results > Car with the look of the circuit: the title is an HTML label, so the axes carry no name.
// The legend goes on its own row under the title (or is left out when the titles already say which curve is which).
function tidy(o, xName, { legend = true, right = 16 } = {}) {
  o.grid = { left: 44, right, top: legend && o.legend && o.legend.show !== false ? 62 : 30, bottom: 40, containLabel: false };
  if (o.legend && o.legend.show !== false && legend) {
    o.legend = { ...o.legend, top: 24, left: 40, right: 8, type: 'plain', icon: 'roundRect', itemWidth: 14, itemHeight: 3, itemGap: 12,
      textStyle: { color: '#8a8a8a', fontSize: 11 }, formatter: n => n.toUpperCase() };
  } else o.legend = { show: false };
  for (const a of [].concat(o.yAxis)) { a.name = ''; a.splitNumber = 4; a.axisLabel = { ...a.axisLabel, fontSize: 11, margin: 8 }; }
  o.xAxis.axisLabel = { ...o.xAxis.axisLabel, fontSize: 11, margin: 8 };
  o.xAxis.name = xName; o.xAxis.nameLocation = 'middle'; o.xAxis.nameGap = 24; o.xAxis.nameTextStyle = { color: '#8a8a8a', fontSize: 11 };
  return o;
}

// Same look as the charts along the circuit: the title is an HTML label, so the vertical axis has no name.
function alignCurve(o) {
  o.grid = { left: 44, right: 14, top: 36, bottom: 38, containLabel: false };
  o.yAxis.name = ''; o.yAxis.splitNumber = 4; o.yAxis.axisLabel = { ...o.yAxis.axisLabel, fontSize: 11, margin: 8 };
  o.xAxis.axisLabel = { ...o.xAxis.axisLabel, fontSize: 11, margin: 8 };
  o.xAxis.name = 'ENGINE SPEED (rpm)'; o.xAxis.nameLocation = 'middle'; o.xAxis.nameGap = 22; o.xAxis.nameTextStyle = { color: '#8a8a8a', fontSize: 11 };
  return o;
}

function profileChart(p, r) {
  const a = p.profile_requested, m = p.profile_max;
  const s = [ser('0–200–0 km/h', a.distance_m, kmh(a.speed_ms), { color: COLORS[0], width: 1.5 }), ser('0–max–0', m.distance_m, kmh(m.speed_ms), { color: COLORS[3], width: 2.6 })];
  const peak = d => d.reduce((b, q) => (q[1] > b[1] ? q : b), d[0]);
  const mark = (d, color, text) => ({ silent: true, data: [{ coord: peak(d), symbol: 'circle', symbolSize: 6, itemStyle: { color }, label: { show: true, formatter: text, color, fontSize: 11, position: 'top', distance: 7 } }] });
  s[0].markPoint = mark(s[0].data, COLORS[0], Math.round(peak(s[0].data)[1]) + ' km/h');
  s[1].markPoint = mark(s[1].data, COLORS[3], Math.round(peak(s[1].data)[1]) + ' km/h · 98 % of top speed');
  if (r) {
    s.push(ser('ref 200', r.profile_requested.distance_m, kmh(r.profile_requested.speed_ms), { color: COLORS[0], faint: true, width: 1.5 }));
    s.push(ser('ref max', r.profile_max.distance_m, kmh(r.profile_max.speed_ms), { color: COLORS[3], faint: true, width: 2.6 }));
  }
  const o = base('Distance (m)', 'Speed (km/h)', null);
  o.series = s;
  return o;
}

function ggPolygon(d) {
  const up = [], lo = [];
  d.ay_g.forEach((ay, i) => {
    if (d.ax_accel_g[i] !== null) up.push([ay, d.ax_accel_g[i]]);
    if (d.ax_brake_g[i] !== null) lo.push([ay, -Math.abs(d.ax_brake_g[i])]);
  });
  const poly = up.concat(lo.reverse());
  if (poly.length) poly.push(poly[0]);
  return poly;
}

function nearestDiagram(gg, speed) {
  let best = null;
  for (const d of gg.diagrams) if (best === null || Math.abs(d.speed_kmh - speed) < Math.abs(best.speed_kmh - speed)) best = d;
  return best && Math.abs(best.speed_kmh - speed) < 12 ? best : null;
}

function ggStraightChart(gg, speed, ref) {
  const st = gg.straight;
  const o = base('Speed (km/h)', 'Acceleration (g)', null);
  o.tooltip = { trigger: 'axis', formatter: ps => `${ps[0].value[0]} km/h<br>${(+ps[0].value[1]).toFixed(3)} g · gear ${st.gear[st.speed_kmh.indexOf(ps[0].value[0])]}` };
  const line = ser('Acceleration', st.speed_kmh, st.ax_accel_g, { color: COLORS[0], width: 2 });
  line.markLine = { silent: true, symbol: 'none', lineStyle: { color: COLORS[1], width: 1.2 }, label: { formatter: speed + ' km/h' }, data: [{ xAxis: speed }] };
  o.series = [line];
  if (ref) o.series.push(ser('ref', ref.straight.speed_kmh, ref.straight.ax_accel_g, { color: COLORS[0], faint: true, width: 2 }));
  o.yAxis.min = 0;
  return o;
}

// g-g diagram: round grid, smooth green outline whose fill fades towards the centre, and its three extremes written on the axes.
function ggChart(gg, idx, ref) {
  const el = document.getElementById('ch-gg');
  const side = Math.max(Math.min(el.clientHeight - 70, el.clientWidth - 120), 100);   // the plot is a square: same scale on both axes
  let lim = 0.5;
  for (const d of gg.diagrams) for (const k of ['ay_g', 'ax_accel_g', 'ax_brake_g']) for (const x of d[k]) if (x !== null) lim = Math.max(lim, Math.abs(x));
  lim = Math.ceil(lim * 10) / 10 + 0.1;
  const G = COLORS[2], d = gg.diagrams[idx];
  const poly = ggPolygon(d);
  const fade = { type: 'radial', x: 0.5, y: 0.5, r: 0.55, colorStops: [{ offset: 0, color: 'rgba(47,207,122,0)' }, { offset: 0.55, color: 'rgba(47,207,122,0.03)' }, { offset: 1, color: 'rgba(47,207,122,0.30)' }] };
  const outline = { name: 'Limit', type: 'line', data: poly, smooth: 0.35, showSymbol: false, lineStyle: { width: 1.2, color: G }, areaStyle: { color: fade }, z: 3 };
  const num = (a, f) => a.filter(q => q !== null).reduce(f);
  const accel = num(d.ax_accel_g, (p, q) => Math.max(p, q)), brake = num(d.ax_brake_g.map(Math.abs), (p, q) => Math.max(p, q)), lat = num(d.ay_g.map(Math.abs), (p, q) => Math.max(p, q));
  const pt = (coord, text, pos) => ({ coord, label: { show: true, formatter: text, color: G, fontSize: 11, position: pos, distance: 8 } });
  outline.markPoint = { silent: true, symbol: 'circle', symbolSize: 5, itemStyle: { color: G },
    data: [pt([0, accel], accel.toFixed(2) + ' g accel', 'top'), pt([0, -brake], brake.toFixed(2) + ' g brake', 'bottom'), pt([lat, 0], lat.toFixed(2) + ' g lateral', 'top')] };
  const ring = r => { const a = []; for (let i = 0; i <= 120; i++) a.push([r * Math.cos(i / 120 * 2 * Math.PI), r * Math.sin(i / 120 * 2 * Math.PI)]); return { type: 'line', data: a, showSymbol: false, silent: true, lineStyle: { width: 0.8, color: '#3a3a3a' }, z: 0, tooltip: { show: false } }; };
  const s = [outline];
  for (let r = 0.5; r <= lim - 0.05; r += 0.5) s.push(ring(r));
  const rd = ref && nearestDiagram(ref, d.speed_kmh);
  if (rd) s.push({ name: 'ref', type: 'line', data: ggPolygon(rd), smooth: 0.35, showSymbol: false, lineStyle: { width: 1.2, color: G, opacity: 0.3 }, z: 1 });
  const o = base('Lateral acceleration (g)', 'Longitudinal acceleration (g)', null);
  const axis = { lineStyle: { color: '#3a3a3a' } };
  o.grid = { left: Math.max((el.clientWidth - side) / 2, 60), top: 36, width: side, height: side, containLabel: false };
  o.xAxis = { type: 'value', name: 'LATERAL ACCELERATION (g)', nameLocation: 'middle', nameGap: 30, nameTextStyle: { color: '#8a8a8a', fontSize: 11 }, min: -lim, max: lim, splitLine: { show: false }, axisLine: axis, axisTick: { show: false }, axisLabel: { fontSize: 11, formatter: v => String(Math.round(v * 10) / 10) } };
  o.yAxis = { type: 'value', min: -lim, max: lim, splitLine: { show: false }, axisLine: axis, axisTick: { show: false }, axisLabel: { fontSize: 11, formatter: v => String(Math.round(v * 10) / 10) } };
  o.tooltip = { trigger: 'none' };
  o.series = s;
  return o;
}

function interp(x, xs, ys) {        // linear interpolation, xs increasing
  if (x <= xs[0]) return ys[0];
  let lo = 0, hi = xs.length - 1;
  if (x >= xs[hi]) return ys[hi];
  while (hi - lo > 1) { const m = (lo + hi) >> 1; if (xs[m] <= x) lo = m; else hi = m; }
  return ys[lo] + (ys[hi] - ys[lo]) * (x - xs[lo]) / (xs[hi] - xs[lo]);
}

// All the charts along the circuit have the same plot area horizontally, so that the same distance is at the same x.
// The title of each chart is an HTML label (it also names the vertical axis), so the axes carry no name here.
function alignAlong(o, c, last = false) {
  o.grid = { left: 40, right: 40, top: 26, bottom: last ? 38 : 20, containLabel: false };
  if (o.legend && o.legend.show !== false) {
    o.legend = { ...o.legend, top: 2, right: 6, left: 'auto', type: 'plain', icon: 'roundRect', itemWidth: 14, itemHeight: 3, itemGap: 14,
      textStyle: { color: '#8a8a8a', fontSize: 11 }, formatter: n => n.toUpperCase() };
  }
  for (const a of [].concat(o.yAxis)) { a.name = ''; a.splitNumber = 3; a.axisLabel = { ...a.axisLabel, fontSize: 11, margin: 8 }; }
  o.xAxis.axisLabel = { ...o.xAxis.axisLabel, fontSize: 11, margin: 8 };
  o.xAxis.name = last ? 'DISTANCE (m)' : '';
  if (last) { o.xAxis.nameLocation = 'middle'; o.xAxis.nameGap = 20; o.xAxis.nameTextStyle = { color: '#8a8a8a', fontSize: 11 }; }
  o.xAxis.min = 0;
  o.xAxis.max = Math.ceil(c.position_m[c.position_m.length - 1] / 500) * 500;   // the same round limit in all of them
  o.xAxis.scale = false;
  return o;
}

function lapSpeedChart(c, r) {
  const s = [ser('Speed', c.position_m, c.speed_kmh, { color: COLORS[3], width: 2.5 }), ser('Cornering limit', c.position_m, c.limit_kmh, { color: COLORS[6], dash: true, width: 1.5 })];
  if (r) s.push(ser('ref speed', r.position_m, r.speed_kmh, { color: COLORS[3], faint: true, width: 2.5 }));
  const o = base('Distance (m)', 'Speed (km/h)', ['Speed', 'Cornering limit']);
  o.yAxis.min = Math.floor(Math.min(...c.speed_kmh) * 0.8 / 10) * 10;
  o.yAxis.max = Math.ceil(Math.max(...c.speed_kmh) * 1.15 / 10) * 10;
  o.series = s;
  return o;
}

// Time difference against the reference: green below the axis (faster), red above (slower), grey while there is nothing to compare.
// The line is split in a red and a green series that meet at the exact zero crossings.
function lapDeltaChart(c, r) {
  const xs = c.position_m;
  const d = r ? xs.map((p, i) => c.time_s[i] - interp(p, r.position_m, r.time_s)) : xs.map(() => 0);
  const o = base('Distance (m)', 'Time difference (s)', null);
  if (d.every(v => Math.abs(v) < 1e-6)) {
    o.series = [ser('Δt', xs, d, { color: COLORS[6], width: 2 })];
    o.yAxis.min = -1; o.yAxis.max = 1;
    return o;
  }
  const pos = [], neg = [];
  const push = (x, y) => { pos.push([x, y > 1e-12 ? y : y < -1e-12 ? null : 0]); neg.push([x, y < -1e-12 ? y : y > 1e-12 ? null : 0]); };
  for (let i = 0; i < xs.length; i++) {
    if (i > 0 && d[i - 1] * d[i] < 0) { const t = d[i - 1] / (d[i - 1] - d[i]); const x = xs[i - 1] + t * (xs[i] - xs[i - 1]); pos.push([x, 0]); neg.push([x, 0]); }
    push(xs[i], d[i]);
  }
  const line = (name, data, color) => ({ name, type: 'line', data, showSymbol: false, symbolSize: 7, connectNulls: false, z: 3, itemStyle: { color }, lineStyle: { width: 2 * 0.62, color }, areaStyle: { color, opacity: 0.1, origin: 0 } });
  o.series = [line('Slower', pos, COLORS[1]), line('Faster', neg, COLORS[2])];
  // Only the difference is written, with the sign and in the colour of the line.
  o.tooltip = { trigger: 'axis', formatter: ps => { const q = ps.find(s => s.value && s.value[1] !== null && Math.abs(s.value[1]) > 1e-12) || ps[0]; const v = q.value[1]; return `<span style="color:${q.color}">${v > 0 ? '+' : v < 0 ? '−' : ''}${Math.abs(v).toFixed(2)} s</span>`; } };
  return o;
}

function lapAccChart(c) {
  const o = base('Distance (m)', 'Acceleration (g)', ['Longitudinal', 'Lateral (+ left)']);
  o.series = [ser('Longitudinal', c.position_m, c.long_accel_g, { color: COLORS[0], width: 1.5 }), ser('Lateral (+ left)', c.position_m, c.lat_accel_g, { color: COLORS[5], width: 1.5 })];
  return o;
}

function lapGearChart(c) {
  const o = base('Distance (m)', 'Gear', null);
  o.yAxis = { type: 'value', name: 'Gear', minInterval: 1, min: 0, nameTextStyle: { align: 'left' } };
  // Off the throttle (braking, holding the corner speed) there is no gear: the line has a gap there.
  o.series = [ser('Gear', c.position_m, c.gear, { color: COLORS[2], step: 'end', width: 2 })];
  return o;
}

function lapHeightChart(c) {
  const o = base('Distance (m)', 'Height (m)', ['Height', 'Bank (+ tilts left)']);
  o.yAxis = [{ type: 'value', name: 'Height (m)', scale: true }, { type: 'value', name: 'Bank (°)', scale: true, splitLine: { show: false }, axisLabel: { color: COLORS[4] }, axisLine: { show: false } }];
  o.series = [ser('Height', c.position_m, c.z_m, { color: COLORS[0] }), ser('Bank (+ tilts left)', c.position_m, c.bank_deg, { color: COLORS[4], y: 1 })];
  return o;
}

function whatifChart(rows) {
  const r = rows.slice().reverse();
  const o = {
    animation: false, grid: { left: 8, right: 24, top: 40, bottom: 40, containLabel: true },
    tooltip: { trigger: 'axis', axisPointer: { type: 'shadow' }, valueFormatter: v => v.toFixed(3) + ' s' },
    legend: { top: 0, data: ['Datum reduced', 'Datum increased'] },
    xAxis: { type: 'value', name: 'Lap time change (s)', nameLocation: 'middle', nameGap: 30 },
    yAxis: { type: 'category', data: r.map(x => x.label) },
    series: [{ name: 'Datum reduced', type: 'bar', data: r.map(x => x.delta_minus_s), itemStyle: { color: COLORS[0] } },
      { name: 'Datum increased', type: 'bar', data: r.map(x => x.delta_plus_s), itemStyle: { color: COLORS[6] } }],
  };
  return o;
}

// ---------------------------------------------------------------- circuit map
function colorAt(t) { return `hsl(${Math.round(215 * (1 - Math.min(Math.max(t, 0), 1)))},85%,58%)`; }     // blue (low), green, yellow, red (high)

// Colour scales of the circuit map. The same value always gives the same colour; only the ends of the scale follow the lap.
const clamp01 = t => Math.min(Math.max(t, 0), 1);
const speedColor = t => `hsl(${Math.round(215 * clamp01(t))},85%,58%)`;                // slow red, yellow, green, fast blue
const mixRgb = (a, b, t) => a.map((x, i) => x + (b[i] - x) * t);
const hslRgb = (h, s, l) => { const a = s * Math.min(l, 1 - l), f = n => { const k = (n + h / 30) % 12; return l - a * Math.max(-1, Math.min(k - 3, 9 - k, 1)); }; return [f(0) * 255, f(8) * 255, f(4) * 255]; };
const LONG_GREY = [61, 61, 61], LONG_RED = hslRgb(355, 1, 0.56), LONG_GREEN = hslRgb(140, 1, 0.5), LONG_TEAL = hslRgb(172, 1, 0.5);
// Braking: grey to red; acceleration: grey to green to turquoise. Each side reaches full colour at its own maximum of the lap.
function longitudinalColor(amin, amax) {
  return a => {
    if (Math.abs(a) < 0.03) return rgbCss(LONG_GREY);
    if (a < 0) return rgbCss(mixRgb(LONG_GREY, LONG_RED, Math.pow(Math.min(Math.min(a / amin, 1) / 0.35, 1), 0.6)));
    const t = Math.min(a / amax, 1);
    return rgbCss(t < 0.12 ? mixRgb(LONG_GREY, LONG_GREEN, Math.pow(t / 0.12, 0.6)) : mixRgb(LONG_GREEN, LONG_TEAL, (t - 0.12) / 0.88));
  };
}
const rgbCss = c => `rgb(${c.map(Math.round).join(',')})`;
// One colour per gear, always the same (turbo scale: blue for the lowest, red for the highest). No gear: dark grey.
const TURBO_6 = ['#4559cb', '#23c3e4', '#61fc6c', '#dde037', '#fa7b1f', '#b71d02'], NO_GEAR_COLOR = '#666666';
const hexRgb = h => [1, 3, 5].map(i => parseInt(h.slice(i, i + 2), 16));
// n gears: the six colours for 6 gears; another number of gears takes n points along the same scale (always the same for a car).
function gearPalette(n) {
  if (n === 6) return TURBO_6.slice();
  return Array.from({ length: n }, (_, k) => {
    const x = n === 1 ? 0 : 5 * k / (n - 1), i = Math.min(Math.floor(x), 4);
    return rgbCss(mixRgb(hexRgb(TURBO_6[i]), hexRgb(TURBO_6[i + 1]), x - i));
  });
}

// o: {values, vmin, vmax, unit, start, dir (+1/-1), onPoints}. Returns a function that finds the nearest point to a click.
function drawMap(canvas, xs, ys, o = {}) {
  const W = canvas.clientWidth, H = canvas.clientHeight;
  if (!W || !H) return null;
  const dpr = window.devicePixelRatio || 1;
  canvas.width = Math.round(W * dpr); canvas.height = Math.round(H * dpr);
  const g = canvas.getContext('2d');
  g.setTransform(dpr, 0, 0, dpr, 0, 0);
  g.clearRect(0, 0, W, H);
  const n = xs.length, pad = 14, padTop = 34;                         // the top has more room: the word FINISH can sit above the line
  const minx = Math.min(...xs), maxx = Math.max(...xs), miny = Math.min(...ys), maxy = Math.max(...ys);
  const sc = Math.min((W - 2 * pad) / Math.max(maxx - minx, 1e-9), (H - pad - padTop) / Math.max(maxy - miny, 1e-9));
  const ox = (W - sc * (maxx - minx)) / 2, oy = pad + (H - pad - padTop - sc * (maxy - miny)) / 2;
  const px = x => ox + sc * (x - minx), py = y => H - (oy + sc * (y - miny));
  g.lineWidth = 5; g.lineCap = 'round'; g.lineJoin = 'round';
  const span = (o.vmax - o.vmin) || 1;
  for (let i = 0; i < n; i++) {
    const j = (i + 1) % n;
    g.strokeStyle = o.values ? (o.colorFn ? o.colorFn(o.values[i]) : colorAt((o.values[i] - o.vmin) / span)) : '#6b6b6b';
    g.beginPath(); g.moveTo(px(xs[i]), py(ys[i])); g.lineTo(px(xs[j]), py(ys[j])); g.stroke();
  }
  const dir = o.dir || 1;
  const s = o.start || 0, sb = (s + dir + n) % n;
  const ang = Math.atan2(py(ys[sb]) - py(ys[s]), px(xs[sb]) - px(xs[s]));
  g.save(); g.translate(px(xs[s]), py(ys[s])); g.rotate(ang);
  g.strokeStyle = '#f2f2f2'; g.lineWidth = 3; g.beginPath(); g.moveTo(0, -14); g.lineTo(0, 14); g.stroke(); g.restore();
  g.fillStyle = '#b5b5b5'; g.font = '500 11px ' + FONT; if ('letterSpacing' in g) g.letterSpacing = '1px'; g.fillText('FINISH', px(xs[s]) + 10, py(ys[s]) - 12);
  // The direction of the lap: a hairline pointer coming out of the finish line (one only: it does not cover the colours).
  g.save(); g.translate(px(xs[s]), py(ys[s])); g.rotate(ang); g.strokeStyle = '#f2f2f2'; g.lineWidth = 1.1; g.lineCap = 'round'; g.lineJoin = 'round';
  g.beginPath(); g.moveTo(0, 14); g.lineTo(18, 14); g.moveTo(13, 9.5); g.lineTo(18, 14); g.lineTo(13, 18.5); g.stroke(); g.restore();
  if (o.values && o.swatches) {                                        // one swatch per category
    const sw = 17, gap = 3, bw = o.swatches.length * (sw + gap) - gap, bx = W - bw - 14, by = H - 26;
    g.font = '500 11px ' + FONT; g.textAlign = 'center';
    o.swatches.forEach((w, k) => { g.fillStyle = w.color; g.fillRect(bx + k * (sw + gap), by, sw, 8); g.fillStyle = '#8a8a8a'; g.fillText(w.label, bx + k * (sw + gap) + sw / 2, by - 3); });
    g.textAlign = 'left';
  } else if (o.values) {                                               // colour bar
    const bw = 120, bx = W - bw - 14, by = H - 26;
    for (let k = 0; k < bw; k++) { g.fillStyle = o.colorFn ? o.colorFn(o.vmin + span * k / (bw - 1)) : colorAt(k / (bw - 1)); g.fillRect(bx + k, by, 1, 8); }
    g.fillStyle = '#8a8a8a'; g.font = '500 11px ' + FONT;
    g.textAlign = 'left'; g.fillText(o.vmin.toFixed(o.digits ?? 0), bx, by - 3);
    g.textAlign = 'right'; g.fillText((o.zero && o.vmax > 0 ? '+' : '') + o.vmax.toFixed(o.digits ?? 0) + ' ' + (o.unit || ''), bx + bw, by - 3);
    if (o.zero && o.vmin < 0 && o.vmax > 0) { g.textAlign = 'center'; g.fillText('0', bx + bw * (0 - o.vmin) / span, by + 19); }   // below the bar
    g.textAlign = 'left';
  }
  if ('letterSpacing' in g) g.letterSpacing = '0px';
  const pick = (cx, cy, maxDist = Infinity) => {          // the point of the circuit nearest to (cx, cy), or -1 if it is farther than maxDist
    let best = -1, bd = maxDist * maxDist;
    for (let i = 0; i < n; i++) { const d = (px(xs[i]) - cx) ** 2 + (py(ys[i]) - cy) ** 2; if (d < bd) { bd = d; best = i; } }
    return best;
  };
  pick.at = i => [px(xs[i]), py(ys[i])];                   // canvas position of point i
  return pick;
}
