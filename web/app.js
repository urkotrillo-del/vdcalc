// Glue: state, navigation, files, calls to the Python worker.
const S = {
  reg: null, vehicle: null, env: { environment: { gravity: 9.81, air_density: 1.225 } }, circuit: null, circuitName: '', preview: null,
  res: null, reference: null, dirty: true, view: 'car-data', ggIdx: 0, whatif: null, seq: 0, busy: false,
};
const $ = id => document.getElementById(id);

// ---------------------------------------------------------------- worker
let worker = null, bootExtra = null;
const pending = new Map();
let nextId = 1;
function startWorker() {
  worker = new Worker('worker.js?v=' + Date.now(), { type: 'module' });
  worker.onmessage = e => {
    const m = e.data;
    if (m.type === 'status') { $('start-btn').textContent = m.text; return; }
    const p = pending.get(m.id);
    if (!p) return;
    if (m.type === 'progress') { if (p.onProgress) p.onProgress(m.done, m.total); return; }
    pending.delete(m.id);
    m.ok ? p.resolve(m.result) : p.reject(new Error(m.error));
  };
}
startWorker();
let ready = Promise.resolve();                 // set while a new worker is booting: other calls wait for it
async function call(fn, args = [], onProgress = null, extra = {}) {
  if (fn !== '__boot__') await ready;
  return new Promise((resolve, reject) => {
    const id = nextId++;
    pending.set(id, { resolve, reject, onProgress });
    worker.postMessage({ id, fn, args, ...extra });
  });
}
// The only way to stop a running calculation is to end the worker and start a new one (it keeps no state of its own).
async function restartWorker() {
  worker.terminate();
  for (const p of pending.values()) p.reject(new Error('cancelled'));
  pending.clear();
  startWorker();
  ready = call('__boot__', [], null, bootExtra);
  await ready;
}
const callJson = async (fn, args, onProgress) => JSON.parse(await call(fn, args, onProgress));

// While calculating, the header buttons are switched off so nobody clicks them half-way.
let whatifRunning = false, whatifToken = 0;
async function cancelWhatIf() {
  whatifToken++; whatifRunning = false;
  $('wi-bar').hidden = true; $('btn-whatif').disabled = false; $('card-whatif').hidden = true; S.whatif = null;
  await restartWorker();
}
const setToolsBusy = on => { for (const id of ['env-toggle', 'btn-ref']) $(id).disabled = on || whatifRunning; };
function showBusy(text) { $('busy-text').textContent = text; $('busy').classList.toggle('dim', S.view.endsWith('-res')); $('busy').hidden = false; setToolsBusy(true); }
function hideBusy() { $('busy').hidden = true; setToolsBusy(false); }

// ---------------------------------------------------------------- helpers
const envForCalc = () => Object.fromEntries(Object.entries(S.env.environment).filter(([, v]) => v !== null && v !== '' && !Number.isNaN(v)));
const circuitForCalc = () => S.circuit;
let validateTimer = null;

let envChanged = false;
function markDirty() { refreshRefForm(); S.dirty = true; S.whatif = null; $('card-whatif').hidden = true; scheduleValidate(); }

function scheduleValidate() {
  clearTimeout(validateTimer);
  validateTimer = setTimeout(() => runValidate().catch(e => { if (e.message === 'cancelled') scheduleValidate(); else throw e; }), 250);
}

async function runValidate() {
  const r = await callJson('validate', [JSON.stringify(S.vehicle), JSON.stringify(envForCalc())]);
  const loose = showIssues($('form-car'), r.issues).filter(i => !i.key.startsWith('environment.') && i.key !== 'powertrain.torque_curve');
  $('issues-car').innerHTML = issueHtml(loose);
  showIssues($('env-body'), r.issues.filter(i => i.key.startsWith('environment.')));
  const curve = r.issues.filter(i => i.key === 'powertrain.torque_curve');
  $('curve-msg').innerHTML = issueHtml(curve);
}

function download(name, text, type = 'application/json') {
  const a = document.createElement('a');
  a.href = URL.createObjectURL(new Blob([text], { type }));
  a.download = name; document.body.append(a); a.click(); a.remove();
  setTimeout(() => URL.revokeObjectURL(a.href), 2000);
}
const readFile = file => new Promise((res, rej) => { const f = new FileReader(); f.onload = () => res(f.result); f.onerror = rej; f.readAsText(file); });

// ---------------------------------------------------------------- car data
function onFormChange(key) {
  if (key === 'general.drive' || key === 'tires.mode') updateVisibility($('form-car'), S.vehicle);
  markDirty();
}

function buildCarForm() {
  buildForm($('form-car'), S.reg.groups, S.vehicle, onFormChange, ['environment']);
  refreshRefForm();
  showCurve();
}

function showCurve() {
  const c = S.vehicle.powertrain.torque_curve;
  if (!c || !c.rpm || !c.rpm.length) { $('curve-info').textContent = 'No curve loaded.'; return; }
  const peak = Math.max(...c.torque_nm);
  $('curve-info').textContent = `${c.rpm.length} points · ${Math.min(...c.rpm).toFixed(0)} to ${Math.max(...c.rpm).toFixed(0)} rpm · peak ${peak.toFixed(0)} N·m. CSV format: two columns, rpm and torque in N·m.`;
  if (S.view === 'car-data') chart('ch-curve', alignCurve(curveChart(c)));
}

async function importCurve(file) {
  const r = await callJson('parse_torque_curve', [await readFile(file)]);
  if (!r.ok) { $('curve-msg').innerHTML = issueHtml(r.issues); return; }
  S.vehicle.powertrain.torque_curve = r.curve;
  showCurve(); $('curve-msg').innerHTML = issueHtml(r.issues); markDirty();
}

async function loadCar(file) {
  const r = await callJson('parse_vehicle', [await readFile(file)]);
  if (!r.ok) { $('issues-car').innerHTML = issueHtml(r.issues); return; }
  S.vehicle = r.data; buildCarForm(); markDirty();
}

function saveCar() {
  const name = (S.vehicle.name || 'car').replace(/[^\w\- ]+/g, '').trim().replace(/\s+/g, '_') || 'car';
  download(name + '.json', JSON.stringify({ format_version: 1, ...S.vehicle }, null, 2));
}

// ---------------------------------------------------------------- circuit data
async function refreshCircuit() {
  const c = S.circuit;
  const r = await callJson('circuit_preview', [JSON.stringify(c)]);
  S.preview = r;
  $('issues-circuit').innerHTML = issueHtml(r.issues || []);
  $('circuit-info').innerHTML = r.ok
    ? `<b>${escapeHtml(S.circuitName)}</b> · ${c.x.length} points · length ${(r.length_m / 1000).toFixed(3)} km · height: ${c.z ? 'yes' : 'no'} · banking: ${c.bank_deg ? 'yes' : 'no'} · direction: ${c.reverse ? 'reversed' : 'as in the file'}`
    : '<b>This circuit cannot be used.</b>';
  drawCircuitData();
}

function drawCircuitData() {
  if (S.view !== 'circuit-data' || !S.circuit) return;
  const c = S.circuit;
  S.mapPick = drawMap($('map-data'), c.x, c.y, { start: c.start, dir: c.reverse ? -1 : 1 });
}

async function importCircuit(file) {
  const r = await callJson('parse_circuit', [await readFile(file)]);
  if (!r.ok) { $('issues-circuit').innerHTML = issueHtml(r.issues); return; }
  S.circuit = { ...r.circuit, reverse: false, start: 0 }; S.circuitName = file.name;
  await refreshCircuit(); markDirty();
}

// ---------------------------------------------------------------- results
async function ensureResults() {
  if (!S.dirty && S.res) return;
  showBusy('Calculating…');
  if (whatifRunning) await cancelWhatIf();      // the data changed: the What if in progress no longer applies
  try {
    S.res = await callJson('calculate_all', [JSON.stringify(S.vehicle), JSON.stringify(envForCalc()), JSON.stringify(circuitForCalc())]);
    S.dirty = false;
  } catch (e) {
    S.res = { ok: false, issues: [{ level: 'error', key: '', message: 'Unexpected error: ' + e.message }] };
  }
  hideBusy();
}

function renderView(view) {
  const ref = S.reference;
  if (view === 'car-data') { showCurve(); }
  else if (view === 'circuit-data') drawCircuitData();
  else if (S.res) {
    if (view === 'car-res') renderCarRes(S.res, ref);
    if (view === 'perf-res') renderPerfRes(S.res, ref);
    if (view === 'circuit-res') renderCircuitRes(S.res, ref);
  }
}

async function go(view) {
  S.view = view;
  document.querySelectorAll('nav button').forEach(b => b.classList.toggle('active', b.dataset.view === view));
  document.querySelectorAll('.view').forEach(s => { s.hidden = s.id !== 'view-' + view; });
  window.scrollTo(0, 0);
  if (view.endsWith('-res')) await ensureResults();
  if (S.view === view) renderView(view);
}

// ---------------------------------------------------------------- tools
async function toggleReference() {
  if (S.reference) { S.reference = null; updateRefUi(); renderView(S.view); return; }
  await ensureResults();
  if (!S.res.ok) { $('ref-state').textContent = 'Correct the data errors first.'; $('ref-state').classList.add('open'); return; }
  S.reference = JSON.parse(JSON.stringify(S.res));
  S.reference.name = S.vehicle.name || 'car';
  S.reference.input = JSON.parse(JSON.stringify(S.vehicle));
  updateRefUi(); renderView(S.view);
}
function refreshRefForm() {
  const rv = S.reference && S.reference.input ? S.reference.input : null;
  showReferenceValues($('form-car'), rv, S.vehicle);
  $('ref-car-tag').textContent = rv ? 'Reference: ' + S.reference.name : '';
}
function updateRefUi() {
  // While the button slides away from the pointer, the hover look is held back.
  const tools = document.querySelector('.tools'); tools.classList.add('settling'); clearTimeout(updateRefUi.t); updateRefUi.t = setTimeout(() => tools.classList.remove('settling'), 700);
  $('btn-ref').textContent = S.reference ? 'Clear reference' : 'Fix reference';
  $('btn-ref').classList.toggle('on', !!S.reference);
  refreshRefForm();
  if (S.reference) $('ref-state').textContent = 'Reference: ' + S.reference.name;   // the text stays while it fades out
  $('ref-state').classList.toggle('open', !!S.reference);
}

async function runWhatIf() {
  if (!S.res || !S.res.circuit) return;
  const bar = $('wi-bar'), fill = bar.firstElementChild;
  $('card-whatif').hidden = false; $('wi-body').innerHTML = ''; bar.hidden = false; fill.style.width = '0';
  $('card-whatif').scrollIntoView({ behavior: 'smooth', block: 'start' });
  const token = ++whatifToken;
  $('btn-whatif').disabled = true; whatifRunning = true; setToolsBusy(true);
  let r;
  try {
    r = await callJson('whatif', [JSON.stringify(S.vehicle), JSON.stringify(envForCalc()), JSON.stringify(circuitForCalc()), 0.05],
      (d, t) => { fill.style.width = (100 * d / t).toFixed(0) + '%'; });
  } catch (e) {
    if (token !== whatifToken) return;          // cancelled: the cancelling code has already tidied up
    r = { ok: false, issues: [{ message: 'Unexpected error: ' + e.message }] };
  }
  if (token !== whatifToken) return;
  bar.hidden = true; $('btn-whatif').disabled = false; whatifRunning = false; setToolsBusy(false);
  if (S.dirty) { S.whatif = null; $('card-whatif').hidden = true; return; }   // the data changed while it was running: the result no longer applies
  S.whatif = r;
  renderWhatif(S.whatif);
}

// ---------------------------------------------------------------- start
async function boot() {
  const q = new URLSearchParams(location.search).get('pyodide');
  const indexURL = new URL(q || 'https://cdn.jsdelivr.net/pyodide/v314.0.7/full/', location.href).href.replace(/\/?$/, '/');
  bootExtra = { indexURL, zipURL: new URL('vdcalc.zip', location.href).href };
  await call('__boot__', [], null, bootExtra);
  S.reg = await callJson('registry', []);
  const [carText, circText] = await Promise.all([fetch('examples/mx5_nd2.json', { cache: 'no-store' }).then(r => r.text()), fetch('examples/example_circuit.csv', { cache: 'no-store' }).then(r => r.text())]);
  S.vehicle = (await callJson('parse_vehicle', [carText])).data;
  const c = await callJson('parse_circuit', [circText]);
  S.circuit = { ...c.circuit, reverse: false, start: 0 }; S.circuitName = 'Example circuit (invented)';
  buildCarForm();
  const envGroup = S.reg.groups.find(g => g.group === 'environment');
  const envBody = $('env-body'); envBody.innerHTML = '';
  for (const p of envGroup.params) envBody.append(makeField(p, S.env, () => { envChanged = true; markDirty(); }));
  await refreshCircuit();
  await runValidate();
  $('start-btn').disabled = false; $('start-btn').textContent = 'Start'; $('start-note').hidden = true;
}

function wire() {
  document.querySelectorAll('nav button').forEach(b => b.addEventListener('click', () => go(b.dataset.view)));
  $('start-btn').addEventListener('click', () => { $('start').classList.add('gone'); go('car-data'); });
  $('btn-load-car').addEventListener('click', () => $('file-car').click());
  $('file-car').addEventListener('change', e => { if (e.target.files[0]) loadCar(e.target.files[0]); e.target.value = ''; });
  $('btn-save-car').addEventListener('click', saveCar);
  $('btn-curve').addEventListener('click', () => $('file-curve').click());
  $('file-curve').addEventListener('change', e => { if (e.target.files[0]) importCurve(e.target.files[0]); e.target.value = ''; });
  $('btn-load-circuit').addEventListener('click', () => $('file-circuit').click());
  $('file-circuit').addEventListener('change', e => { if (e.target.files[0]) importCircuit(e.target.files[0]); e.target.value = ''; });
  $('btn-reverse').addEventListener('click', async () => { S.circuit.reverse = !S.circuit.reverse; await refreshCircuit(); markDirty(); });
  $('map-data').addEventListener('click', async e => {
    if (!S.mapPick) return;
    const r = e.target.getBoundingClientRect();
    S.circuit.start = S.mapPick(e.clientX - r.left, e.clientY - r.top);
    await refreshCircuit(); markDirty();
  });
  $('btn-ref').addEventListener('click', toggleReference);
  $('btn-whatif').addEventListener('click', runWhatIf);
  $('map-color').addEventListener('change', () => renderView('circuit-res'));
  $('gg-speed').addEventListener('input', e => { S.ggIdx = +e.target.value; drawGG(); });
  document.addEventListener('click', e => { if (!$('env').contains(e.target) && !$('env').classList.contains('closed')) $('env-toggle').click(); });
  $('env-toggle').addEventListener('click', async () => {
    const c = $('env').classList.toggle('closed'); $('env-toggle').textContent = c ? 'Environment ▸' : 'Environment ▾';
    // Leaving the panel after changing a value: recalculate the page that is showing.
    if (c && envChanged) { envChanged = false; if (S.view.endsWith('-res')) { await ensureResults(); renderView(S.view); } }
  });
  let t = null;
  window.addEventListener('resize', () => { clearTimeout(t); t = setTimeout(() => renderView(S.view), 200); });
}

wire();
boot().catch(e => { $('start-btn').textContent = 'Could not load the engine'; $('start-note').hidden = false; $('start-note').textContent = String(e.message || e); console.error(e); });
