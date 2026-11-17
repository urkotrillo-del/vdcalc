// Input forms generated from the registry of the core.
const getPath = (o, k) => k.split('.').reduce((n, p) => (n == null ? undefined : n[p]), o);
function setPath(o, k, v) {
  const parts = k.split('.');
  let n = o;
  for (const p of parts.slice(0, -1)) { if (typeof n[p] !== 'object' || n[p] === null) n[p] = {}; n = n[p]; }
  n[parts[parts.length - 1]] = v;
}

const GROUP_TITLE = {};
// Notes that belong to a choice, not to a validation: they stay while that choice is selected.
const CHOICE_NOTES = {
  'powertrain.curve_measured_at': {
    wheel_inertial: 'An inertial dyno already subtracts part of the rotating inertia, measured at the sweep acceleration. The model also applies the inertia of each gear, so the drive force may be slightly underestimated (a few per cent in the highest gears).',
  },
};
const prettyUnit = u => (u === '-' ? '' : u.replace(/\*/g, '·').replace(/([a-zA-Z])2\b/g, '$1²').replace(/([a-zA-Z])3\b/g, '$1³'));
let PARAM_BY_KEY = {};

function fieldValue(p, data) {
  const v = getPath(data, p.key);
  return v === undefined || v === null ? '' : (Array.isArray(v) ? v.join(', ') : v);
}

function isVisible(p, data) {
  if (!p.visible_if) return true;
  const [other, values] = p.visible_if;
  let v = getPath(data, other);
  if (v === undefined || v === null) v = PARAM_BY_KEY[other] ? PARAM_BY_KEY[other].default : undefined;
  return values.includes(v);
}

function makeField(p, data, onChange) {
  const row = document.createElement('div');
  row.className = 'field';
  row.dataset.key = p.key;
  const label = document.createElement('label');
  const text = document.createElement('span');
  text.textContent = p.label;
  const tip = document.createElement('span');
  tip.className = 'tip'; tip.tabIndex = 0; tip.textContent = 'i';
  tip.dataset.tip = p.help + (p.min !== null && p.max !== null ? ` Allowed: ${p.min} to ${p.max}.` : '') +
    (p.typical ? ` Usual: ${p.typical[0]} to ${p.typical[1]}.` : '');
  label.append(text, tip);
  let input;
  if (p.kind === 'choice') {
    input = document.createElement('select');
    for (const c of p.choices) { const o = document.createElement('option'); o.value = c; o.textContent = c; input.append(o); }
    input.value = getPath(data, p.key) ?? p.default ?? p.choices[0];
  } else {
    input = document.createElement('input');
    input.type = p.kind === 'float' ? 'number' : 'text';
    if (p.kind === 'float') input.step = 'any';
    input.value = fieldValue(p, data);
    if (p.default !== null && p.default !== undefined) input.placeholder = String(p.default);
    else if (p.optional) input.placeholder = 'optional';
  }
  input.id = 'f-' + p.key;
  label.htmlFor = input.id;
  const unit = document.createElement('span');
  unit.className = 'unit';
  unit.textContent = prettyUnit(p.unit);
  const msg = document.createElement('div');
  msg.className = 'fmsg';
  const ref = document.createElement('span');
  ref.className = 'refv';
  row.append(label, input, unit, ref, msg);
  if (p.kind !== 'float' && p.kind !== 'choice') row.classList.add('wide');
  if (p.kind === 'choice') row.classList.add('choice');
  const notes = CHOICE_NOTES[p.key];
  const note = document.createElement('div');
  note.className = 'fnote';
  const showNote = () => { note.textContent = (notes && notes[input.value]) || ''; };
  if (notes) { row.append(note); showNote(); }
  input.addEventListener('input', () => {
    showNote();
    let v;
    if (p.kind === 'choice') v = input.value;
    else if (p.kind === 'float') v = input.value === '' ? null : Number(input.value);
    else v = input.value.trim() === '' ? [] : input.value.split(/[\s,;]+/).filter(Boolean).map(t => (isNaN(Number(t)) ? t : Number(t)));
    setPath(data, p.key, v);
    onChange(p.key);
  });
  return row;
}

// Builds one card per group inside `container`. `groups` comes from webapi.registry().
function buildForm(container, groups, data, onChange, skip = []) {
  container.innerHTML = '';
  PARAM_BY_KEY = {};
  for (const g of groups) for (const p of g.params) PARAM_BY_KEY[p.key] = p;
  const name = document.createElement('div');
  name.className = 'card';
  name.innerHTML = '<h3>Name</h3>';
  const nameRow = document.createElement('div');
  nameRow.className = 'field';
  nameRow.style.gridTemplateColumns = '1fr';
  const nameInput = document.createElement('input');
  nameInput.type = 'text'; nameInput.value = data.name || '';
  nameInput.addEventListener('input', () => { data.name = nameInput.value; onChange('name'); });
  nameRow.append(nameInput);
  name.append(nameRow);
  container.append(name);
  for (const g of groups) {
    if (skip.includes(g.group)) continue;
    const card = document.createElement('div');
    card.className = 'card';
    card.innerHTML = `<h3>${g.label}</h3>`;
    for (const p of g.params) card.append(makeField(p, data, onChange));
    container.append(card);
  }
  updateVisibility(container, data);
}

function updateVisibility(container, data) {
  container.querySelectorAll('.field[data-key]').forEach(row => {
    const p = PARAM_BY_KEY[row.dataset.key];
    row.hidden = !isVisible(p, data);
  });
}

// Shows the issues of the validation next to the fields; returns the ones without a field.
function showIssues(container, issues) {
  container.querySelectorAll('.field').forEach(r => { r.classList.remove('bad', 'warn'); const m = r.querySelector('.fmsg'); if (m) m.textContent = ''; });
  const loose = [];
  for (const i of issues) {
    if (CHOICE_NOTES[i.key]) continue;   // shown by the field itself, whatever else is wrong
    const row = container.querySelector(`.field[data-key="${i.key}"]`);
    if (row && !row.hidden) {
      const cls = i.level === 'error' ? 'bad' : 'warn';
      if (!row.classList.contains('bad')) row.classList.add(cls);
      const m = row.querySelector('.fmsg');
      m.textContent = (m.textContent ? m.textContent + ' ' : '') + i.message;
    } else loose.push(i);
  }
  return loose;
}

function issueHtml(list) {
  return list.map(i => `<div class="issue ${i.level}">${escapeHtml(i.message)}</div>`).join('');
}
function escapeHtml(s) { return String(s).replace(/[&<>"]/g, c => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' }[c])); }

// Shows the value of the reference car next to each field (same keys as the form).
function showReferenceValues(container, refVehicle, data) {
  container.classList.toggle('withref', !!refVehicle);
  container.querySelectorAll('.field[data-key]').forEach(row => {
    const p = PARAM_BY_KEY[row.dataset.key], span = row.querySelector('.refv');
    if (!span) return;
    if (!refVehicle) { span.textContent = ''; return; }
    const v = fieldValue(p, refVehicle), cur = fieldValue(p, data);
    span.textContent = v === '' ? '–' : String(v);
    span.classList.toggle('diff', String(v) !== String(cur) && !(v === '' && cur === ''));
  });
}
