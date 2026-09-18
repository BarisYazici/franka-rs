// Schema-driven controls. Every bound, default, unit and detent comes from the owner's published
// schema entry (`spec`); nothing here knows a number. Sliders run on an integer position 0..SLIDER_N
// mapped linearly or logarithmically onto [min, max]; the paired entry carries the schema min/max
// as its own attributes so the DOM itself says where the bounds came from.
const SLIDER_N = 1024;
const PERCENT = 100;

function sliderMap(spec, i) {
  const min = Array.isArray(spec.min) ? spec.min[i] : spec.min;
  const max = Array.isArray(spec.max) ? spec.max[i] : spec.max;
  const smax = spec.slider_max != null ? Math.min(max, spec.slider_max) : max; // narrower slider, same schema max
  if (spec.scale === 'log') {
    const l0 = Math.log(min), l1 = Math.log(smax);
    return { min, max, smax,
      toPos: v => Math.round(SLIDER_N * (Math.log(Math.min(Math.max(v, min), smax)) - l0) / (l1 - l0)),
      toVal: p => Math.exp(l0 + (l1 - l0) * p / SLIDER_N) };
  }
  return { min, max, smax,
    toPos: v => Math.round(SLIDER_N * (Math.min(Math.max(v, min), smax) - min) / (smax - min)),
    toVal: p => min + (smax - min) * p / SLIDER_N };
}

// About three significant figures, at most three decimals; not a bound of anything.
function fmt(v, spec) {
  if (typeof v !== 'number') return String(v);
  if (spec && spec.type === 'u32') return String(Math.round(v));
  const a = Math.abs(v);
  if (a === 0) return '0';
  if (a < 1e-2) return v.toExponential(2);
  return v.toFixed(Math.min(3, Math.max(0, 2 - Math.floor(Math.log10(a)))));
}

function el(tag, cls, text) {
  const e = document.createElement(tag);
  if (cls) e.className = cls;
  if (text != null) e.textContent = text;
  return e;
}

// One scalar slider+entry pair (also used per element of a small array).
function scalarControl(spec, i, onInput) {
  const m = sliderMap(spec, i);
  const wrap = el('span', 'ctl');
  const range = el('input'); range.type = 'range'; range.min = 0; range.max = SLIDER_N;
  const entry = el('input'); entry.type = 'number';
  entry.min = m.min; entry.max = m.max; entry.step = spec.type === 'u32' ? 1 : 'any';
  entry.dataset.boundSource = 'schema';
  const unit = Array.isArray(spec.unit) ? spec.unit[i] : spec.unit;
  const detent = el('span', 'hint');
  let value = m.min;
  const show = () => {
    range.value = m.toPos(value);
    entry.value = spec.type === 'u32' ? Math.round(value) : +fmt(value, spec) === value ? value : fmt(value, spec);
    const off = spec.off_at != null && value === (Array.isArray(spec.off_at) ? spec.off_at[i] : spec.off_at);
    detent.textContent = off ? 'off' : (unit || '');
  };
  range.oninput = () => { value = spec.type === 'u32' ? Math.round(m.toVal(+range.value)) : m.toVal(+range.value); show(); onInput(value); };
  entry.onchange = () => { const v = +entry.value; if (Number.isFinite(v)) { value = v; show(); onInput(value); } };
  wrap.append(range, entry, detent);
  return { wrap, range, entry, get: () => value, set: v => { value = v; show(); }, map: m };
}

// Builds the control for one schema entry. Returns {row, get, set, spec, error, flash, slew, disable}.
function buildControl(name, spec, onEdit) {
  const row = el('div', 'param');
  const label = el('div', 'name');
  label.append(document.createTextNode(name.replace(/_/g, ' ')));
  if (spec.unit && !Array.isArray(spec.unit)) label.append(el('span', 'unit', spec.unit));
  label.append(el('span', 'dot'));
  label.title = [spec.policy && `policy: ${spec.policy}`,
    spec.policy === 'step_up_gate_down' && '(raising takes effect at once, lowering is ramped by the node)',
    spec.slew_tau_s && `slew τ ${spec.slew_tau_s} s`, spec.note].filter(Boolean).join('\n');
  const ctl = el('div', 'ctl');
  const err = el('div', 'err');
  row.append(label, ctl, err);
  const arr = /\[(\d+)\]$/.exec(spec.type);
  const n = arr ? +arr[1] : 0;
  let parts = [], get, set;
  if (spec.type === 'bool') {
    const cb = el('input'); cb.type = 'checkbox';
    cb.onchange = () => onEdit(name, cb.checked);
    ctl.append(cb);
    get = () => cb.checked; set = v => { cb.checked = !!v; };
  } else if (n && n <= 3) {
    for (let i = 0; i < n; i++) {
      const c = scalarControl(spec, i, () => onEdit(name, get()));
      const axis = spec.labels ? spec.labels[i] : spec.norm ? ['v', 'a', 'j'][i] : String(i);
      ctl.append(el('span', 'axis', axis), c.wrap);
      parts.push(c);
    }
    if (spec.norm) { const h = el('span', 'hint'); ctl.append(h); parts.norm = h; }
    get = () => parts.map(p => p.get());
    set = v => { v.forEach((x, i) => parts[i].set(x)); showNorm(); };
    const showNorm = () => { if (parts.norm) parts.norm.textContent = 'norm · per axis ' + get().map(x => fmt(x / Math.sqrt(3))).join(' / '); };
    const inner = onEdit; onEdit = (nm, v) => { showNorm(); inner(nm, v); };
  } else if (n) {
    for (let i = 0; i < n; i++) {
      const e = el('input'); e.type = 'number'; e.min = spec.min[i]; e.max = spec.max[i]; e.step = 'any';
      e.dataset.boundSource = 'schema';
      e.title = spec.labels ? spec.labels[i] : `${name}[${i}]`;
      e.onchange = () => onEdit(name, get());
      ctl.append(e); parts.push(e);
    }
    get = () => parts.map(e => +e.value);
    set = v => v.forEach((x, i) => { parts[i].value = x; });
  } else {
    const c = scalarControl(spec, 0, v => onEdit(name, v));
    ctl.append(c.wrap); parts.push(c);
    get = () => c.get(); set = v => c.set(v);
  }
  const slew = el('span', 'slew'); ctl.append(slew);
  const inputs = () => Array.from(row.querySelectorAll('input'));
  return {
    row, spec, get, set, ctl,
    error: msg => { err.textContent = msg || ''; },
    flash: () => inputs().forEach(i => { i.classList.remove('flash'); void i.offsetWidth; i.classList.add('flash'); }),
    slew: frac => { slew.textContent = frac > 0 ? `slewing ${Math.round(frac * PERCENT)} %` : ''; },
    disable: d => inputs().forEach(i => { i.disabled = d; }),
  };
}

// Group order is presentation only; groups the schema names that are not listed go last.
const GROUP_ORDER = ['gains', 'feedforward', 'ik', 'envelope', 'teleop'];

function groupsOf(schema) {
  const g = {};
  for (const [name, spec] of Object.entries(schema.params)) (g[spec.group || 'other'] ||= []).push(name);
  return Object.keys(g).sort((a, b) => (GROUP_ORDER.indexOf(a) + 1 || 99) - (GROUP_ORDER.indexOf(b) + 1 || 99)).map(k => [k, g[k]]);
}

// A group is behind the lock when the schema marks any of its fields as widening the envelope.
function groupLocked(schema, names) {
  return names.some(n => schema.params[n].danger === 'confirm_above');
}
