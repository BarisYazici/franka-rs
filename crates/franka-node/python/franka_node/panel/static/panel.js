// Panel state: load from the owners, edit locally, Apply as one batch (node then teleop),
// optimistic versioning, restart/reflex/unreachable surfaces, presets, before/after via the bridge.
const OWNERS = ['node', 'teleop'];
const LOCK_S = 60;
const S = {
  arm: null, clientId: 1e3 + Math.floor(Math.random() * 9e3), schemas: {}, loaded: {}, edits: {},
  controls: {}, sections: {}, reach: {}, inflight: false, lockUntil: 0, lastApplied: {}, presets: [], es: null,
};
const $ = id => document.getElementById(id);

// ---- loading -------------------------------------------------------------------------------
async function loadArms() {
  const r = await api.get('/api/arms');
  const arms = Object.keys(r.arms || {});
  $('arms').innerHTML = arms.map(a => `<option>${a}</option>`).join('');
  const want = new URLSearchParams(location.search).get('arm');
  S.arm = arms.includes(want) ? want : arms[0];
  // Nothing to show has two causes worth telling apart: no zenoh session at all, or a session
  // whose owners did not answer the discovery query.
  if (!S.arm) { $('title').textContent = r.linked ? 'franka · on the bus, but no owner answered' : 'franka · no zenoh node on the bus'; return; }
  $('arms').value = S.arm;
  S.edits = {}; S.loaded = {}; S.schemas = {}; S.lastApplied = {}; disarm();  // nothing of the previous arm survives
  await loadAll();
  if (S.es) S.es.close();
  S.es = api.events(S.arm, { hello: onHello, current: onCurrent, status: onStatus, metrics: onMetrics, close: onSseClose, open: onSseOpen });
}

async function loadAll() {
  await Promise.all(OWNERS.map(loadOwner));
  $('title').textContent = `franka ${S.arm}`;
  refreshPresets();
}

async function loadOwner(owner) {
  const [schema, cur] = await Promise.all([api.get(`/api/${S.arm}/${owner}/schema`), api.get(`/api/${S.arm}/${owner}/params`)]);
  if (!schema.params) { S.reach[owner] = false; disarm(); renderReach(); if (schema.error) showError(`${owner} schema: ${schema.error}`); return; }
  S.schemas[owner] = schema;
  render(owner);
  if (cur.params) { S.loaded[owner] = cur; S.edits[owner] = {}; showLoaded(owner); }
  S.reach[owner] = !!cur.params; renderReach();
}

function showLoaded(owner) {
  disarm();
  const cur = S.loaded[owner];
  for (const [name, c] of Object.entries(S.controls[owner])) {
    if (name in cur.params) c.set(cur.params[name]);
    c.error(''); c.row.classList.remove('edited');
    c.slew((cur.slewing || {})[name] || 0);
  }
  updateAdvisories(); updateButtons(); renderVersions();
}

// ---- rendering -----------------------------------------------------------------------------
function render(owner) {
  const schema = S.schemas[owner];
  for (const sec of Object.values(S.sections[owner] || {})) sec.remove();
  S.sections[owner] = {}; S.controls[owner] = {};
  let first = true;
  for (const [group, names] of groupsOf(schema)) {
    const sec = el('section'); sec.dataset.owner = owner; sec.dataset.group = group;
    const h = el('h2', null, `${group} (${owner})`); h.append(el('span', 'grow'));
    sec.append(h);
    const controls = names.map(name => {
      const c = buildControl(name, schema.params[name], onEdit.bind(null, owner));
      sec.append(c.row); S.controls[owner][name] = c; return c;
    });
    if (groupLocked(schema, names)) {
      const b = el('button', null, `🔒 unlock (${LOCK_S} s)`); b.onclick = () => lock(sec, controls, b, false); h.append(b);
      h.append(el('span', 'caveat', 'confirm_above fields need a second click on Apply'));
      lock(sec, controls, b, true);
    }
    const adv = el('div', 'advisories'); sec.append(adv); sec.adv = adv;
    if (owner === 'node' && schema.derived && first) sec.append(renderDerived(schema.derived));
    first = false;
    $('groups').append(sec); S.sections[owner][group] = sec;
  }
}

function renderDerived(d) {
  const box = el('div', 'derived');
  const items = { 'leash': d.leash && `${fmt(d.leash.translation * 1e3)} mm / ${fmt(d.leash.rotation)} rad`, 'max lead': d.max_lead != null && `${fmt(d.max_lead * 1e3)} mm / ${fmt(d.max_lead_rotation)} rad`,
    'max step': d.max_step != null && `${fmt(d.max_step * 1e3)} mm / ${fmt(d.max_step_rotation)} rad`, 'rate': d.rate_hz && `${d.rate_hz} Hz`, 'torque low-pass': d.cutoff_frequency && `${d.cutoff_frequency} Hz` };
  box.append(el('span', null, 'read-only (restart to change): '));
  for (const [k, v] of Object.entries(items)) if (v) { const s = el('span', null, k + ' '); s.append(el('b', null, v)); box.append(s); }
  return box;
}

// Disables the inputs themselves (keyboard included), not just the pointer; relocks after LOCK_S.
function lock(sec, controls, btn, locked) {
  sec.classList.toggle('locked', locked);
  controls.forEach(c => c.disable(locked));
  btn.textContent = `🔒 unlock (${LOCK_S} s)`;
  if (locked) return;
  const until = Date.now() + LOCK_S * 1e3;
  const tick = () => {
    const left = Math.ceil((until - Date.now()) / 1e3);
    if (left <= 0) lock(sec, controls, btn, true); else { btn.textContent = `🔓 ${left} s`; setTimeout(tick, 1e3); }
  };
  tick();
}

function showError(msg) { $('errbar').textContent = msg; $('errbar').classList.add('show'); }

// ---- editing -------------------------------------------------------------------------------
function onEdit(owner, name, value) {
  const loaded = S.loaded[owner] && S.loaded[owner].params[name];
  if (JSON.stringify(value) === JSON.stringify(loaded)) delete S.edits[owner][name]; else S.edits[owner][name] = value;
  S.controls[owner][name].row.classList.toggle('edited', name in S.edits[owner]);
  S.controls[owner][name].error('');
  disarm();
  updateAdvisories(); updateButtons();
}

// The red button is armed for one exact set of edits; anything that changes what would be sent
// (an edit, a revert, a reload, a preset recall) puts it back to a first click.
function disarm() {
  $('confirmbar').classList.remove('show'); $('apply').classList.remove('danger'); $('apply').onclick = () => apply(null);
}

function pending(owner) { return S.loaded[owner] ? { ...S.loaded[owner].params, ...S.edits[owner] } : null; }

function updateAdvisories() {
  const node = pending('node'), nd = S.schemas.node && S.schemas.node.derived;
  const list = [];
  if (node && nd) {
    const cap = feedforwardCap(node.velocity_feedforward_gain, node.cartesian_stiffness, node.budget, nd);
    if (cap) list.push({ ...cap, group: 'feedforward' });
    list.push(...nodeAdvisories(node, nd, S.loaded.node.params));
  }
  const tel = pending('teleop');
  if (tel && S.schemas.teleop) list.push(...teleopAdvisories(tel, S.schemas.teleop, nd, node));
  for (const owner of OWNERS) for (const sec of Object.values(S.sections[owner] || {})) {
    sec.adv.innerHTML = '';
    for (const a of list) if (a.group === sec.dataset.group) sec.adv.append(el('div', `adv ${a.severity}`, a.text));
  }
}

function editCount() { return OWNERS.reduce((n, o) => n + Object.keys(S.edits[o] || {}).length, 0); }

function updateButtons() {
  const n = editCount();
  const sendable = OWNERS.every(o => !Object.keys(S.edits[o] || {}).length || S.reach[o]);
  $('apply').textContent = n ? `Apply ${n}` : 'Apply';
  $('apply').disabled = !n || S.inflight || !sendable || !S.es || S.es.readyState !== 1;
  $('revert').disabled = !n;
}

// ---- apply ---------------------------------------------------------------------------------
// `confirmed` is the confirmFor() map the operator agreed to with the second click on the red
// button, or null for a first click. A map agreed for other edits counts as null.
async function apply(confirmed) {
  if (confirmed && confirmed.key !== editsKey(S.edits)) confirmed = null;
  const need = {};
  for (const o of OWNERS) if (S.edits[o] && S.loaded[o]) need[o] = crossings(S.schemas[o], S.edits[o], S.loaded[o].params);
  const names = OWNERS.flatMap(o => need[o] || []);
  if (names.length && !confirmed) {
    $('confirmbar').textContent = `crossing confirm_above on ${names.join(', ')} — this widens what the arm may do. Click Apply again to confirm.`;
    $('confirmbar').classList.add('show'); $('apply').classList.add('danger'); $('apply').onclick = () => apply(confirmFor(need, S.edits)); return;
  }
  disarm();
  const body = applyBody(OWNERS, S.schemas, S.loaded, S.edits, S.clientId, confirmed);
  S.inflight = true; updateButtons();
  let r;
  try { r = await api.post(`/api/${S.arm}/apply`, body); } catch (e) { showError(`apply failed: ${e.message} — nothing changed`); return; } finally { S.inflight = false; updateButtons(); }
  for (const o of OWNERS) if (r[o]) handleReply(o, r[o], body[o]);
  updateButtons();
}

function handleReply(owner, r, sent) {
  const ctls = S.controls[owner];
  if (r.ok) {
    S.lastApplied[owner] = { ...S.loaded[owner].params, ...sent.params };
    S.loaded[owner] = { ...S.loaded[owner], version: r.version, params: r.params, slewing: r.slewing || {} };
    S.edits[owner] = {}; showLoaded(owner);
    for (const c of r.clamped || []) {
      ctls[c.field].flash();
      ctls[c.field].error(`snapped to ${fmt(c.stored)} (asked ${fmt(c.requested)}) — schema out of date, reloading`);
    }
    if ((r.clamped || []).length) api.get(`/api/${S.arm}/${owner}/schema`).then(s => { if (s.params) { S.schemas[owner] = s; render(owner); showLoaded(owner); } });
    fadeSlew(owner);
    return;
  }
  const where = r.field && ctls[r.field] ? ctls[r.field] : null;
  if (where) where.error(`${r.reason}: ${r.error}`); else showError(`${owner} ${r.reason}: ${r.error}`);
  if (r.reason === 'stale') showStale(owner, r.version, 'the owner', true);
  if (r.reason === 'needs_confirm') {
    $('confirmbar').textContent = `owner requires confirm on ${r.field} — Apply again to confirm`; $('confirmbar').classList.add('show');
    $('apply').classList.add('danger'); $('apply').onclick = () => apply(confirmFor({ [owner]: [r.field] }, S.edits));
  }
}

const SLEW_VISIBLE = 2e-2; // remaining fraction below which the ghost is not worth drawing

// `slewing` is the fraction of the step still to go (1 at acceptance); decay it locally with the
// schema's tau until the next `current` refreshes it.
function fadeSlew(owner) {
  const slew = S.loaded[owner].slewing || {};
  const t0 = Date.now();
  const tick = () => {
    let any = false;
    for (const [name, f0] of Object.entries(slew)) {
      const tau = S.schemas[owner].params[name].slew_tau_s * 1e3;
      const f = tau ? f0 * Math.exp(-(Date.now() - t0) / tau) : 0;
      S.controls[owner][name].slew(f > SLEW_VISIBLE ? f : 0); any = any || f > SLEW_VISIBLE;
    }
    if (any) requestAnimationFrame(tick);
  };
  tick();
}

function revert() {
  for (const o of OWNERS) if (S.loaded[o]) { S.edits[o] = {}; showLoaded(o); }
  $('errbar').classList.remove('show');
}

// ---- stream events -------------------------------------------------------------------------
function onHello(h) { S.reach = h.reach || {}; renderReach(); if (h.status) onStatus(h.status); if (h.channels) renderChannels(h.channels); }
function onSseOpen() { $('header').classList.remove('offline'); updateButtons(); }
function onSseClose() { $('header').classList.add('offline'); $('phase').textContent = 'bridge stream lost — edits kept, Apply disabled'; updateButtons(); }

function onCurrent(c) {
  const owner = c.owner; if (!S.loaded[owner]) return;
  S.reach[owner] = true; renderReach();
  const was = S.loaded[owner];
  if (c.boot_id !== was.boot_id && c.version === 0) {
    showResetBanner(owner, `${owner} restarted — running TOML values. `, was.version);
    S.loaded[owner] = c; S.edits[owner] = {}; showLoaded(owner); return;
  }
  if (was.origin && !c.origin && c.version !== was.version) {
    // the node forgets a session's tuning when the session ends; nothing else on the wire says so
    showResetBanner(owner, `${owner}: the session ended — the arm is back on the TOML values. `, was.version);
    S.loaded[owner] = c; S.edits[owner] = {}; showLoaded(owner); return;
  }
  if (c.version !== was.version) {
    const mine = c.origin && c.origin.by === S.clientId;
    if (mine || !editCount()) { S.loaded[owner] = c; showLoaded(owner); }
    else showStale(owner, c.version, c.origin ? c.origin.by : '?', false);
  } else for (const [name, f] of Object.entries(c.slewing || {})) S.controls[owner][name] && S.controls[owner][name].slew(f);
}

function showResetBanner(owner, text, lastVersion) {
  $('restartbar').textContent = text;
  if (S.lastApplied[owner]) {
    const b = el('button', null, `Re-apply last set v${lastVersion}`);
    // goes through the normal confirm gate: this is exactly when a re-widened envelope needs a look
    b.onclick = () => { for (const [n, v] of Object.entries(S.lastApplied[owner])) { S.controls[owner][n].set(v); onEdit(owner, n, v); } apply(null); };
    $('restartbar').append(b);
  }
  $('restartbar').classList.add('show');
}

function showStale(owner, version, by, withDiff) {
  const bar = $('stalebar'); bar.innerHTML = `${owner} changed by ${by} → v${version} while you were editing. `;
  const reload = el('button', null, 'Reload'); reload.onclick = () => { S.edits[owner] = {}; loadOwner(owner).then(() => bar.classList.remove('show')); };
  const keep = el('button', null, 'Keep my edits');
  keep.onclick = async () => { const kept = { ...S.edits[owner] }; await loadOwner(owner); for (const [n, v] of Object.entries(kept)) { S.controls[owner][n].set(v); onEdit(owner, n, v); } bar.classList.remove('show'); };
  bar.append(reload, keep); bar.classList.add('show');
  if (withDiff) api.get(`/api/${S.arm}/${owner}/params`).then(cur => { if (cur.params) bar.append(diffTable(S.edits[owner], cur.params, 'my edit', `v${cur.version}`)); });
}

function onMetrics(m) {
  live.push(m);
  if (m.reach) { S.reach = m.reach; renderReach(); }
  if (m.channels) renderChannels(m.channels);
}

// The node's status entry merged with the state-derived condition (reflex lives there).
function onStatus(st) {
  if (st.phase == null) return;  // nothing from either channel yet; renderChannels says so
  $('header').classList.toggle('reflex', !!st.reflex);
  $('phase').textContent = (st.reflex ? `REFLEX (${st.robot_mode_name || st.robot_mode}${st.has_errors ? ', errors' : ''}) — values you apply now are stored, not tested · ` : '')
    + `phase ${st.phase} · mode ${st.robot_mode_name || st.robot_mode || '?'} · holder ${st.holder}`
    + (['idle', 'acquired'].includes(st.phase) ? ' · no session: edits are stored, slew happens at enable' : '');
}

// Says when the page cannot know the arm's condition, instead of showing nothing.
function renderChannels(ch) {
  const parts = [];
  if (ch.state_age_s == null) parts.push(ch.state_decode_failures ? `STATE UNDECODABLE (${ch.state_decode_failures} samples) — reflex and metrics cannot be shown` : 'no state from the node yet — reflex cannot be shown');
  if (ch.status_age_s == null) parts.push('no status from the node yet');
  $('channels').textContent = parts.join(' · ');
  $('header').classList.toggle('blind', ch.state_age_s == null);
}

function renderReach() {
  for (const o of OWNERS) for (const sec of Object.values(S.sections[o] || {})) {
    sec.classList.toggle('unreachable', !S.reach[o]);
    sec.querySelector('h2').title = S.reach[o] ? '' : `${o} unreachable`;
  }
  updateButtons();
}

function renderVersions() {
  $('versions').textContent = OWNERS.filter(o => S.loaded[o]).map(o => `${o} v${S.loaded[o].version}${S.loaded[o].dirty ? '*' : ''}`).join(' · ');
}

// ---- presets -------------------------------------------------------------------------------
async function refreshPresets() {
  const r = await api.get(`/api/${S.arm}/presets`); S.presets = r.presets || [];
  $('presets').innerHTML = '<option value="">Preset…</option>' + S.presets.map(p => `<option value="${p.name}">${p.name}${p.builtin ? ' (built-in)' : ''}</option>`).join('');
  $('presets').onchange = () => { $('recall').disabled = $('compare').disabled = !$('presets').value; };
}

function selectedPreset() { return S.presets.find(p => p.name === $('presets').value); }

function recallPreset() {
  const p = selectedPreset(); if (!p) return;
  for (const o of OWNERS) if (p[o] && S.loaded[o]) for (const [n, v] of Object.entries(p[o])) if (S.controls[o][n]) { S.controls[o][n].set(v); onEdit(o, n, v); }
}

function diffTable(a, b, ha, hb) {
  const t = el('table', 'diff'); t.innerHTML = `<tr><td>field</td><td>${ha}</td><td>${hb}</td></tr>`;
  for (const k of new Set([...Object.keys(a || {}), ...Object.keys(b || {})])) {
    const same = JSON.stringify(a && a[k]) === JSON.stringify(b && b[k]);
    const tr = el('tr'); tr.append(el('td', null, k), el('td', same ? '' : 'changed', JSON.stringify(a && a[k])), el('td', same ? '' : 'changed', JSON.stringify(b && b[k]))); t.append(tr);
  }
  return t;
}

function comparePreset() {
  const p = selectedPreset(); if (!p) return;
  const out = $('comparebody'); out.innerHTML = '';
  for (const o of OWNERS) if (p[o] && S.loaded[o]) { out.append(el('h2', null, o), diffTable(p[o], S.loaded[o].params, p.name, 'current')); }
  if (p.snapshot) out.append(el('div', 'caveat', 'snapshot at save: ' + JSON.stringify(p.snapshot)));
  $('compareout').style.display = '';
}

// ---- wiring --------------------------------------------------------------------------------
const live = new Live($('live'));
$('load').onclick = () => loadAll();
$('apply').onclick = () => apply(null);
$('revert').onclick = revert;
$('arms').onchange = () => { history.replaceState(null, '', `?arm=${$('arms').value}`); loadArms(); };
$('recall').onclick = recallPreset;
$('compare').onclick = comparePreset;
$('closecompare').onclick = () => { $('compareout').style.display = 'none'; };
$('savepreset').onclick = () => $('presetbox').classList.toggle('show');
$('pcancel').onclick = () => $('presetbox').classList.remove('show');
$('psave').onclick = async () => {
  const r = await api.post(`/api/${S.arm}/presets`, { name: $('pname').value, note: $('pnote').value, client_id: S.clientId });
  if (r.ok) { $('presetbox').classList.remove('show'); refreshPresets(); } else showError(r.error);
};
$('marker').onclick = () => api.post(`/api/${S.arm}/markers`, { label: 'manual' });
$('clearmarkers').onclick = () => api.del(`/api/${S.arm}/markers`);
loadArms();
