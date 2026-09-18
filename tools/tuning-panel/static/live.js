// Live tiles: sparklines over the last 60 s of bridge metrics, Apply markers, before/after.
const LIVE_S = 60;
const SPARK_W = 600;  // canvas pixels; CSS scales it
const SPARK_H = 40;
const TILES = [
  { key: 'j4_rms', label: 'j4 jitter', unit: 'rad/s', lines: true },
  { key: 'lag_ms', label: 'lag', unit: 'ms' },
  { key: 'track_p99_mm', label: 'track err p99', unit: 'mm' },
];

class Live {
  constructor(root) {
    this.root = root;
    this.hist = [];
    this.tiles = {};
    for (const t of TILES) {
      const box = el('div', 'metric');
      const val = el('span', 'val'); const head = el('span', 'head', t.label);
      const canvas = el('canvas'); canvas.width = SPARK_W; canvas.height = SPARK_H;
      const ba = el('div', 'ba'); const sub = el('div', 'sub');
      box.append(head, val, canvas, ba, sub);
      root.append(box);
      this.tiles[t.key] = { ...t, val, canvas, ba, sub };
    }
    const misc = el('div', 'metric'); this.misc = el('div', 'sub'); misc.append(el('span', 'head', 'context'), this.misc);
    root.append(misc);
  }

  push(m) {
    if (!m.ok) {
      this.misc.textContent = m.reason === 'stale_state' ? `NO STATE for ${fmt(m.age_s)} s — the numbers above are old` : 'no state from the node';
      this.root.classList.add('stale');
      return;
    }
    this.root.classList.remove('stale');
    this.hist.push(m);
    while (this.hist.length && this.hist[0].t < m.t - LIVE_S) this.hist.shift();
    for (const t of Object.values(this.tiles)) this.draw(t, m);
    const h = m.dq_headroom;
    const headroom = h ? `dq headroom j${h.joint + 1} ${Math.round(h.fraction * PERCENT)} % (release ${Math.round(h.release * PERCENT)} %)` : 'dq headroom: waiting for the node schema / teleop release fraction';
    this.misc.textContent = `${headroom} · peak j4 since marker ${fmt(m.j4_peak_since_marker)} · `
      + (m.valid ? '' : 'REFLEX: windows discarded · ') + `${m.markers.length} marker(s), shared by every open tab`;
  }

  draw(t, m) {
    const v = m[t.key];
    t.val.textContent = v == null ? '–' : `${fmt(v)} ${t.unit}`;
    const ctx = t.canvas.getContext('2d'), W = t.canvas.width, H = t.canvas.height;
    ctx.clearRect(0, 0, W, H);
    const xs = this.hist.map(h => h[t.key]).filter(x => x != null);
    const max = Math.max(1e-9, ...xs, t.lines ? Math.max(...Object.values(m.lines)) : 0);
    const x = tt => W * (1 - (m.t - tt) / LIVE_S);
    if (t.lines) for (const [name, y] of Object.entries(m.lines)) {
      ctx.strokeStyle = name === 'burst' ? '#e5564f' : name === 'sim_pass' ? '#e0a52a' : '#4fc37a';
      ctx.setLineDash([2, 3]); ctx.beginPath(); ctx.moveTo(0, H - H * y / max); ctx.lineTo(W, H - H * y / max); ctx.stroke();
    }
    ctx.setLineDash([]); ctx.strokeStyle = '#5aa9ff'; ctx.beginPath();
    let started = false;
    for (const h of this.hist) {
      if (h[t.key] == null) continue;
      const px = x(h.t), py = H - H * h[t.key] / max;
      started ? ctx.lineTo(px, py) : ctx.moveTo(px, py); started = true;
    }
    ctx.stroke();
    ctx.strokeStyle = '#c88cff';
    for (const mk of m.markers) { const px = x(mk.t); if (px >= 0) { ctx.beginPath(); ctx.moveTo(px, 0); ctx.lineTo(px, H); ctx.stroke(); } }
    t.ba.innerHTML = '';
    if (m.before && m.after) {
      const b = m.before[t.key], a = m.after[t.key], verdict = m.compare[t.key];
      const arrow = verdict === 'better' ? '↓ better' : verdict === 'worse' ? '↑ worse' : verdict === 'same' ? '≈ same' : 'collecting…';
      t.ba.append(el('span', 'hint', `before ${b == null ? '–' : fmt(b)} → after ${a == null ? '–' : fmt(a)}`), el('span', verdict || 'same', arrow));
      t.sub.textContent = `marker ${m.markers[m.markers.length - 1].label}: before = ${m.windows.before_s} s up to it, after = from ${m.windows.after_skip_s} s past it, rolling`;
    } else t.sub.textContent = 'apply a change to get a before/after';
  }
}
