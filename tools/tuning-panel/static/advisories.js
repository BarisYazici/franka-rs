// Advisories: never block, always visible. The numbers they compare against come from the node
// schema's `derived` block and the pending slider values; the few constants here are physics or
// operator policy (DESIGN-ui 2.2), not bounds.
const HAND_PEAK_MPS = 0.7;      // a fast hand, for the spatial_scale vs budget share note
const J4_BAND_HZ = 15;          // 3x the j4 dominant band, below which a cutoff eats the signal
const NEAR_BOUND = 0.10;        // teleop clamp within 10 % of the node bound
const ZETA_BAND = [0.3, 2];     // damping ratios outside this get a note (underdamped / sluggish)

// DESIGN-rt 7.3. The leash is fixed at startup; with the feedforward below 1 the steady speed on
// axis i is capped at leash / tau_i / (1 - gain), tau_i = Dx_i / Kx_i. The Cartesian preset's
// damping scales with sqrt(K/K_ref) while stiffness scales with K/K_ref (the node's rule), so
// tau_i = D0_i / (K0_i * sqrt(ratio)).
function feedforwardCap(gain, cartesianStiffness, budget, derived) {
  const p = derived && derived.cartesian_preset, leash = derived && derived.leash;
  if (!p || !leash || gain == null) return null;
  const ratio = cartesianStiffness / p.reference;
  const share = budget ? budget[0] / Math.sqrt(3) : null;
  const axes = ['x', 'y', 'z'].map((ax, i) => {
    const tau = p.damping[i] / (p.stiffness[i] * Math.sqrt(ratio));
    const vmax = gain >= 1 ? Infinity : leash.translation / tau / (1 - gain);
    return { ax, tau, vmax, binds: share != null && vmax < share };
  });
  const worst = axes.reduce((a, b) => (b.vmax < a.vmax ? b : a));
  const text = gain >= 1
    ? `feedforward at 1: damping acts on the velocity error, the ${fmt(leash.translation * 1e3)} mm leash caps nothing`
    : `⚠ feedforward ${fmt(gain)}: the fixed ${fmt(leash.translation * 1e3)} mm leash becomes a speed cap — `
      + axes.map(a => `${a.ax} ${fmt(a.vmax)} m/s (τ ${fmt(a.tau)} s)`).join(', ')
      + (share != null ? ` vs budget share ${fmt(share)} m/s per axis` : '')
      + (worst.binds ? ` — ${worst.ax} BINDS: the arm cannot reach the budget and no knob relieves it` : '');
  return { text, severity: gain >= 1 ? 'ok' : worst.binds ? 'bad' : 'warn' };
}

function dampingRatios(K, D, inertia) {
  if (!inertia) return null;
  return K.map((k, i) => (k > 0 && inertia[i] > 0 ? D[i] / (2 * Math.sqrt(k * inertia[i])) : null));
}

// Returns [{text, severity}] for the pending node values.
function nodeAdvisories(values, derived) {
  const out = [];
  const z = values.joint_stiffness && values.joint_damping && dampingRatios(values.joint_stiffness, values.joint_damping, derived.joint_inertia);
  if (z) {
    const bad = z.map((v, i) => (v != null && (v < ZETA_BAND[0] || v > ZETA_BAND[1]) ? `j${i + 1} ζ ${fmt(v)}` : null)).filter(Boolean);
    out.push({ text: 'ζ ' + z.map(v => (v == null ? '–' : fmt(v))).join(' ') + (bad.length ? ` — outside ${ZETA_BAND.join('…')}: ` + bad.join(', ') : ''), severity: bad.length ? 'warn' : 'ok', group: 'gains' });
  }
  if (values.ik_nullspace_gain === 0) out.push({ text: 'nullspace gain 0: posture term off, the elbow will not fold back', severity: 'warn', group: 'ik' });
  if (values.velocity_feedforward_cutoff != null && values.velocity_feedforward_cutoff < J4_BAND_HZ)
    out.push({ text: `feedforward cutoff ${fmt(values.velocity_feedforward_cutoff)} Hz is below ${J4_BAND_HZ} Hz (3× the j4 band): it filters the motion, not just the ripple`, severity: 'warn', group: 'feedforward' });
  return out;
}

// Teleop values against the node bounds the schema reports (`bound` names a `derived` key).
function teleopAdvisories(values, teleopSchema, nodeDerived, nodeValues) {
  const out = [];
  for (const [name, spec] of Object.entries(teleopSchema.params)) {
    if (!spec.bound || !nodeDerived || nodeDerived[spec.bound] == null) continue;
    const bound = nodeDerived[spec.bound];
    const v = values[name];
    if (v > bound) out.push({ text: `${name} ${fmt(v)} exceeds node ${spec.bound} ${fmt(bound)}: the owner will refuse or clamp`, severity: 'bad', group: 'teleop' });
    else if (v > bound * (1 - NEAR_BOUND)) out.push({ text: `${name} ${fmt(v)} within ${NEAR_BOUND * PERCENT} % of node ${spec.bound} ${fmt(bound)}`, severity: 'warn', group: 'teleop' });
  }
  if (values.spatial_scale != null && nodeValues && nodeValues.budget) {
    const share = nodeValues.budget[0] / Math.sqrt(3);
    if (values.spatial_scale * HAND_PEAK_MPS > share)
      out.push({ text: `spatial scale ${fmt(values.spatial_scale)} × ${HAND_PEAK_MPS} m/s hand peak = ${fmt(values.spatial_scale * HAND_PEAK_MPS)} m/s, above the budget's per-axis share ${fmt(share)} m/s: the OTG will lag the hand`, severity: 'warn', group: 'teleop' });
  }
  const seen = teleopSchema.derived && teleopSchema.derived.node_bounds_seen;
  if (seen && nodeDerived) for (const [k, v] of Object.entries(seen)) {
    if (nodeDerived[k] != null && nodeDerived[k] !== v)
      out.push({ text: `client believes ${k} = ${fmt(v)}, node says ${fmt(nodeDerived[k])} — the client read a stale schema`, severity: 'bad', group: 'teleop' });
  }
  return out;
}
