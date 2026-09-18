// Pure request building for Apply, kept DOM-free so it can be tested under node.

// Fields in `edits` whose new value crosses the schema's confirm_above from below.
function crossings(schema, edits, current) {
  const out = [];
  for (const [name, v] of Object.entries(edits)) {
    const p = schema.params[name]; if (!p || p.danger !== 'confirm_above') continue;
    const thr = p.confirm_above, c = current[name];
    const crossed = Array.isArray(thr) ? thr.some((t, i) => v[i] > t && (c == null || c[i] <= t)) : v > thr && (c == null || c <= thr);
    if (crossed) out.push(name);
  }
  return out;
}

// Identity of a set of pending edits; a confirm agreed for one set never applies to another.
function editsKey(edits) { return JSON.stringify(edits); }

// A confirm map: what the operator agreed to with the second click, bound to the edits it was
// computed from. `owners` is {owner: [names]}.
function confirmFor(owners, edits) { return { owners, key: editsKey(edits) }; }

// One set request per owner with edits. `confirmed` is a confirmFor() map or null; it is honoured
// only while the edits are the ones it was agreed for, and an owner only ever receives its own names.
function applyBody(owners, schemas, loaded, edits, clientId, confirmed) {
  const live = confirmed && confirmed.key === editsKey(edits) ? confirmed.owners : {};
  const body = {};
  for (const o of owners) {
    if (!edits[o] || !Object.keys(edits[o]).length || !loaded[o]) continue;
    body[o] = { client_id: clientId, base_version: loaded[o].version, confirm: live[o] || [], params: edits[o] };
  }
  return body;
}
