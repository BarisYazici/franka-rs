// Thin fetch/SSE layer over the bridge. Owner refusals arrive as HTTP 200 with ok:false;
// callers branch on `ok`, never on status.
const api = {
  async get(path) { const r = await fetch(path); return r.json(); },
  async post(path, body) {
    const r = await fetch(path, { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(body) });
    return r.json();
  },
  // the bridge's write gate wants the JSON content type on every write, body or not
  async del(path) { const r = await fetch(path, { method: 'DELETE', headers: { 'Content-Type': 'application/json' } }); return r.json(); },
  // handlers: {hello, current, status, metrics, open, close}
  events(arm, handlers) {
    const es = new EventSource(`/api/${arm}/events`);
    for (const ev of ['hello', 'current', 'status', 'metrics']) {
      es.addEventListener(ev, e => handlers[ev] && handlers[ev](JSON.parse(e.data)));
    }
    es.onopen = () => handlers.open && handlers.open();
    es.onerror = () => handlers.close && handlers.close();
    return es;
  },
};
