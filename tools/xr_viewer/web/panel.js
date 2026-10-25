// Movable, resizable window showing the robot's thermal feed (/thermal/view) in the desktop page.
//
// Layout rules are pure functions (unit-tested in Node); createThermalPanel() wires them to the DOM.
// The panel only displays frames that arrive from the bridge. It adds no text of its own except
// facts about the stream (frame rate, seconds since the last frame).

export const HEADER_H = 30;                 // px, title bar
export const MIN_W = 180;                   // px, image width
export const MARGIN = 8;                    // px from the window edge
export const DEFAULT_W = 360;
export const STALE_AFTER_S = 1.5;           // no frame for this long -> "no frames" overlay
export const CORNERS = ['br', 'bl', 'tl', 'tr'];   // cycle order of the corner button
const STORAGE_KEY = 'sphereScout.thermalPanel.v1';
const KEY_STEP = 20;

// Frame rate from the bridge's send times (seconds), not from arrival times: a browser that was busy
// delivers queued events in a burst, which would read as a huge rate. null until 3 stamps exist.
export function rateFromStamps(stamps) {
  if (stamps.length < 3) return null;
  const span = stamps[stamps.length - 1] - stamps[0];
  return span > 0 ? (stamps.length - 1) / span : null;
}

// ----------------------------------------------------------------------- layout rules
export const panelHeight = (w, aspect) => w / aspect + HEADER_H;

// Keep the whole panel inside the viewport; rect = { x, y, w } (w = image width).
export function clampRect(r, vw, vh, aspect) {
  const maxW = Math.max(MIN_W, Math.min(vw - 2 * MARGIN, (vh - 2 * MARGIN - HEADER_H) * aspect));
  const w = Math.min(Math.max(r.w, MIN_W), maxW);
  const h = panelHeight(w, aspect);
  const x = Math.min(Math.max(r.x, MARGIN), Math.max(MARGIN, vw - MARGIN - w));
  const y = Math.min(Math.max(r.y, MARGIN), Math.max(MARGIN, vh - MARGIN - h));
  return { x, y, w };
}

export function cornerRect(corner, w, vw, vh, aspect) {
  const h = panelHeight(w, aspect);
  const x = corner.endsWith('l') ? MARGIN : vw - MARGIN - w;
  const y = corner.startsWith('t') ? MARGIN : vh - MARGIN - h;
  return clampRect({ x, y, w }, vw, vh, aspect);
}

export function nearestCorner(r, vw, vh, aspect) {
  const cx = r.x + r.w / 2, cy = r.y + panelHeight(r.w, aspect) / 2;
  return (cy < vh / 2 ? 't' : 'b') + (cx < vw / 2 ? 'l' : 'r');
}

export const nextCorner = (c) => CORNERS[(CORNERS.indexOf(c) + 1) % CORNERS.length];

// Saved state from localStorage; null if missing or damaged.
export function parseSaved(text) {
  try {
    const o = JSON.parse(text);
    if (![o.x, o.y, o.w].every(Number.isFinite) || o.w <= 0) return null;
    return { x: o.x, y: o.y, w: o.w, visible: o.visible !== false };
  } catch { return null; }
}

// -------------------------------------------------------------------------- DOM wiring
function safeStorage() {
  try { const s = window.localStorage; s.getItem(STORAGE_KEY); return s; } catch { return null; }
}

export function createThermalPanel(doc = document, storage = safeStorage()) {
  const $ = (id) => doc.getElementById(id);
  const root = $('thermal'), bar = $('thermal-bar'), canvas = $('thermal-canvas');
  const grip = $('thermal-grip'), meta = $('thermal-meta'), stale = $('thermal-stale');
  const ctx = canvas.getContext('2d');
  const vp = () => ({ vw: window.innerWidth, vh: window.innerHeight });

  let aspect = 4 / 3;
  let visible = true;
  let rect = null;
  let lastFrameAt = null;
  const stamps = [];                           // bridge send times of the most recent frames
  let decoding = false, queued = null;

  function apply() {
    root.hidden = !visible;
    root.style.left = `${Math.round(rect.x)}px`;
    root.style.top = `${Math.round(rect.y)}px`;
    root.style.width = `${Math.round(rect.w)}px`;
    canvas.style.aspectRatio = String(aspect);
  }
  function save() {
    try { storage?.setItem(STORAGE_KEY, JSON.stringify({ ...rect, visible })); } catch { /* private mode: skip */ }
  }
  function place(next, persist = true) {
    const { vw, vh } = vp();
    rect = clampRect(next, vw, vh, aspect);
    apply();
    if (persist) save();
  }

  // initial state: saved layout if any, otherwise bottom-right
  const { vw, vh } = vp();
  const saved = storage ? parseSaved(storage.getItem(STORAGE_KEY)) : null;
  visible = saved ? saved.visible : true;
  rect = saved ? clampRect(saved, vw, vh, aspect) : cornerRect('br', DEFAULT_W, vw, vh, aspect);
  apply();
  window.addEventListener('resize', () => place(rect, false));

  // ---- drag by the title bar, resize by the corner grip (pointer events: mouse, pen, touch)
  let drag = null;
  bar.addEventListener('pointerdown', (e) => {
    if (e.target.closest('button')) return;
    drag = { kind: 'move', sx: e.clientX, sy: e.clientY, r: { ...rect } };
    bar.setPointerCapture(e.pointerId);
    root.classList.add('dragging');
  });
  grip.addEventListener('pointerdown', (e) => {
    drag = { kind: 'size', sx: e.clientX, sy: e.clientY, r: { ...rect } };
    grip.setPointerCapture(e.pointerId);
    root.classList.add('dragging');
    e.preventDefault();
  });
  const onMove = (e) => {
    if (!drag) return;
    const dx = e.clientX - drag.sx, dy = e.clientY - drag.sy;
    if (drag.kind === 'move') place({ ...drag.r, x: drag.r.x + dx, y: drag.r.y + dy }, false);
    else place({ ...drag.r, w: drag.r.w + Math.max(dx, dy * aspect) }, false);
  };
  const onUp = () => { if (drag) { drag = null; root.classList.remove('dragging'); save(); } };
  for (const el of [bar, grip]) {
    el.addEventListener('pointermove', onMove);
    el.addEventListener('pointerup', onUp);
    el.addEventListener('pointercancel', onUp);
  }

  // ---- keyboard: arrows move, Shift+arrows resize (title bar focused)
  bar.addEventListener('keydown', (e) => {
    const k = { ArrowLeft: [-1, 0], ArrowRight: [1, 0], ArrowUp: [0, -1], ArrowDown: [0, 1] }[e.key];
    if (!k) return;
    e.preventDefault();
    if (e.shiftKey) place({ ...rect, w: rect.w + (k[0] + k[1]) * KEY_STEP });
    else place({ ...rect, x: rect.x + k[0] * KEY_STEP, y: rect.y + k[1] * KEY_STEP });
  });

  // ---- buttons
  const toggle = (on = !visible) => { visible = on; apply(); save(); };
  $('thermal-hide').addEventListener('click', () => toggle(false));
  $('thermal-corner').addEventListener('click', () => {
    const { vw, vh } = vp();
    place(cornerRect(nextCorner(nearestCorner(rect, vw, vh, aspect)), rect.w, vw, vh, aspect));
  });

  // ---- frames from the bridge
  async function draw(m) {
    decoding = true;
    try {
      const bin = atob(m.jpg);
      const bytes = new Uint8Array(bin.length);
      for (let i = 0; i < bin.length; i++) bytes[i] = bin.charCodeAt(i);
      const bmp = await createImageBitmap(new Blob([bytes], { type: 'image/jpeg' }));
      if (canvas.width !== bmp.width || canvas.height !== bmp.height) {
        canvas.width = bmp.width; canvas.height = bmp.height;
        const a = bmp.width / bmp.height;
        if (Math.abs(a - aspect) > 1e-3) { aspect = a; place(rect, false); }
      }
      ctx.drawImage(bmp, 0, 0);
      bmp.close();
    } catch { /* undecodable frame: keep the previous one; the stale timer reports a silent stream */ }
    decoding = false;
    if (queued) { const q = queued; queued = null; draw(q); }
  }
  function push(m) {
    lastFrameAt = performance.now();
    if (Number.isFinite(m.t)) { stamps.push(m.t); if (stamps.length > 12) stamps.shift(); }
    if (decoding) queued = m; else draw(m);       // drop in between: always show the newest frame
  }

  // facts about the stream, twice a second
  setInterval(() => {
    const now = performance.now();
    const fps = rateFromStamps(stamps);
    const age = lastFrameAt === null ? null : (now - lastFrameAt) / 1000;
    meta.textContent = age === null || age > STALE_AFTER_S || fps === null ? '' : `${fps.toFixed(0)} Hz`;
    stale.hidden = age !== null && age <= STALE_AFTER_S;
    stale.textContent = age === null ? 'no frames received yet' : `no frames for ${age.toFixed(1)} s`;
  }, 500);

  return { push, toggle, isVisible: () => visible, getRect: () => ({ ...rect }) };
}
