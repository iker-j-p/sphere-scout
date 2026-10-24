// Run: node tools/xr_viewer/test/test_panel.mjs   (layout rules only; the DOM wiring is browser-tested)
import assert from 'node:assert/strict';
import { rateFromStamps, clampRect, cornerRect, nearestCorner, nextCorner, parseSaved, panelHeight,
         MARGIN, MIN_W, HEADER_H, CORNERS } from '../web/panel.js';

const VW = 1100, VH = 700, A = 4 / 3;
const inside = (r) => {
  const h = panelHeight(r.w, A);
  return r.x >= MARGIN - 1e-9 && r.y >= MARGIN - 1e-9 && r.x + r.w <= VW - MARGIN + 1e-9 && r.y + h <= VH - MARGIN + 1e-9;
};

// clamping keeps the whole panel on screen, whatever the input
for (const r of [{ x: -500, y: -500, w: 360 }, { x: 5000, y: 5000, w: 360 }, { x: 300, y: 300, w: 5000 },
                 { x: 300, y: 300, w: 1 }, { x: 0, y: 900, w: 700 }]) {
  assert.ok(inside(clampRect(r, VW, VH, A)), `inside: ${JSON.stringify(r)}`);
}
assert.equal(clampRect({ x: 10, y: 10, w: 1 }, VW, VH, A).w, MIN_W, 'minimum width');
const big = clampRect({ x: 0, y: 0, w: 5000 }, VW, VH, A);
assert.ok(Math.abs(big.w - (VH - 2 * MARGIN - HEADER_H) * A) < 1e-9, 'maximum width is limited by the window height, not just its width');
assert.deepEqual(clampRect({ x: 100, y: 120, w: 360 }, VW, VH, A), { x: 100, y: 120, w: 360 }, 'a valid rect is untouched');

// a tiny window never produces NaN or a panel larger than the window can hold
const tiny = clampRect({ x: 50, y: 50, w: 400 }, 200, 150, A);
assert.ok(Number.isFinite(tiny.x + tiny.y + tiny.w) && tiny.w >= MIN_W, 'tiny viewport stays finite');

// corners
const br = cornerRect('br', 360, VW, VH, A), tl = cornerRect('tl', 360, VW, VH, A);
assert.deepEqual(tl, { x: MARGIN, y: MARGIN, w: 360 });
assert.equal(br.x, VW - MARGIN - 360); assert.ok(Math.abs(br.y - (VH - MARGIN - panelHeight(360, A))) < 1e-9);
for (const c of CORNERS) assert.equal(nearestCorner(cornerRect(c, 360, VW, VH, A), VW, VH, A), c, 'corner round trip ' + c);
let c = 'br'; const seen = [];
for (let i = 0; i < 4; i++) { c = nextCorner(c); seen.push(c); }
assert.deepEqual(seen, ['bl', 'tl', 'tr', 'br'], 'the corner button cycles through all four corners');

// saved state: valid, hidden, damaged
assert.deepEqual(parseSaved('{"x":10,"y":20,"w":300,"visible":false}'), { x: 10, y: 20, w: 300, visible: false });
assert.equal(parseSaved('{"x":10,"y":20,"w":300}').visible, true, 'visible by default');
for (const bad of [null, '', 'not json', '{"x":1}', '{"x":"a","y":1,"w":2}', '{"x":1,"y":1,"w":0}', '{"x":1,"y":1,"w":-5}']) {
  assert.equal(parseSaved(bad), null, 'rejects ' + bad);
}
// frame rate comes from send times, so a burst of late-delivered frames still reads the true rate
const steady = Array.from({ length: 10 }, (_, i) => 100 + i * 0.125);          // 8 Hz
assert.ok(Math.abs(rateFromStamps(steady) - 8) < 1e-9, 'steady 8 Hz');
assert.equal(rateFromStamps([1, 1.1]), null, 'needs at least 3 stamps');
assert.equal(rateFromStamps([5, 5, 5]), null, 'zero span is not a rate');
console.log('ok test_panel');
