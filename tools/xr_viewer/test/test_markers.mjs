// Run: node tools/xr_viewer/test/test_markers.mjs
import { register } from 'node:module';
import assert from 'node:assert/strict';
register('./hooks.mjs', import.meta.url);
const THREE = await import('three');
const { createFrames } = await import('../web/frames.js');
const { MarkerLayer, MARKER } = await import('../web/markers.js');

const near = (a, b, msg, eps = 1e-6) => assert.ok(Math.abs(a - b) < eps, `${msg}: ${a} vs ${b}`);
const f = createFrames(); f.applySpawn([0, 0, 0, 0]);
const layer = new MarkerLayer(f.dyn);

const sphere = (ns, p, s, c, q = [0, 0, 0, 1]) =>
  ({ ns, id: 0, type: MARKER.SPHERE, p, q, s, c, pts: [], cols: [], text: '' });
const est = [3.97, 0.3, 0.3];
// shape copied from a real bridge /events message (CONFIRMED, from the fake demo)
const set1 = [
  { ns: 'status', id: 0, type: MARKER.TEXT, p: [3.97, 0.3, 0.8], q: [0, 0, 0, 1], s: [0, 0, 0.12], c: [1, 1, 1, 1], pts: [], cols: [], text: 'depth 3.97 m  lateral +0.30 m  height 0.30 m\nCONFIRMED' },
  sphere('estimate', est, [0.15, 0.15, 0.15], [0, 1, 0, 1]),
  sphere('uncertainty', est, [0.07, 0.07, 0.56], [0, 1, 0, 0.3]),
  { ns: 'rays', id: 0, type: MARKER.LINE_LIST, p: [0, 0, 0], q: [0, 0, 0, 1], s: [0.01, 0, 0], c: [0.75, 0.75, 0.75, 1],
    pts: [0.1, -0.8, 0.15, 3.97, 0.3, 0.3, 0.1, 0.8, 0.15, 3.97, 0.3, 0.3],
    cols: [0.75, 0.75, 0.75, 0.7, 0.75, 0.75, 0.75, 0.7, 1, 0.1, 0.1, 0.7, 1, 0.1, 0.1, 0.7], text: '' },
];
const r = layer.update(set1, -0.0);
assert.equal(r.statusText.split('\n')[1], 'CONFIRMED', 'status text returned for the HUD');
assert.equal(r.estimate.ns, 'estimate');
assert.equal(f.dyn.children.length, 3 + 2, 'estimate, uncertainty, rays + drop line + halo (text is not a 3D object)');
assert.equal(layer.halo.material.sizeAttenuation, false, 'halo keeps a fixed screen size');
near(layer.halo.position.x, 3.97, 'halo sits on the estimate');

// the estimate lands where the headset frame says: 4 m ahead (-z), 0.3 m to the left (-x), 0.3 m up
f.viewRoot.updateMatrixWorld(true);
const e = layer.live.get('estimate:0').obj.getWorldPosition(new THREE.Vector3());
near(e.x, -0.3, 'estimate x (left is -x)'); near(e.y, 0.3, 'estimate y (up)'); near(e.z, -3.97, 'estimate z (ahead is -z)');

// unit sphere (radius 0.5) scaled by full diameters; ellipsoid long axis is the marker's z
const u = layer.live.get('uncertainty:0').obj;
near(u.scale.z, 0.56, 'ellipsoid z diameter'); near(u.material.opacity, 0.3, 'ellipsoid opacity');
assert.equal(u.material.depthTest, false, 'drawn through walls');

// quaternion: 90 deg about odom x turns the ellipsoid's z axis to odom -y
const s2 = Math.SQRT1_2;
layer.update([sphere('uncertainty', est, [0.07, 0.07, 0.56], [0, 1, 0, 0.3], [s2, 0, 0, s2])], 0);
const axis = new THREE.Vector3(0, 0, 1).applyQuaternion(layer.live.get('uncertainty:0').obj.quaternion);
near(axis.x, 0, 'axis x'); near(axis.y, -1, 'axis y'); near(axis.z, 0, 'axis z');

// rays: per-vertex colour, red for the rejected view
layer.update(set1, 0);
const rays = layer.live.get('rays:0').obj;
assert.equal(rays.geometry.attributes.position.count, 4);
const col = rays.geometry.attributes.color.array;
near(col[0], 0.75, 'used ray grey'); near(col[6], 1.0, 'rejected ray red r'); near(col[7], 0.1, 'rejected ray red g');

// drop line runs from the estimate to the floor
layer.update(set1, -0.5);
const drop = layer.dropLine.geometry.attributes.position.array;
near(drop[2], 0.3, 'drop top z'); near(drop[5], -0.5, 'drop bottom z (floor)');

// markers that disappear are removed and disposed (no leaks across 5 Hz updates)
for (let i = 0; i < 50; i++) layer.update(set1, 0);
assert.equal(f.dyn.children.length, 5, 'no accumulation after 50 updates');
layer.update([], 0);
assert.equal(f.dyn.children.length, 0, 'empty set clears everything, including the drop line and halo');
assert.equal(layer.halo, null);
assert.equal(layer.live.size, 0);

// a marker whose type changes is rebuilt, not mutated into the wrong object
layer.update([sphere('x', [0, 0, 0], [1, 1, 1], [1, 0, 0, 1])], 0);
layer.update([{ ns: 'x', id: 0, type: MARKER.LINE_STRIP, p: [0, 0, 0], q: [0, 0, 0, 1], s: [0.04, 0, 0], c: [1, 0.75, 0, 1], pts: [0, 0, 0, 1, 0, 0], cols: [], text: '' }], 0);
assert.ok(layer.live.get('x:0').obj.isLine, 'type change rebuilds the object');
console.log('ok test_markers');
