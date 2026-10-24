// Run: node tools/xr_viewer/test/test_frames.mjs   (Node 20+; no npm install needed)
import { register } from 'node:module';
import assert from 'node:assert/strict';
register('./hooks.mjs', import.meta.url);
const THREE = await import('three');
const { createFrames } = await import('../web/frames.js');

const near = (a, b, msg, eps = 1e-9) => assert.ok(Math.abs(a - b) < eps, `${msg}: ${a} vs ${b}`);
const nearV = (v, e, msg) => { near(v.x, e[0], msg + ' x'); near(v.y, e[1], msg + ' y'); near(v.z, e[2], msg + ' z'); };

function threePos(root, obj) { root.updateMatrixWorld(true); return obj.getWorldPosition(new THREE.Vector3()); }
function odomOfWorldPoint(f, p) {
  const o = new THREE.Object3D(); o.position.set(...p); f.worldGroup.add(o);
  f.viewRoot.updateMatrixWorld(true);
  return f.rosRoot.worldToLocal(o.getWorldPosition(new THREE.Vector3()));
}

// 1. odom axes in the headset frame: +x forward (-z), +y left (-x), +z up (+y)
{
  const f = createFrames(); f.applySpawn([0, 0, 0, 0]);
  for (const [odom, expect, name] of [[[1, 0, 0], [0, 0, -1], '+x'], [[0, 1, 0], [-1, 0, 0], '+y'], [[0, 0, 1], [0, 1, 0], '+z'],
                                      [[4, 0.3, 0.3], [-0.3, 0.3, -4], 'survivor fixture']]) {
    const o = new THREE.Object3D(); o.position.set(...odom); f.dyn.add(o);
    nearV(threePos(f.viewRoot, o), expect, 'odom ' + name + ' in headset frame');
  }
}
// 2. floor: world z = 0 sits at headset y = 0 even when the robot spawns above it
{
  const f = createFrames(); const floorOdomZ = f.applySpawn([1, 2, 0.5, 0]);
  near(floorOdomZ, -0.5, 'floor in odom z');
  const floorPoint = new THREE.Object3D(); floorPoint.position.set(1, 2, 0); f.worldGroup.add(floorPoint);
  nearV(threePos(f.viewRoot, floorPoint), [0, 0, 0], 'floor under the spawn is at the headset origin');
}
// 3. yaw: same numbers as test_triangulation.test_bearing_and_world_to_odom
{
  const f = createFrames(); f.applySpawn([1.0, 2.0, 0.1, Math.PI / 2]);
  nearV(odomOfWorldPoint(f, [1.0, 3.0, 0.1]), [1.0, 0.0, 0.0], 'world point 1 m ahead of a +y-facing robot');
  nearV(odomOfWorldPoint(f, [0.0, 2.0, 0.1]), [0.0, 1.0, 0.0], 'world point 1 m to its left (-x world)');
}
// 4. general pose vs the localizer formula p_odom = Rz(-yaw) (p_world - spawn)
{
  const spawn = [3.2, -1.7, 0.4, 0.83], pw = [7.1, 4.4, 1.9];
  const f = createFrames(); f.applySpawn(spawn);
  const c = Math.cos(spawn[3]), s = Math.sin(spawn[3]), dx = pw[0] - spawn[0], dy = pw[1] - spawn[1];
  nearV(odomOfWorldPoint(f, pw), [c * dx + s * dy, -s * dx + c * dy, pw[2] - spawn[2]], 'general pose');
}
console.log('ok test_frames (4 groups)');
