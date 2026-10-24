// Draws the localizer's MarkerArray (as forwarded by bridge.py) into a three.js group.
// No DOM, so it is unit-testable in Node. Markers arrive in odom/ROS axes and the parent group
// is the odom frame, so poses are used as-is.
//
// Everything is drawn without depth testing, on top of the semi-transparent building, so the
// estimate stays visible through walls.
import * as THREE from 'three';

export const MARKER = { SPHERE: 2, LINE_STRIP: 4, LINE_LIST: 5, TEXT: 9 };   // visualization_msgs/Marker

const rgb = (c) => new THREE.Color(c[0], c[1], c[2]);
const HALO_SIZE = 0.06;                 // fraction of the screen height, constant at any distance

// Ring + centre dot, so the estimate is findable from across a 24 m building. null without a
// DOM (unit tests in Node): the sprite then draws as a plain square.
function ringTexture() {
  if (typeof document === 'undefined') return null;
  const c = document.createElement('canvas');
  c.width = c.height = 128;
  const g = c.getContext('2d');
  g.strokeStyle = '#fff'; g.fillStyle = '#fff';
  g.lineWidth = 9; g.beginPath(); g.arc(64, 64, 52, 0, Math.PI * 2); g.stroke();
  g.beginPath(); g.arc(64, 64, 7, 0, Math.PI * 2); g.fill();
  return new THREE.CanvasTexture(c);
}

function dispose(o) {
  o.geometry?.dispose();
  o.material?.dispose();
  o.parent?.remove(o);
}

function make(m) {
  if (m.type === MARKER.SPHERE) {
    return new THREE.Mesh(new THREE.SphereGeometry(0.5, 24, 16),
      new THREE.MeshBasicMaterial({ transparent: true, depthTest: false, depthWrite: false }));
  }
  const mat = new THREE.LineBasicMaterial({ vertexColors: true, transparent: true, depthTest: false });
  return m.type === MARKER.LINE_LIST ? new THREE.LineSegments(new THREE.BufferGeometry(), mat)
                                      : new THREE.Line(new THREE.BufferGeometry(), mat);
}

function apply(o, m) {
  if (m.type === MARKER.SPHERE) {
    o.position.set(...m.p);
    o.quaternion.set(...m.q);
    o.scale.set(m.s[0], m.s[1], m.s[2]);          // full diameters along the marker's own axes
    o.material.color.copy(rgb(m.c));
    o.material.opacity = m.c[3];
    o.renderOrder = m.ns === 'uncertainty' ? 9 : 10;   // solid estimate stays readable on top
    return;
  }
  const n = m.pts.length / 3;
  const col = new Float32Array(n * 3);
  for (let i = 0; i < n; i++) {
    const c = m.cols.length ? m.cols.slice(4 * i, 4 * i + 3) : m.c;
    col.set([c[0], c[1], c[2]], 3 * i);
  }
  o.geometry.dispose();
  o.geometry = new THREE.BufferGeometry();
  o.geometry.setAttribute('position', new THREE.BufferAttribute(new Float32Array(m.pts), 3));
  o.geometry.setAttribute('color', new THREE.BufferAttribute(col, 3));
  o.material.opacity = m.cols.length ? Math.max(m.cols[3], 0.5) : m.c[3];
  o.frustumCulled = false;
  o.renderOrder = m.ns === 'size_prior' ? 12 : 6;
}

export class MarkerLayer {
  constructor(parent) {
    this.parent = parent;
    this.live = new Map();          // "ns:id" -> { type, obj }
    this.dropLine = null;
    this.halo = null;
  }

  // markers: the complete current set. floorZ: odom z of the world floor, for the drop line.
  // Returns { statusText, estimate } for the HUD.
  update(markers, floorZ = 0) {
    const seen = new Set();
    let estimate = null, statusText = '';
    for (const m of markers) {
      if (m.type === MARKER.TEXT) { if (m.ns === 'status') statusText = m.text; continue; }
      if (m.ns === 'estimate') estimate = m;
      const key = `${m.ns}:${m.id}`;
      seen.add(key);
      let e = this.live.get(key);
      if (!e || e.type !== m.type) {
        if (e) dispose(e.obj);
        e = { type: m.type, obj: make(m) };
        this.parent.add(e.obj);
        this.live.set(key, e);
      }
      apply(e.obj, m);
    }
    for (const [key, e] of this.live) if (!seen.has(key)) { dispose(e.obj); this.live.delete(key); }

    // vertical line from the estimate to the floor: makes its height readable in 3D
    if (this.dropLine) { dispose(this.dropLine); this.dropLine = null; }
    if (estimate) {
      const [x, y, z] = estimate.p;
      const g = new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(x, y, z), new THREE.Vector3(x, y, floorZ)]);
      this.dropLine = new THREE.Line(g, new THREE.LineBasicMaterial({ color: rgb(estimate.c), transparent: true, opacity: 0.8, depthTest: false }));
      this.dropLine.renderOrder = 9;
      this.parent.add(this.dropLine);
    }

    // halo: fixed on-screen size ring around the estimate (display only)
    if (this.halo && !estimate) { this.halo.material.map?.dispose(); dispose(this.halo); this.halo = null; }
    if (estimate) {
      if (!this.halo) {
        this.halo = new THREE.Sprite(new THREE.SpriteMaterial({
          map: ringTexture(), transparent: true, depthTest: false, depthWrite: false, sizeAttenuation: false }));
        this.halo.scale.set(HALO_SIZE, HALO_SIZE, 1);
        this.halo.renderOrder = 13;
        this.parent.add(this.halo);
      }
      this.halo.position.set(...estimate.p);
      this.halo.material.color.copy(rgb(estimate.c));
    }
    return { statusText, estimate };
  }
}
