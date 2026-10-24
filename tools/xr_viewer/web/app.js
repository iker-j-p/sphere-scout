// Sphere-Scout operator view: desktop free camera, or WebXR first person (Quest).
//
// Frames: see frames.js. Everything ROS is drawn in ROS axes inside the odom frame (origin =
// robot spawn = the operator's start point); the Gazebo world hangs under it, placed with the
// inverse of spawn_pose (same transform as world_to_odom in the localizer).
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { GLTFLoader } from 'three/addons/loaders/GLTFLoader.js';
import { VRButton } from 'three/addons/webxr/VRButton.js';
import { createFrames } from './frames.js';
import { MarkerLayer } from './markers.js';
import { createThermalPanel } from './panel.js';

const $ = (id) => document.getElementById(id);
const DOLLHOUSE_SCALE = 0.04;              // tabletop model: 1 m of world = 4 cm
const DOLLHOUSE_POS = new THREE.Vector3(0, 0.85, -0.7);
const XRAY_OPACITY = 0.16;
const TRAIL_MAX = 800;
const TRAIL_STEP_M = 0.05;
const MOVE_SPEED = 1.6;                    // m/s with the left stick (first person)
const SNAP_DEG = 30;

// ------------------------------------------------------------------ scene and frames
const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
renderer.setSize(window.innerWidth, window.innerHeight);
renderer.xr.enabled = true;
renderer.xr.setReferenceSpaceType('local-floor');
document.body.appendChild(renderer.domElement);

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x1b1f24);
scene.add(new THREE.HemisphereLight(0xffffff, 0x445566, 1.4));
const sun = new THREE.DirectionalLight(0xffffff, 1.2);
sun.position.set(5, 10, 3);
scene.add(sun);

const camera = new THREE.PerspectiveCamera(60, window.innerWidth / window.innerHeight, 0.05, 600);
const rig = new THREE.Group();             // the user's body in XR; identity on desktop
rig.add(camera);
scene.add(rig);

const { viewRoot, rosRoot, worldGroup, dyn, applySpawn: placeWorld } = createFrames();
scene.add(viewRoot);

const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;

// -------------------------------------------------------------------------- config
let config = { spawn_pose: [0, 0, 0, 0], mesh: '/models/apartment.glb' };
let floorOdomZ = 0;                        // world z = 0 (ground plane) expressed in odom z
let buildingBox = null;                    // three-space bounds of the mesh

function applySpawn(spawn) { floorOdomZ = placeWorld(spawn); }

// ----------------------------------------------------------------------- the building
const glassMaterials = [];
function loadBuilding() {
  const info = $('status');
  new GLTFLoader().load(config.mesh, (gltf) => {
    gltf.scene.traverse((o) => {
      if (!o.isMesh) return;
      const mats = Array.isArray(o.material) ? o.material : [o.material];
      o.material = mats.map((m) => {
        const c = m.clone();
        c.transparent = true; c.opacity = XRAY_OPACITY; c.depthWrite = false; c.side = THREE.DoubleSide;
        glassMaterials.push(c);
        return c;
      });
      if (o.material.length === 1) o.material = o.material[0];
    });
    worldGroup.add(gltf.scene);
    const grid = new THREE.GridHelper(60, 60, 0x5c6b7a, 0x33404c);   // 1 m cells at world z = 0
    grid.rotation.x = Math.PI / 2;
    worldGroup.add(grid);
    scene.updateMatrixWorld(true);
    buildingBox = new THREE.Box3().setFromObject(gltf.scene);
    frameDesktopCamera();
    if (info.textContent.startsWith('loading')) info.textContent = 'waiting for /survivor/markers...';
  }, (e) => {
    if (e.lengthComputable) $('status').textContent = `loading building ${Math.round(100 * e.loaded / e.total)}%`;
  }, (err) => { $('warn').style.display = 'block'; $('warn').textContent = 'could not load the building mesh: ' + err; });
}

function frameDesktopCamera() {
  // look at the building from the operator's side, like a dollhouse
  const center = buildingBox.getCenter(new THREE.Vector3());
  const radius = buildingBox.getSize(new THREE.Vector3()).length() / 2;
  const toOperator = new THREE.Vector3(0, 0, 0).sub(center).setY(0);
  if (toOperator.lengthSq() < 1e-6) toOperator.set(0, 0, 1);
  toOperator.normalize().multiplyScalar(Math.cos(0.6)).setY(Math.sin(0.6));
  camera.position.copy(center).addScaledVector(toOperator, radius * 1.6);
  controls.target.copy(center);
  controls.update();
}

// ---------------------------------------------------------------- operator reference
const axes = new THREE.AxesHelper(1.0);    // display size only; odom origin = operator / start
axes.material.depthTest = false;
axes.renderOrder = 8;
dyn.add(axes);

const robot = new THREE.Group();           // display glyph, NOT to scale
const cone = new THREE.Mesh(new THREE.ConeGeometry(0.15, 0.4, 16),
  new THREE.MeshBasicMaterial({ color: 0xffffff, depthTest: false }));
cone.rotation.z = -Math.PI / 2;            // cone axis +y -> +x (heading)
cone.renderOrder = 11;
robot.add(cone);
robot.visible = false;
dyn.add(robot);

const trailPts = new Float32Array(TRAIL_MAX * 3);
let trailN = 0;
const trailGeom = new THREE.BufferGeometry();
trailGeom.setAttribute('position', new THREE.BufferAttribute(trailPts, 3));
trailGeom.setDrawRange(0, 0);
const trail = new THREE.Line(trailGeom, new THREE.LineBasicMaterial({ color: 0xffffff, transparent: true, opacity: 0.6, depthTest: false }));
trail.frustumCulled = false;
trail.renderOrder = 7;
dyn.add(trail);
function clearTrail() { trailN = 0; trailGeom.setDrawRange(0, 0); }

function onRobot(m) {
  robot.position.set(...m.p);
  robot.quaternion.set(...m.q);
  robot.visible = true;
  const last = trailN ? new THREE.Vector3(trailPts[3 * trailN - 3], trailPts[3 * trailN - 2], trailPts[3 * trailN - 1]) : null;
  if (!last || last.distanceTo(robot.position) > TRAIL_STEP_M) {
    if (trailN === TRAIL_MAX) { trailPts.copyWithin(0, 3); trailN--; }
    trailPts.set(m.p, 3 * trailN++);
    trailGeom.attributes.position.needsUpdate = true;
    trailGeom.setDrawRange(0, trailN);
  }
}

// ------------------------------------------------------------------------- markers
const markerLayer = new MarkerLayer(dyn);
let statusText = '';

function onMarkers(msg) {
  statusText = markerLayer.update(msg.markers, floorOdomZ).statusText;
  $('status').textContent = statusText || 'no target yet';
  hudDirty = true;
}

// ----------------------------------------------------------------- head-locked VR panel
const hudCanvas = document.createElement('canvas');
hudCanvas.width = 640; hudCanvas.height = 320;
const hudTex = new THREE.CanvasTexture(hudCanvas);
const hudMesh = new THREE.Mesh(new THREE.PlaneGeometry(0.64, 0.32),
  new THREE.MeshBasicMaterial({ map: hudTex, transparent: true, depthTest: false }));
hudMesh.position.set(0, -0.28, -1.1);
hudMesh.renderOrder = 100;
hudMesh.visible = false;
camera.add(hudMesh);
let hudDirty = true;
function drawHud() {
  const g = hudCanvas.getContext('2d');
  g.clearRect(0, 0, 640, 320);
  g.fillStyle = 'rgba(15,18,22,0.78)'; g.fillRect(0, 0, 640, 320);
  g.fillStyle = '#e8eaed'; g.font = '24px monospace'; g.textBaseline = 'top';
  (statusText || 'no target yet').split('\n').slice(0, 9).forEach((l, i) => g.fillText(l, 14, 12 + i * 33, 612));
  hudTex.needsUpdate = true;
  hudDirty = false;
}

// ---------------------------------------------------------------------------- XR
let dollhouse = false;
function setDollhouse(on) {
  dollhouse = on;
  viewRoot.scale.setScalar(on ? DOLLHOUSE_SCALE : 1);
  viewRoot.position.copy(on ? DOLLHOUSE_POS : new THREE.Vector3());
}
renderer.xr.addEventListener('sessionstart', () => {
  controls.enabled = false;
  rig.position.set(0, 0, 0); rig.rotation.set(0, 0, 0);
  camera.position.set(0, 0, 0);            // pose now comes from the headset
  hudMesh.visible = true;
  setDollhouse(false);                     // first person at the start point, outside the structure
});
renderer.xr.addEventListener('sessionend', () => {
  controls.enabled = true;
  hudMesh.visible = false;
  setDollhouse(false);
  rig.position.set(0, 0, 0); rig.rotation.set(0, 0, 0);
  if (buildingBox) frameDesktopCamera();
});

const prevButtons = new Map();
const tmpDir = new THREE.Vector3(), tmpRight = new THREE.Vector3(), tmpHead = new THREE.Vector3();
const UP = new THREE.Vector3(0, 1, 0);
function pressed(src, idx) {
  const now = !!src.gamepad.buttons[idx]?.pressed, k = `${src.handedness}${idx}`;
  const edge = now && !prevButtons.get(k);
  prevButtons.set(k, now);
  return edge;
}
function turnRig(angle) {
  tmpHead.setFromMatrixPosition(renderer.xr.getCamera().matrixWorld);
  rig.position.sub(tmpHead).applyAxisAngle(UP, angle).add(tmpHead);
  rig.rotation.y += angle;
}
let snapReady = true;
function pollControllers(dt) {
  const session = renderer.xr.getSession();
  if (!session) return;
  for (const src of session.inputSources) {
    const gp = src.gamepad;
    if (!gp) continue;
    const x = gp.axes[2] ?? 0, y = gp.axes[3] ?? 0;
    if (src.handedness === 'left' && !dollhouse && Math.hypot(x, y) > 0.15) {
      renderer.xr.getCamera().getWorldDirection(tmpDir);
      tmpDir.y = 0; tmpDir.normalize();
      tmpRight.crossVectors(tmpDir, UP);
      rig.position.addScaledVector(tmpDir, -y * MOVE_SPEED * dt).addScaledVector(tmpRight, x * MOVE_SPEED * dt);
    }
    if (src.handedness === 'right') {
      if (Math.abs(x) > 0.7 && snapReady) { turnRig(-Math.sign(x) * THREE.MathUtils.degToRad(SNAP_DEG)); snapReady = false; }
      if (Math.abs(x) < 0.3) snapReady = true;
      if (pressed(src, 4)) setDollhouse(!dollhouse);
      if (pressed(src, 5)) clearTrail();
    }
  }
}

// -------------------------------------------------------------------- UI and stream
let xray = true;
function toggleXray() {
  xray = !xray;
  for (const m of glassMaterials) { m.opacity = xray ? XRAY_OPACITY : 1; m.transparent = xray; m.depthWrite = !xray; m.needsUpdate = true; }
}
$('xray').addEventListener('click', toggleXray);
$('trail').addEventListener('click', clearTrail);
const thermal = createThermalPanel();
$('thermalBtn').addEventListener('click', () => thermal.toggle());
window.addEventListener('keydown', (e) => {
  if (e.key === 'x' || e.key === 'X') toggleXray();
  if (e.key === 'c' || e.key === 'C') clearTrail();
  if (e.key === 't' || e.key === 'T') thermal.toggle();
});
window.addEventListener('resize', () => {
  camera.aspect = window.innerWidth / window.innerHeight;
  camera.updateProjectionMatrix();
  renderer.setSize(window.innerWidth, window.innerHeight);
});

if (!window.isSecureContext) {
  $('warn').style.display = 'block';
  $('warn').textContent = 'Not a secure context: WebXR (headset) is unavailable here. The desktop view works. ' +
    'For the headset use http://localhost (PC browser + Link) or the bridge with --https.';
}

function connect() {
  const es = new EventSource('/events');
  es.onopen = () => { $('conn').className = 'ok'; $('conn').textContent = 'connected to bridge'; };
  es.onerror = () => { $('conn').className = 'bad'; $('conn').textContent = 'bridge not reachable, retrying...'; };
  es.onmessage = (ev) => {
    const m = JSON.parse(ev.data);
    if (m.type === 'markers') onMarkers(m);
    else if (m.type === 'robot') onRobot(m);
    else if (m.type === 'thermal') thermal.push(m);
  };
}

// ------------------------------------------------------------------------- main loop
document.body.appendChild(VRButton.createButton(renderer));
let lastT = 0;
renderer.setAnimationLoop((t) => {
  const dt = Math.min((t - lastT) / 1000, 0.1);
  lastT = t;
  if (renderer.xr.isPresenting) { pollControllers(dt); if (hudDirty) drawHud(); } else controls.update();
  renderer.render(scene, camera);
});

fetch('/config.json').then((r) => r.json()).then((c) => {
  config = c;
  applySpawn(c.spawn_pose);
  loadBuilding();
  connect();
});
