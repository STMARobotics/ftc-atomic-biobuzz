import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { GLTFLoader } from 'three/addons/loaders/GLTFLoader.js';

const $ = (id) => document.getElementById(id);
const DEG = Math.PI / 180;
let state = null;

// ------------------------------------------------------------------ scene

const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
renderer.shadowMap.enabled = true;
$('view').appendChild(renderer.domElement);

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x14171c);
const camera = new THREE.PerspectiveCamera(50, 1, 0.5, 3000);
camera.position.set(-70, 95, 95);
const controls = new OrbitControls(camera, renderer.domElement);
controls.target.set(0, 6, 0);
controls.enableDamping = true;

scene.add(new THREE.HemisphereLight(0xffffff, 0x334455, 0.9));
const sun = new THREE.DirectionalLight(0xffffff, 1.6);
sun.position.set(60, 160, 40);
sun.castShadow = true;
Object.assign(sun.shadow.camera, { left: -90, right: 90, top: 90, bottom: -90, near: 1, far: 400 });
sun.shadow.mapSize.set(2048, 2048);
scene.add(sun);

// field is Pedro's: 0-144 in, drawn centered. field (x, y) -> three (x - 72, 0, 72 - y)
const fieldToThree = (x, y) => new THREE.Vector3(x - 72, 0, 72 - y);

const field = new THREE.Mesh(new THREE.PlaneGeometry(144, 144),
  new THREE.MeshStandardMaterial({ color: 0x585d64, roughness: 0.95 }));
field.rotation.x = -Math.PI / 2;
field.receiveShadow = true;
scene.add(field);
const grid = new THREE.GridHelper(144, 6, 0x9aa3ad, 0x9aa3ad);
grid.position.y = 0.05;
scene.add(grid);

const wallMat = new THREE.MeshStandardMaterial({ color: 0xb8c4d0, transparent: true, opacity: 0.25 });
for (const [w, d, x, z] of [[146, 1, 0, -72.5], [146, 1, 0, 72.5], [1, 146, -72.5, 0], [1, 146, 72.5, 0]]) {
  const wall = new THREE.Mesh(new THREE.BoxGeometry(w, 12, d), wallMat);
  wall.position.set(x, 6, z);
  scene.add(wall);
}
// Pedro axes at the field origin: red = +x, green = +y
const origin = fieldToThree(0, 0).setY(0.5);
scene.add(new THREE.ArrowHelper(new THREE.Vector3(1, 0, 0), origin, 18, 0xff5555, 4, 3));
scene.add(new THREE.ArrowHelper(new THREE.Vector3(0, 0, -1), origin, 18, 0x55ff77, 4, 3));

// ------------------------------------------------------------------ robot

const robot = new THREE.Group();
scene.add(robot);
const parts = {};
let config = null;
const balls = [];
let launchMarker = null;

function material(cfg) {
  const opacity = cfg.opacity ?? 1;
  return new THREE.MeshStandardMaterial({
    color: cfg.color || '#999999', roughness: 0.6, metalness: 0.1,
    transparent: opacity < 1, opacity,
  });
}

function placeholder(cfg) {
  const c = cfg.center || [0, 0, 0];
  let mesh;
  if (cfg.cylinder) {
    const { radius, width } = cfg.cylinder;
    mesh = new THREE.Mesh(new THREE.CylinderGeometry(radius, radius, width, 28), material(cfg));
    // cylinders line up with the part's axis (wheels default to sideways)
    const axis = new THREE.Vector3(...(cfg.axis || [0, 0, 1])).normalize();
    mesh.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), axis);
  } else {
    mesh = new THREE.Mesh(new THREE.BoxGeometry(...(cfg.box || [1, 1, 1])), material(cfg));
  }
  mesh.position.set(...c);
  mesh.castShadow = true;
  mesh.receiveShadow = true;
  return mesh;
}

function loadGlb(cfg, holder) {
  new GLTFLoader().load(`models/${cfg.file}`, (gltf) => {
    const model = gltf.scene;
    model.scale.setScalar(cfg.scale ?? 1);
    if (cfg.rotation) model.rotation.set(cfg.rotation[0] * DEG, cfg.rotation[1] * DEG, cfg.rotation[2] * DEG);
    model.traverse((o) => { if (o.isMesh) { o.castShadow = true; o.receiveShadow = true; } });
    holder.clear();
    holder.add(model);
  }, undefined, (err) => console.warn(`couldn't load models/${cfg.file}, using the placeholder`, err));
}

async function buildRobot() {
  config = await (await fetch('robot.json', { cache: 'no-store' })).json();
  for (const cfg of config.parts) {
    const pivot = cfg.pivot || [0, 0, 0];
    const parent = cfg.parent ? parts[cfg.parent] : null;
    const parentPivot = parent ? (parent.cfg.pivot || [0, 0, 0]) : [0, 0, 0];
    // group sits on the hinge, holder shifts back so robot-frame coordinates (and CAD) line up
    const group = new THREE.Group();
    group.position.set(pivot[0] - parentPivot[0], pivot[1] - parentPivot[1], pivot[2] - parentPivot[2]);
    (parent ? parent.group : robot).add(group);
    const holder = new THREE.Group();
    holder.position.set(-pivot[0], -pivot[1], -pivot[2]);
    group.add(holder);
    holder.add(placeholder(cfg));
    if (cfg.file) loadGlb(cfg, holder);
    parts[cfg.name] = { cfg, group, axis: new THREE.Vector3(...(cfg.axis || [0, 1, 0])).normalize(), spin: 0 };
  }

  const stack = config.ballStack;
  for (let i = 0; i < 5; i++) {
    const ball = new THREE.Mesh(new THREE.SphereGeometry(stack.radius, 20, 14), material({ color: stack.color }));
    ball.position.set(stack.bottom[0], stack.bottom[1] + i * stack.spacing, stack.bottom[2]);
    ball.castShadow = true;
    robot.add(ball);
    balls.push(ball);
  }

  const turret = parts.turret;
  launchMarker = new THREE.Object3D();
  const p = config.launcher.point;
  const tp = turret ? turret.cfg.pivot : [0, 0, 0];
  launchMarker.position.set(p[0] - tp[0], p[1] - tp[1], p[2] - tp[2]);
  (turret ? turret.group : robot).add(launchMarker);
}

// ------------------------------------------------------------------ shots

const shots = [];
let lastShots = null;
const shotGeo = new THREE.SphereGeometry(1.4, 16, 12);
const shotMat = new THREE.MeshStandardMaterial({ color: 0xffd23f, emissive: 0x332200 });

function launch() {
  if (!launchMarker || !state) return;
  const l = config.launcher;
  const pos = launchMarker.getWorldPosition(new THREE.Vector3());
  const forward = new THREE.Vector3(1, 0, 0).applyQuaternion(launchMarker.getWorldQuaternion(new THREE.Quaternion()));
  forward.y = 0;
  forward.normalize();
  const speed = l.inPerSecAtTarget * Math.max(0, state.flywheelRpm + 350) / l.targetRpm;
  const vel = forward.multiplyScalar(Math.cos(l.elevationDeg * DEG) * speed);
  vel.y = Math.sin(l.elevationDeg * DEG) * speed;
  const mesh = new THREE.Mesh(shotGeo, shotMat);
  mesh.position.copy(pos);
  scene.add(mesh);
  shots.push({ mesh, vel, life: 0 });
}

function stepShots(dt) {
  for (let i = shots.length - 1; i >= 0; i--) {
    const s = shots[i];
    s.vel.y -= 386 * dt;
    s.mesh.position.addScaledVector(s.vel, dt);
    s.life += dt;
    if (s.mesh.position.y < 1.4 || s.life > 4) {
      scene.remove(s.mesh);
      shots.splice(i, 1);
    }
  }
}

// ------------------------------------------------------------------ apply state

const shownPose = { x: 72, y: 72, h: 0, ready: false };

function angleLerp(a, b, t) {
  let d = b - a;
  while (d > Math.PI) d -= 2 * Math.PI;
  while (d < -Math.PI) d += 2 * Math.PI;
  return a + d * t;
}

function updateRobot(dt) {
  if (!state || !config) return;
  const r = state.robot;
  const t = shownPose.ready ? 1 - Math.exp(-dt * 25) : 1;
  shownPose.x += (r.x - shownPose.x) * t;
  shownPose.y += (r.y - shownPose.y) * t;
  shownPose.h = angleLerp(shownPose.h, r.h, t);
  shownPose.ready = true;

  const before = robot.position.clone();
  robot.position.copy(fieldToThree(shownPose.x, shownPose.y));
  robot.rotation.y = shownPose.h;
  if ($('follow').checked) {
    const delta = robot.position.clone().sub(before);
    camera.position.add(delta);
    controls.target.add(delta);
  }

  for (const part of Object.values(parts)) {
    const d = part.cfg.driver;
    if (!d) continue;
    let angle = null;
    if (d.type === 'servo') {
      const s = state.servos[d.servo];
      if (s) angle = (d.angleAt0 + (d.angleAt1 - d.angleAt0) * s.pos) * DEG;
    } else if (d.type === 'spin') {
      const m = state.motors[d.motor];
      // slowed way down so you can see it turn instead of a strobe
      if (m) part.spin += (m.rpm * (d.scale ?? 1)) / 60 * 2 * Math.PI * dt * 0.08;
      angle = part.spin;
    } else if (d.type === 'turret') {
      angle = (state.turretDeg - (d.forwardDeg ?? 0)) * (d.clockwisePositive ? -1 : 1) * DEG;
    }
    if (angle !== null) part.group.quaternion.setFromAxisAngle(part.axis, angle);
  }

  balls.forEach((b, i) => { b.visible = i < state.balls; });

  if (lastShots === null || state.shots < lastShots) lastShots = state.shots;
  while (lastShots < state.shots) { launch(); lastShots++; }
}

// ------------------------------------------------------------------ driver station UI

let opmodesShown = '';

function renderUi() {
  if (!state) return;
  const names = JSON.stringify(state.opmodes);
  if (names !== opmodesShown) {
    opmodesShown = names;
    const sel = $('opmode');
    const keep = sel.value;
    sel.innerHTML = '';
    for (const group of [['TeleOp', false], ['Autonomous', true]]) {
      const og = document.createElement('optgroup');
      og.label = group[0];
      for (const o of state.opmodes.filter((o) => o.auto === group[1])) {
        const opt = document.createElement('option');
        opt.textContent = o.name;
        og.appendChild(opt);
      }
      sel.appendChild(og);
    }
    if (keep) sel.value = keep;
  }
  const phase = state.phase;
  $('phase').textContent = state.opmode ? `${phase} (${state.opmode})` : phase;
  $('phase').className = `phase-${phase}`;
  $('start').disabled = phase !== 'INIT';
  $('stop').disabled = !(phase === 'INIT' || phase === 'RUNNING');
  $('battery').textContent = `${state.battery.toFixed(2)} V`;
  $('fly').textContent = `${Math.round(state.flywheelRpm)} motor rpm`;
  $('balls').textContent = state.balls;
  $('shots').textContent = state.shots;
  $('pose').textContent = `${state.robot.x.toFixed(1)}, ${state.robot.y.toFixed(1)}, ${(state.robot.h / DEG).toFixed(0)}°`;
  $('telemetry').textContent = state.telemetry.length ? state.telemetry.join('\n') : '-';
  $('warnings').innerHTML = '';
  for (const w of state.warnings) {
    const div = document.createElement('div');
    div.textContent = `⚠ ${w}`;
    $('warnings').appendChild(div);
  }
  $('error').textContent = state.error || '';
}

async function cmd(action) {
  const q = action === 'init' ? `&opmode=${encodeURIComponent($('opmode').value)}` : '';
  const res = await fetch(`cmd?action=${action}${q}`, { method: 'POST' });
  if (!res.ok) $('error').textContent = await res.text();
}
for (const a of ['init', 'start', 'stop', 'reset']) $(a).addEventListener('click', () => cmd(a));

const events = new EventSource('events');
events.onmessage = (e) => { state = JSON.parse(e.data); renderUi(); };
events.onerror = () => { $('phase').textContent = 'sim not running'; $('phase').className = 'phase-STOPPED'; };

// ------------------------------------------------------------------ controller + keyboard

const keys = new Set();
const typing = () => document.activeElement && document.activeElement.tagName === 'SELECT';
window.addEventListener('keydown', (e) => {
  if (typing()) return;
  keys.add(e.code);
  if (['Space', 'ArrowUp', 'ArrowDown', 'ArrowLeft', 'ArrowRight'].includes(e.code)) e.preventDefault();
});
window.addEventListener('keyup', (e) => keys.delete(e.code));
window.addEventListener('blur', () => keys.clear());

const dead = (v) => (Math.abs(v) < 0.06 ? 0 : v);

function blankInput() {
  return {
    lx: 0, ly: 0, rx: 0, ry: 0, lt: 0, rt: 0, a: false, b: false, x: false, y: false,
    lb: false, rb: false, up: false, down: false, left: false, right: false, start: false, back: false,
    ls: false, rs: false,
  };
}

function readInput() {
  const g = blankInput();
  const pad = [...(navigator.getGamepads ? navigator.getGamepads() : [])].find((p) => p && p.connected);
  if (pad) {
    const btn = (i) => !!pad.buttons[i] && pad.buttons[i].pressed;
    const val = (i) => (pad.buttons[i] ? pad.buttons[i].value : 0);
    Object.assign(g, {
      lx: dead(pad.axes[0] || 0), ly: dead(pad.axes[1] || 0), rx: dead(pad.axes[2] || 0), ry: dead(pad.axes[3] || 0),
      a: btn(0), b: btn(1), x: btn(2), y: btn(3), lb: btn(4), rb: btn(5), lt: val(6), rt: val(7),
      back: btn(8), start: btn(9), ls: btn(10), rs: btn(11),
      up: btn(12), down: btn(13), left: btn(14), right: btn(15),
    });
    $('pad').textContent = pad.id.slice(0, 32);
  } else {
    $('pad').textContent = 'keyboard (press a pad button to connect)';
  }
  // keyboard adds on top
  const k = (code) => keys.has(code);
  if (k('KeyW')) g.ly = -1;
  if (k('KeyS')) g.ly = 1;
  if (k('KeyA')) g.lx = -1;
  if (k('KeyD')) g.lx = 1;
  if (k('KeyQ')) g.rx = -1;
  if (k('KeyE')) g.rx = 1;
  if (k('ShiftLeft') || k('ShiftRight')) g.lt = 1;
  if (k('Space')) g.rt = 1;
  g.b ||= k('KeyF');
  g.x ||= k('KeyX');
  g.rb ||= k('KeyR');
  g.a ||= k('KeyG');
  g.up ||= k('ArrowUp');
  g.down ||= k('ArrowDown');
  g.left ||= k('ArrowLeft');
  g.right ||= k('ArrowRight');
  g.start ||= k('Enter');
  return g;
}

// send on every change, and keep re-sending while something is held. An idle page (another
// laptop just watching) never sends, so it can't fight whoever is driving
const NEUTRAL = JSON.stringify(blankInput());
let lastSent = NEUTRAL;
let lastSentAt = 0;
setInterval(() => {
  const body = JSON.stringify(readInput());
  const now = performance.now();
  if (body === lastSent && (body === NEUTRAL || now - lastSentAt < 250)) return;
  lastSent = body;
  lastSentAt = now;
  fetch('gamepad', { method: 'POST', body }).catch(() => {});
}, 20);

// ------------------------------------------------------------------ loop

function resize() {
  renderer.setSize(window.innerWidth, window.innerHeight);
  camera.aspect = window.innerWidth / window.innerHeight;
  camera.updateProjectionMatrix();
}
window.addEventListener('resize', resize);
resize();

const clock = new THREE.Clock();
function frame() {
  const dt = Math.min(clock.getDelta(), 0.1);
  updateRobot(dt);
  stepShots(dt);
  controls.update();
  renderer.render(scene, camera);
  requestAnimationFrame(frame);
}

buildRobot().then(frame).catch((e) => { $('error').textContent = `robot.json: ${e}`; frame(); });
