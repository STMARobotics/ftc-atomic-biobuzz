/*
 * Shoot-on-the-move zone solver.
 *
 * Goal: let the HOOD absorb as much of the shot as possible so the flywheel RPM changes as little as possible.
 *
 *   Phase 1  On the cell's axis, find which ball speeds let a hood in [HOOD_MIN, HOOD_MAX] hit each distance,
 *            including the swing that driving at ROBOT_SPEED adds. Pick the fewest RPM presets that cover it.
 *   Phase 2  For every spot on the field, with the "robot checks the shot" physics solver (full 3D flight,
 *            robot velocity added, drag), classify the spot:
 *              A  SOTM with the RPM held at a preset (hood does all the work)
 *              B  SOTM, but the flywheel has to change speed
 *              C  stationary shots only
 *              D  no shot
 *
 * Run:  node tools/sotm_zones.js                (prints the presets and zone areas)
 *       node tools/sotm_zones.js --json out.json (also writes the field grid for the map)
 * Options: --hood=65,75   --speed=24 (in/s)   --step=4 (grid spacing, in)
 *
 * All the physics inputs are the same estimates used everywhere else (3 in wheel at 0.45 slip, drag from a 25 g
 * wiffle ball, the tilted cell opening from the manual). "A hit" means the ball goes through the opening with the
 * ball's radius counted AND still goes through if the hood is off by 1 deg, the speed by 1.5%, or the aim by 0.7 deg.
 */
const fs = require("fs");
const IN = 0.0254, G = 9.81;
const TL = 30 * Math.PI / 180;
const NI = [0, -Math.sin(TL), -Math.cos(TL)], TV = [0, Math.cos(TL), -Math.sin(TL)];
const CY = 59.55 * IN, TX = 59.25 * IN, TZ = 88 * IN, EXH = (248.2 / 25.4 + 1.5) * IN, FWD = 6 * IN;
const kD = 1.2 * 0.5 * Math.PI * Math.pow(2.8 * IN / 2, 2) / (2 * 0.025);
const K = 0.45 * Math.PI * 3 * IN / 60;                    // ball m/s per wheel rpm (3 in wheel, 0.45 slip)

const args = process.argv.slice(2);
const arg = (n, d) => { const a = args.find(x => x.startsWith("--" + n + "=")); return a ? a.split("=")[1] : d; };
const [HOOD_MIN, HOOD_MAX] = arg("hood", "65,75").split(",").map(Number);
const ROBOT_SPEED = Number(arg("speed", "24")) * IN;
const STEP = Number(arg("step", "4"));
const SPAN = Number(arg("spanrpm", "500")) * K;
const THR = Number(arg("thr", "0.9"));
const RPM_MIN = Number(arg("rpmmin", "2500")), RPM_MAX = Number(arg("rpmmax", "4800")), V_MIN = RPM_MIN * K, V_MAX = RPM_MAX * K;

function flight(px0, pz0, Vx, Vz, yaw, hdDeg, v) {
  const hd = hdDeg * Math.PI / 180, dt = 0.003;
  let px = px0 + Math.sin(yaw) * FWD, py = EXH, pz = pz0 + Math.cos(yaw) * FWD;
  let vx = Vx + v * Math.cos(hd) * Math.sin(yaw), vy = v * Math.sin(hd), vz = Vz + v * Math.cos(hd) * Math.cos(yaw);
  let s0 = (px - TX) * NI[0] + (py - CY) * NI[1] + (pz - TZ) * NI[2];
  for (let i = 0; i < 700; i++) {
    const sp = Math.sqrt(vx * vx + vy * vy + vz * vz);
    vx -= kD * sp * vx * dt; vy += (-G - kD * sp * vy) * dt; vz -= kD * sp * vz * dt;
    px += vx * dt; py += vy * dt; pz += vz * dt;
    if (py < 0) return null;
    const s = (px - TX) * NI[0] + (py - CY) * NI[1] + (pz - TZ) * NI[2];
    if (s0 < 0 && s >= 0) {
      const cn = Math.max((vx * NI[0] + vy * NI[1] + vz * NI[2]) / Math.max(sp, 1e-9), 0.2);
      return { a: ((py - CY) * TV[1] + (pz - TZ) * TV[2]) / IN, b: (px - TX) / IN, cn, t: (i + 1) * dt };
    }
    s0 = s;
  }
  return null;
}
const hitOK = r => r && Math.abs(r.a) <= 7 - 1.8 / r.cn && Math.abs(r.b) <= 8.2;
const PERT = [[0, 0, 0], [1, 0, 0], [-1, 0, 0], [0, 0.015, 0], [0, -0.015, 0], [0, 0, 0.7], [0, 0, -0.7]];
function robust(px, pz, Vx, Vz, yaw, hood, v) {
  for (const [dh, dv, dy] of PERT) if (!hitOK(flight(px, pz, Vx, Vz, yaw + dy * Math.PI / 180, hood + dh, v * (1 + dv)))) return false;
  return true;
}
function aimYaw0(px, pz, Vx, Vz) {
  const tof = 0.7;
  return Math.atan2(TX - Vx * tof - px, TZ - Vz * tof - pz);
}
/** For a given hood and speed, find the aim that puts the ball through the opening's centre line sideways. */
function solveYaw(px, pz, Vx, Vz, hood, v, yaw) {
  for (let it = 0; it < 4; it++) {
    const r = flight(px, pz, Vx, Vz, yaw, hood, v);
    if (!r) return null;
    if (Math.abs(r.b) < 0.05) return { yaw, a: r.a, b: r.b };
    const r2 = flight(px, pz, Vx, Vz, yaw + 0.5 * Math.PI / 180, hood, v);
    if (!r2) return null;
    const d = (r2.b - r.b) / 0.5;
    if (Math.abs(d) < 1e-3) return null;
    yaw -= 0.9 * (r.b / d) * Math.PI / 180;
  }
  const r = flight(px, pz, Vx, Vz, yaw, hood, v);
  return r ? { yaw, a: r.a, b: r.b } : null;
}
/** Speed fixed: use only the hood (and aim) to hit. Returns {hood, yaw} or null. */
function hoodOnly(px, pz, Vx, Vz, v) {
  let yaw = aimYaw0(px, pz, Vx, Vz), prev = null, best = null;
  for (let h = HOOD_MIN; h <= HOOD_MAX + 1e-9; h += 1) {
    const s = solveYaw(px, pz, Vx, Vz, h, v, yaw);
    if (!s) { prev = null; continue; }
    yaw = s.yaw;
    if (prev && (prev.a > 0) !== (s.a > 0)) {
      const f = prev.a / (prev.a - s.a), h0 = prev.h + f * (h - prev.h);
      const s2 = solveYaw(px, pz, Vx, Vz, h0, v, yaw);
      if (s2 && robust(px, pz, Vx, Vz, s2.yaw, h0, v)) {
        const cand = { hood: h0, yaw: s2.yaw };
        if (!best || Math.abs(h0 - 70) < Math.abs(best.hood - 70)) best = cand;
      }
    }
    prev = { h, a: s.a };
  }
  return best;
}
/** Speed may change: the least change from vCentre that works, within +/- span. */
function freeSolve(px, pz, Vx, Vz, vCentre, span) {
  const cands = [];
  for (let d = 0; d <= span + 1e-9; d += 0.1) { cands.push(vCentre + d); if (d > 0) cands.push(vCentre - d); }
  for (const v of cands) {
    if (v < V_MIN || v > V_MAX) continue;
    const r = hoodOnly(px, pz, Vx, Vz, v);
    if (r) return { v, ...r };
  }
  return null;
}

/* ------------------------------------------------------------------------------- Phase 1 */
const tofAt = {};                                         // measured flight time on the axis, per distance
const tofOf = d => tofAt[d] || 0.55 + (d - 44) * 0.0093;   // fallback guess where no shot exists
function onAxisBand(dist) {
  const px = TX, pz = TZ + dist * IN, good = [];
  let tsum = 0;
  for (let v = V_MIN; v <= V_MAX + 1e-9; v += 0.05) {
    const r = hoodOnly(px, pz, 0, 0, v);
    if (r) { good.push(v); tsum += flight(px, pz, 0, 0, r.yaw, r.hood, v).t; }
  }
  if (good.length) tofAt[dist] = tsum / good.length;
  return good;
}
console.log(`hood ${HOOD_MIN} to ${HOOD_MAX} deg, flywheel ${RPM_MIN} to ${RPM_MAX} rpm, driving ${(ROBOT_SPEED / IN).toFixed(0)} in/s`);
console.log("Phase 1: which ball speeds let the hood alone cover each distance (on the cell's axis)...");
const dists = [], bands = {};
for (let d = 30; d <= 104; d += 2) { const g = onAxisBand(d); bands[d] = new Set(g.map(x => Math.round(x * 100))); if (g.length) dists.push(d); }
const swing = d => Math.round(ROBOT_SPEED / IN * tofOf(d) / 2) * 2;
function scoreAt(d, v) {                                // share of the driving swing around d that preset v can hit with the hood alone
  const s = swing(d);
  let n = 0, ok = 0;
  for (let dd = d - s; dd <= d + s; dd += 2) {
    if (dd < 30 || dd > 104) continue;                  // beyond where any shot is possible, nothing to cover
    n++; if (bands[dd] && bands[dd].has(Math.round(v * 100))) ok++;
  }
  return n ? ok / n : 1;
}
const speeds = []; for (let v = V_MIN; v <= V_MAX + 1e-9; v += 0.05) speeds.push(Math.round(v * 100) / 100);
const coverDists = dists.filter(d => d >= 32 && d <= 100);
function best(Kp) {
  let bestSet = null, bestCov = -1;
  const rec = (start, chosen) => {
    if (chosen.length === Kp) {
      let cov = 0;
      for (const d of coverDists) cov += Math.max(...chosen.map(v => scoreAt(d, v)));
      if (cov > bestCov) { bestCov = cov; bestSet = chosen.slice(); }
      return;
    }
    for (let i = start; i < speeds.length; i += 3) { chosen.push(speeds[i]); rec(i + 1, chosen); chosen.pop(); }
  };
  rec(0, []);
  return { set: bestSet, cov: bestCov / coverDists.length };
}
const presets = {};
for (const Kp of [1, 2, 3, 4]) {
  const r = best(Kp); presets[Kp] = r;
  console.log(`  ${Kp} preset${Kp > 1 ? "s" : " "}: ${r.set.map(v => Math.round(v / K / 10) * 10 + " rpm").join(", ")}  the hood alone covers ${(r.cov * 100).toFixed(0)}% of the driving swing, averaged over 32 to 100 in`);
}
module.exports = { presets, K, flight, hoodOnly, solveYaw, robust, aimYaw0, hitOK, freeSolve };
if (args.includes("--windows")) {
  console.log("\nHow wide a stretch of distances one fixed ball speed covers with the hood alone, for this hood range:");
  let widest = 0;
  for (let v = 5.0; v <= 8.0 + 1e-9; v += 0.25) {
    let lo = null, hi = null;
    for (let d = 28; d <= 104; d += 2) if (hoodOnly(TX, TZ + d * IN, 0, 0, v)) { if (lo === null) lo = d; hi = d; }
    if (lo !== null) { widest = Math.max(widest, hi - lo + 2); console.log(`  ${v.toFixed(2)} m/s (${String(Math.round(v / K / 10) * 10).padStart(4)} rpm): ${lo} to ${hi} in  (${hi - lo + 2} in wide)`); }
  }
  console.log(`  widest window: ${widest} in`);
  process.exit(0);
}
if (require.main === module && args.some(a => a.startsWith("--json"))) {
  let PK = 1; while (PK < 4 && (presets[PK].cov < 0.9) && presets[PK + 1].cov - presets[PK].cov >= 0.03) PK++;
  const PRESETS = presets[PK].set.slice().sort((a, b) => a - b);
  console.log(`\nPhase 2 uses ${PK} preset${PK > 1 ? "s" : ""}: ${PRESETS.map(v => Math.round(v / K / 10) * 10).join(", ")} rpm`);
  const DIRS = 16, X0 = 6, X1 = 138, Z0 = 6, Z1 = 138;
  const nx = Math.floor((X1 - X0) / STEP) + 1, nz = Math.floor((Z1 - Z0) / STEP) + 1;
  const grid = { cls: [], fHood: [], fFree: [], dRpm: [], pre: [], vLo: [], vHi: [] };
  const t0 = Date.now(); let done = 0;
  for (let iz = 0; iz < nz; iz++) for (let ix = 0; ix < nx; ix++) {
    const x = X0 + ix * STEP, z = Z0 + iz * STEP;
    const idx = iz * nx + ix;
    const wall = x < 9 || x > 135 || z < 9 || z > 135;
    const ROBOT_HALF = 9;                                              // robot centre can't get closer than this to the hive
    const hive = x > 46 - ROBOT_HALF && x < 98 + ROBOT_HALF && z > 51 - ROBOT_HALF && z < 93 + ROBOT_HALF;
    const set = (c, fh = 0, ff = 0, dr = 0, pre = 0, lo = 0, hi = 0) => { grid.cls[idx] = c; grid.fHood[idx] = fh; grid.fFree[idx] = ff; grid.dRpm[idx] = dr; grid.pre[idx] = pre; grid.vLo[idx] = lo; grid.vHi[idx] = hi; };
    if (wall || hive) { set(-1); continue; }
    const px = x * IN, pz = z * IN;
    if (pz <= TZ) { set(3); continue; }                                  // beside or behind the cell
    // stationary: which speeds work with the hood alone from here
    let vLo = null, vHi = null;
    for (let v = V_MIN; v <= V_MAX + 1e-9; v += 0.1) if (hoodOnly(px, pz, 0, 0, v)) { if (vLo === null) vLo = v; vHi = v; }
    if (vLo === null) { set(3); continue; }
    const vMid = (vLo + vHi) / 2;
    const inBand = PRESETS.filter(p => p >= vLo - 1e-6 && p <= vHi + 1e-6);
    const pre = inBand.length ? inBand.reduce((a, b) => Math.abs(a - vMid) < Math.abs(b - vMid) ? a : b) : null;
    let nh = 0, nf = 0, maxd = 0;
    for (let k = 0; k < DIRS; k++) {
      const ang = 2 * Math.PI * k / DIRS, Vx = ROBOT_SPEED * Math.cos(ang), Vz = ROBOT_SPEED * Math.sin(ang);
      if (pre !== null && hoodOnly(px, pz, Vx, Vz, pre)) { nh++; nf++; continue; }
      const f = freeSolve(px, pz, Vx, Vz, pre !== null ? pre : vMid, SPAN);
      if (f) { nf++; maxd = Math.max(maxd, Math.abs(f.v - (pre !== null ? pre : vMid)) / K); }
    }
    const fh = nh / DIRS, ff = nf / DIRS;
    const cls = fh >= THR ? 0 : ff >= THR ? 1 : 2;
    set(cls, +fh.toFixed(3), +ff.toFixed(3), Math.round(maxd), pre === null ? 0 : Math.round(pre / K / 10) * 10, +vLo.toFixed(2), +vHi.toFixed(2));
    if (++done % 20 === 0) console.log(`  ${done} stationary-capable spots done (${((Date.now() - t0) / 1000).toFixed(0)} s)`);
  }
  const names = ["A SOTM, RPM held at a preset (hood does the work)", "B SOTM, flywheel has to change speed", "C stationary shots only", "D no shot"];
  const counts = [0, 0, 0, 0]; let usable = 0;
  grid.cls.forEach(c => { if (c >= 0) { counts[c]++; usable++; } });
  console.log("\nZones over the reachable field (excluding the hive and a 9 in wall margin):");
  counts.forEach((n, i) => console.log(`  ${names[i]}: ${(n / usable * 100).toFixed(0)}%  (${n} of ${usable} spots)`));
  const out = { hood: [HOOD_MIN, HOOD_MAX], rpmRange: [RPM_MIN, RPM_MAX], robotSpeed: Math.round(ROBOT_SPEED / IN), step: STEP, x0: X0, z0: Z0, nx, nz,
    presets: PRESETS.map(v => Math.round(v / K / 10) * 10), grid, counts, usable };
  fs.writeFileSync(arg("json", "zones.json"), JSON.stringify(out));
  console.log("wrote " + arg("json", "zones.json"));
}
