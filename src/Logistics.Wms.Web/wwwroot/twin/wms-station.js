/* WMS-driven variant of the station below (derived from simulation/canvas/project/app.js): the robots, conveyors, operators and
   safety are unchanged, but every decision comes from the Logistics WMS in the parent page and every event is booked there —
   R1 tasks (with the occupancy of its transfer heads), routing at the decision point, picks allocated at the stops, sortation
   targets from the bar-code arch, put-wall orders and lane steps, restock, exception tote. Robots the WMS reports as not available
   (fault, manual, maintenance) stand still. The WMS clock follows the simulation time (it is passed with every call).

   Station: logistics application — goods-to-robot order fulfilment, after the ABB Robotics video "ABB Robotics | Logistics"
   (youtube.com/watch?v=uUGB30RUQlA, Cainiao / Dasu warehouse: an IRB 6700 inside an automated storage and retrieval cylinder shelf
   of red bar-coded bins picks the bins an order wave needs and sends them on a conveyor; at the picking station two IRB 1200 with
   3D vision and vacuum grippers pick the goods out of the bins onto the operating line ("wave picking — each wave may include ten or
   twenty orders, the system adds up the orders"); at the sortation station two IRB 360 scan the bar codes and put the goods into the
   order cases; storage, picking and sortation completely automatic), rebuilt with library robots: R1 IRB 6700-150/3.20 (fork in
   side mode, A6 axis = fork plane) in the cylinder, R2 / R3 IRB 1300-12/1.4 (4-cup vacuum gripper, 3D camera over each bin stop),
   S1 / S2 IRB 930-22/1.05 SCARAs for the IRB 360 deltas (the IRB 920T-6/0.55 named in the feasibility study cannot reach the belt and
   four put-wall cases with one elbow; the 1.05 m IRB 930 can) — kinematics and joint limits from the RobotStudio PIM.xml.
   Units m, Z-up, R1 base = cylinder centre = origin.
   Changes against the video (which shows neither): bins that run empty go out on a restock loop through the west port of the
   cylinder to an operator outside (refill, back in); the order cases sit on 8 powered put-wall lanes through the north fence — a
   finished case runs out to the packing operator, an empty one runs in (rolling waves: a new order starts as soon as its lane has an
   empty case); no-read items run off the belt end into an exception tote outside; one guarded zone with guard-locked doors. */
(function () {
'use strict';
const RC = window.RobotCell; const { mat, box, cylZ, cylX, clamp, rand, lerp } = RC.h;
const S = RC.sim;
const TAU = 2 * Math.PI, D2R = Math.PI / 180;

// ------------------------------------------------------------------ WMS bridge (set up by twin.js: RC.wms.bridge, RC.wms.init)
const WMS = RC.wms = RC.wms || {};
Object.assign(WMS, { robots: {}, orders: null, opts: null, tote: 0, errors: [], calls: 0, pollReq: null, pollAt: 0 });
/* asynchronous WMS call; the returned object completes later: { done, ok, value, error } (generators wait on r.done) */
function wmsCall(name, ...args) {
  const r = { done: false, ok: false, value: null, error: null };
  WMS.calls++;
  if (!WMS.bridge) { r.done = true; r.error = 'WMS not connected'; return r; }
  WMS.bridge.invoke(name, S.t, ...args).then(v => {
    r.ok = !!(v && v.ok); r.value = v ? v.data : null; r.error = v && !v.ok ? v.error || 'refused' : null; r.done = true;
    if (r.error) wmsError(name, r.error);
  }, e => { r.error = String((e && e.message) || e); r.done = true; wmsError(name, r.error); });
  return r;
}
function wmsError(name, msg) { WMS.errors.push({ t: S.t, name, msg }); if (WMS.errors.length > 30) WMS.errors.shift(); console.warn('WMS ' + name + ': ' + msg); }
/* the WMS gives a robot work only in Auto mode without a fault; the 3D robot stands still otherwise */
const robotOk = id => { const r = WMS.robots[id.toUpperCase()]; return !r || r.available; };
const robotWhy = id => { const r = WMS.robots[id.toUpperCase()]; return !r ? '' : r.state === 'Fault' ? 'WMS: fault ' + (r.faultCode || '') : 'WMS: ' + r.mode + ' mode'; };
const wrap = a => Math.atan2(Math.sin(a), Math.cos(a));

// ------------------------------------------------------------------ layout (m, s)
const K = {
  /* cylinder shelf: 26 columns × 8 levels, bins radial (0.6 deep), front face r 2.0; columns left out: east port (25, 0, 1) for the
     picking loop, west port (12, 13, 14) for the restock loop, north passage (6, 7) for maintenance */
  ring: { rIn: 2.0, rOut: 2.65, rc: 2.3, cols: 26, lev: 8, z0: 0.30, pitch: 0.37, h: 3.75, gaps: [25, 0, 1, 12, 13, 14, 6, 7] },   // top ring 3.75: the R1 wrist passes under it at level 8
  bin: { L: 0.6, W: 0.4, H: 0.28, wall: 0.02, fl: 0.03 },
  rob: {
    r1: { base: { x: 0, y: 0 }, yaw: 0, ped: 0.4, tool: 0.30, beam: 0.55, rS: 0.9 },   // L-fork under the flange: TCP = fork plane on the A6 axis, bin centre 0.55 m ahead (wrist 0.1 m inside the shelf face)
    r2: { base: { x: 4.3, y: 1.15 }, yaw: Math.PI, ped: 0.4, tool: 0.16 },             // 4-cup vacuum gripper, TCP = cup face; A1 = 0 faces west (bin ↔ belt swings pass the west side)
    r3: { base: { x: 5.8, y: 1.15 }, yaw: Math.PI, ped: 0.4, tool: 0.16 },
    s1: { base: { x: 7.3, y: 2.55 }, yaw: Math.PI, ped: 1.05, tool: 0.12, elbow: -1, pickX: 7.2 },   // A1 = 0 faces west, elbow −1: belt + 4 lanes in one arm configuration
    s2: { base: { x: 9.1, y: 2.55 }, yaw: 0, ped: 1.05, tool: 0.12, elbow: 1, pickX: 9.2 },         // mirror image
  },
  zc: 0.75, laneY: 0.38, vConv: 0.45, vRat: 0.35,
  belt: { y: 1.95, x0: 3.75, x1: 10.35, z: 0.80, w: 0.40, v: 0.45, scanX: 6.35, endX: 10.3 },
  cam: { z: 2.2, postY: -0.95 },
  pw: { y0: 3.0, y1: 4.8, yIn: 3.2, yOut: 4.55, z: 0.58, w: 0.37, v: 0.4, xs: { s1: [6.7, 7.1, 7.5, 7.9], s2: [8.5, 8.9, 9.3, 9.7] }, curtainY: 4.3, roof: 0.92 },
  cs: { L: 0.35, W: 0.30, H: 0.22 },                                                      // order case (carton) on the put-wall lane
  fence: { x0: -3.2, x1: 10.6, y0: -3.2, y1: 3.45, h: 2.0 },
  rst: { tunX: -4.25, opX: -5.45, roof: 1.12 },
  ex: { x: 11.4, y: 1.95, cap: 25 },
  door: { n: [0.5, 1.4], s: [4.6, 5.5] },
  scanR: 4.0,
  walk: { v: 1.0, x0: 4.2, x1: 11.95, y0: -3.75, y1: 5.1, idle: { x: 8.2, y: 5.1 } },
};
const F = K.fence, RG = K.ring, BN = K.bin, BL = K.belt, PW = K.pw;
const DTH = TAU / RG.cols;
const Z = { r1In: K.zc - 0.015, r1Up: K.zc + 0.015, pCarry: 1.28, sCarry: 1.14, caseRim: PW.z + K.cs.H, binFloor: K.zc + BN.fl, rim: K.zc + BN.H };
const motion = {
  r1: { vMax: 2.5, vSlow: 0.1, slowZone: 0.03, accel: 8, yawR: 0.5 },    // IRB 6700, fork 45 kg + bin ≤ 20 kg (programmed, assumption)
  r2: { vMax: 2.2, vSlow: 0.05, slowZone: 0.02, accel: 10, yawR: 0.1 },  // IRB 1300, 4-cup gripper + carton ≤ 2 kg
  r3: { vMax: 2.2, vSlow: 0.05, slowZone: 0.02, accel: 10, yawR: 0.1 },
  s1: { vMax: 2.5, vSlow: 0.05, slowZone: 0.02, accel: 12, yawR: 0.05 }, // IRB 930, bellows cup + carton ≤ 2 kg
  s2: { vMax: 2.5, vSlow: 0.05, slowZone: 0.02, accel: 12, yawR: 0.05 },
};
const V0 = Object.fromEntries(Object.entries(motion).map(([k, m]) => [k, m.vMax]));
const ctx = id => RC.ctxById(id);
const zStop = () => S.estop || S.zone.stopped;

// ------------------------------------------------------------------ catalogue: 40 SKUs, cartons in three sizes (grid per bin layer)
const SIZE = {
  S: { l: 0.12, w: 0.09, h: 0.06, nx: 4, ny: 4, kg: 0.35 },
  M: { l: 0.16, w: 0.12, h: 0.08, nx: 3, ny: 3, kg: 0.7 },
  L: { l: 0.24, w: 0.17, h: 0.11, nx: 2, ny: 2, kg: 1.6 },
};
for (const z of Object.values(SIZE)) z.cap = z.nx * z.ny * 2;
const LABELS = [0x33b24a, 0xd8262b, 0x2f8fff, 0xf2c200, 0x9b59b6, 0xff7a1a];
const SKUS = [];                                                             // filled from the WMS (code, size), see loadSkus()
function loadSkus() { SKUS.length = 0; (WMS.init.skus || []).forEach((k, i) => SKUS.push({ i, code: k.code, size: k.size, lab: LABELS[i % LABELS.length] })); }
const dimOf = sku => SIZE[SKUS[sku].size];

// ------------------------------------------------------------------ state
const G = {}; const M = {}; const GEO = {};
const SLOTS = []; const BINS = []; const ITEMPOOL = []; const BELT = []; const LANES = []; const ORDERS = { queue: [], active: [], seq: 0, done: 0, doneT: [] };
const LOOPS = {}; const ST = {}; const EXC = { n: 0, items: [] };
Object.assign(S, { mode: 'zipf', speedPct: 100, noRead: 0.005, refillT: 35, linesMax: 4, qtyMax: 3, interlocks: {} });

function mats() {
  Object.assign(M, {
    frame: mat(0x9aa3ab, { metalness: 0.5, roughness: 0.4 }), dark: mat(0x22272c, { metalness: 0.4, roughness: 0.5 }), black: mat(0x15181b, { roughness: 0.6 }),
    white: mat(0xe9ecef, { roughness: 0.5, metalness: 0.2 }), truss: mat(0xdfe3e7, { roughness: 0.45, metalness: 0.35 }), steel: mat(0x8d949a, { metalness: 0.7, roughness: 0.35 }),
    bin: mat(0xd42a2f, { roughness: 0.55 }), binIn: mat(0x7a1418, { roughness: 0.7 }), label: mat(0xf4f4f4, { roughness: 0.6 }),
    carton: mat(0xc49a6c, { roughness: 0.8 }), case: mat(0xb88a58, { roughness: 0.85 }), roller: mat(0xb7bec5, { metalness: 0.6, roughness: 0.35 }),
    belt: mat(0x2b3036, { roughness: 0.8 }), chain: mat(0x3a4048, { metalness: 0.5, roughness: 0.5 }), blue: mat(0x2f5fa8, { roughness: 0.6 }), sick: mat(0x2a9fd6, { roughness: 0.5 }),
    yellow: mat(0xf2c200, { roughness: 0.5, metalness: 0.2 }), green: mat(0x2f8a4e, { roughness: 0.55 }), grey: mat(0x6d747b, { roughness: 0.6 }), red: mat(0xd8262b, { roughness: 0.5 }),
    tote: mat(0x2f5fa8, { roughness: 0.7 }), pallet: mat(0xa3794b, { roughness: 0.85 }),
    skirt: new THREE.MeshStandardMaterial({ color: 0x3a434b, transparent: true, opacity: 0.35, side: THREE.DoubleSide, roughness: 0.8, depthWrite: false }),
    proj: new THREE.MeshBasicMaterial({ color: 0x9cff3a, transparent: true, opacity: 0.16, side: THREE.DoubleSide, depthWrite: false }),
    scanBeam: new THREE.MeshBasicMaterial({ color: 0xff3030, transparent: true, opacity: 0.5, side: THREE.DoubleSide, depthWrite: false }),
    shirt: mat(0x2f5fa8, { roughness: 0.8 }), shirt2: mat(0x2f8a4e, { roughness: 0.8 }), pants: mat(0x2a2f38, { roughness: 0.85 }), skin: mat(0xd9a882, { roughness: 0.7 }),
  });
  M.labs = LABELS.map(c => mat(c, { roughness: 0.6 }));
  M.cartonFocus = mat(0xff8a3d, { roughness: 0.55, emissive: 0x7a2a00, emissiveIntensity: 0.7 });   // cartons of the order in focus
  for (const [k, z] of Object.entries(SIZE)) GEO[k] = { box: new THREE.BoxGeometry(z.l, z.w, z.h), lab: new THREE.BoxGeometry(z.l * 0.35, z.w * 0.5, 0.002) };
  GEO.binWallL = new THREE.BoxGeometry(BN.L, BN.wall, BN.H); GEO.binWallS = new THREE.BoxGeometry(BN.wall, BN.W - 2 * BN.wall, BN.H); GEO.binFloor = new THREE.BoxGeometry(BN.L - 0.02, BN.W - 0.02, BN.fl);
  GEO.binLab = new THREE.BoxGeometry(0.004, 0.14, 0.07);
}
const lampMat = c => mat(c, { emissive: c, emissiveIntensity: 1.2 });
const setLamp = (m, c) => { if (!m) return; m.material.color.set(c); m.material.emissive.set(c); };

// ------------------------------------------------------------------ items (cartons): pooled groups, origin = bottom centre, local x = long side
function takeItem(sku) {
  const sz = SKUS[sku].size; let it = ITEMPOOL.find(q => q.free && q.sz === sz);
  if (!it) { const g = new THREE.Group(); RC.scene.add(g); const b = new THREE.Mesh(GEO[sz].box, M.carton); b.position.z = SIZE[sz].h / 2; b.castShadow = false; g.add(b);
    const lab = new THREE.Mesh(GEO[sz].lab, M.labs[0]); lab.position.set(SIZE[sz].l * 0.2, 0, SIZE[sz].h + 0.001); lab.castShadow = false; g.add(lab); it = { g, lab, sz, free: true }; ITEMPOOL.push(it); }
  it.free = false; it.sku = sku; it.g.children[0].material = M.carton; it.lab.material = M.labs[LABELS.indexOf(SKUS[sku].lab)]; it.g.visible = true; it.g.rotation.set(0, 0, 0);
  if (it.g.parent !== RC.scene) RC.scene.attach(it.g);
  Object.assign(it, { order: null, line: null, x: 0, read: false, target: '', state: '' }); return it;
}
function freeItem(it) { it.free = true; it.g.visible = false; if (it.g.parent !== RC.scene) RC.scene.attach(it.g); }
/* grid position of the k-th carton in a bin (layer 0 first), bin-local, bottom of the carton */
function gridPos(sku, k) {
  const z = dimOf(sku), per = z.nx * z.ny, layer = Math.floor(k / per), i = k % per, ix = i % z.nx, iy = Math.floor(i / z.nx);
  const px = (BN.L - 2 * BN.wall) / z.nx, py = (BN.W - 2 * BN.wall) / z.ny;
  return { x: (ix - (z.nx - 1) / 2) * px, y: (iy - (z.ny - 1) / 2) * py, z: BN.fl + layer * z.h };
}

// ------------------------------------------------------------------ bins: rack slots (instanced), live bins (groups with cartons)
function slotPose(s) { return { x: RG.rc * Math.cos(s.th), y: RG.rc * Math.sin(s.th), z: s.zb }; }
function setInst(s) {
  const m = new THREE.Matrix4(), on = !!s.bin, p = slotPose(s);
  for (const [im, dz, sc] of [[G.instBin, BN.H / 2, 1], [G.instTop, BN.H + 0.001, 1], [G.instLab, BN.H / 2, 1]]) {
    const q = new THREE.Matrix4().makeRotationZ(s.th); m.makeTranslation(p.x, p.y, p.z + dz); m.multiply(q);
    if (im === G.instLab) m.multiply(new THREE.Matrix4().makeTranslation(-BN.L / 2 - 0.003, 0, 0));
    if (!on) m.multiply(new THREE.Matrix4().makeScale(1e-4, 1e-4, 1e-4));
    im.setMatrixAt(s.i, m); im.instanceMatrix.needsUpdate = true;
  }
}
function makeLive(b) {                                                      // a bin leaves the rack: group + cartons
  if (!b.g) { const g = new THREE.Group(); RC.scene.add(g);
    for (const s of [-1, 1]) { const w = new THREE.Mesh(GEO.binWallL, M.bin); w.position.set(0, s * (BN.W / 2 - BN.wall / 2), BN.H / 2); g.add(w);
      const e = new THREE.Mesh(GEO.binWallS, M.bin); e.position.set(s * (BN.L / 2 - BN.wall / 2), 0, BN.H / 2); g.add(e); }
    const f = new THREE.Mesh(GEO.binFloor, M.binIn); f.position.z = BN.fl / 2; g.add(f);
    const l = new THREE.Mesh(GEO.binLab, M.label); l.position.set(-BN.L / 2 - 0.003, 0, BN.H * 0.55); g.add(l);
    g.traverse(o => { if (o.isMesh) o.castShadow = o === g.children[0]; });
    b.g = g; }
  b.g.visible = true; if (b.g.parent !== RC.scene) RC.scene.attach(b.g); b.items = [];
  for (let k = 0; k < b.n; k++) { const it = takeItem(b.sku), p = gridPos(b.sku, k); b.g.add(it.g); it.g.position.set(p.x, p.y, p.z); it.g.rotation.set(0, 0, 0); b.items.push(it); }
}
function unLive(b) { for (const it of b.items || []) freeItem(it); b.items = []; if (b.g) b.g.visible = false; }
function placeBin(b, x, y, z, yaw) { b.g.position.set(x, y, z); b.g.rotation.set(0, 0, yaw || 0); }

// ------------------------------------------------------------------ zone conveyors: picking loop (east port) and restock loop (west port)
function mkLoop(id, pts, kinds) { const L = LOOPS[id] = { id, zones: pts.map((p, i) => ({ x: p[0], y: p[1], k: kinds[i] || '', bin: null, res: null })), moves: 0 }; return L; }
function initLoops() {
  const y = K.laneY;
  mkLoop('pick', [[2.1, y], [2.85, y], [3.6, y], [4.3, y], [5.05, y], [5.8, y], [6.6, y], [6.6, -y], [5.85, -y], [5.1, -y], [4.35, -y], [3.6, -y], [2.85, -y], [2.1, -y]],
                 ['H', '', 'DEC', 'P1', '', 'P2', 'END', 'RIN', '', '', '', '', '', 'HIN']);
  mkLoop('rst', [[-2.1, y], [-2.8, y], [-3.5, y], [-4.75, y], [-4.75, -y], [-3.5, -y], [-2.8, -y], [-2.1, -y]], ['H', '', '', 'OP', 'RIN', '', '', 'HIN']);
}
const zoneOf = (L, k) => L.zones.findIndex(z => z.k === k);
const loopBins = L => L.zones.reduce((a, z) => a + (z.bin ? 1 : 0) + (z.res && z.res !== z.bin && typeof z.res === 'object' ? 0 : 0), 0);
function canLeave(L, b, i) {
  const z = L.zones[i];
  if (z.k === 'H' && !b.placed) return false;
  if (z.k === 'HIN' || (L.id === 'rst' && z.k === 'OP' && !b.refilled)) return false;
  if ((z.k === 'P1' || z.k === 'P2') && b.station === z.k && !b.done) return false;
  if (L.id === 'pick' && z.k === 'DEC' && !decide(b)) return false;
  if (i + 1 < L.zones.length && (L.zones[i + 1].k === 'P1' || L.zones[i + 1].k === 'P2')) {                // a bin passes a stop only when it is free
    const P = ST[L.zones[i + 1].k]; if (P && P.toolIn && P.toolIn()) return false; }
  return true;
}
function onArrive(L, b, i) {
  const z = L.zones[i];
  if (L.id === 'pick' && (z.k === 'P1' || z.k === 'P2') && b.station === z.k) { arrive(b, z.k); ST[z.k].valid = false; b.arrT = S.t; S.stat.presented++; }
  if (L.id === 'rst' && z.k === 'OP') b.opT = S.t;
}
function updateLoop(L, dt) {
  const v = K.vConv;
  for (let i = L.zones.length - 1; i >= 0; i--) {
    const z = L.zones[i], b = z.bin; if (!b) continue;
    if (b.mv) {
      if (zStop()) continue;
      const m = b.mv, sp = Math.abs(m.b.y - m.a.y) > 0.5 ? K.vRat : v; m.s = Math.min(m.L, m.s + sp * dt);
      const u = m.s / m.L; placeBin(b, lerp(m.a.x, m.b.x, u), lerp(m.a.y, m.b.y, u), K.zc, b.g.rotation.z);
      if (m.s >= m.L) { z.bin = null; const t = L.zones[m.j]; t.bin = b; t.res = null; b.zi = m.j; b.mv = null; L.moves++; onArrive(L, b, m.j); }
      continue;
    }
    if (zStop()) continue;
    const j = i + 1; if (j >= L.zones.length) continue;
    const t = L.zones[j]; if (t.bin || t.res) continue;
    if (!canLeave(L, b, i)) continue;
    if (L.id === 'pick' && (t.k === 'P1' || t.k === 'P2') && t.k !== b.station && b.station === null && false) continue;
    t.res = b; b.mv = { a: { x: z.x, y: z.y }, b: { x: t.x, y: t.y }, s: 0, L: Math.hypot(t.x - z.x, t.y - z.y), j };
  }
}
/* the restock tunnel curtain is muted while a bin crosses it (between the zones at x −3.5 and −4.75) */
const rstMuted = () => LOOPS.rst.zones.some(z => z.bin && z.bin.mv && Math.min(z.bin.mv.a.x, z.bin.mv.b.x) < K.rst.tunX && Math.max(z.bin.mv.a.x, z.bin.mv.b.x) > K.rst.tunX);

// ------------------------------------------------------------------ WMS: orders, waves, demand and allocation live in the WMS
/* decision point before P1, taken when the bin could move on into P1: the WMS routes it to P2 / P1, sends it back (no open demand)
   or lets it wait (both stops busy or their pickers down) */
function decide(b) {
  if (b.station) return true;                                                // routed before (e.g. restored from the WMS)
  if (b.done) return true;                                                   // going back without stopping
  const P = LOOPS.pick, z1 = P.zones[zoneOf(P, 'P1')];
  if (z1.bin || z1.res) return false;
  if (!b.routeReq) { if ((b.routeT || 0) > S.t) return false; b.routeReq = wmsCall('Route', b.id); return false; }
  const r = b.routeReq; if (!r.done) return false;
  b.routeReq = null;
  if (!r.ok || !r.value) { b.routeT = S.t + 2; return false; }
  if (r.value.decision === 'Station') { b.station = r.value.station; return true; }
  if (r.value.decision === 'Return') { b.station = null; b.done = true; return true; }
  b.routeT = S.t + 1; return false;                                          // wait and ask again
}
/* bin arrived at its stop: the WMS allocates its cartons to the oldest open order lines */
function arrive(b, k) { b.picks = null; b.pi = 0; b.arriveReq = wmsCall('Arrive', b.id, k); }
function resolveArrivals() {
  for (const z of LOOPS.pick.zones) { const b = z.bin; if (!b || !b.arriveReq || !b.arriveReq.done) continue;
    const r = b.arriveReq; b.arriveReq = null; b.picks = r.ok && r.value ? r.value : []; b.pi = 0; if (!b.picks.length) b.done = true; }
}

// ------------------------------------------------------------------ build: cylinder shelf
function buildRing() {
  const rI = RG.rIn, rO = RG.rOut, H = RG.h, gap = new Set(RG.gaps);
  const N = RG.cols * RG.lev;
  G.instBin = new THREE.InstancedMesh(new THREE.BoxGeometry(BN.L, BN.W, BN.H), M.bin, N);
  G.instTop = new THREE.InstancedMesh(new THREE.BoxGeometry(BN.L - 0.04, BN.W - 0.04, 0.002), M.binIn, N);
  G.instLab = new THREE.InstancedMesh(new THREE.BoxGeometry(0.004, 0.14, 0.07), M.label, N);
  for (const im of [G.instBin, G.instTop, G.instLab]) { im.frustumCulled = false; im.castShadow = im === G.instBin; im.receiveShadow = true; RC.scene.add(im); }
  for (let k = 0; k < RG.cols; k++) { if (gap.has(k)) continue;
    for (let j = 0; j < RG.lev; j++) { const s = { i: SLOTS.length, k, j, th: wrap(k * DTH), zb: RG.z0 + j * RG.pitch, bin: null, res: false }; SLOTS.push(s); } }
  // posts at every column boundary next to a used column (inner + outer face), rails per slot, X braces on the outer face, top / bottom rings
  const used = k => !gap.has(((k % RG.cols) + RG.cols) % RG.cols);
  const posts = []; for (let k = 0; k < RG.cols; k++) if (used(k) || used(k - 1)) posts.push(k - 0.5);
  const postI = new THREE.InstancedMesh(new THREE.BoxGeometry(0.05, 0.05, H), M.truss, posts.length * 2), m = new THREE.Matrix4();
  posts.forEach((k, i) => { const th = k * DTH; [rI - 0.03, rO].forEach((r, n) => { m.makeTranslation(r * Math.cos(th), r * Math.sin(th), H / 2); m.multiply(new THREE.Matrix4().makeRotationZ(th)); postI.setMatrixAt(i * 2 + n, m); }); });
  const rails = new THREE.InstancedMesh(new THREE.BoxGeometry(rO - rI + 0.02, 0.03, 0.02), M.truss, SLOTS.length * 2);
  SLOTS.forEach((s, i) => { for (const n of [0, 1]) { const tv = (n ? 1 : -1) * 0.17, c = Math.cos(s.th), sn = Math.sin(s.th), rm = (rI + rO) / 2;
    m.makeTranslation(rm * c - tv * sn, rm * sn + tv * c, s.zb - 0.01); m.multiply(new THREE.Matrix4().makeRotationZ(s.th)); rails.setMatrixAt(i * 2 + n, m); } });
  const cols = []; for (let k = 0; k < RG.cols; k++) if (used(k)) cols.push(k);
  const chord = 2 * rO * Math.sin(DTH / 2), seg = 2 * RG.pitch, nSeg = Math.floor((H - 0.2) / seg), dl = Math.hypot(chord, seg), ang = Math.atan2(seg, chord);
  const brace = new THREE.InstancedMesh(new THREE.BoxGeometry(dl, 0.025, 0.035), M.truss, cols.length * nSeg * 2);
  let bi = 0; for (const k of cols) { const th = k * DTH, rr = rO * Math.cos(DTH / 2) + 0.01; for (let q = 0; q < nSeg; q++) for (const sg of [-1, 1]) {
    m.makeTranslation(rr * Math.cos(th), rr * Math.sin(th), 0.15 + seg * (q + 0.5)); m.multiply(new THREE.Matrix4().makeRotationZ(th + Math.PI / 2)); m.multiply(new THREE.Matrix4().makeRotationY(sg * ang)); brace.setMatrixAt(bi++, m); } }
  const ringB = new THREE.InstancedMesh(new THREE.BoxGeometry(chord + 0.02, 0.06, 0.08), M.truss, cols.length * 4);
  cols.forEach((k, i) => { const th = k * DTH; [[rI - 0.03, H - 0.04], [rO, H - 0.04], [rI - 0.03, 0.05], [rO, 0.05]].forEach(([r, z], n) => { const rr = r * Math.cos(DTH / 2);
    m.makeTranslation(rr * Math.cos(th), rr * Math.sin(th), z); m.multiply(new THREE.Matrix4().makeRotationZ(th + Math.PI / 2)); ringB.setMatrixAt(i * 4 + n, m); }); });
  for (const im of [postI, rails, brace, ringB]) { im.frustumCulled = false; im.castShadow = im !== brace; im.receiveShadow = true; RC.scene.add(im); }
  // back stops at r 2.62 per slot (instanced), floor ring marking, cylinder control box at the north passage
  const stops = new THREE.InstancedMesh(new THREE.BoxGeometry(0.02, 0.34, 0.06), M.truss, SLOTS.length); stops.frustumCulled = false;
  SLOTS.forEach((s, i) => { m.makeTranslation(2.62 * Math.cos(s.th), 2.62 * Math.sin(s.th), s.zb + 0.03); m.multiply(new THREE.Matrix4().makeRotationZ(s.th)); stops.setMatrixAt(i, m); }); RC.scene.add(stops);
  { const g = new THREE.RingGeometry(rI - 0.05, rO + 0.05, 104); const r = new THREE.Mesh(g, mat(0x3a4048, { roughness: 0.9 })); r.position.z = 0.003; RC.scene.add(r); }
}

// ------------------------------------------------------------------ build: conveyors (rollers + chain heads + right-angle transfers)
function rollerLane(x0, x1, y, w, z, parent) {
  const L = Math.abs(x1 - x0), xc = (x0 + x1) / 2;
  for (const s of [-1, 1]) box(L, 0.04, 0.1, M.frame, xc, y + s * (w / 2 + 0.02), z - 0.05, parent);
  const n = Math.floor(L / 0.1), r = new THREE.InstancedMesh(new THREE.CylinderGeometry(0.025, 0.025, w, 10), M.roller, n), m = new THREE.Matrix4(); r.frustumCulled = false;
  for (let i = 0; i < n; i++) { m.makeTranslation(Math.min(x0, x1) + 0.05 + i * L / n, y, z - 0.025); r.setMatrixAt(i, m); } (parent || RC.scene).add(r);
  for (let x = Math.min(x0, x1) + 0.15; x < Math.max(x0, x1); x += 1.2) for (const s of [-1, 1]) box(0.05, 0.05, z - 0.1, M.dark, x, y + s * (w / 2 + 0.02), (z - 0.1) / 2, parent);
}
function chainHead(x0, x1, y, z) {                                          // fork transfer head: two chains at ±0.17 (the fork tines run between them)
  const L = Math.abs(x1 - x0), xc = (x0 + x1) / 2;
  for (const s of [-1, 1]) { box(L, 0.05, 0.06, M.chain, xc, y + s * 0.17, z - 0.03); box(0.05, 0.05, z - 0.06, M.dark, xc, y + s * 0.17, (z - 0.06) / 2); }
  box(0.04, 0.5, 0.06, M.yellow, Math.max(x0, x1) + (x1 > x0 ? 0.02 : -0.02) * 0, y, 0.2);
}
function ratTable(x, z) {                                                   // right-angle transfer: chains along y across both lanes
  const y = K.laneY;
  box(0.66, 2 * y + 0.5, 0.04, M.dark, x, 0, z - 0.09); for (const dx of [-0.22, 0, 0.22]) box(0.04, 2 * y + 0.42, 0.04, M.chain, x + dx, 0, z - 0.02);
  for (const dx of [-0.28, 0.28]) for (const dy of [-y - 0.2, y + 0.2]) box(0.05, 0.05, z - 0.1, M.dark, x + dx, dy, (z - 0.1) / 2);
}
function buildConveyors() {
  const z = K.zc, y = K.laneY, w = 0.46;
  // picking loop: chain heads inside the east port, rollers to the RAT at x 6.6
  for (const s of [1, -1]) { chainHead(1.78, 2.42, s * y, z); rollerLane(2.42, 6.28, s * y, w, z); }
  ratTable(6.6, z);
  box(0.06, 0.06, 0.12, M.blue, 6.95, 0, z - 0.06);
  // restock loop: chain heads inside the west port, rollers out through the tunnel to the operator RAT at x −4.75
  for (const s of [1, -1]) { chainHead(-1.78, -2.42, s * y, z); rollerLane(-2.42, -4.43, s * y, w, z); }
  ratTable(-4.75, z);
  // stops: pop-up blades at P1 / P2 (visual), bin presence sensors
  G.blades = {};
  for (const k of ['P1', 'P2']) { const zn = LOOPS.pick.zones[zoneOf(LOOPS.pick, k)]; G.blades[k] = box(0.02, 0.36, 0.06, M.yellow, zn.x + 0.32, y, z - 0.03); }
}
/* 3D camera over each bin stop: post south of the return lane, cantilever over both lanes; frustum shown while imaging */
function buildCameras() {
  for (const k of ['P1', 'P2']) {
    const zn = LOOPS.pick.zones[zoneOf(LOOPS.pick, k)], x = zn.x, y = zn.y, zc = K.cam.z;
    box(0.1, 0.1, zc + 0.15, M.green, x, K.cam.postY, (zc + 0.15) / 2);
    box(0.08, y - K.cam.postY + 0.05, 0.08, M.green, x, (y + K.cam.postY) / 2, zc + 0.14);
    box(0.36, 0.2, 0.12, M.black, x, y, zc + 0.04); cylZ(0.04, 0.03, M.dark, x, y, zc - 0.035, null, 16);
    const geo = new THREE.BufferGeometry(), a = [x, y, zc - 0.05], hx = BN.L / 2 - 0.02, hy = BN.W / 2 - 0.02, zf = Z.binFloor + 0.002, c = [[x - hx, y - hy], [x + hx, y - hy], [x + hx, y + hy], [x - hx, y + hy]], v = [];
    for (let i = 0; i < 4; i++) { const p = c[i], q = c[(i + 1) % 4]; v.push(...a, p[0], p[1], zf, q[0], q[1], zf); }
    geo.setAttribute('position', new THREE.Float32BufferAttribute(v, 3)); const pr = new THREE.Mesh(geo, M.proj); pr.visible = false; RC.scene.add(pr);
    ST[k] = Object.assign(ST[k] || {}, { k, x, y, zi: zoneOf(LOOPS.pick, k), proj: pr, valid: false, imgT: 0, images: 0, rob: k === 'P1' ? 'r2' : 'r3' });
  }
}
/* frustum of the 3D camera: from the lens (z 2.15) to the bin floor outline; a point inside it hides the bin */
function inCone(P, x, y, z) {
  const zs = K.cam.z - 0.05; if (z >= zs) return false;
  const f = (zs - z) / (zs - Z.binFloor), hx = (BN.L / 2) * f + 0.05, hy = (BN.W / 2) * f + 0.05;
  return Math.abs(x - P.x) < hx && Math.abs(y - P.y) < hy;
}
function coneBlocked(P) {
  for (const c of RC.ctxs) { if (c.id !== 'r2' && c.id !== 'r3') continue; const t = c.tcp;
    for (const [dx, dy] of [[0, 0], [-0.07, -0.05], [0.07, 0.05], [0.07, -0.05], [-0.07, 0.05]]) if (inCone(P, t.x + dx, t.y + dy, t.z)) return c.id;
    if (inCone(P, t.x, t.y, t.z + 0.3)) return c.id; }
  return '';
}

// ------------------------------------------------------------------ build: item belt, bar-code scanner, exception chute
function buildBelt() {
  const B = BL, L = B.x1 - B.x0, xc = (B.x0 + B.x1) / 2;
  box(L, B.w, 0.02, M.belt, xc, B.y, B.z - 0.01);
  for (const s of [-1, 1]) box(L, 0.04, 0.12, M.frame, xc, B.y + s * (B.w / 2 + 0.03), B.z - 0.04);
  for (let x = B.x0 + 0.2; x < B.x1; x += 1.3) for (const s of [-1, 1]) box(0.05, 0.05, B.z - 0.1, M.dark, x, B.y + s * (B.w / 2 + 0.03), (B.z - 0.1) / 2);
  cylX(0.05, 0.42, M.steel, B.x0, B.y, B.z - 0.05); cylX(0.05, 0.42, M.steel, B.x1, B.y, B.z - 0.05);
  box(0.25, 0.2, 0.2, M.blue, B.x0 + 0.15, B.y - B.w / 2 - 0.2, B.z - 0.25);                           // drive
  // bar-code scanner arch (SICK camera from above + side), top 1.15
  const x = B.scanX; for (const s of [-1, 1]) box(0.05, 0.05, 1.15, M.dark, x, B.y + s * 0.32, 0.575);
  box(0.06, 0.69, 0.05, M.dark, x, B.y, 1.15); box(0.09, 0.12, 0.08, M.sick, x, B.y, 1.08); box(0.08, 0.05, 0.09, M.sick, x, B.y + 0.28, 0.92);
  G.scanBeam = box(0.002, B.w, 0.02, M.scanBeam, x, B.y, B.z + 0.13); G.scanBeam.visible = false; G.scanBeam.castShadow = false;
  // exception chute: belt end → covered chute through a sealed panel in the east fence → tote outside
  const z0 = B.z - 0.03, x1 = K.ex.x - 0.15, z1 = 0.5, Lc = Math.hypot(x1 - B.x1, z0 - z1);
  const ch = box(Lc, 0.34, 0.012, M.steel, (B.x1 + x1) / 2, B.y, (z0 + z1) / 2); ch.rotation.y = Math.atan2(z0 - z1, x1 - B.x1);
  for (const s of [-1, 1]) { const w = box(Lc, 0.008, 0.3, M.steel, 0, s * 0.17, 0.15, ch); w.castShadow = false; }
  const cover = box(Lc, 0.36, 0.008, M.steel, 0, 0, 0.3, ch); cover.castShadow = false;
  box(0.45, 0.4, 0.3, M.tote, K.ex.x, B.y, 0.15); G.exFill = box(0.4, 0.34, 1, M.carton, K.ex.x, B.y, 0.2); G.exFill.visible = false; G.exFill.castShadow = false;
}

// ------------------------------------------------------------------ build: put wall (8 powered case lanes through the north fence, tunnel hoods)
function caseMesh() {
  const g = new THREE.Group(), C = K.cs; RC.scene.add(g);
  box(C.L, C.W, 0.01, M.case, 0, 0, 0.005, g);
  for (const s of [-1, 1]) { box(C.L, 0.008, C.H, M.case, 0, s * (C.W / 2 - 0.004), C.H / 2, g); box(0.008, C.W, C.H, M.case, s * (C.L / 2 - 0.004), 0, C.H / 2, g); }
  g.traverse(o => { if (o.isMesh) o.castShadow = false; }); return g;
}
function buildPutWall() {
  for (const [sid, xs] of Object.entries(PW.xs)) {
    xs.forEach((x, i) => {
      const L = PW.y1 - PW.y0, yc = (PW.y0 + PW.y1) / 2;
      for (const s of [-1, 1]) box(0.03, L, 0.08, M.frame, x + s * (PW.w / 2 + 0.015), yc, PW.z - 0.04);
      const n = Math.floor(L / 0.1), r = new THREE.InstancedMesh(new THREE.CylinderGeometry(0.02, 0.02, PW.w, 8).rotateZ(Math.PI / 2), M.roller, n), m = new THREE.Matrix4(); r.frustumCulled = false;
      for (let k = 0; k < n; k++) { m.makeTranslation(x, PW.y0 + 0.05 + k * L / n, PW.z - 0.02); r.setMatrixAt(k, m); } RC.scene.add(r);
      for (const y of [PW.y0 + 0.1, PW.y1 - 0.1]) box(0.34, 0.05, PW.z - 0.08, M.dark, x, y, (PW.z - 0.08) / 2);
      const lamp = cylZ(0.018, 0.03, lampMat(0x22ff66), x, PW.yOut + 0.27, PW.z + 0.02, null, 10);     // put-to-light / done lamp at the operator end
      const cs = caseMesh();
      LANES.push({ sid, i, id: sid.toUpperCase() + '-' + (i + 1), x, u: 0, state: 'ready', order: null, items: [], cs, lamp, outT: 0, claimOp: false, cases: 0 });
    });
    // tunnel hood skirts (side walls to the floor below the hood) — the hood itself is in safety()
    const x0 = xs[0] - PW.w / 2 - 0.05, x1 = xs[xs.length - 1] + PW.w / 2 + 0.05, Ld = PW.curtainY - F.y1;
    for (const xx of [x0, x1]) { const m = box(0.01, Ld, PW.roof, M.skirt, xx, (F.y1 + PW.curtainY) / 2, PW.roof / 2); m.castShadow = false; }
  }
  // case supply cart + packing cart outside (visual)
  box(0.8, 0.5, 0.06, M.dark, 6.0, 5.75, 0.4); for (let k = 0; k < 4; k++) box(0.36, 0.31, 0.22, M.case, 5.8 + (k % 2) * 0.4, 5.75, 0.54 + Math.floor(k / 2) * 0.23);
  box(1.0, 0.6, 0.06, M.dark, 10.3, 5.75, 0.4);
}
function showLane(ln) {
  const y = lerp(PW.yIn, PW.yOut, ln.u); ln.cs.position.set(ln.x, y, PW.z); ln.cs.visible = ln.state !== 'empty';
  /* items packed by their real size (rows along x, rows across y, then the next layer); the fixed 0.16 × 0.12 grid with 0.06 m layers made
     the M / L cartons interpenetrate (flow-path sweep) */
  const C = K.cs; let cx = -C.L / 2 + 0.01, cy = -C.W / 2 + 0.01, cz = 0.012, rowD = 0, layH = 0;
  ln.items.forEach(it => { const d = SIZE[it.sz]; let a = d.l, b = d.w, rot = 0;
    if (cx + a > C.L / 2 - 0.01 && cx + b <= C.L / 2 - 0.01) { a = d.w; b = d.l; rot = Math.PI / 2; }        // turned 90° when only that fits the row
    if (cx + a > C.L / 2 - 0.01) { cx = -C.L / 2 + 0.01; cy += rowD; rowD = 0; }                          // next row
    if (cy + b > C.W / 2 - 0.01) { cx = -C.L / 2 + 0.01; cy = -C.W / 2 + 0.01; cz += layH; layH = 0; rowD = 0; }   // next layer
    it.g.position.set(ln.x + cx + a / 2, y + cy + b / 2, PW.z + cz); it.g.rotation.set(0, 0, rot);
    cx += a; rowD = Math.max(rowD, b); layH = Math.max(layH, d.h); });
}

// ------------------------------------------------------------------ build: restock station outside the west port (operator table, stock pallet)
function buildRestock() {
  const y = K.laneY, R = K.rst;
  for (const s of [-1, 1]) { const m = box(R.tunX - F.x0, 0.01, R.roof, M.skirt, (R.tunX + F.x0) / 2, s * (y + 0.29), R.roof / 2); m.castShadow = false; }
  box(1.0, 1.2, 0.12, M.pallet, -5.6, 1.75, 0.06); for (let k = 0; k < 6; k++) box(0.42, 0.36, 0.3, M.carton, -5.85 + (k % 2) * 0.46, 1.5 + Math.floor(k / 2 % 2) * 0.42, 0.27 + Math.floor(k / 4) * 0.3);
  G.rstLamp = cylZ(0.02, 0.05, lampMat(0x22ff66), -4.75, y + 0.33, 1.05, null, 12); box(0.06, 0.06, 0.3, M.dark, -4.75, y + 0.33, 0.88);
  box(0.12, 0.06, 0.1, M.dark, -5.2, y + 0.35, 1.0); G.rstBtn = cylZ(0.025, 0.02, lampMat(0x2fa8ff), -5.2, y + 0.39, 1.0, null, 12); G.rstBtn.rotation.x = Math.PI / 2;
}

// ------------------------------------------------------------------ robot pedestals + tools
function pedestal(b, h, w) {
  const m = mat(0x394047, { metalness: 0.35, roughness: 0.5 }); w = w || 0.6;
  box(w, w, 0.03, m, b.x, b.y, 0.015); box(w * 0.7, w * 0.7, h - 0.06, m, b.x, b.y, 0.03 + (h - 0.06) / 2); box(w * 0.83, w * 0.83, 0.03, M.dark, b.x, b.y, h - 0.015);
  RC.floorLine(b.x, b.y, w + 0.1, w + 0.1).material = mat(0x3a4048, { roughness: 0.9 });
}
function buildTools() {
  { const c = ctx('r1'), g = new THREE.Group(); c.robot.toolMount.add(g); const Tl = K.rob.r1.tool;            // L-fork: flange down, mast to the fork plane, tines ahead (+X = tool yaw)
    cylZ(0.13, 0.04, M.dark, 0, 0, -0.02, g, 24); box(0.1, 0.3, 0.04, M.frame, 0.02, 0, -0.06, g);               // flange adapter + head plate
    box(0.08, 0.3, Tl - 0.08, M.frame, 0.04, 0, -0.08 - (Tl - 0.08) / 2, g);                                      // mast
    box(0.02, 0.34, 0.24, M.yellow, 0.225, 0, -Tl + 0.13, g);                                                     // back rest (bin end)
    for (const s of [-1, 1]) box(0.62, 0.05, 0.02, M.steel, 0.55, s * 0.09, -Tl - 0.01, g);                       // tines (top = TCP plane)
    box(0.24, 0.24, 0.03, M.dark, 0.13, 0, -Tl - 0.015, g); box(0.03, 0.03, 0.03, M.blue, 0.11, 0.13, -Tl + 0.2, g);   // tine root, bin sensor
    c.robot.setToolOffset(Tl); }
  for (const id of ['r2', 'r3']) { const c = ctx(id), g = new THREE.Group(); c.robot.toolMount.add(g); const Tl = K.rob[id].tool;  // 4-cup vacuum gripper
    cylZ(0.04, 0.02, M.yellow, 0, 0, -0.01, g, 20); box(0.04, 0.04, 0.08, M.dark, 0, 0, -0.06, g); box(0.12, 0.08, 0.015, M.frame, 0, 0, -0.105, g);
    for (const [dx, dy] of [[-0.04, -0.025], [0.04, -0.025], [-0.04, 0.025], [0.04, 0.025]]) { cylZ(0.004, 0.03, M.steel, dx, dy, -0.128, g, 8); cylZ(0.011, 0.015, M.black, dx, dy, -Tl + 0.0075, g, 12); }
    box(0.03, 0.02, 0.02, M.blue, 0.04, 0.04, -0.06, g); c.robot.setToolOffset(Tl); }
  for (const id of ['s1', 's2']) { const c = ctx(id), g = new THREE.Group(); c.robot.toolMount.add(g); const Tl = K.rob[id].tool;   // bellows cup
    cylZ(0.03, 0.02, M.yellow, 0, 0, -0.01, g, 16); cylZ(0.012, 0.07, M.steel, 0, 0, -0.055, g, 10); cylZ(0.022, 0.03, M.black, 0, 0, -Tl + 0.015, g, 14);
    c.robot.setToolOffset(Tl); }
}
function grab(c, m) {                                                       // carried object keeps its pose relative to the TCP (yaw only)
  m.updateWorldMatrix(true, false); const p = new THREE.Vector3().setFromMatrixPosition(m.matrixWorld); const q = new THREE.Quaternion(); m.getWorldQuaternion(q); const e = new THREE.Euler().setFromQuaternion(q, 'ZYX');
  if (m.parent !== RC.scene) RC.scene.attach(m);
  const dx = p.x - c.tcp.x, dy = p.y - c.tcp.y, cy = Math.cos(-c.tcp.yaw), sy = Math.sin(-c.tcp.yaw);
  c.held = { m, lx: dx * cy - dy * sy, ly: dx * sy + dy * cy, dz: c.tcp.z - p.z, rz: e.z - c.tcp.yaw };
}

// ------------------------------------------------------------------ R1 (IRB 6700, fork) in the cylinder: retrieve / store / restock
const R1 = K.rob.r1;
const headOf = (lp, k) => { const L = LOOPS[lp], i = zoneOf(L, k), z = L.zones[i]; return { L, i, z, x: z.x, y: z.y, yaw: z.x > 0 ? 0 : (z.y > 0 ? Math.PI : -Math.PI) }; };
function freeSlot(cur) {                                                    // nearest free slot to the robot's angle, mid levels preferred
  let best = null, bs = 1e9;
  for (const s of SLOTS) { if (s.bin || s.res) continue; const sc = Math.abs(wrap(s.th - cur)) + 0.35 * Math.abs(s.zb - 1.4); if (sc < bs) { bs = sc; best = s; } }
  return best;
}
function pickBinFor(sku, cur) {
  let best = null, bs = 1e9;
  for (const b of BINS) { if (b.sku !== sku || b.where !== 'rack' || b.n <= 0 || b.res) continue; const s = b.slot, sc = -b.n * 0.05 + Math.abs(wrap(s.th - cur)) + 0.35 * Math.abs(s.zb - 1.4); if (sc < bs) { bs = sc; best = b; } }
  return best;
}
const pickLoopCount = () => LOOPS.pick.zones.filter(z => z.bin).length;
const rstCount = () => LOOPS.rst.zones.filter(z => z.bin).length;
/* what R1's controller sees at its transfer heads, sent with every task request */
function r1Heads() {
  const hin = headOf('pick', 'HIN'), rin = headOf('rst', 'HIN'), hout = headOf('pick', 'H'), rout = headOf('rst', 'H');
  const at = h => h.z.bin && !h.z.bin.mv ? h.z.bin.id : null, free = h => !h.z.bin && !h.z.res;
  return [at(hin), at(rin), free(hout), free(rout)];
}
const slotOf = code => { const m = /C(\d+)-L(\d+)/.exec(code || ''); return m ? SLOTS.find(q => q.k === +m[1] - 1 && q.j === +m[2] - 1) : null; };
/* WMS ASRS task → the physical move (source: where the bin really is) */
function r1Task(v) {
  const b = BINS.find(q => q.id === v.binId); if (!b) return null;
  const hin = headOf('pick', 'HIN'), rin = headOf('rst', 'HIN'), hout = headOf('pick', 'H'), rout = headOf('rst', 'H');
  switch (v.kind) {
    case 'Retrieve': return b.slot ? { kind: 'retrieve', v, b, slot: b.slot, dst: hout } : null;
    case 'Store': { const s = slotOf(v.toSlot); return s ? { kind: 'store', v, b, src: hin, slot: s } : null; }
    case 'StoreRestocked': { const s = slotOf(v.toSlot); return s ? { kind: 'storeRst', v, b, src: rin, slot: s } : null; }
    case 'ReturnToRestock': return { kind: 'toRst', v, b, src: hin, dst: rout };
    case 'RackToRestock': return b.slot ? { kind: 'rackToRst', v, b, slot: b.slot, dst: rout } : null;
  }
  return null;
}
function* progR1() {
  const c = ctx('r1'), B = RC.motion.bind(c), { move, dwell, waitUntil, setStep } = B, rS = R1.rS, BM = R1.beam;
  /* TCP = fork plane under the wrist, bin centre BM ahead along the yaw. Swing with the TCP at r 0.9 (fork tip 1.75 inside the shelf face
     2.0 and the conveyor heads 1.8) in the direction that keeps A1 inside ±170°
     (relative angles never pass ±180°, the west port gap); z and the fork yaw (radial) change along the arc */
  const swing = function* (x, y, z, yaw) {
    const a0 = Math.atan2(c.tcp.y, c.tcp.x), a1 = Math.atan2(y, x), r1 = Math.hypot(x, y);
    const yw = yaw === undefined ? a1 : yaw;
    if (Math.abs(a1 - a0) < 0.25) { yield* move({ x, y, z, yaw: yw }); return; }
    const n = Math.max(3, Math.ceil(Math.abs(a1 - a0) / 0.3)), via = [], r0 = Math.hypot(c.tcp.x, c.tcp.y);
    for (let i = 1; i < n; i++) { const u = i / n, a = a0 + (a1 - a0) * u, r = Math.min(rS, lerp(r0, r1, u)); via.push({ x: r * Math.cos(a), y: r * Math.sin(a), z: lerp(c.tcp.z, z, u), yaw: lerp(c.tcp.yaw, yw, u) }); }
    yield* move({ x, y, z, yaw: yw }, { via });
  };
  const radial = (th, r) => ({ x: r * Math.cos(th), y: r * Math.sin(th) });
  const take = function* (src, b) {                                         // src: { slot } | head
    if (src.slot) { const s = src.slot, th = s.th, a = radial(th, rS), p = radial(th, RG.rc - BM), q = radial(th, RG.rc);
      yield* swing(a.x, a.y, s.zb - 0.015, th);
      yield* move({ x: p.x, y: p.y, z: s.zb - 0.015 }, { vMax: 1.4 }); yield* move({ x: p.x, y: p.y, z: s.zb + 0.015 }, { vMax: 0.3 });
      s.bin = null; b.slot = null; setInst(s); makeLive(b); placeBin(b, q.x, q.y, s.zb + 0.015, th); grab(c, b.g); b.where = 'r1';
      yield* move({ x: a.x, y: a.y, z: s.zb + 0.015 }, { vMax: 1.4 }); s.res = false; return; }
    const h = src, sx = h.x - (BM + 0.65) * Math.cos(h.yaw), ix = h.x - BM * Math.cos(h.yaw);
    yield* swing(sx, h.y, Z.r1In, h.yaw);
    yield* move({ x: ix, y: h.y, z: Z.r1In }, { vMax: 1.2 }); yield* move({ x: ix, y: h.y, z: Z.r1Up }, { vMax: 0.3 });
    h.z.bin = null; b.zi = -1; placeBin(b, h.x, h.y, Z.r1Up, b.g.rotation.z); grab(c, b.g); b.where = 'r1';
    yield* move({ x: sx, y: h.y, z: Z.r1Up }, { vMax: 1.2 });
  };
  const put = function* (dst, b) {
    if (dst.slot) { const s = dst.slot, th = s.th, a = radial(th, rS), p = radial(th, RG.rc - BM);
      yield* swing(a.x, a.y, s.zb + 0.015, th);
      yield* move({ x: p.x, y: p.y, z: s.zb + 0.015 }, { vMax: 1.4 }); yield* move({ x: p.x, y: p.y, z: s.zb - 0.015 }, { vMax: 0.3 });
      c.held = null; unLive(b); s.bin = b; b.slot = s; b.where = 'rack'; setInst(s); s.res = false;
      yield* move({ x: a.x, y: a.y, z: s.zb - 0.015 }, { vMax: 1.4 }); return; }
    const h = dst, sx = h.x - (BM + 0.65) * Math.cos(h.yaw), ix = h.x - BM * Math.cos(h.yaw);
    yield* swing(sx, h.y, Z.r1Up, h.yaw);
    yield* move({ x: ix, y: h.y, z: Z.r1Up }, { vMax: 1.2 }); yield* move({ x: ix, y: h.y, z: Z.r1In }, { vMax: 0.3 });
    c.held = null; placeBin(b, h.x, h.y, K.zc, 0); h.z.bin = b; h.z.res = null; b.zi = h.i; b.loop = h.L.id; b.where = h.L.id; b.placed = false;
    Object.assign(b, { station: null, done: false, picks: [], allocated: false, refilled: false });
    yield* move({ x: sx, y: h.y, z: Z.r1In }, { vMax: 1.2 }); b.placed = true;
  };
  Object.assign(c.tcp, { x: rS, y: 0, z: 1.2, yaw: 0 });
  while (true) {
    let t = null;
    setStep(2, 'R1 · waiting for a task', 'asking the WMS for the next bin move', {});
    yield* waitUntil(() => robotOk('r1'), robotWhy('r1') || 'R1 not available');
    const req = wmsCall('R1Next', ...r1Heads());
    yield* waitUntil(() => req.done, 'WMS');
    if (!req.ok || !req.value) { yield* dwell(0.6); continue; }
    t = r1Task(req.value);
    if (!t) { const cx = wmsCall('R1Cancel', req.value.id); yield* waitUntil(() => cx.done, 'WMS'); yield* dwell(1); continue; }
    if (t.src) yield* waitUntil(() => t.src.z.bin === t.b && !t.b.mv, 'bin at the transfer head');
    if (t.dst) yield* waitUntil(() => !t.dst.z.bin && !t.dst.z.res, 'transfer head free');
    c.task = t; const b = t.b; b.res = true; if (t.slot) t.slot.res = true; if (t.dst) t.dst.z.res = 'r1';
    const t0 = S.t;
    if (t.kind === 'retrieve') {
      setStep(2, 'R1 · retrieve bin {b} ({s} · {n} pcs) from column {c} level {l}', 'fork under the bin · lift 15 mm · out radially', { b: b.id, s: SKUS[b.sku].code, n: b.n, c: t.slot.k + 1, l: t.slot.j + 1 });
      yield* take({ slot: t.slot }, b);
      setStep(3, 'R1 · bin {b} onto the picking loop', 'chain head in the east port', { b: b.id });
      yield* put(t.dst, b); S.stat.retrieved++;
    } else if (t.kind === 'store' || t.kind === 'storeRst') {
      setStep(t.kind === 'store' ? 7 : 8, 'R1 · store bin {b} ({n} pcs) in column {c} level {l}', 'dynamic slotting: nearest free slot', { b: b.id, n: b.n, c: t.slot.k + 1, l: t.slot.j + 1 });
      yield* take(t.src, b); yield* put({ slot: t.slot }, b); S.stat.stored++;
    } else {
      setStep(8, 'R1 · empty bin {b} → restock loop', 'west port → operator outside', { b: b.id });
      if (t.kind === 'toRst') yield* take(t.src, b); else yield* take({ slot: t.slot }, b);
      yield* put(t.dst, b); S.stat.toRst++;
    }
    b.res = false; c.task = null; c.n = (c.n || 0) + 1; c.lastCycle = S.t - t0; c.cyc.push(c.lastCycle); if (c.cyc.length > 40) c.cyc.shift();
    const done = wmsCall('R1Complete', t.v.id); yield* waitUntil(() => done.done, 'WMS');
  }
}

// ------------------------------------------------------------------ R2 / R3 (IRB 1300): 3D vision picking from the bin at P1 / P2 onto the item belt
const binAt = P => { const z = LOOPS.pick.zones[P.zi], b = z.bin; return b && !b.mv && b.station === P.k ? b : null; };
const PLACE = {};                                                            // belt place claims per robot
function beltFree(x, l) { return BELT.every(it => Math.abs(it.x - x) > (SIZE[it.sz].l + l) / 2 + 0.07 && !(it.x < x && it.x > x - 0.75)); }
function* progPick(id) {
  const c = ctx(id), P = ST[id === 'r2' ? 'P1' : 'P2'], B = RC.motion.bind(c), { move, dwell, waitUntil, setStep } = B;
  const stage = { x: P.x, y: 1.55, z: Z.pCarry }, R = id.toUpperCase(), yw = c.robot.yaw || 0;
  /* bin (south) and belt (north) lie on opposite sides of the base: swing round the west side (relative angle 0), radius ≥ 0.6 m */
  const go = function* (x, y, z) {
    const bx = c.base.x, by = c.base.y, a0 = wrap(Math.atan2(c.tcp.y - by, c.tcp.x - bx) - yw), a1 = wrap(Math.atan2(y - by, x - bx) - yw);
    if (Math.abs(a1 - a0) < 0.4) { yield* move({ x, y, z, yaw: 0 }); return; }
    const r0 = Math.hypot(c.tcp.x - bx, c.tcp.y - by), r1 = Math.hypot(x - bx, y - by), n = Math.ceil(Math.abs(a1 - a0) / 0.3), via = [];
    for (let i = 1; i < n; i++) { const u = i / n, a = a0 + (a1 - a0) * u + yw, r = Math.max(0.6, lerp(r0, r1, u)); via.push({ x: bx + r * Math.cos(a), y: by + r * Math.sin(a), z }); }
    yield* move({ x, y, z, yaw: 0 }, { via });
  };
  Object.assign(c.tcp, { x: stage.x, y: stage.y, z: stage.z, yaw: 0 });
  P.toolIn = () => c.tcp.z < Z.rim + 0.25 && Math.abs(c.tcp.x - P.x) < BN.L / 2 + 0.1 && Math.abs(c.tcp.y - P.y) < BN.W / 2 + 0.1;
  while (true) {
    setStep(id === 'r2' ? 4 : 4, '{r} · waiting for a bin at {p}', '3D image after every pick · robot outside the camera view', { r: R, p: P.k });
    yield* go(stage.x, stage.y, stage.z);
    let b = null;
    yield* waitUntil(() => robotOk(id), robotWhy(id) || 'robot not available');
    yield* waitUntil(() => { b = binAt(P); return robotOk(id) && b && !b.done && b.picks && b.pi < b.picks.length && P.valid && b.items.length > 0; }, 'no bin / 3D imaging');
    const t0 = S.t, it = b.items[b.items.length - 1], pk = b.picks[b.pi];
    wmsCall('RobotBegin', R, 'pick:' + pk.id, 'pick ' + SKUS[b.sku].code + ' from bin ' + b.id + ' for order ' + pk.orderId);
    it.g.updateWorldMatrix(true, false); const wp = new THREE.Vector3().setFromMatrixPosition(it.g.matrixWorld), h = SIZE[it.sz].h;
    setStep(4, '{r} · pick {s} for order {o} from bin {b}', '4-cup vacuum · top layer first · {n} left in the bin', { r: R, s: SKUS[b.sku].code, o: pk.orderId, b: b.id, n: b.items.length - 1 });
    yield* go(wp.x, wp.y, Z.pCarry);
    yield* move({ x: wp.x, y: wp.y, z: wp.z + h }, { surface: wp.z + h });
    c.vac = true; yield* dwell(0.12);
    b.items.pop(); b.n--; grab(c, it.g); c.item = it; it.pick = pk.id; it.orderId = pk.orderId; b.pi++; P.valid = false; S.stat.picked++;
    yield* move({ x: wp.x, y: wp.y, z: Z.pCarry });
    yield* go(P.x, BL.y, Z.pCarry);
    if (b.pi >= b.picks.length) b.done = true;                               // tool has left the bin: the stop may release it
    setStep(5, '{r} · place on the item belt', 'belt holds while the gripper is down · free gap ≥ 70 mm', { r: R });
    yield* waitUntil(() => !zStop() && beltFree(P.x, SIZE[it.sz].l), 'no free gap on the belt');
    PLACE[id] = P.x;
    yield* move({ x: P.x, y: BL.y, z: BL.z + h }, { surface: BL.z + h });
    c.vac = false; yield* dwell(0.06); c.held = null; c.item = null;
    it.x = P.x; it.state = 'belt'; it.g.position.set(it.x, BL.y, BL.z); it.g.rotation.set(0, 0, 0); BELT.push(it);
    wmsCall('PickDone', it.pick);                                           // booked out of the bin, tracked by its pick id
    yield* move({ x: P.x, y: BL.y, z: Z.pCarry }); delete PLACE[id];
    c.n = (c.n || 0) + 1; c.lastCycle = S.t - t0; c.cyc.push(c.lastCycle); if (c.cyc.length > 30) c.cyc.shift();
  }
}
function updateCams(dt) {
  for (const P of Object.values(ST)) {
    const b = binAt(P);
    if (!b || b.done) { P.proj.visible = false; P.imgT = 0; continue; }
    if (!P.valid) { if (!coneBlocked(P) && !zStop()) { P.imgT += dt; P.proj.visible = true; P.proj.material.opacity = 0.1 + 0.08 * Math.sin(S.t * 30); if (P.imgT >= 0.45) { P.valid = true; P.imgT = 0; P.images++; P.proj.visible = false; } } else { P.imgT = 0; P.proj.visible = false; } }
  }
  S.interlocks.image = Object.values(ST).every(P => !P.proj.visible || !coneBlocked(P));
  S.interlocks.binStop = Object.values(ST).every(P => !P.toolIn || !P.toolIn() || !LOOPS.pick.zones.some(z => z.bin && z.bin.mv && (z.x === P.x || z.bin.mv.b.x === P.x) && Math.abs(z.y - P.y) < 0.1));
}

// ------------------------------------------------------------------ item belt: scanner, holds for the sorters / placing, exception end
const SID = { S1: 's1', S2: 's2' };
function updateBelt(dt) {
  const B = BL; let halt = zStop() ? 'stop' : '';
  if (!halt && Object.keys(PLACE).some(id => ctx(id).tcp.z < Z.pCarry - 0.03)) halt = 'place';
  for (const sid of ['s1', 's2']) { const c = ctx(sid), X = K.rob[sid].pickX;
    if (!halt && BELT.some(it => it.target === sid && it.x >= X - 1e-4 && robotOk(sid))) halt = 'sort';
    if (!halt && c.tcp.z < Z.sCarry - 0.02 && Math.hypot(c.tcp.x - X, c.tcp.y - B.y) < 0.25) halt = 'sort'; }
  if (!halt && BELT.some(it => it.target === 'wait' && it.x >= K.rob.s1.pickX - 0.05)) halt = 'scan';
  if (!halt && EXC.n >= K.ex.cap && BELT.some(it => it.x > B.endX - 0.3)) halt = 'exception tote full';
  G.beltHalt = halt; if (halt) { S.stat.haltT += dt; }
  else for (const it of BELT) { let nx = it.x + B.v * dt; if (it.target === 's1' || it.target === 's2') { const X = K.rob[it.target].pickX; if (it.x <= X && nx > X) nx = X; } it.x = nx; }
  G.scanBeam.visible = false;
  for (let i = BELT.length - 1; i >= 0; i--) { const it = BELT[i];
    if (!it.read && it.x >= B.scanX) { it.read = true; G.scanBeam.visible = true; S.stat.scans++;
      const nr = Math.random() < S.noRead; if (nr) S.stat.noRead++;
      it.target = 'wait'; it.scanReq = wmsCall('Scan', it.pick, nr ? null : SKUS[it.sku].code); }
    if (it.scanReq && it.scanReq.done) { const r = it.scanReq, v = r.ok ? r.value : null; it.scanReq = null;
      if (!v || v.target === 'Exception') { it.target = 'ex'; it.reason = v ? v.reason : 'refused'; } else { it.target = v.target.toLowerCase(); it.laneId = v.lane; } }
    if ((it.target === 's1' || it.target === 's2') && !robotOk(it.target)) { it.target = 'ex'; it.reason = 'sorter down'; wmsCall('Missed', it.pick); }
    if ((it.target === 's1' || it.target === 's2') && it.x > K.rob[it.target].pickX + 0.01) { it.target = 'ex'; it.reason = 'missed'; wmsCall('Missed', it.pick); }
    if (it.x >= B.x1 - 0.02) { BELT.splice(i, 1); toException(it); continue; }
    it.g.position.set(it.x, B.y, B.z); }
  S.interlocks.belt = !Object.keys(PLACE).length || !!halt;
}
function toException(it) {
  EXC.n++; S.stat.exc++; freeItem(it);                                     // booked by the WMS at the arch
  G.exFill.visible = EXC.n > 0; G.exFill.scale.z = Math.max(0.002, 0.25 * EXC.n / K.ex.cap); G.exFill.position.z = 0.04 + G.exFill.scale.z / 2;
}

// ------------------------------------------------------------------ S1 / S2 (IRB 930): pick at the belt stop, put into the order case
function* progSort(id) {
  const c = ctx(id), Rk = K.rob[id], B = RC.motion.bind(c), { move, dwell, waitUntil, setStep } = B, X = Rk.pickX, Rn = id.toUpperCase();
  const yw = c.robot.yaw || 0;
  /* arc round the base in the robot's relative angle (never through ±180° behind the base: one elbow, A1 ±145°) */
  const go = function* (x, y, z) {
    const bx = c.base.x, by = c.base.y, a0 = wrap(Math.atan2(c.tcp.y - by, c.tcp.x - bx) - yw), a1 = wrap(Math.atan2(y - by, x - bx) - yw);
    if (Math.abs(a1 - a0) < 0.3) { yield* move({ x, y, z }); return; }
    const r0 = Math.hypot(c.tcp.x - bx, c.tcp.y - by), r1 = Math.hypot(x - bx, y - by), n = Math.ceil(Math.abs(a1 - a0) / 0.3), via = [];
    for (let i = 1; i < n; i++) { const u = i / n, a = a0 + (a1 - a0) * u + yw, r = Math.max(0.55, Math.min(0.62, lerp(r0, r1, u))); via.push({ x: bx + r * Math.cos(a), y: by + r * Math.sin(a), z }); }
    yield* move({ x, y, z }, { via });
  };
  Object.assign(c.tcp, { x: X, y: BL.y, z: Z.sCarry, yaw: 0 });
  while (true) {
    setStep(id === 's1' ? 6 : 6, '{r} · waiting for an item', 'scanned at the arch · belt stops the item under the cup', { r: Rn });
    yield* move({ x: X, y: BL.y, z: Z.sCarry });
    let it = null;
    yield* waitUntil(() => robotOk(id), robotWhy(id) || 'robot not available');
    yield* waitUntil(() => !!(it = BELT.find(q => q.target === id && Math.abs(q.x - X) < 1e-3)) && robotOk(id), 'no item for this sorter');
    const t0 = S.t, h = SIZE[it.sz].h, ln = LANES.find(l => l.id === it.laneId);
    if (!ln) { it.target = 'ex'; wmsCall('Missed', it.pick); continue; }
    wmsCall('RobotBegin', Rn, 'sort:' + it.pick, 'put ' + SKUS[it.sku].code + ' (carton #' + it.pick + ') into lane ' + ln.id);
    setStep(6, '{r} · {s} → order {o} (lane {l})', 'bellows cup · bar code read at the arch', { r: Rn, s: SKUS[it.sku].code, o: it.orderId, l: ln.id });
    yield* move({ x: X, y: BL.y, z: BL.z + h }, { surface: BL.z + h });
    c.vac = true; yield* dwell(0.08); BELT.splice(BELT.indexOf(it), 1); grab(c, it.g); c.item = it; it.state = 'held';
    yield* move({ x: X, y: BL.y, z: Z.sCarry });
    yield* go(ln.x, PW.yIn, Z.sCarry);
    yield* waitUntil(() => ln.state === 'active' && ln.u === 0, 'case lane moving');
    const zd = Z.caseRim + 0.03 + h; c.inLane = ln;
    yield* move({ x: ln.x, y: PW.yIn, z: zd }, { surface: zd });
    c.vac = false; yield* dwell(0.05); c.held = null; c.item = null; ln.items.push(it); showLane(ln);
    S.stat.sorted++; S.out.items.push(S.t); if (S.out.items.length > 400) S.out.items.shift();
    const sr = wmsCall('Sort', it.pick, ln.id);
    yield* move({ x: ln.x, y: PW.yIn, z: Z.sCarry }); c.inLane = null;
    yield* waitUntil(() => sr.done, 'WMS');
    if (sr.ok && sr.value === true) completeOrder(ln);
    yield* go(X, BL.y, Z.sCarry);
    c.n = (c.n || 0) + 1; c.lastCycle = S.t - t0; c.cyc.push(c.lastCycle); if (c.cyc.length > 30) c.cyc.shift();
  }
}
/* the WMS completed the lane's order: the case runs out to the packing operator */
function completeOrder(ln) {
  ORDERS.done++; ORDERS.doneT.push(S.t); if (ORDERS.doneT.length > 200) ORDERS.doneT.shift();
  ln.state = 'out'; ln.order = null;
}
/* put-wall lanes: a finished case runs out (only with the sorter's tool clear of the lane), the operator swaps it, an empty case runs in */
const toolOverLane = ln => { const c = ctx(ln.sid); return c.tcp.z < Z.sCarry - 0.03 && Math.abs(c.tcp.x - ln.x) < 0.25 && c.tcp.y > PW.yIn - 0.25; };
function updateLanes(dt) {
  for (const ln of LANES) {
    if (ln.state === 'out' && !zStop() && !toolOverLane(ln)) { ln.u = Math.min(1, ln.u + PW.v * dt / (PW.yOut - PW.yIn)); if (ln.u >= 1) { ln.state = 'atOuter'; ln.outT = S.t; ln.req = wmsCall('Lane', ln.id, 'CaseAtOperator'); } }
    else if (ln.state === 'in' && !zStop()) { ln.u = Math.max(0, ln.u - PW.v * dt / (PW.yOut - PW.yIn)); if (ln.u <= 0) { ln.state = 'ready'; ln.req = wmsCall('Lane', ln.id, 'EmptyCaseInPosition'); } }
    if (ln.req && ln.req.done) { const r = ln.req; ln.req = null; if (r.ok && r.value) applyLane(ln, r.value); }
    showLane(ln);
    setLamp(ln.lamp, ln.state === 'atOuter' ? 0xff6a13 : ln.state === 'active' ? 0x22ff66 : ln.state === 'out' || ln.state === 'in' ? 0x2fa8ff : 0x444444);
  }
  S.interlocks.lanes = LANES.every(ln => !(ln.state === 'out' || ln.state === 'in') || !toolOverLane(ln));
}
/* a WMS lane state onto a lane that stands still (moving cases and the operator's swap are driven from here) */
function applyLane(ln, w) {
  if (ln.req || !(ln.state === 'ready' || ln.state === 'active')) return;
  if (w.state === 'Active') { ln.state = 'active'; ln.order = { id: w.orderId }; }
  else if (w.state === 'Ready') { ln.state = 'ready'; ln.order = null; }
  else if (w.state === 'Outbound') { ln.state = 'out'; ln.order = null; }
}
const pwMuted = sid => LANES.some(ln => ln.sid === sid && (ln.state === 'out' || ln.state === 'in'));

// ------------------------------------------------------------------ operators: OP1 restock (west, outside), OP2 put wall + exception tote (north / east)
function makeOperator(shirt) {
  const g = new THREE.Group(); RC.scene.add(g);
  for (const dx of [-0.1, 0.1]) cylZ(0.07, 0.85, M.pants, dx, 0, 0.425, g, 12);
  box(0.42, 0.24, 0.62, shirt, 0, 0, 1.16, g);
  const head = new THREE.Mesh(new THREE.SphereGeometry(0.11, 20, 14), M.skin); head.position.set(0, 0, 1.6); head.castShadow = true; g.add(head);
  const arms = [-0.26, 0.26].map(dx => { const a = new THREE.Group(); a.position.set(dx, 0, 1.42); a.rotation.x = 0.2; g.add(a); cylZ(0.045, 0.62, shirt, 0, 0, -0.31, a, 10); return a; });
  g.userData.arms = arms; return g;
}
function updateOp1(dt) {                                                    // restock: refill the empty bin at the operator stop, press "done"
  const op = G.op1, L = LOOPS.rst, z = L.zones[zoneOf(L, 'OP')], b = z.bin && !z.bin.mv ? z.bin : null;
  if (b && !b.refilled) { op.t += dt; op.busy += dt; op.task = ['refill bin {b} with {s} ({n} → {c} pcs)', { b: b.id, s: SKUS[b.sku].code, n: b.n, c: dimOf(b.sku).cap }];
    if (op.t >= S.refillT && !b.rstReq && (b.rstT || 0) <= S.t) b.rstReq = wmsCall('Restock', b.id);
    if (b.rstReq && b.rstReq.done) { const r = b.rstReq; b.rstReq = null;
      if (!r.ok || !r.value) b.rstT = S.t + 5;
      else { if (r.value.sku >= 0 && r.value.sku !== b.sku) { for (const it of b.items) freeItem(it); b.items = []; b.n = 0; b.sku = r.value.sku; }
        const q = r.value.qty, add = q - b.n; for (let k = b.n; k < q; k++) { const it = takeItem(b.sku), p = gridPos(b.sku, k); b.g.add(it.g); it.g.position.set(p.x, p.y, p.z); b.items.push(it); }
        b.n = q; b.refilled = true; op.t = 0; op.n++; S.stat.refills++; S.stat.restocked += add; } } }
  else { op.t = 0; op.task = null; }
  op.g.rotation.z = -Math.PI / 2 + Math.PI; op.g.position.set(K.rst.opX, b ? K.laneY : 0, 0);
  op.g.userData.arms.forEach((a, j) => { a.rotation.x = op.task ? 1.0 + (j ? 0.25 : -0.25) * Math.sin(S.t * 4) : 0.2; });
  setLamp(G.rstLamp, b && !b.refilled ? 0xff6a13 : b ? 0x2fa8ff : 0x22ff66);
}
const WK = K.walk;
function perim(p) {
  const c = [{ x: clamp(p.x, WK.x0, WK.x1), y: WK.y0 }, { x: WK.x1, y: clamp(p.y, WK.y0, WK.y1) }, { x: clamp(p.x, WK.x0, WK.x1), y: WK.y1 }, { x: WK.x0, y: clamp(p.y, WK.y0, WK.y1) }];
  let best = 0, bd = 1e9; c.forEach((q, i) => { const d = Math.hypot(q.x - p.x, q.y - p.y); if (d < bd) { bd = d; best = i; } });
  const Wd = WK.x1 - WK.x0, Hd = WK.y1 - WK.y0, q = c[best];
  const s = best === 0 ? q.x - WK.x0 : best === 1 ? Wd + q.y - WK.y0 : best === 2 ? Wd + Hd + WK.x1 - q.x : 2 * Wd + Hd + WK.y1 - q.y;
  return { q, s };
}
function route(a, b) {
  const corners = [{ s: 0, x: WK.x0, y: WK.y0 }, { s: WK.x1 - WK.x0, x: WK.x1, y: WK.y0 }, { s: WK.x1 - WK.x0 + WK.y1 - WK.y0, x: WK.x1, y: WK.y1 }, { s: 2 * (WK.x1 - WK.x0) + WK.y1 - WK.y0, x: WK.x0, y: WK.y1 }];
  const pa = perim(a), pb = perim(b), Lt = 2 * (WK.x1 - WK.x0 + WK.y1 - WK.y0); let d = pb.s - pa.s; if (d > Lt / 2) d -= Lt; if (d < -Lt / 2) d += Lt;
  const rel = s => { let x = (s - pa.s) % Lt; if (x < 0) x += Lt; return d >= 0 ? x : (Lt - x) % Lt; };
  const cs = corners.filter(cn => { const x = rel(cn.s); return x > 1e-6 && x < Math.abs(d) - 1e-6; }).sort((p, q) => rel(p.s) - rel(q.s));
  return [pa.q, ...cs.map(cn => ({ x: cn.x, y: cn.y })), pb.q, b];
}
function op2Task() {
  const ln = LANES.filter(q => q.state === 'atOuter' && !q.claimOp).sort((a, b) => a.outT - b.outT)[0];
  if (ln) { ln.claimOp = true; return { kind: 'case', ln, spot: { x: ln.x, y: WK.y1 }, face: -Math.PI / 2, dur: 14, label: ['lane {l}: full case → packing, empty case in', { l: ln.id }] }; }
  if (EXC.n >= 8 || (EXC.n > 0 && S.t - (EXC.lastT || 0) > 600)) { EXC.claim = true; return { kind: 'ex', spot: { x: WK.x1, y: K.ex.y }, face: Math.PI, dur: 25, label: ['exception tote: {n} items → re-scan / restock', { n: EXC.n }] }; }
  return null;
}
function updateOp2(dt) {
  const op = G.op2, v = WK.v;
  if (!op.task && !EXC.claim) { const t = op2Task(); if (t) { op.task = Object.assign({ t: 0, leg: 0 }, t); op.task.pts = route({ x: op.x, y: op.y }, t.spot); } else if (!op.home) { op.home = route({ x: op.x, y: op.y }, WK.idle); op.hl = 0; } }
  const stepTo = p => { const dx = p.x - op.x, dy = p.y - op.y, L = Math.hypot(dx, dy), st = Math.max(0, v * dt); if (L <= st) { op.x = p.x; op.y = p.y; op.walk += L; return true; } op.x += dx / L * st; op.y += dy / L * st; op.walk += st; op.face = Math.atan2(dy, dx); return false; };
  const T_ = op.task;
  if (T_) { op.home = null;
    if (T_.leg < T_.pts.length) { if (stepTo(T_.pts[T_.leg])) T_.leg++; }
    else { op.face = T_.face; T_.t += dt; op.busy += dt;
      if (T_.kind === 'case' && T_.t > 0.5 * T_.dur && T_.ln.items.length) { for (const it of T_.ln.items) freeItem(it); T_.ln.items = []; S.stat.cases++; }
      if (T_.t >= T_.dur) {
        if (T_.kind === 'case') { T_.ln.state = 'in'; T_.ln.claimOp = false; T_.ln.req = wmsCall('Lane', T_.ln.id, 'CaseReplaced'); }
        else { S.stat.exRestocked += EXC.n; EXC.n = 0; EXC.claim = false; EXC.lastT = S.t; G.exFill.visible = false; wmsCall('ToteEmptied'); }
        op.task = null; op.n++; op.home = route({ x: op.x, y: op.y }, WK.idle); op.hl = 0; } } }
  else if (op.home) { if (op.hl < op.home.length) { if (stepTo(op.home[op.hl])) op.hl++; } }
  if (EXC.claim && !op.task) EXC.claim = false;
  op.g.position.set(op.x, op.y, 0); op.g.rotation.z = op.face - Math.PI / 2;
  op.g.userData.arms.forEach((a, j) => { a.rotation.x = T_ && T_.leg >= T_.pts.length ? 1.0 + (j ? 0.25 : -0.25) * Math.sin(S.t * 4) : 0.2; });
}

// ------------------------------------------------------------------ zone: one guarded zone, guard-locked doors north (cylinder passage) and south (picking)
function freeZone() { return { open: false, door: 0, unlockT: 0, resetT: 0, stopped: false }; }
function updateZone(dt) {
  const Zn = S.zone;
  if (Zn.open) { Zn.stopped = true; Zn.resetT = 0; if (RC.ctxs.every(c => c.v < 1e-3)) Zn.unlockT += dt; if (Zn.unlockT >= 0.5) Zn.door = Math.min(1, Zn.door + dt / 1.5); }
  else { Zn.unlockT = 0; if (Zn.door > 0) Zn.door = Math.max(0, Zn.door - dt / 1.5); else if (Zn.stopped) { Zn.resetT += dt; if (Zn.resetT >= 1.5) { Zn.stopped = false; Zn.resetT = 0; } } }
  S.interlocks.zone = !Zn.stopped; S.interlocks.estop = !S.estop;
  setLamp(G.stack, S.estop ? 0xff3a3a : Zn.stopped ? 0xffb020 : 0x22ff66);
  const N = RC.elec && RC.elec.nodes, lamp = (nd, ok) => { if (nd && nd.lamp) nd.lamp.material.emissiveIntensity = ok ? 1.2 : 0.15; };
  if (N) { lamp(N.RC1, !zStop()); lamp(N.RC2, !zStop()); lamp(N.MCP, !S.estop); lamp(N.SC1, !zStop()); }
}
const holdOf = () => () => S.estop ? 'E-STOP · cell stopped' : S.zone && S.zone.stopped ? 'protective stop · cell door' : '';

// ------------------------------------------------------------------ figures
const avg = a => a.length ? a.reduce((p, q) => p + q, 0) / a.length : null;
function rate(T_, win) { const n = T_.length; if (n < 4) return null; let i = 0; if (win) while (i < n - 4 && T_[n - 1] - T_[i] > win) i++; return (n - 1 - i) / (T_[n - 1] - T_[i]) * 3600; }

// ------------------------------------------------------------------ WMS state → cell, polling
/* bins where the WMS has them: rack slots, loop zones (in their order), stops; cartons on the belt; R1 starts without a task */
function loadWmsState() {
  const I = WMS.init; loadSkus();
  BINS.length = 0;
  const byLoc = {};
  for (const w of I.bins) {
    const b = { id: w.id, sku: w.sku, n: w.qty, slot: null, where: 'rack', g: null, items: [], res: false, mv: null, station: null, done: false, picks: [], pi: 0,
                allocated: false, placed: true, refilled: false, zi: -1, loop: null };
    BINS.push(b);
    if (w.loc === 'Rack') { const s = SLOTS.find(q => q.k === w.col && q.j === w.level); if (s) { s.bin = b; b.slot = s; continue; } }
    (byLoc[w.loc] = byLoc[w.loc] || []).push({ b, w });
  }
  const put = (L, i, b) => { const z = L.zones[i]; makeLive(b); placeBin(b, z.x, z.y, K.zc, 0); z.bin = b; b.zi = i; b.loop = L.id; b.where = L.id; };
  const order = list => (list || []).sort((p, q) => String(p.w.moved || '').localeCompare(String(q.w.moved || '')));
  const P = LOOPS.pick, R = LOOPS.rst, zi = (L, k) => zoneOf(L, k);
  for (const { b, w } of order(byLoc.AtStation)) { const k = w.station || 'P2'; put(P, zi(P, k), b); b.station = k; b.picks = (I.stations[k] || []).slice(); b.done = !b.picks.length; }
  const free = (L, idx) => idx.filter(i => !L.zones[i].bin);
  let slots = free(P, [zi(P, 'DEC'), 1, 0]);
  for (const { b, w } of order(byLoc.PickInbound)) { const i = slots.shift(); if (i === undefined) break; put(P, i, b); b.station = w.station; }
  slots = free(P, [13, 12, 11, 10, 9, 8, 7]);
  for (const { b } of order(byLoc.PickReturn)) { const i = slots.shift(); if (i === undefined) break; put(P, i, b); b.done = true; }
  slots = free(R, [zi(R, 'OP'), 2, 1, 0]);
  for (const { b } of order(byLoc.RestockStation)) { const i = slots.shift(); if (i === undefined) break; put(R, i, b); }
  slots = free(R, [7, 6, 5, 4]);
  for (const { b } of order(byLoc.RestockReturn)) { const i = slots.shift(); if (i === undefined) break; put(R, i, b); b.refilled = true; }
  // cartons on the belt: unscanned ones before the arch, routed ones before their sorter
  let xa = BL.scanX - 0.35, xs = { s1: K.rob.s1.pickX - 0.05, s2: K.rob.s2.pickX - 0.05 };
  for (const w of I.belt || []) {
    const it = takeItem(w.sku); it.pick = w.pick; it.orderId = w.orderId; it.state = 'belt';
    if (w.status === 'Routed' && w.sorter) { const sid = w.sorter.toLowerCase(); it.read = true; it.target = sid; it.laneId = w.lane; it.x = xs[sid]; xs[sid] -= 0.3; }
    else { it.read = false; it.x = Math.max(BL.x0 + 0.1, xa); xa -= 0.3; }
    it.g.position.set(it.x, BL.y, BL.z); BELT.push(it);
  }
}
/* label over each put-wall lane: lane, order reference, sorted / ordered cartons (orange: the order in focus) */
function laneLabel(ln, w) {
  const txt = w && w.reference ? [ln.id + ' · ' + w.reference, w.sorted + ' / ' + w.items + ' cartons'] : [ln.id, w ? laneWord(w.state) : ''];
  const focus = !!(w && w.orderId && w.orderId === WMS.focus), key = txt.join('|') + focus;
  if (ln.labelKey === key) return; ln.labelKey = key;
  if (!ln.label) {
    const cv = document.createElement('canvas'); cv.width = 512; cv.height = 150;
    const tex = new THREE.CanvasTexture(cv); const sp = new THREE.Sprite(new THREE.SpriteMaterial({ map: tex, depthTest: false, transparent: true }));
    sp.scale.set(0.4, 0.117, 1); sp.position.set(ln.x, PW.yIn + 0.45, PW.z + 0.62); sp.renderOrder = 20; RC.scene.add(sp); ln.label = { cv, tex, sp };
  }
  const { cv, tex } = ln.label, g = cv.getContext('2d');
  g.clearRect(0, 0, cv.width, cv.height);
  g.fillStyle = focus ? 'rgba(255,106,19,.92)' : 'rgba(20,24,29,.82)'; g.strokeStyle = focus ? '#ffd0b0' : '#4a5561'; g.lineWidth = 4;
  g.beginPath(); if (g.roundRect) g.roundRect(4, 4, cv.width - 8, cv.height - 8, 18); else g.rect(4, 4, cv.width - 8, cv.height - 8); g.fill(); g.stroke();
  g.fillStyle = '#ffffff'; g.font = '600 46px "Segoe UI", system-ui, sans-serif'; g.textBaseline = 'middle'; g.fillText(fit(g, txt[0], cv.width - 36), 18, 50);
  g.fillStyle = focus ? '#fff3ea' : '#c6ccd2'; g.font = '40px "Segoe UI", system-ui, sans-serif'; g.fillText(txt[1], 18, 108);
  if (w && w.items) { g.fillStyle = 'rgba(255,255,255,.18)'; g.fillRect(18, 132, cv.width - 36, 8); g.fillStyle = focus ? '#ffffff' : '#35c26b'; g.fillRect(18, 132, (cv.width - 36) * Math.min(1, w.sorted / w.items), 8); }
  tex.needsUpdate = true;
}
const fit = (g, t, w) => { if (g.measureText(t).width <= w) return t; while (t.length > 3 && g.measureText(t + '…').width > w) t = t.slice(0, -1); return t + '…'; };
const laneWord = st => ({ Ready: 'empty case, waiting for an order', Outbound: 'finished case → packing', AtOperator: 'at the packing operator', Inbound: 'empty case coming in' })[st] || st;
const esc = t => String(t == null ? '' : t).replace(/[&<>"]/g, c => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' })[c]);
/* the orders panel at the top of the Process panel */
function renderOrders() {
  const el = document.getElementById('wms-orders'); if (!el) return;
  const lanes = WMS.lanes || [], q = WMS.queue || [], o = WMS.orders || {};
  const rows = lanes.map(w => { const f = w.orderId && w.orderId === WMS.focus, pct = w.items ? Math.round(100 * w.sorted / w.items) : 0;
    return '<li class="' + (f ? 'focus' : '') + '" data-order="' + (w.orderId || '') + '"><b>' + esc(w.id) + '</b><span class="ref">' + (w.reference ? esc(w.reference) : '<i>' + esc(laneWord(w.state)) + '</i>') + '</span>'
      + (w.reference ? '<span class="num">' + w.sorted + '/' + w.items + '</span><span class="pbar"><i style="width:' + pct + '%"></i></span>' : '') + '</li>'; }).join('');
  const queue = q.length ? q.map(x => '<li class="' + (x.id === WMS.focus ? 'focus' : '') + '" data-order="' + x.id + '"><span class="ref">' + esc(x.reference) + (x.priority ? ' <em>P' + x.priority + '</em>' : '') + '</span><span class="lines">' + esc(x.lines) + '</span></li>').join('')
    : '<li class="none">No order waiting. Create one on the Orders page' + (WMS.opts && WMS.opts.generateOrders ? '' : ' (or switch on Generate demo orders)') + '.</li>';
  const html = '<div class="oh"><span>Orders on the put wall</span><span>' + (o.completed || 0) + ' done</span></div><ul class="lanes">' + rows + '</ul>'
    + '<div class="oh"><span>Queue · ' + (o.queued || 0) + '</span><span>released when a lane has an empty case</span></div><ul class="queue">' + queue + '</ul>';
  if (el.innerHTML !== html) el.innerHTML = html;
}
/* point the 3D view at an order: its lane label and its cartons turn orange, the camera goes to the put wall */
WMS.focusOrder = id => {
  WMS.focus = id || null;
  for (const w of WMS.lanes || []) { const ln = LANES.find(l => l.id === w.id); if (ln) laneLabel(ln, w); }
  renderOrders();
  if (WMS.focus && (WMS.lanes || []).some(w => w.orderId === WMS.focus) && RC.setView) RC.setView('putwall');
};
function tintFocus() {
  const f = WMS.focus, paint = it => { if (!it || !it.g) return; const m = f && it.orderId === f ? M.cartonFocus : M.carton; if (it.g.children[0].material !== m) it.g.children[0].material = m; };
  for (const it of BELT) paint(it); for (const ln of LANES) for (const it of ln.items) paint(it); for (const c of RC.ctxs) paint(c.item);
}
function applyPoll(p) {
  if (!p) return;
  WMS.lanes = p.lanes || WMS.lanes; WMS.queue = p.queue || WMS.queue || [];
  for (const w of WMS.lanes || []) { const ln = LANES.find(l => l.id === w.id); if (ln) laneLabel(ln, w); }
  WMS.robots = {}; for (const r of p.robots || []) WMS.robots[r.id] = r;
  WMS.orders = p.orders || WMS.orders; WMS.opts = p.options || WMS.opts; WMS.tote = p.tote;
  if (WMS.opts) { S.noRead = WMS.opts.noReadRate; S.mode = WMS.opts.pareto ? 'zipf' : 'flat'; }
  renderOrders();
  for (const w of p.lanes || []) { const ln = LANES.find(l => l.id === w.id); if (ln) applyLane(ln, w); }
  // a picker that went down: the WMS released the bin at its stop
  for (const k of ['P1', 'P2']) { const P = ST[k], b = binAt(P); if (b && !robotOk(P.rob) && !b.done && !(ctx(P.rob).held)) { b.picks = []; b.done = true; } }
}
/* once a real second: lanes, robots, order figures (the WMS tops up the order queue) */
function syncWms() {
  if (WMS.pollReq && WMS.pollReq.done) { const r = WMS.pollReq; WMS.pollReq = null; if (r.ok) applyPoll(r.value); }
  const now = performance.now();
  if (!WMS.pollReq && now - WMS.pollAt > 1000) { WMS.pollAt = now; WMS.pollReq = wmsCall('Poll'); }
}

// ------------------------------------------------------------------ station definition
const rob = (id, type, prog, extra) => Object.assign({ id, type, color: 0xf4f4f2, motion: motion[id], pedestal: K.rob[id].ped, yaw: K.rob[id].yaw, base: K.rob[id].base, glb: type, program: prog }, extra || {});
RC.defineStation({
  id: 'logistics_application_wms', K,
  robots: [rob('r1', 'irb6700', progR1), rob('r2', 'irb1300', () => progPick('r2')), rob('r3', 'irb1300', () => progPick('r3')),
           rob('s1', 'irb930', () => progSort('s1'), { elbow: K.rob.s1.elbow }), rob('s2', 'irb930', () => progSort('s2'), { elbow: K.rob.s2.elbow })],
  floor: { w: 19.5, l: 12.0, x: 3.3, y: 0.7 },
  views: { iso: [-2.3, 0.78, 19.5, [3.6, 0.3, 0.8]], top: [-Math.PI / 2, 1.52, 19.0, [3.4, 0.7, 0]], front: [-Math.PI / 2, 0.2, 16.5, [3.4, 0.5, 1.2]],
           cylinder: [-2.4, 0.95, 8.5, [0, 0, 1.4]], picking: [-1.2, 0.62, 4.6, [5.0, 0.8, 1.0]], sorting: [-1.75, 0.6, 4.6, [8.2, 2.5, 1.0]],
           putwall: [1.35, 0.5, 5.2, [8.2, 4.0, 0.8]], restock: [-2.3, 0.62, 6.5, [-4.0, 0, 0.8]] },
  build() {
    mats(); initLoops();
    for (const c of RC.ctxs) { c.hold = holdOf(); c.cyc = []; }
    buildRing(); buildConveyors(); buildCameras(); buildBelt(); buildPutWall(); buildRestock();
    pedestal(K.rob.r1.base, K.rob.r1.ped, 1.1); for (const id of ['r2', 'r3']) pedestal(K.rob[id].base, K.rob[id].ped); for (const id of ['s1', 's2']) pedestal(K.rob[id].base, K.rob[id].ped, 0.5);
    buildTools();
    // bins: the WMS's bins (placed in reset)
    loadSkus();
    // HMI + stack light at the south door, floor marking of the walkway
    box(0.05, 0.05, 1.2, M.dark, 4.3, F.y0 - 0.12, 0.6); box(0.36, 0.05, 0.26, M.dark, 4.3, F.y0 - 0.16, 1.35); box(0.32, 0.01, 0.2, mat(0x2f8fff, { emissive: 0x1f5faf, emissiveIntensity: 0.6 }), 4.3, F.y0 - 0.19, 1.35);
    G.stack = cylZ(0.03, 0.1, lampMat(0x22ff66), 3.9, F.y0, F.h + 0.08, null, 12);
    RC.floorLine((WK.x0 + WK.x1) / 2, WK.y1, WK.x1 - WK.x0, 0.05); RC.floorLine(WK.x1, (WK.y0 + WK.y1) / 2, 0.05, WK.y1 - WK.y0); RC.floorLine((WK.x0 + WK.x1) / 2, WK.y0, WK.x1 - WK.x0, 0.05);
    RC.floorLine(K.rst.opX, 0, 0.6, 1.4);
    G.op1 = { g: makeOperator(M.shirt), t: 0, busy: 0, n: 0, task: null };
    G.op2 = { g: makeOperator(M.shirt2), x: WK.idle.x, y: WK.idle.y, face: 0, task: null, home: null, walk: 0, busy: 0, n: 0 };
  },
  safety() {
    const y = K.laneY, xs1 = PW.xs.s1, xs2 = PW.xs.s2, bl = xs => [xs[0] - PW.w / 2 - 0.05, xs[xs.length - 1] + PW.w / 2 + 0.05];
    const b1 = bl(xs1), b2 = bl(xs2);
    return {
      fence: F,
      runs: [{ side: 'front' }, { side: 'top', openings: [b1, b2] }, { side: 'left', openings: [[-y - 0.29, y + 0.29]] }, { side: 'right' }],
      walls: [],
      curtains: [{ x: K.rst.tunX, from: -y - 0.29, to: y + 0.29, z0: 0.15, h: 1.0, model: 'F3SG-PG' },
                 { y: PW.curtainY, from: b1[0], to: b1[1], z0: 0.15, h: 0.75, model: 'F3SG-PG 30 mm' }, { y: PW.curtainY, from: b2[0], to: b2[1], z0: 0.15, h: 0.75, model: 'F3SG-PG 30 mm' }],   // hand detection: S 528 mm needed, 970 mm to the sorter
      doors: [{ side: 'top', from: K.door.n[0], to: K.door.n[1] }, { side: 'front', from: K.door.s[0], to: K.door.s[1] }],
      scanners: { r: K.scanR, model: 'OS32C', opacity: 0.1, units: [
        { x: F.x0 + 0.05, y: F.y0 + 0.05, a: Math.PI / 4 }, { x: F.x0 + 0.05, y: F.y1 - 0.05, a: -Math.PI / 4 }, { x: 0.2, y: 0.62, a: Math.PI / 4 }, { x: -0.62, y: -0.5, a: -Math.PI * 3 / 4 },
        { x: 3.0, y: F.y0 + 0.05, a: Math.PI / 2 }, { x: 7.0, y: F.y0 + 0.05, a: Math.PI / 2 }, { x: F.x1 - 0.05, y: F.y0 + 0.05, a: 3 * Math.PI / 4 }, { x: F.x1 - 0.05, y: F.y1 - 0.05, a: -3 * Math.PI / 4 },
        { x: 4.6, y: F.y1 - 0.05, a: -Math.PI / 2 }] },
      hoods: [{ axis: 'x', x0: K.rst.tunX, x1: F.x0, y0: -y - 0.29, y1: y + 0.29, zBase: K.rst.roof },
              { axis: 'y', x0: b1[0], x1: b1[1], y0: F.y1, y1: PW.curtainY, zBase: PW.roof }, { axis: 'y', x0: b2[0], x1: b2[1], y0: F.y1, y1: PW.curtainY, zBase: PW.roof }],
      stations: [{ x: K.door.n[1] + 0.3, y: F.y1 + 0.3 }, { x: K.door.s[1] + 0.3, y: F.y0 - 0.3 }, { x: -5.1, y: -0.85 }, { x: 8.2, y: 4.75 }, { x: 10.3, y: 4.75 }, { x: F.x1 + 0.3, y: 1.2 }, { x: -1.6, y: F.y0 - 0.3 }],
      state: () => {
        const Zn = S.zone, dtx = RC.t(Zn.door > 0 ? 'open' : Zn.open ? 'unlocking' : Zn.stopped ? 'closed · reset pending' : 'closed · locked');
        const mu = [rstMuted(), pwMuted('s1'), pwMuted('s2')];
        return { doorOpen: [Zn.door, Zn.door], doorText: ['doors {a}', { a: dtx }], curtainMuted: mu,
                 curtainText: ['restock tunnel {a} · put wall S1 {b} · S2 {c}', { a: RC.t(mu[0] ? 'muted (bin passing)' : 'active'), b: RC.t(mu[1] ? 'muted (case passing)' : 'active'), c: RC.t(mu[2] ? 'muted (case passing)' : 'active') }],
                 scannerText: S.estop ? 'E-STOP' : 'field clear (restart interlock)', fenceText: S.estop ? 'E-STOP · cell stopped' : 'one guarded zone · cylinder inside the fence' };
      },
    };
  },
  electrical() { return elecPlan(); },
  reset() {
    if (!G.trimmed) { G.trimmed = true; RC.scene.traverse(o => { if (o.isMesh && !o.isInstancedMesh && o.castShadow && o.geometry) { if (!o.geometry.boundingSphere) o.geometry.computeBoundingSphere(); if (o.geometry.boundingSphere.radius < 0.08) o.castShadow = false; } }); }
    for (const c of RC.ctxs) { c.held = null; c.item = null; c.task = null; c.waitAcc = 0; c.busyAcc = 0; c.n = 0; c.cyc = []; c.waiting = ''; c.vac = false; c.inLane = null; }
    for (const it of ITEMPOOL) freeItem(it);
    BELT.length = 0; for (const k in PLACE) delete PLACE[k];
    for (const L of Object.values(LOOPS)) { for (const z of L.zones) { z.bin = null; z.res = null; } L.moves = 0; }
    for (const s of SLOTS) { s.bin = null; s.res = false; }
    loadWmsState();
    for (const s of SLOTS) setInst(s);
    for (const P of Object.values(ST)) Object.assign(P, { valid: false, imgT: 0, images: 0 });
    ORDERS.queue = []; ORDERS.active = []; ORDERS.seq = 0; ORDERS.done = 0; ORDERS.doneT = [];
    const laneState = { Ready: ['ready', 0], Active: ['active', 0], Outbound: ['out', 0], AtOperator: ['atOuter', 1], Inbound: ['in', 1] };
    for (const ln of LANES) { const w = (WMS.init.lanes || []).find(q => q.id === ln.id), st = laneState[w ? w.state : 'Ready'] || ['ready', 0];
      Object.assign(ln, { u: st[1], state: st[0], order: w && w.orderId ? { id: w.orderId } : null, items: [], claimOp: false, outT: 0, req: null }); showLane(ln); }
    EXC.n = WMS.init.tote || 0; EXC.claim = false; EXC.lastT = 0; G.exFill.visible = EXC.n > 0;
    S.zone = freeZone(); S.zone.open = !!S.toggles.door; S.estop = !!S.toggles.estop;
    S.stat = { retrieved: 0, stored: 0, toRst: 0, presented: 0, picked: 0, sorted: 0, scans: 0, noRead: 0, exc: 0, exRestocked: 0, refills: 0, restocked: 0, cases: 0, haltT: 0 };
    S.out = { items: [], leadT: [] };
    Object.assign(G.op1, { t: 0, busy: 0, n: 0, task: null });
    Object.assign(G.op2, { x: WK.idle.x, y: WK.idle.y, face: 0, task: null, home: null, walk: 0, busy: 0, n: 0 });
    for (const k in motion) motion[k].vMax = V0[k] * S.speedPct / 100;
    S.interlocks = { zone: true, estop: !S.estop, image: true, binStop: true, belt: true, lanes: true, reachAll: true };
    applyPoll(WMS.init);
  },
  update(dt) {
    if (!(dt >= 0)) return;                                                  // see stepTo in updateOp2
    for (const c of RC.ctxs) { if (c.waiting) c.waitAcc += dt; else c.busyAcc += dt; }
    updateZone(dt); syncWms(); resolveArrivals(); tintFocus(); updateLoop(LOOPS.pick, dt); updateLoop(LOOPS.rst, dt); updateCams(dt); updateBelt(dt); updateLanes(dt); updateOp1(dt); updateOp2(dt);
    for (const k of ['P1', 'P2']) { const P = ST[k], b = binAt(P); G.blades[k].position.z = b && !b.done ? K.zc - 0.03 + 0.05 : K.zc - 0.08; }
    S.interlocks.reachAll = RC.ctxs.every(c => c.reach);
  },
  onPose(tcp, c) {
    const h = c.held; if (!h) return;
    const cy = Math.cos(tcp.yaw), sy = Math.sin(tcp.yaw);
    h.m.position.set(tcp.x + h.lx * cy - h.ly * sy, tcp.y + h.lx * sy + h.ly * cy, tcp.z - h.dz); h.m.rotation.set(0, 0, tcp.yaw + h.rz, 'ZYX');
  },
  setMode(m) { S.mode = m === 'flat' ? 'flat' : 'zipf'; wmsCall('SetOption', 'pareto', S.mode === 'zipf' ? 1 : 0); },
  setToggle(id, on) { if (id === 'door') S.zone.open = on; else if (id === 'estop') S.estop = on; },
  setNumber(id, v) {
    if (id === 'speed' && v > 0) { S.speedPct = clamp(v, 10, 100); for (const k in motion) motion[k].vMax = V0[k] * S.speedPct / 100; }
    else if (id === 'no-read' && v >= 0) { S.noRead = clamp(v, 0, 20) / 100; WMS.bridge && WMS.bridge.invoke('SetOption', 'no-read', v); }
    else if (id === 'refill-t' && v > 0) S.refillT = clamp(v, 5, 300);
    else if (id === 'lines-max' && v > 0) { S.linesMax = Math.round(clamp(v, 1, 6)); WMS.bridge && WMS.bridge.invoke('SetOption', 'lines-max', S.linesMax); }
    else if (id === 'qty-max' && v > 0) { S.qtyMax = Math.round(clamp(v, 1, 5)); WMS.bridge && WMS.bridge.invoke('SetOption', 'qty-max', S.qtyMax); }
  },
  hud() {
    const out = {}, s = S.stat, r = rate(S.out.items, 900), ro = rate(ORDERS.doneT, 1800);
    out['m-rate'] = r ? ['{r} items / h sorted · {o} orders / h', { r: r.toFixed(0), o: ro ? ro.toFixed(0) : '—' }] : '— (measuring)';
    const wo = WMS.orders;
    out['m-orders'] = wo ? ['WMS: {d} done · {a} active · {q} queued · lead time {l} s', { d: wo.completed, a: wo.active, q: wo.queued, l: wo.leadSeconds ? wo.leadSeconds.toFixed(0) : '—' }] : '—';
    out['m-ordrate'] = wo ? ['{n} orders in the last hour', { n: wo.lastHour }] : '—';
    const rs = Object.values(WMS.robots);
    out['m-wms'] = rs.length ? rs.map(r => r.id + ' ' + (r.state === 'Fault' ? '⚠ ' + (r.faultCode || 'fault') : r.mode !== 'Auto' ? r.mode.toLowerCase() : r.state === 'Working' ? 'working' : 'idle')).join(' · ') : '—';
    out['m-wmscalls'] = ['{n} calls · {e} refused', { n: WMS.calls, e: WMS.errors.length }];
    out['m-bins'] = ['{p} presented · {i} items / bin · {r} retrieved · {s} stored', { p: s.presented, i: s.presented ? (s.picked / s.presented).toFixed(2) : '—', r: s.retrieved, s: s.stored }];
    const rack = BINS.filter(b => b.where === 'rack').length, freeS = SLOTS.filter(q => !q.bin).length, empty = BINS.filter(b => b.n <= 0).length;
    out['m-rack'] = ['{b} bins in the shelf · {f} free slots · {e} empty · loop {l} · restock {q}', { b: rack, f: freeS, e: empty, l: pickLoopCount(), q: rstCount() }];
    const cyc = id => { const a = ctx(id).cyc; return a.length ? avg(a).toFixed(1) : '—'; };
    const busy = id => (100 * ctx(id).busyAcc / Math.max(S.t, 1e-9)).toFixed(0);
    out['m-r1'] = ['{b} % busy · {c} s per bin move', { b: busy('r1'), c: cyc('r1') }];
    out['m-pick'] = ['R2 {a} % · R3 {b} % busy · {c} s per item · {n} picked', { a: busy('r2'), b: busy('r3'), c: cyc('r2'), n: s.picked }];
    out['m-sort'] = ['S1 {a} % · S2 {b} % busy · {c} s per item', { a: busy('s1'), b: busy('s2'), c: cyc('s1') }];
    out['m-belt'] = ['{n} items · {h} · halted {t} %', { n: BELT.length, h: RC.t(G.beltHalt ? 'holds: ' + G.beltHalt : 'running'), t: (100 * s.haltT / Math.max(S.t, 1e-9)).toFixed(0) }];
    out['m-lanes'] = LANES.map(l => l.id.slice(-1) === '1' ? l.id.slice(0, 2) + ' ' + RC.t(l.state).slice(0, 3) : RC.t(l.state).slice(0, 3)).join(' · ');
    out['m-cam'] = ['P1 {a} · P2 {b} images', { a: ST.P1.images, b: ST.P2.images }];
    out['m-exc'] = ['{e} to the exception tote ({n} in the tote) · {s} scans', { e: s.exc, n: EXC.n, s: s.scans }];
    out['m-rst'] = ['{r} bins refilled ({p} pcs)', { r: s.refills, p: s.restocked }];
    out['m-op1'] = G.op1.task ? [G.op1.task[0], G.op1.task[1]] : 'idle';
    const op = G.op2; out['m-op2'] = ['{t} · {u} % busy · {w} m walked', { t: op.task ? RC.t(op.task.label[0], op.task.label[1]) : RC.t('idle'), u: (100 * op.busy / Math.max(S.t, 1e-9)).toFixed(0), w: Math.round(op.walk) }];
    out['m-demand'] = RC.t(S.mode === 'flat' ? 'flat demand (all SKUs alike)' : 'Pareto demand (top 20 % SKUs ≈ 55 % lines)');
    return out;
  },
  i18n: { zh: {
    "Logistics application · cylinder ASRS → 3D vision picking → bar-code sortation into order cases · IRB 6700 + 2 × IRB 1300 + 2 × IRB 930": "物流應用 · 圓筒自動倉儲 → 3D 視覺揀貨 → 條碼分揀至訂單箱 · IRB 6700＋2 × IRB 1300＋2 × IRB 930",
    "Orders on the put wall → R1 retrieves the bin from the cylinder shelf → picking loop → R2 / R3 pick with 3D vision onto the item belt → bar-code arch → S1 / S2 put into the order case → case out to packing · bins back to the shelf, empty bins to restock": "放置牆上之訂單 → R1 由圓筒料架取出料箱 → 揀貨迴路 → R2／R3 以 3D 視覺揀至物品輸送帶 → 條碼門架 → S1／S2 放入訂單箱 → 訂單箱送出包裝 · 料箱回存料架、空料箱送補貨",
    "R1 · cylinder shelf (fork)": "R1 · 圓筒料架（貨叉）",
    "R2 · picking P1 (3D vision)": "R2 · 揀貨 P1（3D 視覺）",
    "R3 · picking P2 (3D vision)": "R3 · 揀貨 P2（3D 視覺）",
    "S1 · sortation, lanes S1-1…4": "S1 · 分揀，通道 S1-1…4",
    "S2 · sortation, lanes S2-1…4": "S2 · 分揀，通道 S2-1…4",
    "Rolling waves: each of the 8 put-wall lanes with an empty case takes the next order; the WMS adds up the open lines of all active orders per SKU": "滾動波次：8 條放置牆通道中有空訂單箱者接下一張訂單；WMS 依 SKU 彙總所有進行中訂單之未完成行",
    "R1 (IRB 6700, fork) retrieves a bin of the next open SKU from the cylinder shelf (26 columns × 8 levels, 144 slots) and sets it on the picking loop": "R1（IRB 6700，貨叉）由圓筒料架（26 欄 × 8 層、144 料位）取出下一個未完成 SKU 之料箱，放上揀貨迴路",
    "The bin runs through the east port to stop P2 (or P1 while P2 is busy); a bin without open demand runs straight back": "料箱經東埠送至停止位 P2（P2 忙碌時為 P1）；無未完成需求之料箱直接返回",
    "R2 / R3 (IRB 1300, 4-cup vacuum) pick the cartons of every active order for that SKU — the 3D camera images the bin before each pick": "R2／R3（IRB 1300，4 吸盤真空）為所有進行中訂單揀取該 SKU 之紙箱 — 每次揀取前 3D 相機拍攝料箱",
    "Cartons go onto the item belt into a free gap (the belt holds while the gripper is down); the bar-code arch reads each carton (no-read → exception tote)": "紙箱放入物品輸送帶之空隙（夾爪下降時輸送帶保持）；條碼門架讀取每一紙箱（無法讀取 → 例外料盒）",
    "S1 / S2 (IRB 930) take their cartons at the belt stop and put them into the order case on the put-wall lane": "S1／S2（IRB 930）於輸送帶停止位取其紙箱，放入放置牆通道上之訂單箱",
    "The bin returns on the inner lane; R1 stores it in the nearest free slot (dynamic slotting) — returns before retrievals": "料箱經內側線返回；R1 存入最近之空料位（動態儲位）— 回存優先於取出",
    "Empty bins go out through the west port to the restock operator (refill, push-button) and come back in to be stored": "空料箱經西埠送至補貨作業員（補貨、按鈕）後送回存放",
    "Finished order: the case runs out through the tunnel to the packing operator, an empty case runs in and the lane takes the next order": "訂單完成：訂單箱經隧道送出至包裝作業員，空訂單箱送入，通道接下一張訂單",
    "Safety: one guarded zone, guard-locked doors north (cylinder passage) / south, 9 × OS32C restart prevention, muted curtains at the restock and put-wall tunnels": "安全：單一防護區、北側（圓筒通道）／南側防護鎖門、9 台 OS32C 防止重啟、補貨與放置牆隧道設遮蔽光幕",
    "Sim time": "模擬時間",
    "Output": "產出",
    "Orders": "訂單",
    "Demand": "需求",
    "Bin presentations": "料箱呈現",
    "Cylinder shelf": "圓筒料架",
    "R1 · shelf robot": "R1 · 料架機器人",
    "R2 / R3 · picking": "R2／R3 · 揀貨",
    "3D images": "3D 拍攝",
    "S1 / S2 · sortation": "S1／S2 · 分揀",
    "Item belt": "物品輸送帶",
    "Put-wall lanes": "放置牆通道",
    "Bar code · exceptions": "條碼 · 例外",
    "Restock": "補貨",
    "Restock operator": "補貨作業員",
    "Packing operator": "包裝作業員",
    "Last cycle": "上一週期",
    "F3SG-PG × 3 (restock tunnel multi-beam, put wall 30 mm; muted while a bin / case passes)": "F3SG-PG × 3（補貨隧道多光束、放置牆 30 mm；料箱／訂單箱通過時遮蔽）",
    "Cell doors locked · running": "工作站門已鎖定 · 運轉中",
    "Cell STOPPED (door)": "工作站已停止（安全門）",
    "E-stop released": "緊急停止已復歸",
    "E-STOP · cell stopped": "緊急停止 · 工作站停止",
    "3D image: field of view clear": "3D 拍攝：視野淨空",
    "Gripper in the 3D field of view": "夾爪位於 3D 視野內",
    "Bin stops: no bin moves under a gripper": "料箱停止位：夾爪下方無料箱移動",
    "Bin moving under a gripper": "夾爪下方料箱移動中",
    "Item belt holds while a gripper places": "夾爪放置時物品輸送帶保持",
    "Belt running under a placing gripper": "放置中夾爪下方輸送帶運轉",
    "Put-wall lanes move only with the sorter clear": "放置牆通道僅於分揀機器人離開時移動",
    "Case lane moving under a sorter": "分揀機器人下方訂單箱通道移動中",
    "All robot TCPs within reach": "所有機器人 TCP 在可達範圍",
    "A robot TCP out of reach": "機器人 TCP 超出可達範圍",
    "Robot speed": "機器人速度",
    "Bar-code no-read": "條碼無法讀取",
    "Restock time per bin": "每箱補貨時間",
    "Lines per order (max)": "每張訂單行數（最大）",
    "Quantity per line (max)": "每行數量（最大）",
    "Pareto demand (fast movers)": "柏拉圖需求（快速流動品）",
    "Flat demand (all SKUs alike)": "平均需求（所有 SKU 相同）",
    "Iso": "等角",
    "Top": "俯視",
    "Front": "正視",
    "Picking": "揀貨",
    "Sortation": "分揀",
    "Put wall": "放置牆",
    "Cell door": "工作站門",
    "E-stop": "緊急停止",
    "Main control panel · line PLC + safety PLC + WMS server": "主控制盤 · 產線 PLC＋安全 PLC＋WMS 伺服器",
    "Conveyor control · picking / restock loops, item belt, put wall": "輸送控制 · 揀貨／補貨迴路、物品輸送帶、放置牆",
    "OmniCore V250XT · R1": "OmniCore V250XT · R1",
    "4 × OmniCore C30 · R2, R3, S1, S2": "4 × OmniCore C30 · R2、R3、S1、S2",
    "R1 · waiting for a task": "R1 · 等待任務",
    "no return, no restock bin, no open demand": "無回存、無補貨料箱、無未完成需求",
    "no bin to move": "無需搬移之料箱",
    "R1 · retrieve bin {b} ({s} · {n} pcs) from column {c} level {l}": "R1 · 由第 {c} 欄第 {l} 層取出料箱 {b}（{s} · {n} 個）",
    "fork under the bin · lift 15 mm · out radially": "貨叉伸入料箱下方 · 抬升 15 mm · 徑向退出",
    "R1 · bin {b} onto the picking loop": "R1 · 料箱 {b} 放上揀貨迴路",
    "chain head in the east port": "東埠鏈條輸送頭",
    "R1 · store bin {b} ({n} pcs) in column {c} level {l}": "R1 · 料箱 {b}（{n} 個）存入第 {c} 欄第 {l} 層",
    "dynamic slotting: nearest free slot": "動態儲位：最近之空料位",
    "R1 · empty bin {b} → restock loop": "R1 · 空料箱 {b} → 補貨迴路",
    "west port → operator outside": "西埠 → 外部作業員",
    "{r} · waiting for a bin at {p}": "{r} · 於 {p} 等待料箱",
    "3D image after every pick · robot outside the camera view": "每次揀取後 3D 拍攝 · 機器人位於相機視野外",
    "no bin / 3D imaging": "無料箱／3D 拍攝中",
    "{r} · pick {s} for order {o} from bin {b}": "{r} · 由料箱 {b} 為訂單 {o} 揀取 {s}",
    "4-cup vacuum · top layer first · {n} left in the bin": "4 吸盤真空 · 由上層先取 · 料箱剩 {n} 個",
    "{r} · place on the item belt": "{r} · 放上物品輸送帶",
    "belt holds while the gripper is down · free gap ≥ 70 mm": "夾爪下降時輸送帶保持 · 空隙 ≥ 70 mm",
    "no free gap on the belt": "輸送帶上無空隙",
    "{r} · waiting for an item": "{r} · 等待紙箱",
    "scanned at the arch · belt stops the item under the cup": "於門架掃描 · 輸送帶將紙箱停於吸盤下方",
    "no item for this sorter": "無此分揀機器人之紙箱",
    "{r} · {s} → order {o} (lane {l})": "{r} · {s} → 訂單 {o}（通道 {l}）",
    "bellows cup · bar code read at the arch": "波紋吸盤 · 於門架讀取條碼",
    "case lane moving": "訂單箱通道移動中",
    "refill bin {b} with {s} ({n} → {c} pcs)": "料箱 {b} 補入 {s}（{n} → {c} 個）",
    "lane {l}: full case → packing, empty case in": "通道 {l}：滿箱 → 包裝、送入空箱",
    "exception tote: {n} items → re-scan / restock": "例外料盒：{n} 件 → 重新掃描／歸位",
    "doors {a}": "門 {a}",
    "restock tunnel {a} · put wall S1 {b} · S2 {c}": "補貨隧道 {a} · 放置牆 S1 {b} · S2 {c}",
    "muted (bin passing)": "遮蔽（料箱通過）",
    "muted (case passing)": "遮蔽（訂單箱通過）",
    "active": "作用中",
    "field clear (restart interlock)": "防護區淨空（防止重啟）",
    "one guarded zone · cylinder inside the fence": "單一防護區 · 圓筒位於圍籬內",
    "E-STOP": "緊急停止",
    "protective stop · cell door": "保護性停止 · 工作站門",
    "open": "開啟",
    "unlocking": "解鎖中",
    "closed · reset pending": "關閉 · 待復歸",
    "closed · locked": "關閉 · 鎖定",
    "idle": "閒置",
    "running": "運轉中",
    "— (measuring)": "—（量測中）",
    "holds: place": "保持：放置中",
    "holds: sort": "保持：分揀取件",
    "holds: stop": "保持：停止",
    "holds: exception tote full": "保持：例外料盒已滿",
    "holds: scan": "保持：等待 WMS 分揀指示",
    "ready": "待命",
    "out": "送出",
    "atOuter": "待換箱",
    "in": "送入",
    "{r} items / h sorted · {o} orders / h": "每小時分揀 {r} 件 · {o} 張訂單",
    "{d} done · {a} active · lead time {l} s": "完成 {d} · 進行中 {a} · 前置時間 {l} 秒",
    "{p} presented · {i} items / bin · {r} retrieved · {s} stored": "呈現 {p} 次 · 每箱 {i} 件 · 取出 {r} · 回存 {s}",
    "{b} bins in the shelf · {f} free slots · {e} empty · loop {l} · restock {q}": "料架內 {b} 箱 · 空料位 {f} · 空箱 {e} · 迴路 {l} · 補貨 {q}",
    "{b} % busy · {c} s per bin move": "稼動 {b} % · 每次搬移 {c} 秒",
    "R2 {a} % · R3 {b} % busy · {c} s per item · {n} picked": "R2 {a} % · R3 {b} % 稼動 · 每件 {c} 秒 · 已揀 {n}",
    "S1 {a} % · S2 {b} % busy · {c} s per item": "S1 {a} % · S2 {b} % 稼動 · 每件 {c} 秒",
    "{n} items · {h} · halted {t} %": "{n} 件 · {h} · 停止 {t} %",
    "P1 {a} · P2 {b} images": "P1 {a} · P2 {b} 次拍攝",
    "{e} no-read → exception tote ({n} in the tote) · {s} scans": "無法讀取 {e} → 例外料盒（盒內 {n}）· 掃描 {s} 次",
    "{r} bins refilled ({p} pcs)": "已補貨 {r} 箱（{p} 個）",
    "{t} · {u} % busy · {w} m walked": "{t} · 稼動 {u} % · 步行 {w} m",
    "flat demand (all SKUs alike)": "平均需求（所有 SKU 相同）",
    "Pareto demand (top 20 % SKUs ≈ 55 % lines)": "柏拉圖需求（前 20 % SKU ≈ 55 % 行數）",
    "WMS: {d} done · {a} active · {q} queued · lead time {l} s": "WMS：完成 {d} · 進行中 {a} · 佇列 {q} · 前置時間 {l} 秒",
    "{n} calls · {e} refused": "{n} 次呼叫 · {e} 次拒絕",
    "{e} to the exception tote ({n} in the tote) · {s} scans": "{e} 件至例外料盒（盒內 {n}）· 掃描 {s} 次",
    "WMS robots": "WMS 機器人",
    "{n} orders in the last hour": "最近一小時 {n} 張訂單",
    "Orders, last hour": "訂單（最近一小時）",
    "WMS link": "WMS 連線",
    "R1 · waiting for a task": "R1 · 等待任務",
    "asking the WMS for the next bin move": "向 WMS 索取下一個料箱搬移",
    "bin at the transfer head": "料箱到達輸送頭",
    "transfer head free": "輸送頭淨空",
    "robot not available": "機器人無法使用",
    "R1 not available": "R1 無法使用",
  } },
  G, K, Z, SLOTS, BINS, BELT, LANES, ORDERS, LOOPS, ST, EXC, SKUS, SIZE, ITEMPOOL, inCone, coneBlocked, toolOverLane, rstMuted, pwMuted, dimOf, route, perim, binAt,
  equipBoxes() { return equipBoxes(); },
});

// ------------------------------------------------------------------ equipment footprints (operator workspace audit) and electrical plan
function keep() {
  const o = [], y = K.laneY, b = (name, x0, x1, y0, y1, h, extra) => o.push(Object.assign({ name, flow: true, h, x0: Math.min(x0, x1), x1: Math.max(x0, x1), y0: Math.min(y0, y1), y1: Math.max(y0, y1) }, extra || {}));
  b('picking loop', 1.75, 6.95, -y - 0.3, y + 0.3, 1.1, { under: false });
  b('restock loop', -1.75, -5.1, -y - 0.3, y + 0.3, 1.1);
  b('item belt', BL.x0 - 0.1, BL.x1 + 0.05, BL.y - 0.3, BL.y + 0.3, 0.95);
  b('exception chute + tote', BL.x1, K.ex.x + 0.25, BL.y - 0.25, BL.y + 0.25, 0.8);
  for (const sid of ['s1', 's2']) { const xs = PW.xs[sid]; b('put wall ' + sid.toUpperCase(), xs[0] - PW.w / 2 - 0.06, xs[xs.length - 1] + PW.w / 2 + 0.06, PW.y0, PW.y1, 0.85); }
  for (const k of ['P1', 'P2']) { const P = ST[k]; b('3D camera post ' + k, P.x - 0.08, P.x + 0.08, K.cam.postY - 0.08, K.cam.postY + 0.08, 2.4, { flow: false }); }
  return o;
}
function equipBoxes() {
  const o = keep().map(k => ({ n: k.name, x0: k.x0, x1: k.x1, y0: k.y0, y1: k.y1 }));
  o.push({ n: 'stock pallet', x0: -6.1, x1: -5.1, y0: 1.15, y1: 2.35 }, { n: 'case cart', x0: 5.6, x1: 6.4, y0: 5.5, y1: 6.0 }, { n: 'packing cart', x0: 9.8, x1: 10.8, y0: 5.45, y1: 6.05 });
  return o;
}
function elecPlan() {
  const zT = 2.4, yN = F.y1 + 0.3, yS = F.y0 - 0.3, xW = F.x0 - 0.3, xE = F.x1 + 0.3, y = K.laneY;
  const dev = [], add = (id, x, y_, z, kind, jb, mount) => dev.push(Object.assign({ id, x, y: y_, z, kind: kind || 'sensor', jb }, mount ? { mount } : {}));
  /* zone conveyors: one zone-controller card (two MDR zones, IO-Link) per zone pair on the lane frame, bin sensors only where a bin is
     held for a decision (heads, decision point, stops, RAT ends); 3D cameras along their cantilever to the post */
  const zc = (L, pre, jbOf) => { for (let i = 0; i < L.zones.length; i += 2) { const a = L.zones[i], b = L.zones[Math.min(i + 1, L.zones.length - 1)];
    add(pre + 'Z' + (i / 2 + 1), (a.x + b.x) / 2, Math.min(a.y, b.y) - 0.27, 0.55, 'motor', jbOf((a.x + b.x) / 2)); } };
  zc(LOOPS.pick, 'P', x => x <= 4.4 ? 'JP1' : 'JP2'); zc(LOOPS.rst, 'R', () => 'JR1');
  for (const k of ['H', 'DEC', 'P1', 'P2', 'END', 'RIN', 'HIN']) { const z = LOOPS.pick.zones[zoneOf(LOOPS.pick, k)]; add('BP' + k, z.x - 0.2, z.y - 0.27, 0.72, 'sensor', z.x <= 4.4 ? 'JP1' : 'JP2'); }
  for (const k of ['P1', 'P2']) { const P = ST[k], j = k === 'P1' ? 'JP1' : 'JP2'; add('V' + k, P.x, P.y, K.cam.z, 'camera', j, [{ x: P.x, y: K.cam.postY }]); add('Y' + k, P.x + 0.32, y - 0.27, 0.65, 'valve', j); }
  for (const k of ['H', 'OP', 'RIN', 'HIN']) { const z = LOOPS.rst.zones[zoneOf(LOOPS.rst, k)]; add('BR' + k, z.x + 0.2, z.y - 0.27, 0.72, 'sensor', z.x < -4 ? 'JR2' : 'JR1'); }
  add('SB1', -5.2, y + 0.35, 1.0, 'sensor', 'JR2'); add('HL1', -4.75, y + 0.33, 1.05, 'sensor', 'JR2');
  // item belt: drive, scanner (top + side), place / pick presence sensors, exception tote sensor
  add('MB', BL.x0 + 0.15, BL.y - BL.w / 2 - 0.2, 0.55, 'motor', 'JB1'); add('BC1', BL.scanX, BL.y, 1.08, 'camera', 'JB1', [{ x: BL.scanX, y: BL.y + 0.32 }]); add('BC2', BL.scanX, BL.y + 0.28, 0.92, 'camera', 'JB1', [{ x: BL.scanX, y: BL.y + 0.32 }]);
  for (const [n, x] of [['BS1', K.rob.s1.pickX], ['BS2', K.rob.s2.pickX], ['BQ2', 4.3], ['BQ3', 5.8]]) add(n, x, BL.y - BL.w / 2 - 0.03, 0.82, 'sensor', x > 7 ? 'JB2' : 'JB1');
  add('BX', K.ex.x, BL.y - 0.25, 0.35, 'sensor', 'JB2');
  // put wall: one drive + case sensor at the robot end + lamp at the operator end per lane (lane frame)
  LANES.forEach((ln, i) => { const j = ln.sid === 's1' ? 'JN1' : 'JN2'; add('ML' + (i + 1), ln.x + PW.w / 2, PW.yOut - 0.1, 0.5, 'motor', j); add('BL' + (i + 1), ln.x - PW.w / 2, PW.yIn, 0.55, 'sensor', j); add('HL' + (i + 2), ln.x, PW.yOut + 0.27, 0.6, 'sensor', j); });
  const jb = (id, x, y_, to, ports) => ({ id, x, y: y_, z: 1.0, to, post: true, ports: ports || 12 });
  return {
    main: { id: 'MCP', x: 1.4, y: -4.25, face: '-y', label: 'Main control panel · line PLC + safety PLC + WMS server' },
    subs: [{ id: 'SC1', x: 7.6, y: -4.25, w: 0.8, d: 0.6, h: 1.8, face: '-y', label: 'Conveyor control · picking / restock loops, item belt, put wall', to: 'MCP' }],
    controllers: [{ id: 'RC1', robots: ['r1'], x: -1.8, y: -4.25, w: 0.8, d: 0.7, h: 1.4, face: '-y', label: 'OmniCore V250XT · R1' },
                  { id: 'RC2', robots: ['r2', 'r3', 's1', 's2'], x: 3.0, y: 4.35, w: 1.8, d: 0.6, h: 1.8, face: '+y', label: '4 × OmniCore C30 · R2, R3, S1, S2' }],
    jbs: [jb('JP1', 3.0, yS, 'SC1'), jb('JP2', 6.2, yS, 'SC1'), jb('JR1', xW, -1.2, 'SC1'), jb('JR2', xW, 1.3, 'SC1'), jb('JC1', -0.55, yN, 'SC1', 8), jb('JB1', 4.6, yN, 'SC1'), jb('JB2', xE, 2.6, 'SC1'), jb('JN1', 6.35, yN, 'SC1'), jb('JN2', 10.05, yN, 'SC1')],
    devices: dev,
    trays: [
      { id: 'TS', kind: 'overhead', z: zT, w: 0.4, pts: [[xW, yS], [xE, yS]] },
      { id: 'TW', kind: 'overhead', z: zT, w: 0.3, pts: [[xW, yS], [xW, -1.0]] }, { id: 'TWL', kind: 'overhead', ladder: true, z: zT, w: 0.3, pts: [[xW, -1.0], [xW, 1.0]] }, { id: 'TW2', kind: 'overhead', z: zT, w: 0.3, pts: [[xW, 1.0], [xW, yN]] },
      { id: 'TE', kind: 'overhead', z: zT, w: 0.3, pts: [[xE, yS], [xE, yN]] },
      { id: 'TN', kind: 'overhead', z: zT, w: 0.4, pts: [[xW, yN], [6.2, yN]] }, { id: 'TNL', kind: 'overhead', ladder: true, z: zT, w: 0.3, pts: [[6.2, yN], [10.2, yN]] }, { id: 'TN2', kind: 'overhead', z: zT, w: 0.3, pts: [[10.2, yN], [xE, yN]] },
      { id: 'F1', kind: 'floor', pts: [[-0.15, yN], [-0.15, 0.75], [-0.75, 0.75], [-0.75, -0.5]] },   // into the cylinder through the north passage, round the pedestal (scanners)
      { id: 'F2', kind: 'floor', pts: [[3.3, yN], [3.3, 1.45], [5.5, 1.45], [5.5, 1.15]] },
      { id: 'F4', kind: 'floor', pts: [[6.0, yN], [6.0, 2.55], [7.05, 2.55]] }, { id: 'F5', kind: 'floor', pts: [[10.4, yN], [10.4, 2.55], [9.35, 2.55]] },
      { id: 'FP', kind: 'floor', pts: [[4.3, yS], [4.3, -0.95]] }, { id: 'FQ', kind: 'floor', pts: [[5.8, yS], [5.8, -0.95]] },
    ],
    keepouts: keep(),
  };
}
})();
