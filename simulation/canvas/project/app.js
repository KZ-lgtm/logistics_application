/* Robot-station simulation engine — generic part (three.js UMD, Z-up, metres, robot base = origin).
   A station registers itself with RC.defineStation({...}); see stations/irb660_cell/station.js for the contract. */
(function () {
'use strict';
const RC = window.RobotCell = window.RobotCell || {};
RC.robots = RC.robots || {};

// ------------------------------------------------------------------ helpers (also exposed as RC.h for stations)
const clamp = (v, a, b) => Math.max(a, Math.min(b, v));
const lerp = (a, b, t) => a + (b - a) * t;
const deg = r => r * 180 / Math.PI;
const rand = (a, b) => a + Math.random() * (b - a);
function mat(color, opts) { return new THREE.MeshStandardMaterial(Object.assign({ color, roughness: 0.6, metalness: 0.15 }, opts || {})); }
function box(w, l, h, m, x, y, z, parent) {
  const mesh = new THREE.Mesh(new THREE.BoxGeometry(w, l, h), m); mesh.position.set(x, y, z);
  mesh.castShadow = true; mesh.receiveShadow = true; (parent || RC.scene).add(mesh); return mesh;
}
function cylZ(r, h, m, x, y, z, parent, seg) {
  const g = new THREE.CylinderGeometry(r, r, h, seg || 24); g.rotateX(Math.PI / 2);
  const mesh = new THREE.Mesh(g, m); mesh.position.set(x, y, z); mesh.castShadow = true; mesh.receiveShadow = true; (parent || RC.scene).add(mesh); return mesh;
}
function cylX(r, h, m, x, y, z, parent, seg) {
  const g = new THREE.CylinderGeometry(r, r, h, seg || 16); g.rotateZ(Math.PI / 2);
  const mesh = new THREE.Mesh(g, m); mesh.position.set(x, y, z); mesh.castShadow = true; mesh.receiveShadow = true; (parent || RC.scene).add(mesh); return mesh;
}
function cylY(r, h, m, x, y, z, parent, seg) {
  const mesh = new THREE.Mesh(new THREE.CylinderGeometry(r, r, h, seg || 16), m); mesh.position.set(x, y, z);
  mesh.castShadow = true; mesh.receiveShadow = true; (parent || RC.scene).add(mesh); return mesh;
}
/* group whose origin sits at pivot P; child meshes keep native coordinates */
function pivotGroup(P, parent) {
  const g = new THREE.Group(); g.position.set(P.x, P.y || 0, P.z);
  const inner = new THREE.Group(); inner.position.set(-P.x, -(P.y || 0), -P.z);
  g.add(inner); g.userData.inner = inner; g.userData.P = { x: P.x, y: P.y || 0, z: P.z };
  (parent || RC.scene).add(g); return g;
}
/* ABB robot paint from the GLB's own materials: the slot with the most triangles is the body paint (→ body colour),
   red slots stay red (ABB labels), light greys become steel, everything else the dark joint/cable finish */
function robotMats(body) {
  return (materials, tri) => {
    const dark = mat(0x22272c, { roughness: 0.5, metalness: 0.4 }), red = mat(0xd8121c, { roughness: 0.5 }), steel = mat(0x8d949a, { roughness: 0.4, metalness: 0.6 });
    const paint = mat(body, { roughness: 0.45, metalness: 0.15 });
    const col = m => (m.pbrMetallicRoughness && m.pbrMetallicRoughness.baseColorFactor) || [0.5, 0.5, 0.5, 1];
    const isRed = m => { const c = col(m); return c[0] > 0.5 && c[1] < 0.25 && c[2] < 0.25; };
    let bodyIdx = -1, best = -1;
    materials.forEach((m, i) => { if (!isRed(m) && tri[i] > best) { best = tri[i]; bodyIdx = i; } });
    return materials.map((m, i) => { if (i === bodyIdx) return paint; if (isRed(m)) return red; const c = col(m); return (c[0] + c[1] + c[2]) / 3 > 0.45 ? steel : dark; });
  };
}
/* walking router (skill rule "Operator workspace"): occupancy grid over the floor, footprints inflated by the body radius; path()
   = A* (8-neighbour) from a point to the nearest free spot of the goal, simplified to waypoints; nearestFree() moves a standing spot
   out of equipment. router({ x0, x1, y0, y1, res, r }) → { clear(), mark(box), temp(boxes), free(x, y), line(p, q), nearestFree(x, y), path(a, b) } */
function router(o) {
  const res = o.res || 0.1, r = o.r === undefined ? 0.25 : o.r, nx = Math.ceil((o.x1 - o.x0) / res), ny = Math.ceil((o.y1 - o.y0) / res), occ = new Uint8Array(nx * ny), tmp = new Uint8Array(nx * ny);
  const ix = x => Math.floor((x - o.x0) / res), iy = y => Math.floor((y - o.y0) / res), cx = i => o.x0 + (i + 0.5) * res, cy = j => o.y0 + (j + 0.5) * res;
  const inG = (i, j) => i >= 0 && j >= 0 && i < nx && j < ny, isB = k => occ[k] === 1 || tmp[k] === 1, blocked = (i, j) => !inG(i, j) || isB(j * nx + i);
  const R = {
    clear() { occ.fill(0); },
    mark(b, into) { const A = into || occ; for (let j = Math.max(0, iy(b.y0 - r)); j <= Math.min(ny - 1, iy(b.y1 + r)); j++) for (let i = Math.max(0, ix(b.x0 - r)); i <= Math.min(nx - 1, ix(b.x1 + r)); i++) {
      const x = cx(i), y = cy(j), dx = Math.max(b.x0 - x, 0, x - b.x1), dy = Math.max(b.y0 - y, 0, y - b.y1); if (dx * dx + dy * dy < r * r) A[j * nx + i] = 1; } },
    temp(boxes) { tmp.fill(0); for (const b of boxes || []) R.mark(b, tmp); },                                   // extra footprints for the next plan (the item about to be set)
    free: (x, y) => !blocked(ix(x), iy(y)),
    line(p, q) { const L = Math.hypot(q[0] - p[0], q[1] - p[1]), n = Math.ceil(L / (res / 2)); for (let m = 1; m < n; m++) if (blocked(ix(p[0] + (q[0] - p[0]) * m / n), iy(p[1] + (q[1] - p[1]) * m / n))) return false; return true; },   // straight walk p → q clear
    nearestFree(x, y) {
      let i0 = clamp(ix(x), 0, nx - 1), j0 = clamp(iy(y), 0, ny - 1); if (!blocked(i0, j0)) return { x, y };
      for (let k = 1; k < Math.max(nx, ny); k++) { let best = null, bd = 1e9;
        for (let j = j0 - k; j <= j0 + k; j++) for (let i = i0 - k; i <= i0 + k; i++) { if (Math.max(Math.abs(i - i0), Math.abs(j - j0)) !== k || blocked(i, j)) continue; const d = Math.hypot(cx(i) - x, cy(j) - y); if (d < bd) { bd = d; best = { x: cx(i), y: cy(j) }; } }
        if (best) return best; }
      return { x, y };
    },
    path(a, b) {
      const a0 = R.nearestFree(a.x, a.y), esc = Math.hypot(a0.x - a.x, a0.y - a.y) > 1e-6;                       // a start inside a footprint just set: shortest step out first
      const g = R.nearestFree(b.x, b.y), s = [clamp(ix(a0.x), 0, nx - 1), clamp(iy(a0.y), 0, ny - 1)], t = [ix(g.x), iy(g.y)], N = nx * ny;
      const cost = new Float32Array(N).fill(Infinity), prev = new Int32Array(N).fill(-1), done = new Uint8Array(N), heap = [];
      const H = (i, j) => Math.hypot(i - t[0], j - t[1]), push = (k, f) => { heap.push([f, k]); let c = heap.length - 1; while (c > 0) { const p = (c - 1) >> 1; if (heap[p][0] <= heap[c][0]) break; [heap[p], heap[c]] = [heap[c], heap[p]]; c = p; } };
      const pop = () => { const top = heap[0], last = heap.pop(); if (heap.length) { heap[0] = last; let c = 0; for (;;) { const l = 2 * c + 1, rr = l + 1; let m = c; if (l < heap.length && heap[l][0] < heap[m][0]) m = l; if (rr < heap.length && heap[rr][0] < heap[m][0]) m = rr; if (m === c) break; [heap[m], heap[c]] = [heap[c], heap[m]]; c = m; } } return top; };
      const k0 = s[1] * nx + s[0], kt = t[1] * nx + t[0]; cost[k0] = 0; push(k0, H(s[0], s[1]));
      while (heap.length) {
        const [, k] = pop(); if (done[k]) continue; done[k] = 1; if (k === kt) break;
        const i = k % nx, j = (k - i) / nx;
        for (let dj = -1; dj <= 1; dj++) for (let di = -1; di <= 1; di++) { if (!di && !dj) continue; const a2 = i + di, b2 = j + dj; if (!inG(a2, b2)) continue;
          const k2 = b2 * nx + a2; if (isB(k2)) continue; if (di && dj && (blocked(i + di, j) || blocked(i, j + dj))) continue;
          const c2 = cost[k] + Math.hypot(di, dj); if (c2 < cost[k2]) { cost[k2] = c2; prev[k2] = k; push(k2, c2 + H(a2, b2)); } }
      }
      let kg = kt;
      if (prev[kt] < 0 && kt !== k0) {                                                                       // enclosed goal: the reachable free cell nearest to it
        let bd = Infinity; kg = k0; for (let k = 0; k < N; k++) if (done[k] && !isB(k)) { const d = Math.hypot(cx(k % nx) - g.x, cy((k - k % nx) / nx) - g.y); if (d < bd) { bd = d; kg = k; } }
        g.x = cx(kg % nx); g.y = cy((kg - kg % nx) / nx);
      }
      const cells = []; for (let k = kg; k >= 0 && k !== k0; k = prev[k]) cells.push(k); cells.reverse();
      const pts = cells.map(k => [cx(k % nx), cy((k - k % nx) / nx)]); if (!pts.length) return esc ? [[a0.x, a0.y], [g.x, g.y]] : [[g.x, g.y]]; pts[pts.length - 1] = [g.x, g.y];
      const seeLine = (p, q) => { const L = Math.hypot(q[0] - p[0], q[1] - p[1]), n = Math.ceil(L / (res / 2)); for (let m = 1; m < n; m++) { const x = p[0] + (q[0] - p[0]) * m / n, y = p[1] + (q[1] - p[1]) * m / n; if (blocked(ix(x), iy(y))) return false; } return true; };
      const out = esc ? [[a0.x, a0.y]] : []; let cur = [a0.x, a0.y], c = 0;                                                            // string-pulling: keep only the turning points
      while (c < pts.length) { let far = c; for (let m = pts.length - 1; m > c; m--) if (seeLine(cur, pts[m])) { far = m; break; } out.push(pts[far]); cur = pts[far]; c = far + 1; }
      return out;
    },
  };
  return R;
}
RC.h = { clamp, lerp, deg, rand, mat, box, cylZ, cylX, cylY, pivotGroup, robotMats, router };

// ------------------------------------------------------------------ minimal GLB parser → { linkName: THREE.Group }
/* mats: material array indexed by the GLB material slot, or a function (materials, triCount[]) → array (see RC.h.robotMats);
   offsets: {nodeName: [x, y, z]} translations the RobotStudio export dropped (robot_pim.json → serial6.meshOffsets) */
RC.parseGLB = function (buf, mats, offsets) {
  const dv = new DataView(buf);
  const jl = dv.getUint32(12, true);
  const json = JSON.parse(new TextDecoder().decode(new Uint8Array(buf, 20, jl)));
  const binOff = 20 + jl + 8;
  offsets = offsets || {};
  if (typeof mats === 'function') {
    const tri = (json.materials || []).map(() => 0);
    for (const m of json.meshes || []) for (const p of m.primitives) if (p.material !== undefined) tri[p.material] += json.accessors[p.indices].count / 3;
    mats = mats(json.materials || [], tri);
  }
  function acc(i) {
    const a = json.accessors[i], bv = json.bufferViews[a.bufferView];
    const off = binOff + (bv.byteOffset || 0) + (a.byteOffset || 0);
    const n = a.type === 'VEC3' ? 3 : a.type === 'VEC2' ? 2 : 1;
    const C = a.componentType === 5126 ? Float32Array : a.componentType === 5125 ? Uint32Array : a.componentType === 5123 ? Uint16Array : Uint8Array;
    return new C(buf.slice(off, off + a.count * n * C.BYTES_PER_ELEMENT));
  }
  const out = {};
  const roots = new Set((json.scenes && json.scenes[0] && json.scenes[0].nodes) || []);   // the mechanism root itself is not a link
  const addMesh = (grp, mi) => {
    for (const p of json.meshes[mi].primitives) {
      const g = new THREE.BufferGeometry();
      g.setAttribute('position', new THREE.BufferAttribute(acc(p.attributes.POSITION), 3));
      if (p.attributes.NORMAL !== undefined) g.setAttribute('normal', new THREE.BufferAttribute(acc(p.attributes.NORMAL), 3)); else g.computeVertexNormals();
      g.setIndex(new THREE.BufferAttribute(acc(p.indices), 1));
      const m = new THREE.Mesh(g, mats[p.material] || mats[0]); m.castShadow = true; m.receiveShadow = true; grp.add(m);
    }
  };
  /* a link = named node with children; every mesh in its subtree belongs to it (RobotStudio exports multi-part links,
     e.g. Link1 = {Link1, Cable}, as an unnamed group under the link node); node transforms are identity in these exports */
  const visit = (grp, ni) => {
    const n = json.nodes[ni]; let g = grp;
    if (n.name && offsets[n.name]) { g = new THREE.Group(); g.position.fromArray(offsets[n.name]); grp.add(g); }
    if (n.mesh !== undefined) addMesh(g, n.mesh); for (const c of n.children || []) visit(g, c);
  };
  json.nodes.forEach((node, idx) => {
    if (!node.name || !node.children || roots.has(idx)) return;
    const grp = new THREE.Group(); grp.name = node.name;
    if (offsets[node.name]) grp.position.fromArray(offsets[node.name]);
    for (const c of node.children) visit(grp, c);
    if (grp.children.length) out[node.name] = grp;
  });
  return out;
};
RC.b64ToBuf = function (b64) {
  const bin = atob(b64); const u8 = new Uint8Array(bin.length);
  for (let i = 0; i < bin.length; i++) u8[i] = bin.charCodeAt(i);
  return u8.buffer;
};

// ------------------------------------------------------------------ i18n (en / zh-TW); stations add their strings with RC.i18n.add
const ZH = {
  'Pause': '暫停', 'Run': '執行', 'Reset': '重置', 'Sim speed': '模擬倍速', 'Language': '語言',
  'Iso': '等角', 'Top': '俯視', 'Front': '正視', 'Robot': '機器人',
  'Drag: orbit · Right-drag / Shift-drag: pan · Wheel: zoom': '拖曳：旋轉 · 右鍵拖曳 / Shift+拖曳：平移 · 滾輪：縮放',
  'Loading robot model…': '載入機器人模型中…',
  'Sim time': '模擬時間', 'Last cycle': '上一週期', 'TCP speed': 'TCP 速度', 'Joints': '關節角度',
  'Robot TCP speed (max {v} mm/s · {s} mm/s in {z} mm zone)': '機器人 TCP 速度（最高 {v} mm/s · {z} mm 區內 {s} mm/s）',
  'Interlocks (collision avoidance)': '互鎖（防碰撞）', 'TCP within reach': 'TCP 在可達範圍', 'TCP out of reach': 'TCP 超出可達範圍',
  'WAIT: ': '等待：', 'Packer': '裝箱機器人', 'Palletizer': '堆疊機器人',
  'Process': '流程', 'Metrics': '數據', 'Controls': '控制', 'Collapse': '收合', 'Expand': '展開', 'Hide': '隱藏', 'Show': '顯示',
  'Safety': '安全防護', 'Safety fence H{h}': '安全圍籬 H{h}', 'Safety door': '安全門', 'Light curtain {m}': '安全光柵 {m}', 'Laser scanner {m} ×{n}': '安全雷射掃描器 {m} ×{n}',
  'Scan fields': '掃描區', 'Opacity': '透明度',
  'closed · locked': '關閉 · 鎖定', 'open': '開啟', 'clear': '無遮斷', 'muted (vehicle passing)': '屏蔽中（搬運車通過）', 'field clear': '防護區淨空', 'field switched (vehicle docking)': '防護區切換（搬運車進站）',
  'perimeter closed': '周界封閉',
  'Timeline': '時間軸', 'Seeking…': '跳轉中…', 'Go to phase {n}: {name}': '跳至階段 {n}：{name}', 'Project time': '專案時間',
  'not simulated in this mode': '此模式不模擬',
};
RC.lang = (navigator.language || '').toLowerCase().startsWith('zh') ? 'zh' : 'en';
const t = (key, args) => {
  if (Array.isArray(key)) { args = key[1]; key = key[0]; }
  let str = RC.lang === 'zh' ? (ZH[key] !== undefined ? ZH[key] : key) : key;
  if (args) for (const k in args) str = str.split('{' + k + '}').join(args[k]);
  return str;
};
RC.t = t;
RC.i18n = { add: d => { if (d && d.zh) Object.assign(ZH, d.zh); }, dict: ZH };
RC.applyLang = () => {
  document.documentElement.lang = RC.lang === 'zh' ? 'zh-Hant' : 'en';
  document.querySelectorAll('[data-i18n]').forEach(el => { const a = el.getAttribute('data-i18n-args'); el.textContent = t(el.getAttribute('data-i18n'), a ? JSON.parse(a) : undefined); });
  if (RC.sim && RC.station) updateHUD();
};
RC.setLang = l => { RC.lang = l === 'zh' ? 'zh' : 'en'; RC.applyLang(); return RC.lang; };

// ------------------------------------------------------------------ simple orbit camera (Z-up)
class Orbit {
  constructor(cam, dom, view) {
    this.cam = cam; this.dom = dom;
    this.t = new THREE.Vector3(); this.set(view[0], view[1], view[2], view[3]);
    let drag = null;
    const down = e => { drag = { x: e.clientX, y: e.clientY, b: e.button, shift: e.shiftKey }; dom.setPointerCapture(e.pointerId); };
    const move = e => {
      if (!drag) return; const dx = e.clientX - drag.x, dy = e.clientY - drag.y; drag.x = e.clientX; drag.y = e.clientY;
      if (drag.b === 2 || drag.shift) {
        const right = new THREE.Vector3().subVectors(cam.position, this.t).cross(new THREE.Vector3(0, 0, 1)).normalize();
        this.t.addScaledVector(right, dx * 0.0025 * this.r).addScaledVector(new THREE.Vector3(0, 0, 1), dy * 0.0025 * this.r);
      } else { this.az -= dx * 0.006; this.el = clamp(this.el + dy * 0.006, 0.05, 1.5); }
      this.update();
    };
    const up = () => { drag = null; };
    dom.addEventListener('pointerdown', down); dom.addEventListener('pointermove', move);
    dom.addEventListener('pointerup', up); dom.addEventListener('pointercancel', up);
    dom.addEventListener('contextmenu', e => e.preventDefault());
    dom.addEventListener('wheel', e => { e.preventDefault(); this.r = clamp(this.r * (1 + Math.sign(e.deltaY) * 0.1), 3, RC.orbitMax || 40); this.update(); }, { passive: false });
  }
  set(az, el, r, t) { this.az = az; this.el = el; this.r = r; if (t) this.t.set(t[0], t[1], t[2]); this.update(); }
  update() {
    const c = Math.cos(this.el);
    this.cam.position.set(this.t.x + this.r * c * Math.cos(this.az), this.t.y + this.r * c * Math.sin(this.az), this.t.z + this.r * Math.sin(this.el));
    this.cam.up.set(0, 0, 1); this.cam.lookAt(this.t);
  }
}

// ------------------------------------------------------------------ scene: renderer, lights, floor, 1 m grid aligned to the robot base
function buildScene(canvas, st) {
  const renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
  renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
  renderer.shadowMap.enabled = true; renderer.shadowMap.type = THREE.PCFSoftShadowMap;
  renderer.outputColorSpace = THREE.SRGBColorSpace;
  renderer.toneMapping = THREE.ACESFilmicToneMapping; renderer.toneMappingExposure = 1.0;
  RC.renderer = renderer;
  const scene = RC.scene = new THREE.Scene();
  scene.background = new THREE.Color(0x1b1f24); scene.fog = new THREE.Fog(0x1b1f24, 28, 60);
  RC.camera = new THREE.PerspectiveCamera(42, 1, 0.1, 200);
  RC.orbit = new Orbit(RC.camera, canvas, st.views[Object.keys(st.views)[0]]);
  scene.add(new THREE.HemisphereLight(0xdfe8ff, 0x2a3a2a, 0.75));
  const sun = new THREE.DirectionalLight(0xffffff, 2.2); sun.position.set(6, -7, 12); sun.castShadow = true;
  const f = st.floor || { w: 14, l: 12, x: 0, y: 0 };
  const half = Math.max(11, Math.max(f.w, f.l) / 2 + 1);                                   // shadow frustum covers the whole cell floor
  sun.shadow.mapSize.set(half > 11 ? 4096 : 2048, half > 11 ? 4096 : 2048); Object.assign(sun.shadow.camera, { left: -half, right: half, top: half, bottom: -half, near: 1, far: 40 + half * 2 }); sun.shadow.bias = -0.0005;
  sun.position.set(f.x + 6, f.y - 7, 12); sun.target.position.set(f.x, f.y, 0); scene.add(sun.target); scene.add(sun);
  const fill = new THREE.DirectionalLight(0x9fb8ff, 0.5); fill.position.set(-8, 6, 6); scene.add(fill);
  RC.orbitMax = Math.max(40, Math.max(f.w, f.l) * 1.3);
  const G = Math.max(20, Math.ceil(Math.max(Math.abs(f.x) + f.w / 2, Math.abs(f.y) + f.l / 2) / 5) * 5 + 5);   // grid / outer floor half-size (m)
  const floorOuter = box(2 * G, 2 * G, 0.02, mat(0x7d8489, { roughness: 0.9 }), 0, 0, -0.011); floorOuter.castShadow = false;
  if (G > 20) scene.fog = new THREE.Fog(0x1b1f24, G * 1.4, G * 3);
  const floorCell = box(f.w, f.l, 0.012, mat(0x2f7a3a, { roughness: 0.85 }), f.x, f.y, -0.006); floorCell.castShadow = false;
  const gridLines = (step, filter, color, opacity) => {
    const v = [];
    for (let i = -G; i <= G; i += step) { if (!filter(i)) continue; v.push(i, -G, 0, i, G, 0, -G, i, 0, G, i, 0); }
    const g = new THREE.BufferGeometry(); g.setAttribute('position', new THREE.Float32BufferAttribute(v, 3));
    const l = new THREE.LineSegments(g, new THREE.LineBasicMaterial({ color, transparent: true, opacity })); l.position.z = 0.002; scene.add(l); return l;
  };
  gridLines(1, i => i % 5 !== 0, 0x4a555d, 0.9); gridLines(5, i => i !== 0, 0x6b7780, 1.0); gridLines(1, i => i === 0, 0x9aa8b3, 1.0);
  RC.floorLine = (x, y, w, l) => { const m = box(w, l, 0.004, mat(0xf2c200, { roughness: 0.8 }), x, y, 0.004); m.castShadow = false; return m; };
}

// ------------------------------------------------------------------ simulation state + motion primitives
/* One robot context per robot. The primary robot's context IS RC.sim (S.tcp, S.v, S.reach, S.step …), so single-robot
   stations keep the original API; extra robots (station.robots[1..]) get their own context objects in RC.ctxs and use
   RC.motion.bind(ctx) for move / dwell / waitUntil / setStep. A context may define hold() → '' | reason: while it returns
   a reason the robot decelerates to standstill and every primitive waits (protective stop of that robot's zone). */
const S = RC.sim = {
  t: 0, speed: 1, running: true, mode: null,
  tcp: { x: 0, y: 0, z: 0, yaw: 0 }, v: 0, reach: true,
  step: 0, status: 'Booting', sub: '', subArgs: null, waiting: '',
  lastCycle: 0, cycleStart: 0,
  interlocks: {}, safety: {}, toggles: {},
};
RC.ctxs = [S];
const held = ctx => (ctx.hold ? ctx.hold() : '') || '';
/* move the TCP: straight line, or an arc around the robot base ('arc': cylindrical interpolation, by default through the
   side away from the A1 wrap, i.e. angles normalised 0…2π from +X; 'dir': +1 / −1 forces a counter-clockwise / clockwise
   swing; optional outward 'bulge'); 'via': [{x, y, z?, yaw?}, …] = polyline through these points with one speed profile (blended
   corners, like a robot zone); 'surface' = z of the surface below → creep at vSlow inside the slow zone and brake before it */
function* moveCtx(ctx, target, opts) {
  opts = opts || {}; const M = ctx.motion || RC.station.motion; const B = ctx.base || { x: 0, y: 0 };
  const a = { x: ctx.tcp.x, y: ctx.tcp.y, z: ctx.tcp.z, yaw: ctx.tcp.yaw, tilt: ctx.tcp.tilt || 0 };
  const b = { x: target.x, y: target.y, z: target.z, yaw: target.yaw !== undefined ? target.yaw : a.yaw, tilt: target.tilt !== undefined ? target.tilt : a.tilt };
  /* tilted tools (serial6 'tilt' mode): tcp.tilt is interpolated along the move; a tilt change counts as |Δtilt| · tiltR of path
     (opts.tiltR or motion.tiltR, default yawR) so a pure reorientation of a long torch takes time too */
  const tR = opts.tiltR !== undefined ? opts.tiltR : (M.tiltR !== undefined ? M.tiltR : (M.yawR || 0));
  const arc = !!opts.arc && !opts.via;
  let L, r0, r1, th0, th1, bulge = 0, poly = null;
  if (opts.via && opts.via.length) {                                        // polyline through via points, blended (no stop at the corners)
    /* via yaw (optional): the tool yaw is reached at that corner; unset corners get the yaw interpolated along the path.
       Each segment counts max(length, |Δyaw| · yawR), so a corner-to-corner turn of a long tool is speed-limited too */
    poly = [Object.assign({}, a)].concat(opts.via.map(p => ({ x: p.x, y: p.y, z: p.z !== undefined ? p.z : a.z, yaw: p.yaw })), [Object.assign({}, b)]); poly[0].g = 0;
    for (let i = 1; i < poly.length; i++) poly[i].g = poly[i - 1].g + Math.hypot(poly[i].x - poly[i - 1].x, poly[i].y - poly[i - 1].y, poly[i].z - poly[i - 1].z);
    for (let i = 1, k = 0; i < poly.length; i++) if (poly[i].yaw !== undefined) {           // fill unset yaws between defined corners
      for (let j = k + 1; j < i; j++) poly[j].yaw = lerp(poly[k].yaw, poly[i].yaw, (poly[j].g - poly[k].g) / Math.max(1e-9, poly[i].g - poly[k].g));
      k = i;
    }
    const yR = opts.yawR !== undefined ? opts.yawR : (M.yawR || 0); poly[0].c = 0;
    for (let i = 1; i < poly.length; i++) poly[i].c = poly[i - 1].c + Math.max(poly[i].g - poly[i - 1].g, Math.abs(poly[i].yaw - poly[i - 1].yaw) * yR);
    L = poly[poly.length - 1].c;
  } else if (arc) {
    r0 = Math.hypot(a.x - B.x, a.y - B.y); r1 = Math.hypot(b.x - B.x, b.y - B.y); th0 = Math.atan2(a.y - B.y, a.x - B.x); th1 = Math.atan2(b.y - B.y, b.x - B.x);
    if (opts.dir) { let d = th1 - th0; if (opts.dir > 0 && d < 0) d += 2 * Math.PI; if (opts.dir < 0 && d > 0) d -= 2 * Math.PI; th1 = th0 + d; }
    else {
      if (th0 < -0.01) th0 += 2 * Math.PI; else if (th0 < 0) th0 = 0;
      if (th1 < -0.01) th1 += 2 * Math.PI; else if (th1 < 0) th1 = 0;
    }
    bulge = opts.bulge !== undefined ? opts.bulge : (M.swingBulge || 0);
    L = Math.hypot((r0 + r1) / 2 * (th1 - th0), r1 - r0, b.z - a.z) + 2 * bulge;
  } else L = Math.hypot(b.x - a.x, b.y - a.y, b.z - a.z);
  /* long tools: 'yawR' (or motion.yawR) = tool radius — a yaw change counts as |Δyaw| · yawR of path, so the tool tip
     never turns faster than vMax (the TCP then slows down); 'vMax' caps this move's speed (e.g. a fork withdrawal) */
  if (!poly) L = Math.max(L, Math.abs(b.yaw - a.yaw) * (opts.yawR !== undefined ? opts.yawR : (M.yawR || 0)));
  L = Math.max(L, Math.abs(b.tilt - a.tilt) * tR);
  if (L < 1e-4) { if (b.tilt !== a.tilt) ctx.tcp.tilt = b.tilt; return; }
  let s = 0; ctx.v = 0;
  while (s < L) {
    const dt = yield;
    let vT = opts.vMax !== undefined ? Math.min(M.vMax, opts.vMax) : M.vMax;
    if (opts.surface !== undefined) {
      const above = ctx.tcp.z - opts.surface;
      if (above < M.slowZone + 1e-4) vT = M.vSlow;
      else if (b.z < a.z) vT = Math.min(vT, M.vSlow + Math.sqrt(2 * M.accel * (above - M.slowZone)));
    }
    vT = Math.min(vT, Math.sqrt(2 * M.accel * Math.max(L - s, 0)) + 0.02);
    const h = held(ctx); if (h) vT = 0; ctx.waiting = h;                    // protective stop: brake to standstill and hold
    ctx.v += clamp(vT - ctx.v, -M.accel * dt, M.accel * dt);
    s = Math.min(L, s + ctx.v * dt);
    const u = s / L;
    if (poly) {
      let i = 1; while (i < poly.length - 1 && poly[i].c < s) i++;
      const p0 = poly[i - 1], p1 = poly[i], w = (s - p0.c) / Math.max(1e-9, p1.c - p0.c);
      ctx.tcp.x = lerp(p0.x, p1.x, w); ctx.tcp.y = lerp(p0.y, p1.y, w); ctx.tcp.z = lerp(p0.z, p1.z, w); ctx.tcp.yaw = lerp(p0.yaw, p1.yaw, w);
      if (b.tilt !== a.tilt) ctx.tcp.tilt = lerp(a.tilt, b.tilt, s / L);
      continue;
    }
    if (arc) { const r = lerp(r0, r1, u) + bulge * Math.sin(Math.PI * u), th = lerp(th0, th1, u); ctx.tcp.x = B.x + r * Math.cos(th); ctx.tcp.y = B.y + r * Math.sin(th); }
    else { ctx.tcp.x = lerp(a.x, b.x, u); ctx.tcp.y = lerp(a.y, b.y, u); }
    ctx.tcp.z = lerp(a.z, b.z, u); ctx.tcp.yaw = lerp(a.yaw, b.yaw, u);
    if (b.tilt !== a.tilt) ctx.tcp.tilt = lerp(a.tilt, b.tilt, u);
  }
  ctx.v = 0;
}
function* dwellCtx(ctx, sec) { let x = 0; while (x < sec) { const dt = yield; const h = held(ctx); ctx.waiting = h; if (!h) x += dt; } }
function* waitUntilCtx(ctx, fn, why) { for (;;) { const h = held(ctx); if (!h && fn()) break; ctx.waiting = h || why; yield; } ctx.waiting = ''; }
function setStepCtx(ctx, n, status, sub, args) { ctx.step = n; ctx.status = status; ctx.sub = sub || ''; ctx.subArgs = args || null; }
function* move(target, opts) { yield* moveCtx(S, target, opts); }
function* dwell(sec) { yield* dwellCtx(S, sec); }
function* waitUntil(fn, why) { yield* waitUntilCtx(S, fn, why); }
function setStep(n, status, sub, args) { setStepCtx(S, n, status, sub, args); }
const bindMotion = ctx => ({
  move: function* (t, o) { yield* moveCtx(ctx, t, o); }, dwell: function* (s) { yield* dwellCtx(ctx, s); },
  waitUntil: function* (f, w) { yield* waitUntilCtx(ctx, f, w); }, setStep: (n, st, sub, a) => setStepCtx(ctx, n, st, sub, a),
});
RC.motion = { move, dwell, waitUntil, setStep, bind: bindMotion };
RC.ctxById = id => RC.ctxs.find(c => c.id === id) || S;

// ------------------------------------------------------------------ robot pose from the TCP target (robot module does IK + link transforms)
function updateRobotPose() {
  const st = RC.station;
  for (const ctx of RC.ctxs) {
    const rb = ctx.robot; const B = ctx.base || { x: 0, y: 0 };
    const p = { x: ctx.tcp.x - B.x, y: ctx.tcp.y - B.y, z: ctx.tcp.z - (ctx.pedestal || 0) };
    const ik = rb.ik(p, ctx.tcp); ctx.reach = ik.reach;                                   // side-mounted tools need the yaw for the wrist centre
    let q1n = ik.q1; if (q1n < -0.01) q1n += 2 * Math.PI; else if (q1n < 0) q1n = 0;   // continuous over the +Y swing
    ctx.q = { q1: ik.q1, q2: ik.q2, q3: ik.q3, q4: ctx.tcp.yaw + Math.PI - q1n,          // A4 cancels A1 → tool yaw = tcp.yaw (world)
              yaw: ctx.tcp.yaw, roll: ctx.tcp.roll || 0, tilt: ctx.tcp.tilt || 0 };          // side tool: axis along yaw, A6 = roll · tilt tool: leaned towards yaw
    if (ctx === S) RC.q = ctx.q;
    rb.apply(ctx.q);
    if (st.onPose) st.onPose(ctx.tcp, ctx);
  }
}

/* stations with a timeline replay a run when the user seeks back: Math.random is seeded per run (RC.seed), a new seed on every
   reset / mode change, the same seed on a seek (other stations keep the native generator, e.g. for stall sweeps) */
const nativeRandom = Math.random;
function seedRandom(seed) {
  let a = seed >>> 0;
  Math.random = () => { a = (a + 0x6D2B79F5) >>> 0; let x = a; x = Math.imul(x ^ (x >>> 15), x | 1); x ^= x + Math.imul(x ^ (x >>> 7), x | 61); return ((x ^ (x >>> 14)) >>> 0) / 4294967296; };
}
function resetSim(seed) {
  if (RC.station.timeline) { RC.seed = seed !== undefined ? seed : Math.floor(nativeRandom() * 4294967296); seedRandom(RC.seed); }
  for (const ctx of RC.ctxs) Object.assign(ctx, { lastCycle: 0, cycleStart: 0, waiting: '', v: 0 });
  S.t = 0;
  RC.station.reset();
  RC.programs = RC.ctxs.map(ctx => { const g = (ctx.def && ctx.def.program || RC.station.program)(); g.next(); return g; });
  RC.program = RC.programs[0];
}
function stepSim(dt) {
  S.t += dt;
  for (const g of RC.programs) g.next(dt);
  RC.station.update(dt);
  updateRobotPose();
  if (RC.animateSafety) RC.animateSafety(dt);
}

// ------------------------------------------------------------------ HUD (ids come from station.json; values from station.hud())
const $ = id => document.getElementById(id);
function setText(id, txt) { const el = $(id); if (el && el.textContent !== txt) el.textContent = txt; }
function updateHUD() {
  const st = RC.station;
  for (let i = 1; i <= (st.ui.steps || []).length; i++) { const el = $('step-' + i); if (el) el.classList.toggle('active', RC.ctxs.some(c => c.step === i)); }
  RC.ctxs.forEach((c, i) => {                                                              // ids carry '-<robot id>' for the 2nd… robot
    const x = i === 0 ? '' : '-' + c.id; const M = c.motion || st.motion;
    setText('hud-status' + x, t(c.status, c.subArgs));                                     // status may use the same {args} as the sub-line
    setText('hud-sub' + x, c.waiting ? t('WAIT: ') + t(c.waiting) : t(c.sub, c.subArgs));
    const sub = $('hud-sub' + x); if (sub) sub.classList.toggle('wait', !!c.waiting);
    setText('m-cycle' + x, c.lastCycle ? c.lastCycle.toFixed(1) + ' s' : '—');
    setText('m-tcpv' + x, Math.round(c.v * 1000) + ' mm/s');
    setText('h-vmax' + x, t('Robot TCP speed (max {v} mm/s · {s} mm/s in {z} mm zone)', { v: Math.round(M.vMax * 1000), s: Math.round(M.vSlow * 1000), z: Math.round(M.slowZone * 1000) }));
    const bar = $('tcp-bar' + x); if (bar) { bar.style.width = (c.v / M.vMax * 100).toFixed(1) + '%'; bar.classList.toggle('slow', c.v > 0 && c.v <= M.vSlow + 0.01); }
    if (c.q && c.robot.jointText) setText('m-q' + x, c.robot.jointText(c.q));
    setText('m-tcp' + x, `X ${Math.round(c.tcp.x * 1000)}  Y ${Math.round(c.tcp.y * 1000)}  Z ${Math.round(c.tcp.z * 1000)} mm`);
  });
  setText('m-time', S.t.toFixed(1) + ' s');
  const vals = st.hud ? st.hud() : {};
  for (const id in vals) { const v = vals[id]; setText(id, Array.isArray(v) ? t(v[0], v[1]) : (typeof v === 'string' ? t(v) : String(v))); }
  const il = (id, ok, okText, badText) => { const el = $(id); if (!el) return; el.classList.toggle('ok', ok); el.classList.toggle('bad', !ok); setText(id, t(ok ? okText : badText)); };
  for (const d of st.ui.interlocks || []) {
    const reach = d.key === 'reach' || d.id === 'il-reach';                                // {key:'reach', robot:'<id>'} = that robot's IK reach
    il(d.id, reach ? RC.ctxById(d.robot).reach : !!S.interlocks[d.key], d.ok, d.bad);
  }
  if (RC.safetyHUD) RC.safetyHUD(setText);
  if (RC.elecHUD) RC.elecHUD(setText);
  if (RC.timeline) RC.timeline.update();
}

// ------------------------------------------------------------------ timeline bar (stations with timeline() → { lens: [per phase], pos, phase (1-based), from, done, label, unit })
/* One segment per process step (width = its length, e.g. planned hours) and a slider for the position. Clicking a segment jumps to
   the start of that phase, releasing the slider jumps to that position: forward = fast simulation from now, backward = reset
   with the same seed and fast simulation up to the target (nothing is drawn meanwhile). Built by the engine inside the view's
   host element, so both shells get it; fitPanels places it between the Process and Metrics panels. */
function buildTimeline(st, canvas) {
  const T = RC.timeline = { seeking: false, dragging: false, lang: null };
  const el = T.el = document.createElement('section'); el.setAttribute('data-rc-timeline', '');
  Object.assign(el.style, { position: 'absolute', top: '14px', left: '360px', right: '330px', background: 'rgba(20,24,29,.86)', border: '1px solid #2c343c',
    borderRadius: '10px', padding: '8px 12px 6px', display: 'flex', flexDirection: 'column', gap: '6px', color: '#e8ecef', font: '12px/1.4 "Segoe UI", system-ui, sans-serif', zIndex: 2 });
  const head = document.createElement('div'); Object.assign(head.style, { display: 'flex', justifyContent: 'space-between', gap: '8px', alignItems: 'baseline' });
  const title = document.createElement('span'); Object.assign(title.style, { fontSize: '11px', textTransform: 'uppercase', letterSpacing: '.8px', color: '#9aa4ad' });
  const now = document.createElement('span'); Object.assign(now.style, { fontVariantNumeric: 'tabular-nums', color: '#e8ecef', whiteSpace: 'nowrap', overflow: 'hidden', textOverflow: 'ellipsis' });
  head.append(title, now);
  const bar = document.createElement('div'); Object.assign(bar.style, { display: 'flex', gap: '2px' });
  const range = document.createElement('input'); range.type = 'range'; range.min = 0; range.step = 'any';
  Object.assign(range.style, { width: '100%', margin: '0', accentColor: '#ff6a13', cursor: 'pointer' });
  el.append(head, bar, range); canvas.parentElement.appendChild(el);
  const lens = st.timeline().lens; const starts = lens.map((_, i) => lens.slice(0, i).reduce((a, b) => a + b, 0));
  range.max = starts[starts.length - 1] + lens[lens.length - 1];
  T.segs = lens.map((len, i) => {
    const b = document.createElement('button'); b.type = 'button'; b.textContent = String(i + 1);
    Object.assign(b.style, { flex: len + ' 1 0', minWidth: '16px', height: '26px', padding: '0', border: '0', borderRadius: '4px', font: 'inherit', fontSize: '11px',
      fontWeight: '600', cursor: 'pointer', color: '#e8ecef', background: '#2c343c' });
    b.addEventListener('click', () => RC.seek(starts[i])); bar.appendChild(b); return b;
  });
  range.addEventListener('input', () => { T.dragging = true; T.update(); });
  range.addEventListener('change', () => { T.dragging = false; RC.seek(parseFloat(range.value)); });
  const names = st.ui.steps || [];
  T.update = () => {
    const tl = st.timeline(); const from = tl.from || 0;
    if (T.lang !== RC.lang) {                                             // labels follow the language switch
      T.lang = RC.lang; title.textContent = t('Timeline'); range.setAttribute('aria-label', t('Project time'));
      T.segs.forEach((b, i) => { const lbl = t('Go to phase {n}: {name}', { n: i + 1, name: t(names[i] || '') }) + ' · ' + lens[i] + ' ' + (tl.unit || 'h'); b.dataset.lbl = lbl; });
    }
    const p = T.dragging ? parseFloat(range.value) : tl.pos;
    T.segs.forEach((b, i) => {
      const end = starts[i] + lens[i], skipped = end <= from + 1e-6;
      const cur = (p >= starts[i] - 1e-6 && p < end - 1e-6) || (i === lens.length - 1 && p >= end - 1e-6);
      const lbl = skipped ? t('not simulated in this mode') : b.dataset.lbl;
      if (b.title !== lbl) { b.title = lbl; b.setAttribute('aria-label', lbl); }
      b.disabled = skipped; b.style.cursor = skipped ? 'not-allowed' : 'pointer'; b.style.opacity = skipped ? '0.35' : '1';
      b.style.background = cur ? '#ff6a13' : p >= end ? 'rgba(255,106,19,.35)' : '#2c343c';
    });
    if (!T.dragging) range.value = tl.pos;
    now.textContent = (T.seeking ? t('Seeking…') + ' · ' : '') + (tl.label ? t(tl.label) : tl.pos.toFixed(1));
  };
  RC.seek = pos => {
    if (T.seeking) return; T.seeking = true; T.update();
    setTimeout(() => {                                                     // let the "Seeking…" label paint first (no rAF: works in a hidden tab)
      try {
        pos = Math.max(pos, st.timeline().from || 0);
        if (pos < st.timeline().pos - 1e-6) resetSim(RC.seed);              // backward: replay the same run
        let ph = 1; while (ph < starts.length && starts[ph] <= pos + 1e-6) ph++;   // the phase that holds pos (a phase start belongs to that phase)
        for (let n = 0; n < 400000; n++) { const tl = st.timeline(); if ((tl.pos >= pos - 1e-6 && (tl.phase === undefined || tl.phase >= ph)) || tl.done) break; stepSim(0.02); }
      } finally { T.seeking = false; updateHUD(); if (RC.fitPanels) RC.fitPanels(); }
    }, 30);
  };
  T.update();
}

// ------------------------------------------------------------------ station registration + bootstrap
RC.defineStation = def => {
  RC.station = def; RC.K = def.K; if (def.i18n) RC.i18n.add(def.i18n); if (def.ui) RC.station.ui = def.ui;
  if (def.robots && def.robots.length) {                    // multi-robot station: robots[0] is the primary (RC.robot / RC.sim)
    const r0 = def.robots[0]; if (!def.robot) def.robot = r0; if (!def.motion) def.motion = r0.motion; if (!def.program) def.program = r0.program;
  }
  return def;
};

RC.start = function (opts) {
  const st = RC.station; if (!st) throw new Error('no station defined');
  if (opts.ui) st.ui = opts.ui;
  const canvas = opts.canvas;
  buildScene(canvas, st);
  /* one context per robot: the primary shares RC.sim, the others are fresh objects (RC.ctxs); each robot module is built once
     at its base position (metres, robot 1 = origin) with its own mesh (opts.glbs[def.glb], falling back to opts.glbB64) */
  const defs = st.robots && st.robots.length ? st.robots : [Object.assign({ id: 'r1', motion: st.motion, program: st.program }, st.robot)];
  RC.ctxs = defs.map((d, i) => {
    const ctx = i === 0 ? S : { tcp: { x: 0, y: 0, z: 0, yaw: 0 }, v: 0, reach: true, q: null, step: 0, status: '', sub: '', subArgs: null, waiting: '', lastCycle: 0, cycleStart: 0 };
    ctx.id = d.id || ('r' + (i + 1)); ctx.def = d; ctx.motion = d.motion || st.motion; ctx.base = d.base || { x: 0, y: 0 }; ctx.pedestal = d.pedestal || 0;
    /* every robot is its own module instance (RC.robots[type].create): kinematics + limits from this robot's PIM block
       (ROBOT_PARAMS.byRobot[<glb key | id>], falling back to the station-wide block for single-robot stations) */
    const fac = RC.robots[d.type]; if (!fac) throw new Error('unknown robot type ' + d.type);
    const RP = window.ROBOT_PARAMS || {}; const own = (RP.byRobot && (RP.byRobot[d.glb] || RP.byRobot[ctx.id])) || RP;
    ctx.robot = fac.create ? fac.create(own, ctx.id) : fac;
    const b64 = (d.glb && opts.glbs && opts.glbs[d.glb]) || opts.glbB64;
    ctx.robot.build(RC.b64ToBuf(b64), { pedestal: ctx.pedestal, yaw: d.yaw, color: d.color, x: ctx.base.x, y: ctx.base.y, elbow: d.elbow, a6Raw: d.a6Raw });   // elbow: SCARA arm configuration · a6Raw: serial6 down-mode A6 unwrapped
    return ctx;
  });
  RC.robot = RC.ctxs[0].robot; S.robots = RC.ctxs;
  st.build();
  if (st.safety && RC.buildSafety) RC.buildSafety(st.safety());
  if (st.electrical && RC.buildElectrical) RC.buildElectrical(st.electrical());   // after safety: safety devices are wired too
  resetSim();
  if (st.timeline) buildTimeline(st, canvas);

  RC.setView = name => { const v = st.views[name]; if (v) RC.orbit.set(v[0], v[1], v[2], v[3]); };
  RC.setSpeed = v => { S.speed = v; };
  RC.setMode = m => { S.mode = m; if (st.setMode) st.setMode(m); resetSim(); };
  RC.setNumber = (id, v) => { if (st.setNumber) st.setNumber(id, v); };
  RC.setToggle = (id, on) => { S.toggles[id] = !!on; if (st.setToggle) st.setToggle(id, !!on); return !!on; };
  RC.reset = () => resetSim();
  RC.toggleRun = () => { S.running = !S.running; return S.running; };
  RC.step = stepSim;

  function resize() {
    const w = canvas.clientWidth || canvas.parentElement.clientWidth, h = canvas.clientHeight || canvas.parentElement.clientHeight;
    if (!w || !h) return;
    RC.renderer.setSize(w, h, false); RC.camera.aspect = w / h; RC.camera.updateProjectionMatrix();
  }
  RC.applyLang();
  window.addEventListener('resize', resize); resize();
  if (window.ResizeObserver) new ResizeObserver(resize).observe(canvas.parentElement);
  /* HUD panels never run off the screen: header stays, body scrolls. Max heights go to CSS variables on <html> (a re-rendering
     shell keeps them): Controls ≤ half the view, Process / Metrics end above the Controls panel where they share its columns. */
  function fitPanels() {
    const P = {}; document.querySelectorAll('[data-rc-panel]').forEach(e => { P[e.dataset.rcPanel] = e; });
    const box = canvas.parentElement.parentElement; if (!box) return;
    const b = box.getBoundingClientRect(), top = Math.max(b.top, 0), bottom = Math.min(b.bottom, window.innerHeight), root = document.documentElement.style;
    const set = (k, v) => { v += 'px'; if (root.getPropertyValue(k) !== v) root.setProperty(k, v); };
    const shown = e => e && e.getClientRects().length > 0;
    set('--rc-max-ctrl', Math.max(120, Math.round((bottom - top) * 0.5)));
    const c = shown(P.ctrl) ? P.ctrl.getBoundingClientRect() : null;
    const tl = layoutTimeline(box.getBoundingClientRect(), P, shown);
    for (const id of ['hdr', 'metrics']) {
      if (!shown(P[id])) continue; const q = P[id].getBoundingClientRect();
      let end = bottom - (id === 'metrics' ? 40 : 14);                          // metrics clears the orbit legend
      if (c && c.left < q.right && c.right > q.left) end = Math.min(end, c.top - 10);
      if (tl && tl.left < q.right && tl.right > q.left && tl.top > q.top) end = Math.min(end, tl.top - 10);   // narrow layout: timeline above Controls
      set('--rc-max-' + id, Math.max(120, Math.round(end - q.top)));
    }
  }
  /* timeline between the Process and Metrics panels (below the dock of hidden panels); too narrow → full width above Controls */
  function layoutTimeline(b, P, shown) {
    const el = RC.timeline && RC.timeline.el; if (!el) return null; const r = e => e.getBoundingClientRect();
    let left = 14, right = 14, top = 14;
    if (shown(P.hdr)) left = r(P.hdr).right - b.left + 12;
    if (shown(P.metrics)) right = b.right - r(P.metrics).left + 12;
    const dock = document.querySelector('[data-rc-dock]'); if (dock && r(dock).width > 0) top = r(dock).bottom - b.top + 8;
    if (b.width - left - right < 380) { left = 14; right = 14; top = (shown(P.ctrl) ? r(P.ctrl).top - b.top : b.height - 14) - el.offsetHeight - 10; }
    const set = (k, v) => { v = Math.round(v) + 'px'; if (el.style[k] !== v) el.style[k] = v; };
    set('left', left); set('right', right); set('top', top);
    return el.getBoundingClientRect();
  }
  RC.fitPanels = fitPanels;
  window.addEventListener('resize', fitPanels); fitPanels();
  if (window.ResizeObserver) { const ro = new ResizeObserver(() => fitPanels()); ro.observe(canvas.parentElement); document.querySelectorAll('[data-rc-panel]').forEach(e => ro.observe(e)); }
  let last = performance.now(), hudT = 0;
  function frame(now) {
    requestAnimationFrame(frame);
    const dt = Math.min(0.05, (now - last) / 1000); last = now;
    if (S.running) { const total = dt * S.speed; const n = Math.max(1, Math.ceil(total / 0.02)); const h = total / n; for (let i = 0; i < n; i++) stepSim(h); }
    hudT += dt; if (hudT > 0.1) { hudT = 0; updateHUD(); fitPanels(); }   // re-fit: a shell may re-render its panels
    RC.renderer.render(RC.scene, RC.camera);
  }
  resize(); requestAnimationFrame(frame);
};
})();

/* Safety equipment builders, driven by a config object returned by station.safety():
   {
     fence: { x0, x1, y0, y1, h, panel, roof, window },  // cell rectangle (metres) and fence height; panel 'laser' = solid laser-safe sheet
                                                          // panels (IEC 60825-4, drawn translucent so the cell stays visible) with laser
                                                          // viewing windows window: {z0, h} on every panel ≥ 1 m and on door leaves;
                                                          // roof: true closes the enclosure at fence height (frame + sheet)
                                                          // panel 'glass' = laminated safety glass / polycarbonate panels to the floor
                                                          // (ISO 14120 fixed guard of a display enclosure in a public space),
                                                          // brushed-steel posts without caps
     runs: [{ side: 'front'|'top'|'left'|'right', openings: [[a, b], ...] }],   // fenced sides; a side not listed has no fence
     walls: [{ y, x0, x1, openings } | { x, y0, y1, openings }],   // inner fence segments (dividers between zones), same height and look as the perimeter
     doors: [{ side, from, to, clear } | { y | x, from, to, out, clear }],   // interlocked, guard-locked doors on a fenced side or on an inner wall line
                                                          // (inner line: out = ±1 = the side the leaf swings to, default −1); clear = clear
                                                          // height of the opening for a vehicle + load: the frame posts and header rise to
                                                          // clear + 0.06 when that exceeds the fence height (the leaf stays fence height; the
                                                          // gap above it is like the fence top) — a header at fence height stops a tall stack
     curtains: [{ side | y | x, from, to, z0, h, model }],   // multi-beam light curtains along a side or an inner wall line (from/to along it)
     scanners: { r, model, units: [{ x, y, a, clip }], opacity, visible },   // laser scanners on the fence, a = facing angle into the cell,
                                                          // clip = {x0,x1,y0,y1} rectangle of the unit's zone (default: the cell) — fields stop at a divider
     hoods: [{ x0, x1, y0, y1, zBase, slot, axis, top }],   // fixed-guard hoods (transparent) up to fence height (or top), optional outfeed slot;
                                                          // axis 'y' (default): side walls along Y, slot wall at slot.y · axis 'x': walls along X, slot wall at slot.x
     stations: [{ x, y }],                               // E-stop / reset stations
     state: () => ({ curtainMuted, scannerSwitched, doorOpen, doorText, curtainText, scannerText })
                                                          // curtainMuted: one boolean for all curtains or one per curtain in the order of `curtains`
                                                          // live status from the station: doorOpen 0..1 animates the leaf (a number for
                                                          // all doors, or one number per door in the order of `doors`),
                                                          // *Text (key or [key, args]) override the Safety HUD rows
   }
   Sides: front = y0 (run along x), top = y1, left = x0 (run along y), right = x1. */
(function () {
'use strict';
const RC = window.RobotCell; const { mat, box, cylZ, clamp } = RC.h;

RC.buildSafety = function (cfg) {
  const F = cfg.fence; const H = F.h; RC.safetyCfg = cfg;
  const glass = F.panel === 'glass';
  const postM = glass ? mat(0xc3c8cd, { roughness: 0.3, metalness: 0.8 }) : mat(0xf2c200, { roughness: 0.5, metalness: 0.2 });
  const meshM = new THREE.MeshStandardMaterial({ color: 0x3a434b, transparent: true, opacity: 0.32, side: THREE.DoubleSide, roughness: 0.8, depthWrite: false });
  const railM = mat(0x22272c, { metalness: 0.4 });
  const laser = F.panel === 'laser', win = laser && F.window;
  const panelM = laser ? new THREE.MeshStandardMaterial({ color: 0x55606a, transparent: true, opacity: 0.55, side: THREE.DoubleSide, roughness: 0.6, metalness: 0.3, depthWrite: false })
               : glass ? new THREE.MeshStandardMaterial({ color: 0xcfe3ee, transparent: true, opacity: 0.16, side: THREE.DoubleSide, roughness: 0.05, metalness: 0.1, depthWrite: false }) : meshM;
  const solid = laser || glass;                                                               // sheet panels reach the floor (no gap under them)
  const winM = new THREE.MeshStandardMaterial({ color: 0x1f8a4c, transparent: true, opacity: 0.35, side: THREE.DoubleSide, roughness: 0.1, metalness: 0.1, depthWrite: false });
  const grp = RC.safety = new THREE.Group(); RC.scene.add(grp);
  /* panel between a and b along a line (horiz: along x at y = fixed), z from z0 to z1, with a laser window band when it is long enough */
  const panel = (horiz, fixed, a, b, z0, z1, parent, off) => {
    const L = b - a, c = (a + b) / 2, o = off || 0, put = (l, zz0, zz1, m) => { const h = zz1 - zz0; if (h <= 0.001) return;
      const pm = horiz ? box(l, 0.02, h, m, c - o, fixed, zz0 + h / 2, parent) : box(0.02, l, h, m, fixed, c - o, zz0 + h / 2, parent); pm.castShadow = false; };
    if (!win || L < 0.6 || win.z0 < z0 || win.z0 + win.h > z1) { put(L, z0, z1, panelM); return; }
    put(L, z0, win.z0, panelM); put(L, win.z0 + win.h, z1, panelM);
    const wl = L - 0.3, w0 = win.z0, w1 = win.z0 + win.h;                                 // window inset 0.15 m from the posts
    const pm = horiz ? box(wl, 0.024, w1 - w0, winM, c - o, fixed, (w0 + w1) / 2, parent) : box(0.024, wl, w1 - w0, winM, fixed, c - o, (w0 + w1) / 2, parent); pm.castShadow = false;
    for (const s of [-1, 1]) { const q = horiz ? box(0.15, 0.02, w1 - w0, panelM, c - o + s * (L / 2 - 0.075), fixed, (w0 + w1) / 2, parent) : box(0.02, 0.15, w1 - w0, panelM, fixed, c - o + s * (L / 2 - 0.075), (w0 + w1) / 2, parent); q.castShadow = false; }
  };
  const post = (x, y) => box(0.07, 0.07, H, postM, x, y, H / 2, grp);
  const cap = (x, y) => glass ? null : box(0.09, 0.09, 0.03, mat(0xd8121c), x, y, H + 0.015, grp);
  const sideGeom = side => side === 'front' ? { horiz: true, fixed: F.y0, a0: F.x0, a1: F.x1 } : side === 'top' ? { horiz: true, fixed: F.y1, a0: F.x0, a1: F.x1 }
                        : side === 'left' ? { horiz: false, fixed: F.x0, a0: F.y0, a1: F.y1 } : { horiz: false, fixed: F.x1, a0: F.y0, a1: F.y1 };
  const lineGeom = o => o.side ? sideGeom(o.side) : o.y !== undefined ? { horiz: true, fixed: o.y, inner: true } : { horiz: false, fixed: o.x, inner: true };   // a side, or an inner wall line
  const run = (horiz, fixed, a0, a1, openings) => {
    const segs = []; let cur = a0;
    for (const [a, b] of openings.slice().sort((p, q) => p[0] - q[0])) { if (a > cur) segs.push([cur, a]); cur = Math.max(cur, b); }
    if (cur < a1) segs.push([cur, a1]);
    for (const [a, b] of segs) {
      const L = b - a, c = (a + b) / 2;
      if (solid) panel(horiz, fixed, a, b, 0.0, H, grp);                                   // laser / glass panels reach the floor (no gap under them)
      else if (horiz) { const m = box(L, 0.02, H - 0.15, meshM, c, fixed, H / 2 + 0.05, grp); m.castShadow = false; }
      else { const m = box(0.02, L, H - 0.15, meshM, fixed, c, H / 2 + 0.05, grp); m.castShadow = false; }
      if (horiz) { box(L, 0.04, 0.04, railM, c, fixed, H - 0.05, grp); box(L, 0.04, 0.04, railM, c, fixed, 0.12, grp); }
      else { box(0.04, L, 0.04, railM, fixed, c, H - 0.05, grp); box(0.04, L, 0.04, railM, fixed, c, 0.12, grp); }
      const n = Math.max(1, Math.round(L / 2));
      for (let i = 0; i <= n; i++) { const p = a + L * i / n; if (horiz) { post(p, fixed); cap(p, fixed); } else { post(fixed, p); cap(fixed, p); } }
    }
  };
  for (const r of cfg.runs || []) {
    const g = sideGeom(r.side);
    const openings = (r.openings || []).concat((cfg.doors || []).filter(d => d.side === r.side).map(d => [d.from, d.to]));
    run(g.horiz, g.fixed, g.a0, g.a1, openings);
  }
  const onLine = (d, w) => !d.side && (w.y !== undefined ? d.y === w.y && d.from >= w.x0 - 1e-6 && d.to <= w.x1 + 1e-6 : d.x === w.x && d.from >= w.y0 - 1e-6 && d.to <= w.y1 + 1e-6);
  for (const w of cfg.walls || []) {
    const openings = (w.openings || []).concat((cfg.doors || []).filter(d => onLine(d, w)).map(d => [d.from, d.to]));
    if (w.y !== undefined) run(true, w.y, w.x0, w.x1, openings); else run(false, w.x, w.y0, w.y1, openings);
  }

  // doors: frame, hinged mesh panel, outside handle, interlock switch with LED (guard locking)
  RC.doors = []; RC.doorLeaves = [];
  for (const d of cfg.doors || []) {
    const g = lineGeom(d); const dw = d.to - d.from; const c = (d.from + d.to) / 2;
    const pt = (a) => g.horiz ? [a, g.fixed] : [g.fixed, a];
    const Hd = Math.max(H, (d.clear || 0) + 0.06);                                            // frame height (clear opening for a tall load)
    post(...pt(d.from)); post(...pt(d.to)); if (Hd > H) for (const a of [d.from, d.to]) { const [x, y] = pt(a); box(0.07, 0.07, Hd - H, postM, x, y, (H + Hd) / 2, grp); if (!glass) box(0.09, 0.09, 0.03, mat(0xd8121c), x, y, Hd + 0.015, grp); } else { cap(...pt(d.from)); cap(...pt(d.to)); }
    if (g.horiz) box(dw, 0.06, 0.06, postM, c, g.fixed, Hd - 0.03, grp); else box(0.06, dw, 0.06, postM, g.fixed, c, Hd - 0.03, grp);
    const door = new THREE.Group(); door.position.set(...pt(d.from), 0); grp.add(door);
    const out = g.inner ? (d.out || -1) : g.fixed === F.y0 || g.fixed === F.x0 ? -1 : 1;   // outside direction along the normal
    RC.doorLeaves.push({ leaf: door, sign: g.horiz ? out : -out });                     // leaf hinged at d.from, swings outward
    if (g.horiz) {
      if (solid) panel(true, 0, 0.06, dw - 0.06, 0.1, H - 0.1, door); else { const dm = box(dw - 0.12, 0.02, H - 0.25, meshM, dw / 2, 0, H / 2 - 0.02, door); dm.castShadow = false; }
      for (const z of [0.15, H - 0.2]) box(dw - 0.12, 0.05, 0.05, postM, dw / 2, 0, z, door);
      for (const x of [0.06, dw - 0.06]) box(0.05, 0.05, H - 0.25, postM, x, 0, H / 2 - 0.02, door);
      box(0.03, 0.06, 0.14, railM, dw - 0.18, out * 0.05, 1.05, door);
      box(0.06, 0.08, 0.16, mat(0x22272c), d.to - 0.05, g.fixed, 1.25, grp);
      RC.doors.push(box(0.03, 0.03, 0.03, mat(0x22ff66, { emissive: 0x22ff66, emissiveIntensity: 1.5 }), d.to - 0.05, g.fixed + out * 0.05, 1.3, grp));
    } else {
      if (solid) panel(false, 0, 0.06, dw - 0.06, 0.1, H - 0.1, door); else { const dm = box(0.02, dw - 0.12, H - 0.25, meshM, 0, dw / 2, H / 2 - 0.02, door); dm.castShadow = false; }
      for (const z of [0.15, H - 0.2]) box(0.05, dw - 0.12, 0.05, postM, 0, dw / 2, z, door);
      for (const y of [0.06, dw - 0.06]) box(0.05, 0.05, H - 0.25, postM, 0, y, H / 2 - 0.02, door);
      box(0.06, 0.03, 0.14, railM, out * 0.05, dw - 0.18, 1.05, door);
      box(0.08, 0.06, 0.16, mat(0x22272c), g.fixed, d.to - 0.05, 1.25, grp);
      RC.doors.push(box(0.03, 0.03, 0.03, mat(0x22ff66, { emissive: 0x22ff66, emissiveIntensity: 1.5 }), g.fixed + out * 0.05, d.to - 0.05, 1.3, grp));
    }
  }

  // roof of a closed (laser / glass) enclosure: sheet at fence height on a frame of cross beams every ≤ 1.5 m
  if (F.roof) {
    const w = F.x1 - F.x0, l = F.y1 - F.y0, cx = (F.x0 + F.x1) / 2, cy = (F.y0 + F.y1) / 2;
    const rm = new THREE.MeshStandardMaterial({ color: glass ? 0xcfe3ee : 0x55606a, transparent: true, opacity: laser ? 0.16 : glass ? 0.12 : 0.22, side: THREE.DoubleSide, roughness: glass ? 0.05 : 0.6, depthWrite: false });
    const roof = box(w, l, 0.015, rm, cx, cy, H + 0.01, grp); roof.castShadow = false; RC.roof = roof;
    const n = Math.max(1, Math.ceil(w / 1.5)); for (let i = 0; i <= n; i++) box(0.05, l, 0.05, railM, F.x0 + w * i / n, cy, H - 0.02, grp);
    box(w, 0.05, 0.05, railM, cx, F.y0, H - 0.02, grp); box(w, 0.05, 0.05, railM, cx, F.y1, H - 0.02, grp);
  }

  // light curtains: emitter/receiver posts, translucent plane, visible beams, floor marking
  RC.curtains = [];
  const cm = mat(0x22272c, { metalness: 0.5 });
  for (const c of cfg.curtains || []) {
    const g = lineGeom(c); const z0 = c.z0 !== undefined ? c.z0 : 0.3; const ch = c.h || 1.7; const len = c.to - c.from;
    const stripeM = mat(0x2f8fff, { emissive: 0x2f8fff, emissiveIntensity: 0.9 });
    const planeM = new THREE.MeshBasicMaterial({ color: 0x2f8fff, transparent: true, opacity: 0.16, side: THREE.DoubleSide, depthWrite: false });
    const beamM = new THREE.LineBasicMaterial({ color: 0x8fc4ff, transparent: true, opacity: 0.8 });
    const pt = a => g.horiz ? [a, g.fixed] : [g.fixed, a];
    for (const a of [c.from, c.to]) {
      box(0.3, 0.3, 0.02, cm, ...pt(a), 0.01, grp); box(0.08, 0.08, H, cm, ...pt(a), H / 2, grp);
      const off = a === c.from ? 0.05 : -0.05;
      if (g.horiz) box(0.02, 0.03, ch, stripeM, a + off, g.fixed, z0 + ch / 2, grp); else box(0.03, 0.02, ch, stripeM, g.fixed, a + off, z0 + ch / 2, grp);
    }
    const plane = new THREE.Mesh(new THREE.PlaneGeometry(len - 0.1, ch), planeM);
    if (g.horiz) { plane.rotation.x = Math.PI / 2; plane.position.set((c.from + c.to) / 2, g.fixed, z0 + ch / 2); }
    else { plane.rotation.set(Math.PI / 2, Math.PI / 2, 0); plane.position.set(g.fixed, (c.from + c.to) / 2, z0 + ch / 2); }   // width along Y, height along Z
    grp.add(plane);
    for (let i = 0; i < 8; i++) {
      const z = z0 + 0.05 + i * (ch - 0.1) / 7;
      const p0 = g.horiz ? new THREE.Vector3(c.from + 0.05, g.fixed, z) : new THREE.Vector3(g.fixed, c.from + 0.05, z);
      const p1 = g.horiz ? new THREE.Vector3(c.to - 0.05, g.fixed, z) : new THREE.Vector3(g.fixed, c.to - 0.05, z);
      grp.add(new THREE.Line(new THREE.BufferGeometry().setFromPoints([p0, p1]), beamM));
    }
    if (g.horiz) box(len, 0.06, 0.004, mat(0xf2c200), (c.from + c.to) / 2, g.fixed, 0.004, grp); else box(0.06, len, 0.004, mat(0xf2c200), g.fixed, (c.from + c.to) / 2, 0.004, grp);
    RC.curtains.push({ stripeM, planeM, beamM });
  }

  // laser scanners on the fence (300 mm), 270° FOV, protective field clipped to the cell rectangle
  RC.scanners = []; const sc = cfg.scanners;
  if (sc && sc.units && sc.units.length) {
    const R = sc.r || 4.0; const op = sc.opacity !== undefined ? sc.opacity : 0.12;
    RC.scanFieldM = new THREE.MeshBasicMaterial({ color: 0xff8a1f, transparent: true, opacity: op, side: THREE.DoubleSide, depthWrite: false });
    RC.scanEdgeM = new THREE.LineBasicMaterial({ color: 0xff8a1f, transparent: true, opacity: Math.min(1, op * 5) });
    RC.scanSweepM = new THREE.MeshBasicMaterial({ color: 0xffb060, transparent: true, opacity: Math.min(1, op * 3), side: THREE.DoubleSide, depthWrite: false });
    const clipRect = (poly, Rc) => {
      const edges = [[Rc.x0, 'x', 1], [Rc.x1, 'x', -1], [Rc.y0, 'y', 1], [Rc.y1, 'y', -1]];
      for (const [v, k, sgn] of edges) {
        const inside = pt => (pt[k] - v) * sgn >= 0; const out = [];
        for (let i = 0; i < poly.length; i++) {
          const a = poly[i], b = poly[(i + 1) % poly.length]; const ia = inside(a), ib = inside(b);
          const cross = () => { const u = (v - a[k]) / (b[k] - a[k]); return { x: a.x + (b.x - a.x) * u, y: a.y + (b.y - a.y) * u }; };
          if (ia && ib) out.push(b); else if (ia && !ib) out.push(cross()); else if (!ia && ib) { out.push(cross()); out.push(b); }
        }
        poly = out; if (!poly.length) break;
      }
      return poly;
    };
    const scanGrp = RC.scanGroup = new THREE.Group(); grp.add(scanGrp);
    sc.units.forEach((u, i) => {
      const Rc = u.clip || F;                                            // the unit's zone rectangle
      const hx = u.x === Rc.x0 ? 0.1 : u.x === Rc.x1 ? -0.1 : 0, hy = u.y === Rc.y0 ? 0.1 : u.y === Rc.y1 ? -0.1 : 0;
      const cx = u.x + hx, cy = u.y + hy;
      box(0.12, 0.12, 0.12, mat(0x1c1f22, { roughness: 0.4 }), cx, cy, 0.30, grp);
      box(0.04, 0.04, 0.24, mat(0x22272c), cx - hx * 0.5, cy - hy * 0.5, 0.12, grp);
      cylZ(0.05, 0.05, mat(0xf2c200, { roughness: 0.4 }), cx, cy, 0.385, grp, 24);
      const pts = [{ x: cx, y: cy }]; const n = 48;
      for (let j = 0; j <= n; j++) { const th = u.a - 3 * Math.PI / 4 + (3 * Math.PI / 2) * j / n; pts.push({ x: cx + R * Math.cos(th), y: cy + R * Math.sin(th) }); }
      const poly = clipRect(pts, Rc); if (!poly.length) return;
      const field = new THREE.Mesh(new THREE.ShapeGeometry(new THREE.Shape(poly.map(q => new THREE.Vector2(q.x, q.y)))), RC.scanFieldM); field.position.z = 0.006 + i * 0.0005; scanGrp.add(field);
      scanGrp.add(new THREE.LineLoop(new THREE.BufferGeometry().setFromPoints(poly.map(q => new THREE.Vector3(q.x, q.y, 0.009))), RC.scanEdgeM));
      const sweep = new THREE.Mesh(new THREE.CircleGeometry(R, 12, 0, 0.05), RC.scanSweepM); sweep.position.set(cx, cy, 0.0085); scanGrp.add(sweep);
      RC.scanners.push({ sweep, centre: u.a, t: i * 0.9, cx, cy, R, rect: Rc });
    });
    RC.scanGroup.visible = sc.visible !== false;
    RC.setScanOpacity = v => { v = clamp(v, 0, 1); RC.scanFieldM.opacity = v; RC.scanEdgeM.opacity = Math.min(1, v * 5); RC.scanSweepM.opacity = Math.min(1, v * 3); };
    RC.setScanVisible = on => { RC.scanGroup.visible = !!on; return RC.scanGroup.visible; };
  }

  // fixed-guard hoods (e.g. over a machine that crosses the fence line): transparent sides + roof, optional outfeed slot wall
  const glassM = new THREE.MeshStandardMaterial({ color: 0x9fc4d8, transparent: true, opacity: 0.22, side: THREE.DoubleSide, roughness: 0.2, depthWrite: false });
  for (const h of cfg.hoods || []) {
    const T = h.top || H;                                                                      // roof height: fence height, or top for a tall load (opt-in)
    const w = h.x1 - h.x0, l = h.y1 - h.y0, cx = (h.x0 + h.x1) / 2, cy = (h.y0 + h.y1) / 2, zb = h.zBase, hh = T - zb;
    if (h.axis === 'x') {
      for (const y of [h.y0, h.y1]) { const m = box(w, 0.02, hh, glassM, cx, y, zb + hh / 2, grp); m.castShadow = false; }
      if (h.slot) { const ew = box(0.02, l, hh - h.slot.h, glassM, h.slot.x, cy, zb + h.slot.h + (hh - h.slot.h) / 2, grp); ew.castShadow = false; }
      box(0.06, l + 0.06, 0.06, postM, h.x0, cy, T, grp); box(0.06, l + 0.06, 0.06, postM, h.x1, cy, T, grp);
    } else {
      for (const x of [h.x0, h.x1]) { const m = box(0.02, l, hh, glassM, x, cy, zb + hh / 2, grp); m.castShadow = false; }
      if (h.slot) { const ew = box(w, 0.02, hh - h.slot.h, glassM, cx, h.slot.y, zb + h.slot.h + (hh - h.slot.h) / 2, grp); ew.castShadow = false; }
      box(w + 0.06, 0.06, 0.06, postM, cx, h.y0, T, grp); box(w + 0.06, 0.06, 0.06, postM, cx, h.y1, T, grp);
    }
    const roof = box(w, l, 0.02, glassM, cx, cy, T, grp); roof.castShadow = false;
    for (const x of [h.x0, h.x1]) for (const y of [h.y0, h.y1]) box(0.06, 0.06, hh, postM, x, y, zb + hh / 2, grp);
  }

  // E-stop / reset stations (yellow box, red mushroom, blue reset button)
  for (const s of cfg.stations || []) {
    const g = new THREE.Group(); g.position.set(s.x, s.y, 0); g.rotation.z = s.rz || 0; grp.add(g);
    box(0.05, 0.05, 1.1, mat(0x22272c), 0, 0, 0.55, g);
    box(0.08, 0.16, 0.22, mat(0xf2c200, { roughness: 0.5 }), 0, 0, 1.2, g);
    const m = cylZ(0.03, 0.03, mat(0xd8121c, { roughness: 0.4 }), -0.055, 0.04, 1.24, g, 16); m.rotation.y = Math.PI / 2;
    const r = cylZ(0.018, 0.02, mat(0x2f6fd6, { roughness: 0.4 }), -0.05, -0.04, 1.24, g, 16); r.rotation.y = Math.PI / 2;
  }
};

RC.animateSafety = function (dt) {
  const cfg = RC.safetyCfg; if (!cfg) return; const F = cfg.fence;
  for (const sc of RC.scanners || []) {
    sc.t += dt;
    const th = sc.centre - Math.PI / 2 + Math.PI * (0.5 + 0.5 * Math.sin(sc.t * 1.6));
    sc.sweep.rotation.z = th;
    const dx = Math.cos(th), dy = Math.sin(th); let L = sc.R; const Rc = sc.rect || F;
    if (dx > 1e-6) L = Math.min(L, (Rc.x1 - sc.cx) / dx); if (dx < -1e-6) L = Math.min(L, (Rc.x0 - sc.cx) / dx);
    if (dy > 1e-6) L = Math.min(L, (Rc.y1 - sc.cy) / dy); if (dy < -1e-6) L = Math.min(L, (Rc.y0 - sc.cy) / dy);
    const k = Math.max(0.02, L / sc.R); sc.sweep.scale.set(k, k, 1);
  }
  const st = cfg.state ? cfg.state() : {};
  const opens = (Array.isArray(st.doorOpen) ? st.doorOpen : [st.doorOpen || 0]).map(v => clamp(v || 0, 0, 1));   // 0 = closed · locked, 1 = fully open
  const doorOpen = Math.max(0, ...opens);
  const muted = Array.isArray(st.curtainMuted) ? st.curtainMuted.map(v => !!v) : (RC.curtains || []).map(() => !!st.curtainMuted);   // one flag per curtain, or one for all
  RC.sim.safety = Object.assign({}, st, { doorClosed: doorOpen === 0, curtainMuted: muted.some(v => v), scannerSwitched: !!st.scannerSwitched });
  const openOf = i => opens[Math.min(i, opens.length - 1)];
  (RC.doorLeaves || []).forEach((d, i) => { d.leaf.rotation.z = d.sign * openOf(i) * 1.75; });
  (RC.doors || []).forEach((led, i) => { const c = openOf(i) > 0 ? 0xff4d4d : 0x22ff66; led.material.color.set(c); led.material.emissive.set(c); });
  (RC.curtains || []).forEach((cu, i) => {
    const m = muted[Math.min(i, muted.length - 1)]; const c = m ? 0x8a9098 : 0x2f8fff;
    cu.planeM.color.set(c); cu.stripeM.color.set(c); cu.stripeM.emissive.set(c); cu.beamM.color.set(m ? 0xb0b6bc : 0x8fc4ff);
  });
};

RC.safetyHUD = function (setText) {
  const cfg = RC.safetyCfg; if (!cfg) return; const t = RC.t; const sf = RC.sim.safety || {};
  const txt = (v, dflt) => Array.isArray(v) ? t(v[0], v[1]) : t(v || dflt);        // stations may override the texts via state()
  setText('m-fence', txt(sf.fenceText, 'perimeter closed'));
  setText('m-door', txt(sf.doorText, sf.doorClosed === false ? 'open' : 'closed · locked'));
  setText('m-curtain', txt(sf.curtainText, sf.curtainMuted ? 'muted (vehicle passing)' : 'clear'));
  setText('m-scanner', txt(sf.scannerText, sf.scannerSwitched ? 'field switched (vehicle docking)' : 'field clear'));
};
})();

/* Electrical layout: main control panel → sub-cabinets / robot controllers → junction boxes → field devices, every
   connection routed on cable trays / floor trunks, with service-clearance and network checks. Config from station.electrical():
   {
     main: { id, x, y, w, d, h, face, label },           // MAIN CONTROL PANEL (MCP): incoming supply + main disconnect, line PLC,
                                                          // safety PLC, power distribution, HMI — exactly one per station
     subs: [{ id, x, y, w, d, h, face, label, feed }],    // sub-cabinets (remote I/O, drives, machine panels), fed from 'feed' (default MCP)
     controllers: [{ id, robot | robots: [], x, y, w, d, h, face, label, cables }],   // robot controllers: power + fieldbus + safety from the MCP,
                                                          // manipulator cables to robot(s) (ctx id); cables = standard lengths (m)
     jbs: [{ id, x, y, z, to, post, ports }],            // junction boxes (8-port M12 by default); devices → JB (cordsets) → JB 'to' cabinet
     devices: [{ id, x, y, z, kind, jb, mount }],        // field devices (motor, sensor, valve, camera, …); jb = explicit JB, else nearest.
                                                         // mount {x, y} | [{x, y}, …]: the post / mast / wall the device's bracket hangs on
                                                         // (a list = along an arm, then a beam, to the post) — the cable runs along the
                                                         // bracket at device height (each leg x, then y) and down / up at the last point,
                                                         // never straight down through the work space under a camera / motor arm.
                                                          // Safety devices (curtains, scanners, door switches, E-stops) are added from station.safety()
     trays: [{ id, kind, z, w, pts: [[x, y], …], maxSpan, host, ladder }],   // perforated tray (U section, lipped rails, 3 m sections;
                                                          // ladder: true = ladder tray, spans up to 6 m over a lane / door). kind:
                                                          // 'overhead' (z ≥ 2.2 m) on floor-standing strut uprights with cross-arms, placed
                                                          //   every ≤ 2.0 m where the floor is free, max span 2.5 m (maxSpan, 6 for a ladder tray);
                                                          // 'under' (hung under its host equipment frame, host = keep-out name);
                                                          // 'floor' (cable bridge with ramps; never across a flow path or lane).
                                                          // Tray ends meeting at one xy join; different levels get a vertical tray on a strut.
     keepouts: [{ x0, x1, y0, y1, name, h, flow, under, deck, vehicle, clearZ }],   // equipment footprints (h = top incl. product,
                                                          // deck = conveying surface: frame ducts 50 mm below it, else at 0.3 m;
                                                          // frame: 'sides' = only the two long-side ducts, for a floor area a cart /
                                                          // pallet / vehicle drives into along its long axis),
                                                          // flow = material-flow path (conveyor, machine, pallet station): no floor bridge,
                                                          // no support, no cordset across it (under: cordsets may pass under it through the
                                                          // frame); vehicle = traffic lane (clearZ = mast / load height + 0.2); trays over
                                                          // any keep-out at z ≥ clearZ or h + 0.3
     maxDrop: 10, attach: 0.6, attachRobot: 1.2, serviceDepth: 1.0, jbPorts: 8   // cordset limit · cabinet footprint / JB → tray gap · robot base →
                                                          // floor-trunk end · clear depth in front of a door · JB ports
   }
   face = direction the cabinet door opens to ('+x' | '-x' | '+y' | '-y'); the service area is in front of it:
   depth = max(serviceDepth, door leaf + 0.5 m escape width) (IEC 60364-7-729 / EN 61439 practice), width = cabinet width.
   RC.elec = { nodes, cables, issues, trays, stats, robotClearance(), setVisible(on) } — verification reads it. */
(function () {
'use strict';
const RC = window.RobotCell; const { mat, box, cylZ } = RC.h;
const FACE = { '+x': [1, 0], '-x': [-1, 0], '+y': [0, 1], '-y': [0, -1] };
const CAB = { main: { w: 1.2, d: 0.6, h: 2.0 }, sub: { w: 0.8, d: 0.4, h: 1.2 }, ctrl: { w: 0.73, d: 0.72, h: 0.97 } };
const DIA = { bus: 0.008, safety: 0.010, power24: 0.012, feeder: 0.022, motor: 0.028, smb: 0.013, customer: 0.020, jbPower: 0.016 };
const ROBOT_CABLES = [3, 7, 15, 22, 30];                 // ABB manipulator cable lengths (m)
const TRAY_DEPTH = { overhead: 0.06, under: 0.06, floor: 0.05 }; const FILL_MAX = 0.4;

RC.buildElectrical = function (cfg) {
  const t = RC.t; const issues = []; const warn = (key, args) => issues.push([key, args || {}]);
  const grp = new THREE.Group(); RC.scene.add(grp); const ov = new THREE.Group(); grp.add(ov);   // equipment · planning overlay
  const E = RC.elec = { cfg, issues, nodes: {}, cables: [], trays: [], stats: {}, overlay: ov };
  const attachMax = cfg.attach || 0.6, attachRobot = cfg.attachRobot || 1.2, maxDrop = cfg.maxDrop || 10, svc = cfg.serviceDepth || 1.0;
  const saf = RC.safetyCfg || {}; const F = saf.fence;
  const keep = (cfg.keepouts || []).concat((saf.doors || []).map((d, i) => {        // fence-door swing areas are keep-outs too
    const L = d.to - d.from, s = d.side;
    if (s === 'front') return { name: 'door swing', x0: d.from, x1: d.to, y0: F.y0 - L, y1: F.y0 };
    if (s === 'top') return { name: 'door swing', x0: d.from, x1: d.to, y0: F.y1, y1: F.y1 + L };
    if (s === 'left') return { name: 'door swing', x0: F.x0 - L, x1: F.x0, y0: d.from, y1: d.to };
    if (s === 'right') return { name: 'door swing', x0: F.x1, x1: F.x1 + L, y0: d.from, y1: d.to };
    const o = d.out || -1;
    return d.y !== undefined ? { name: 'door swing', x0: d.from, x1: d.to, y0: Math.min(d.y, d.y + o * L), y1: Math.max(d.y, d.y + o * L) }
                             : { name: 'door swing', x0: Math.min(d.x, d.x + o * L), x1: Math.max(d.x, d.x + o * L), y0: d.from, y1: d.to };
  }));

  // ---------------------------------------------------------------- materials
  const ral = mat(0xd7d9d4, { roughness: 0.55 }), plinth = mat(0x2b3138, { metalness: 0.4 }), dark = mat(0x22272c);
  const galv = mat(0xa9b0b6, { metalness: 0.7, roughness: 0.35 }), trunkM = mat(0x3a4046, { roughness: 0.7 }), yel = mat(0xf2c200, { roughness: 0.5 });
  const cableBundle = mat(0x15181b, { roughness: 0.8 }), dropM = new THREE.LineBasicMaterial({ color: 0x5ec8ff, transparent: true, opacity: 0.85 });

  // ---------------------------------------------------------------- labels (sprites, overlay)
  const label = (text, x, y, z, color) => {
    const c = document.createElement('canvas'); c.width = 256; c.height = 64; const g = c.getContext('2d');
    g.fillStyle = 'rgba(20,24,29,.85)'; g.fillRect(0, 0, 256, 64); g.strokeStyle = color || '#ff6a13'; g.lineWidth = 4; g.strokeRect(2, 2, 252, 60);
    g.fillStyle = '#e8ecef'; g.font = 'bold 34px Segoe UI, sans-serif'; g.textAlign = 'center'; g.textBaseline = 'middle'; g.fillText(text, 128, 34);
    const tex = new THREE.CanvasTexture(c); if (THREE.SRGBColorSpace) tex.colorSpace = THREE.SRGBColorSpace;
    const s = new THREE.Sprite(new THREE.SpriteMaterial({ map: tex, depthTest: false, transparent: true })); s.scale.set(0.8, 0.2, 1); s.position.set(x, y, z); s.renderOrder = 10; ov.add(s); return s;
  };

  // ---------------------------------------------------------------- cabinets (RAL 7035, plinth, doors facing 'face')
  const cabs = [];
  const cabinet = (c, role) => {
    const D = CAB[role]; const w = c.w || D.w, d = c.d || D.d, h = c.h || D.h; const [fx, fy] = FACE[c.face || '-y'];
    const alongX = fx === 0;                                           // door on a y-face → width along x
    const sx = alongX ? w : d, sy = alongX ? d : w;
    const g = new THREE.Group(); grp.add(g);
    box(sx, sy, 0.1, plinth, c.x, c.y, 0.05, g); box(sx, sy, h - 0.1, ral, c.x, c.y, 0.1 + (h - 0.1) / 2, g);
    const fxp = c.x + fx * (sx / 2 + 0.005), fyp = c.y + fy * (sy / 2 + 0.005);
    const leaves = w > 0.85 ? 2 : 1; const leaf = w / leaves;
    for (let i = 1; i < leaves; i++) { const o = -w / 2 + i * leaf; box(alongX ? 0.01 : 0.012, alongX ? 0.012 : 0.01, h - 0.2, dark, alongX ? c.x + o : fxp, alongX ? fyp : c.y + o, 0.1 + (h - 0.1) / 2, g); }
    for (let i = 0; i < leaves; i++) {                                 // handles
      const o = -w / 2 + (i + 1) * leaf - 0.06; box(alongX ? 0.03 : 0.03, alongX ? 0.03 : 0.03, 0.18, dark, alongX ? c.x + o : fxp + fx * 0.015, alongX ? fyp + fy * 0.015 : c.y + o, 1.1, g);
    }
    if (role === 'main') {                                             // main disconnect (red/yellow rotary) + HMI
      const px = alongX ? c.x - w / 2 + 0.2 : fxp + fx * 0.02, py = alongX ? fyp + fy * 0.02 : c.y - w / 2 + 0.2;
      box(0.12, 0.12, 0.12, yel, px, py, 1.45, g); box(0.05, 0.05, 0.16, mat(0xd8121c), px + fx * 0.06, py + fy * 0.06, 1.45, g);
      const hx = alongX ? c.x + 0.1 : fxp + fx * 0.02, hy = alongX ? fyp + fy * 0.02 : c.y + 0.1;
      box(alongX ? 0.38 : 0.04, alongX ? 0.04 : 0.38, 0.28, dark, hx, hy, 1.5, g);
      box(alongX ? 0.32 : 0.045, alongX ? 0.045 : 0.32, 0.22, mat(0x2f8fff, { emissive: 0x1f5faf, emissiveIntensity: 0.6 }), hx, hy, 1.5, g);
    }
    const lamp = cylZ(0.035, 0.1, mat(0x22ff66, { emissive: 0x22ff66, emissiveIntensity: 1.2 }), c.x, c.y, h + 0.05, g, 12);   // status lamp (stations may drive it: RC.elec.nodes[id].lamp)
    const node = { id: c.id, role, x: c.x, y: c.y, z: h, w, d, h, sx, sy, face: c.face || '-y', label: c.label || '', robots: c.robots || (c.robot ? [c.robot] : []), feed: c.feed, cables: c.cables, insideOk: c.insideOk, entry: h, lamp };
    E.nodes[c.id] = node; cabs.push(node);
    label(c.id, c.x, c.y, h + 0.45, role === 'main' ? '#ff3b3b' : role === 'ctrl' ? '#ff6a13' : '#f2c200');
    // service area: clear depth in front of the door(s) + door swing arc
    const depth = Math.max(svc, leaf + 0.5);
    const r = fx ? { x0: fx > 0 ? c.x + sx / 2 : c.x - sx / 2 - depth, x1: fx > 0 ? c.x + sx / 2 + depth : c.x - sx / 2, y0: c.y - sy / 2, y1: c.y + sy / 2 }
                 : { x0: c.x - sx / 2, x1: c.x + sx / 2, y0: fy > 0 ? c.y + sy / 2 : c.y - sy / 2 - depth, y1: fy > 0 ? c.y + sy / 2 + depth : c.y - sy / 2 };
    node.service = r; node.depth = depth; node.leaf = leaf;
    return node;
  };
  const M = cfg.main; if (!M) warn('No main control panel'); else cabinet(M, 'main');
  for (const c of cfg.subs || []) cabinet(c, 'sub');
  for (const c of cfg.controllers || []) cabinet(c, 'ctrl');

  // ---------------------------------------------------------------- fence lines, flow paths (used by the cordset, tray and support rules)
  const fenceLines = [];
  if (F) {
    const sideLine = { front: [F.x0, F.x1, F.y0, F.y0], top: [F.x0, F.x1, F.y1, F.y1], left: [F.x0, F.x0, F.y0, F.y1], right: [F.x1, F.x1, F.y0, F.y1] };
    for (const r of saf.runs || []) { const [x0, x1, y0, y1] = sideLine[r.side]; fenceLines.push({ x0: x0 - 0.05, x1: x1 + 0.05, y0: y0 - 0.05, y1: y1 + 0.05, name: 'fence' }); }
    for (const w of saf.walls || []) fenceLines.push(w.y !== undefined ? { x0: w.x0, x1: w.x1, y0: w.y - 0.05, y1: w.y + 0.05, name: 'wall' } : { x0: w.x - 0.05, x1: w.x + 0.05, y0: w.y0, y1: w.y1, name: 'wall' });
  }
  const inR = (x, y, k, m) => { m = m || 0; return x > k.x0 - m && x < k.x1 + m && y > k.y0 - m && y < k.y1 + m; };
  const flows = keep.filter(k => k.flow || k.vehicle);                  // material-flow paths (conveyors, machines, pallet / vehicle lanes)
  const needZ = k => k.vehicle ? (k.clearZ || 2.6) : (k.clearZ || (k.h !== undefined ? k.h + 0.3 : 0));   // clear height over a keep-out

  // ---------------------------------------------------------------- junction boxes + field devices
  for (const j of cfg.jbs || []) {
    const z = j.z !== undefined ? j.z : 1.0;
    box(0.3, 0.3, 0.2, ral, j.x, j.y, z, grp); box(0.31, 0.31, 0.02, dark, j.x, j.y, z - 0.1, grp);
    E.nodes[j.id] = { id: j.id, role: 'jb', x: j.x, y: j.y, z, entry: z + 0.1, to: j.to || (M && M.id), devices: [], ports: j.ports || cfg.jbPorts || 8, post: !!j.post };
    label(j.id, j.x, j.y, z + 0.4, '#5ec8ff');
  }
  const jbs = Object.values(E.nodes).filter(n => n.role === 'jb');
  const devices = (cfg.devices || []).map(d => Object.assign({ z: 0.9, kind: 'sensor' }, d));
  const mpath = d => d.mount ? [{ x: d.x, y: d.y }].concat(Array.isArray(d.mount) ? d.mount : [d.mount]) : null;
  const mp = d => { const P = mpath(d); return P ? { x: P[P.length - 1].x, y: P[P.length - 1].y, z: d.z } : d; };   // where the cable leaves the bracket
  const armL = d => { const P = mpath(d); let L = 0; if (P) for (let i = 1; i < P.length; i++) L += Math.abs(P[i].x - P[i - 1].x) + Math.abs(P[i].y - P[i - 1].y); return L; };
  (saf.curtains || []).forEach((c, i) => {                              // safety devices from the safety config
    const pt = a => c.side ? (c.side === 'front' ? [a, F.y0] : c.side === 'top' ? [a, F.y1] : c.side === 'left' ? [F.x0, a] : [F.x1, a]) : c.y !== undefined ? [a, c.y] : [c.x, a];
    [[c.from, 'emitter'], [c.to, 'receiver']].forEach(([a, k]) => { const [x, y] = pt(a); devices.push({ id: `LC${i + 1}-${k[0].toUpperCase()}`, kind: 'safety', x, y, z: 1.0 }); });
  });
  ((saf.scanners && saf.scanners.units) || []).forEach((u, i) => devices.push({ id: 'SC-' + (i + 1), kind: 'safety', x: u.x, y: u.y, z: 0.3 }));
  (saf.doors || []).forEach((d, i) => {
    const s = d.side; const a = d.to; const [x, y] = s ? (s === 'front' ? [a, F.y0] : s === 'top' ? [a, F.y1] : s === 'left' ? [F.x0, a] : [F.x1, a]) : d.y !== undefined ? [a, d.y] : [d.x, a];
    devices.push({ id: 'DS-' + (i + 1), kind: 'safety', x, y, z: 1.25 });
  });
  (saf.stations || []).forEach((s, i) => devices.push({ id: 'ES-' + (i + 1), kind: 'safety', x: s.x, y: s.y, z: 1.2 }));

  // ---------------------------------------------------------------- perforated cable tray (U section, perforated base, lipped side rails, 3 m sections)
  const perfTex = (() => {
    const c = document.createElement('canvas'); c.width = c.height = 64; const g = c.getContext('2d');
    g.fillStyle = '#fff'; g.fillRect(0, 0, 64, 64); g.fillStyle = '#000';
    for (const [x, y] of [[16, 16], [48, 48], [48, 16], [16, 48]]) { g.beginPath(); g.ellipse(x, y, 9, 3.5, 0, 0, Math.PI * 2); g.fill(); }
    const tx = new THREE.CanvasTexture(c); tx.wrapS = tx.wrapT = THREE.RepeatWrapping; return tx;
  })();
  const perfMat = (L, w) => { const tx = perfTex.clone(); tx.needsUpdate = true; tx.repeat.set(Math.max(1, Math.round(L / 0.1)), Math.max(1, Math.round(w / 0.1)));
    return mat(0xa9b0b6, { metalness: 0.7, roughness: 0.35, alphaMap: tx, alphaTest: 0.5, side: THREE.DoubleSide }); };
  const splice = mat(0x7d858c, { metalness: 0.6, roughness: 0.4 });
  /* one straight tray run in a local frame: length along +x from 0 to L, base at z 0, width w, side rails dp with 15 mm lips */
  const trayRun = (parent, L, w, dp, ladder) => {
    if (ladder) { dp = 0.1; for (let u = 0.15; u < L; u += 0.3) box(0.03, w, 0.012, galv, u, 0, 0.008, parent); }   // ladder tray: rungs every 300 mm, 100 mm rails
    else box(L, w, 0.004, perfMat(L, w), L / 2, 0, 0.002, parent);
    for (const e of [-1, 1]) { box(L, 0.003, dp, galv, L / 2, e * w / 2, dp / 2, parent); box(L, 0.018, 0.003, galv, L / 2, e * (w / 2 - 0.009), dp, parent); }
    for (let u = 3; u < L - 0.05; u += 3) for (const e of [-1, 1]) box(0.14, 0.006, dp * 0.8, splice, u, e * (w / 2 + 0.004), dp * 0.45, parent);   // section joints
  };
  /* strut support (41 × 41 channel): floor-standing upright with base plate and anchors, cross-arm under the tray, knee braces */
  const supports = [];
  const upright = (x, y, zTop, ang, w, arm) => {
    const g = new THREE.Group(); g.position.set(x, y, 0); g.rotation.z = ang; grp.add(g);
    box(0.2, 0.2, 0.012, galv, 0, 0, 0.006, g); for (const a of [-1, 1]) for (const b of [-1, 1]) cylZ(0.008, 0.03, dark, a * 0.07, b * 0.07, 0.02, g, 6);
    box(0.041, 0.041, zTop - 0.012, galv, 0, 0, (zTop + 0.012) / 2, g);
    if (arm !== false) {
      box(0.06, w + 0.12, 0.041, galv, 0, 0, zTop - 0.0205, g);
      for (const e of [-1, 1]) { const m = box(0.02, 0.3, 0.02, galv, 0, e * 0.1, zTop - 0.13, g); m.rotation.x = e * Math.PI / 4; }
    }
    supports.push({ x, y, z0: 0, z1: zTop, w: arm !== false ? w + 0.12 : 0.2, ang });
  };
  /* where an upright may stand: not in a keep-out / flow path / lane / door swing, a cabinet or its service area, on a fence line,
     at an E-stop station, a JB post or on a floor bridge, and not beside a robot base */
  const cabRects = () => cabs.map(c => ({ x0: c.x - c.sx / 2, x1: c.x + c.sx / 2, y0: c.y - c.sy / 2, y1: c.y + c.sy / 2 })).concat(cabs.map(c => c.service));
  const blocked = (x, y) => keep.some(k => inR(x, y, k, 0.1)) || cabRects().some(k => inR(x, y, k, 0.1)) || fenceLines.some(k => inR(x, y, k, 0.1))
    || (saf.stations || []).some(s => Math.hypot(x - s.x, y - s.y) < 0.3) || jbs.some(j => j.post && Math.hypot(x - j.x, y - j.y) < 0.25)
    || segs.some(s => s.tray.kind === 'floor' && proj(s, x, y).d < s.tray.w / 2 + 0.1) || (RC.ctxs || []).some(c => Math.hypot(x - c.base.x, y - c.base.y) < 0.9);

  // ---------------------------------------------------------------- trays: geometry + crossing rules (drawn after the junctions are known)
  const segs = [];
  const proj = (s, x, y) => { const u = Math.max(0, Math.min(1, ((x - s.x0) * (s.x1 - s.x0) + (y - s.y0) * (s.y1 - s.y0)) / (s.L * s.L))); const px = s.x0 + u * (s.x1 - s.x0), py = s.y0 + u * (s.y1 - s.y0); return { u, px, py, d: Math.hypot(x - px, y - py) }; };
  for (const tr of cfg.trays || []) {
    const kind = tr.kind || 'overhead', w = tr.w || (kind === 'floor' ? 0.3 : 0.2), z = tr.z !== undefined ? tr.z : kind === 'floor' ? 0.03 : 2.4, dp = TRAY_DEPTH[kind];
    const T = { id: tr.id, kind, w, z, depth: tr.ladder ? 0.1 : dp, len: 0, fill: 0, segs: [], maxSpan: tr.maxSpan || (tr.ladder ? 6 : cfg.maxSpan || 2.5), host: tr.host, ladder: !!tr.ladder }; E.trays.push(T);
    if (kind === 'overhead' && z < 2.2) warn('{t}: overhead tray at z {z} m < 2.2 m headroom', { t: tr.id, z: z.toFixed(2) });
    for (let k = 0; k + 1 < tr.pts.length; k++) {
      const [x0, y0] = tr.pts[k], [x1, y1] = tr.pts[k + 1]; const L = Math.hypot(x1 - x0, y1 - y0); if (L < 1e-6) continue;
      const s = { tray: T, x0, y0, x1, y1, L, z, cables: [], area: 0, split: [0, 1], s0: T.len, cuts: [] }; segs.push(s); T.segs.push(s); T.len += L;
      if (kind === 'floor') {                                          // cable bridge: checker-plate cover, yellow chamfered ramps, end ramps
        const sg = new THREE.Group(); sg.position.set(x0, y0, z); sg.rotation.z = Math.atan2(y1 - y0, x1 - x0); grp.add(sg);
        box(L, w, 0.045, trunkM, L / 2, 0, -0.0075, sg); for (const e of [-1, 1]) { const r = box(L, 0.07, 0.012, yel, L / 2, e * (w / 2 + 0.03), -0.024, sg); r.rotation.x = e * -0.35; }
        for (const u of [k === 0 ? 0 : null, k + 2 === tr.pts.length ? L : null]) if (u !== null) { const r = box(0.07, w, 0.012, yel, u + (u ? 0.03 : -0.03), 0, -0.024, sg); r.rotation.y = (u ? 1 : -1) * 0.35; }
      }
      // crossing rules: floor bridges never cross a flow path or lane; trays over one need its clear height
      for (const k2 of keep) {
        if (!segRect(s, k2) || k2.name === T.host) continue;
        if (kind === 'floor') { if (k2.flow || k2.vehicle) warn('{t}: floor bridge crosses {n}', { t: tr.id, n: k2.name || '' }); }
        else if (z < needZ(k2)) warn('{t}: tray at z {z} m over {n} (needs {c} m)', { t: tr.id, z: z.toFixed(2), n: k2.name || '', c: needZ(k2).toFixed(2) });
      }
    }
  }
  for (const a of segs) for (const b of segs) if (a.tray !== b.tray) for (const [x, y] of [[b.x0, b.y0], [b.x1, b.y1]]) { const p = proj(a, x, y); if (p.d < 0.05) a.split.push(p.u); }
  const dirOf = (dx, dy) => Math.abs(dx) >= Math.abs(dy) ? (dx >= 0 ? '+x' : '-x') : (dy >= 0 ? '+y' : '-y');
  const DV = { '+x': [1, 0], '-x': [-1, 0], '+y': [0, 1], '-y': [0, -1] };
  const tees = [];                                                     // branch take-offs from a run (cabinet branch trays)
  const pieces = [];                                                   // footprint of every horizontal tray piece: no two may overlap
  const piece = (n, x0, y0, x1, y1, w, z) => { const hx = Math.abs(x1 - x0) > 1e-6 ? 0 : w / 2, hy = Math.abs(y1 - y0) > 1e-6 ? 0 : w / 2; pieces.push({ n, z, x0: Math.min(x0, x1) - hx, x1: Math.max(x0, x1) + hx, y0: Math.min(y0, y1) - hy, y1: Math.max(y0, y1) + hy }); };
  const attachTo = {}; const risers = [], vtrays = [];                 // node id → [{ seg, u, d, dz }] — one attach per tray (top AND bottom entry); vtrays: vertical trays between levels (as wide as the upper tray)
  const nodeList = Object.values(E.nodes);
  const robots = (RC.ctxs || []).filter(c => cabs.some(k => k.role === 'ctrl' && k.robots.includes(c.id)));
  /* ceiling-mounted robots (delta: robot module `ceiling`) take the manipulator cable up from the mounting face to an overhead tray */
  const endpoints = nodeList.concat(robots.map(c => ({ id: 'robot:' + c.id, role: 'robot', x: c.base.x, y: c.base.y, entry: c.robot.ceiling ? (c.pedestal || 0) : 0.25, ceiling: !!c.robot.ceiling })));
  const conduit = mat(0x2b3036, { roughness: 0.7 });
  for (const n of endpoints) {
    const per = {};
    for (const s of segs) {
      if (n.role === 'robot' && (s.tray.kind === 'floor') === !!n.ceiling) continue;   // manipulator cables leave the base on the floor (ceiling robots: overhead)
      const p = proj(s, n.x, n.y); const edge = n.sx ? Math.hypot(Math.max(0, Math.abs(p.px - n.x) - n.sx / 2), Math.max(0, Math.abs(p.py - n.y) - n.sy / 2)) : p.d;   // cabinet: from its footprint
      const dz = n.sx && s.tray.kind === 'floor' ? 0.1 : Math.abs(s.z - n.entry); const cost = edge + dz * 0.3; const b = per[s.tray.id];
      if (edge <= (n.role === 'robot' ? attachRobot : attachMax) && (!b || cost < b.cost)) per[s.tray.id] = { s, u: p.u, d: edge, dz, cost, px: p.px, py: p.py };
    }
    let list = Object.values(per);
    if (n.role !== 'robot') {                                           // one overhead entry (the nearest run) + at most one bottom entry (floor bridge)
      const best = k => list.filter(A => (A.s.tray.kind === 'floor') === k).sort((p, q) => p.cost - q.cost)[0];
      list = [best(false), best(true)].filter(Boolean);
    }
    if (!list.length) { warn('{n} is not on a cable tray (> {a} m)', { n: n.id.replace('robot:', 'robot '), a: n.role === 'robot' ? attachRobot : attachMax }); if (n.post) upright(n.x, n.y, n.z - 0.1, 0, 0, false); continue; }
    let postTop = n.post ? n.z - 0.1 : 0;
    for (const A of list) {
      A.s.split.push(A.u);
      if (A.s.tray.kind === 'floor' || n.role === 'robot') continue;       // bottom entry / robot base: the bridge ends at it
      const T = A.s.tray, hl = Math.hypot(A.px - n.x, A.py - n.y), a = Math.atan2(n.y - A.py, n.x - A.px), rim = T.w / 2;
      if (n.sx) {                                                          // cabinet: tee out of the run's side → branch tray over the roof → vertical tray into the gland plate
        const bw = 0.15, zb = T.z, out = hl > rim + 0.05;
        if (out) {
          const d = dirOf(n.x - A.px, n.y - A.py), [ux, uy] = DV[d]; tees.push({ x: A.px, y: A.py, z: zb, dir: d });
          const g = new THREE.Group(); g.position.set(A.px + ux * rim, A.py + uy * rim, zb); g.rotation.z = Math.atan2(uy, ux); grp.add(g);
          const bl = Math.max(0.05, Math.abs(ux ? n.x - A.px : n.y - A.py) - rim + bw / 2); trayRun(g, bl, bw, 0.05);
          piece(n.id + ' branch', A.px + ux * rim, A.py + uy * rim, A.px + ux * (rim + bl), A.py + uy * (rim + bl), bw, zb);
        }
        const bx = out ? n.x : A.px, by = out ? n.y : A.py;               // under the run: straight down from the tray bottom
        const v = new THREE.Group(); v.position.set(bx, by, n.h); v.rotation.order = 'ZYX'; v.rotation.set(0, -Math.PI / 2, a); grp.add(v); trayRun(v, Math.max(0.05, zb - n.h - 0.01), bw, 0.05);
        risers.push({ x: bx, y: by, z0: n.h, z1: zb });
      } else {                                                             // JB: flexible conduit up the JB post (a strut to tray level), into the tray over its rail
        const z0 = n.entry, z1 = T.z; cylZ(0.016, Math.max(0.02, z1 + 0.08 - z0), conduit, n.x + 0.06, n.y, (z0 + z1 + 0.08) / 2, grp, 8);
        const hr = hl - rim + 0.05; if (hr > 0.03) { const m = box(hr, 0.032, 0.032, conduit, n.x - Math.cos(a) * hr / 2, n.y - Math.sin(a) * hr / 2, z1 + 0.08, grp); m.rotation.z = a; }
        if (n.post) postTop = Math.max(postTop, z1 - 0.03);
        risers.push({ x: n.x, y: n.y, z0, z1 });
      }
    }
    if (n.post) { upright(n.x, n.y, postTop, 0, 0, false); n.postTop = postTop; }   // JB post: floor-standing strut (carries the JB and its conduit)
    attachTo[n.id] = list;
  }

  // ---------------------------------------------------------------- fittings: where runs of one level meet (elbow / tee / cross) every run stops at
  // the fitting's edge and the fitting's side rails stay open only towards the connected runs — trays never overlap each other
  const lvl = segs.filter(s => s.tray.kind !== 'floor'); const cand = [];
  for (const s of lvl) cand.push({ x: s.x0, y: s.y0, z: s.z }, { x: s.x1, y: s.y1, z: s.z });
  for (let i = 0; i < lvl.length; i++) for (let j = i + 1; j < lvl.length; j++) {
    const A = lvl[i], B = lvl[j]; if (A.tray === B.tray || Math.abs(A.z - B.z) > 0.1) continue;
    const dx1 = A.x1 - A.x0, dy1 = A.y1 - A.y0, dx2 = B.x1 - B.x0, dy2 = B.y1 - B.y0, den = dx1 * dy2 - dy1 * dx2; if (Math.abs(den) < 1e-9) continue;
    const ua = ((B.x0 - A.x0) * dy2 - (B.y0 - A.y0) * dx2) / den, ub = ((B.x0 - A.x0) * dy1 - (B.y0 - A.y0) * dx1) / den;
    if (ua > -1e-6 && ua < 1 + 1e-6 && ub > -1e-6 && ub < 1 + 1e-6) cand.push({ x: A.x0 + ua * dx1, y: A.y0 + ua * dy1, z: A.z });
  }
  for (const t of tees) cand.push(t);
  const fittings = [];
  for (const P of cand) {
    if (fittings.some(f => Math.hypot(f.x - P.x, f.y - P.y) < 0.05 && Math.abs(f.z - P.z) < 0.1)) continue;
    const dirs = new Set(), contrib = []; let w = 0, ladder = false;
    for (const s of lvl) {
      if (Math.abs(s.z - P.z) > 0.1) continue; const q = proj(s, P.x, P.y); if (q.d > 0.05) continue;
      const ux = (s.x1 - s.x0) / s.L, uy = (s.y1 - s.y0) / s.L; w = Math.max(w, s.tray.w); ladder = ladder || s.tray.ladder;
      if (q.u * s.L < 0.05) { dirs.add(dirOf(ux, uy)); contrib.push({ s, u: q.u, a: 0, b: 1 }); }
      else if ((1 - q.u) * s.L < 0.05) { dirs.add(dirOf(-ux, -uy)); contrib.push({ s, u: q.u, a: 1, b: 0 }); }
      else { dirs.add(dirOf(ux, uy)); dirs.add(dirOf(-ux, -uy)); contrib.push({ s, u: q.u, a: 1, b: 1 }); }
    }
    for (const t of tees) if (Math.hypot(t.x - P.x, t.y - P.y) < 0.05 && Math.abs(t.z - P.z) < 0.1) dirs.add(t.dir);
    if (dirs.size < 2) continue;                                       // free end
    const ds = [...dirs]; if (ds.length === 2 && DV[ds[0]][0] === -DV[ds[1]][0] && DV[ds[0]][1] === -DV[ds[1]][1]) continue;   // straight run / inline joint
    fittings.push({ x0: P.x - w / 2, x1: P.x + w / 2, y0: P.y - w / 2, y1: P.y + w / 2, z: P.z, dirs, ladder });
  }
  // junctions closer than a tray width share one (rectangular) fitting; every run is then cut where it enters a fitting
  for (let merged = true; merged;) {
    merged = false;
    for (let i = 0; i < fittings.length && !merged; i++) for (let j = i + 1; j < fittings.length && !merged; j++) {
      const a = fittings[i], b = fittings[j]; if (Math.abs(a.z - b.z) > 0.1) continue;
      if (Math.min(a.x1, b.x1) - Math.max(a.x0, b.x0) > 0.005 && Math.min(a.y1, b.y1) - Math.max(a.y0, b.y0) > 0.005) {
        fittings[i] = { x0: Math.min(a.x0, b.x0), x1: Math.max(a.x1, b.x1), y0: Math.min(a.y0, b.y0), y1: Math.max(a.y1, b.y1), z: a.z, dirs: new Set([...a.dirs, ...b.dirs]), ladder: a.ladder || b.ladder };
        fittings.splice(j, 1); merged = true;
      }
    }
  }
  for (const f of fittings) { f.x = (f.x0 + f.x1) / 2; f.y = (f.y0 + f.y1) / 2; f.w = Math.max(f.x1 - f.x0, f.y1 - f.y0); }
  for (const s of lvl) for (const f of fittings) {
    if (Math.abs(s.z - f.z) > 0.1) continue; const c = clipT(s, f); if (c) s.cuts.push({ u0: c[0], u1: c[1] });
  }
  const fitRail = mat(0xa9b0b6, { metalness: 0.7, roughness: 0.35 });
  for (const f of fittings) {                                           // fitting: plate + rails on the closed sides only
    const dp = f.ladder ? 0.1 : 0.06, W = f.x1 - f.x0, H = f.y1 - f.y0;
    box(W, H, 0.004, perfMat(W, H), f.x, f.y, f.z + 0.002, grp); pieces.push({ n: 'fitting', z: f.z, x0: f.x0, x1: f.x1, y0: f.y0, y1: f.y1 });
    for (const d of ['+x', '-x', '+y', '-y']) {
      if (f.dirs.has(d)) continue; const [ux, uy] = DV[d], len = ux ? H : W, off = ux ? W / 2 : H / 2;
      box(ux ? 0.003 : len, ux ? len : 0.003, dp, fitRail, f.x + ux * off, f.y + uy * off, f.z + dp / 2, grp);
      box(ux ? 0.018 : len, ux ? len : 0.018, 0.003, fitRail, f.x + ux * (off - 0.009), f.y + uy * (off - 0.009), f.z + dp, grp);
    }
  }
  for (const s of lvl) {                                                // the runs, minus the fitting footprints
    const cuts = s.cuts.map(c => [Math.max(0, c.u0), Math.min(1, c.u1)]).sort((p, q) => p[0] - q[0]); const keepU = []; let cur = 0;
    for (const [c0, c1] of cuts) { if (c0 > cur + 1e-4) keepU.push([cur, c0]); cur = Math.max(cur, c1); }
    if (cur < 1 - 1e-4) keepU.push([cur, 1]);
    const ang = Math.atan2(s.y1 - s.y0, s.x1 - s.x0);
    for (const [u0, u1] of keepU) {
      const L = (u1 - u0) * s.L; if (L < 0.01) continue;
      const g = new THREE.Group(); g.position.set(s.x0 + u0 * (s.x1 - s.x0), s.y0 + u0 * (s.y1 - s.y0), s.z); g.rotation.z = ang; grp.add(g);
      trayRun(g, L, s.tray.w, TRAY_DEPTH[s.tray.kind], s.tray.ladder);
      piece(s.tray.id, s.x0 + u0 * (s.x1 - s.x0), s.y0 + u0 * (s.y1 - s.y0), s.x0 + u1 * (s.x1 - s.x0), s.y0 + u1 * (s.y1 - s.y0), s.tray.w, s.z);
    }
  }
  E.fittings = fittings;
  for (let i = 0; i < pieces.length; i++) for (let j = i + 1; j < pieces.length; j++) {
    const a = pieces[i], b = pieces[j]; if (Math.abs(a.z - b.z) > 0.15) continue;
    const ox = Math.min(a.x1, b.x1) - Math.max(a.x0, b.x0), oy = Math.min(a.y1, b.y1) - Math.max(a.y0, b.y0);
    if (ox > 0.005 && oy > 0.005) warn('Trays {a} and {b} overlap near ({x}, {y})', { a: a.n, b: b.n, x: ((Math.max(a.x0, b.x0) + Math.min(a.x1, b.x1)) / 2).toFixed(1), y: ((Math.max(a.y0, b.y0) + Math.min(a.y1, b.y1)) / 2).toFixed(1) });
  }
  const adj = {}; const f2 = v => (Math.abs(v) < 0.005 ? 0 : v).toFixed(2);   // no '-0.00' vs '0.00' split of one graph node
  const key = (s, u) => s.tray.id + '@' + f2(s.x0 + u * (s.x1 - s.x0)) + ',' + f2(s.y0 + u * (s.y1 - s.y0));
  const edge = (a, b, w, seg) => { (adj[a] = adj[a] || []).push({ to: b, w, seg }); (adj[b] = adj[b] || []).push({ to: a, w, seg }); };
  const pos = {};
  for (const s of segs) {
    const us = [...new Set(s.split.map(u => +u.toFixed(4)))].sort((p, q) => p - q);
    for (let i = 0; i + 1 < us.length; i++) edge(key(s, us[i]), key(s, us[i + 1]), (us[i + 1] - us[i]) * s.L, s);
    for (const u of us) pos[key(s, u)] = { x: s.x0 + u * (s.x1 - s.x0), y: s.y0 + u * (s.y1 - s.y0), z: s.z, s, u };
  }
  const ks = Object.keys(pos); const ladders = [];                     // trays meeting at one xy: joint, or a vertical ladder between levels
  for (let i = 0; i < ks.length; i++) for (let j = i + 1; j < ks.length; j++) {
    const a = pos[ks[i]], b = pos[ks[j]]; if (ks[i].split('@')[0] === ks[j].split('@')[0] || Math.hypot(a.x - b.x, a.y - b.y) >= 0.05) continue;
    edge(ks[i], ks[j], Math.abs(a.z - b.z), null);
    if (Math.abs(a.z - b.z) > 0.1 && !ladders.some(l => Math.hypot(l.x - a.x, l.y - a.y) < 0.05)) {
      const lo = a.z < b.z ? a : b, hi = a.z < b.z ? b : a, w = hi.s.tray.w, ang = Math.atan2(hi.s.y1 - hi.s.y0, hi.s.x1 - hi.s.x0);
      const v = new THREE.Group(); v.position.set(a.x, a.y, lo.z + 0.05); v.rotation.order = 'ZYX'; v.rotation.set(0, -Math.PI / 2, ang); grp.add(v); trayRun(v, hi.z - lo.z - 0.05, w, 0.06);   // vertical tray
      const off = -0.06, ux = Math.cos(ang), uy = Math.sin(ang);                                                                        // bolted to a strut upright behind it
      upright(a.x + ux * off, a.y + uy * off, hi.z, ang, w, true);
      ladders.push({ x: a.x, y: a.y }); risers.push({ x: a.x, y: a.y, z0: lo.z, z1: hi.z }); vtrays.push({ x: a.x, y: a.y, ang, w, z0: lo.z, z1: hi.z });
    }
  }

  for (const id in attachTo) for (const A of attachTo[id]) edge('N:' + id, key(A.s, +A.u.toFixed(4)), A.d + A.dz, null);   // same rounding as the split points

  // ---------------------------------------------------------------- supports along every overhead tray: ≤ 2.0 m apart, max span 2.5 m (maxSpan)
  const along = (T, sPos) => { for (const s of T.segs) if (sPos <= s.s0 + s.L + 1e-6) { const u = Math.max(0, Math.min(1, (sPos - s.s0) / s.L)); return { x: s.x0 + u * (s.x1 - s.x0), y: s.y0 + u * (s.y1 - s.y0), ang: Math.atan2(s.y1 - s.y0, s.x1 - s.x0) }; } const s = T.segs[T.segs.length - 1]; return { x: s.x1, y: s.y1, ang: Math.atan2(s.y1 - s.y0, s.x1 - s.x0) }; };
  const sOf = (T, x, y) => { let best = null; for (const s of T.segs) { const p = proj(s, x, y); if (!best || p.d < best.d) best = { d: p.d, s: s.s0 + p.u * s.L }; } return best; };
  const spacing = cfg.supportSpacing || 2.0;
  for (const T of E.trays) {
    if (T.kind !== 'overhead' || !T.segs.length) continue;
    const held = [];                                                     // positions (along the tray) that already carry it
    for (const l of ladders) { const q = sOf(T, l.x, l.y); if (q && q.d < 0.05) held.push(q.s); }
    for (const j of jbs) if (j.post && j.postTop) { const q = sOf(T, j.x, j.y); if (q && q.d < 0.3) held.push(q.s); }
    for (const c of cabs) { const q = sOf(T, c.x, c.y); if (q && q.d < Math.max(c.sx, c.sy) / 2) held.push(q.s); }     // tray end over a cabinet roof
    for (const e of [0, T.len]) {                                        // a branch end resting on a tray that runs through (T-junction);
      const p = along(T, e);                                             // an inline / corner joint is a splice, not a support
      if (segs.some(s => { if (s.tray === T || s.tray.kind !== 'overhead') return false; const q = proj(s, p.x, p.y); const sp = s.s0 + q.u * s.L; return q.d < 0.05 && sp > 0.05 && sp < s.tray.len - 0.05; })) held.push(e);
    }
    /* greedy walk along the tray: the next support stands at the farthest free spot ≤ spacing after the last one (a JB post, a
       ladder or a branch junction already there counts); where the floor is blocked it takes the next free spot and the span check reports it */
    const free = v => { const p = along(T, v); return !blocked(p.x, p.y); };
    const hs = held.slice().sort((a, b) => a - b); const placed = held.slice();
    const put = v => { const p = along(T, v); if (!supports.some(u => Math.hypot(u.x - p.x, u.y - p.y) < 0.35)) upright(p.x, p.y, T.z - 0.006, p.ang, T.w); placed.push(v); return v; };   // one upright carries both runs at a corner
    let cur = hs.length && hs[0] <= 0.6 ? hs[0] : null;
    if (cur === null) for (let v = 0.1; v <= Math.min(T.len - 0.1, T.maxSpan / 2); v += 0.05) if (free(v)) { cur = put(v); break; }
    if (cur === null) cur = 0;
    while (T.len - cur > 0.6) {
      const nh = hs.find(h => h > cur + 0.05);
      if (nh !== undefined && nh <= cur + spacing) { cur = nh; continue; }
      let nxt = null;
      for (let v = Math.min(cur + spacing, T.len - 0.1); v > cur + 0.3; v -= 0.05) if (free(v)) { nxt = v; break; }
      if (nxt === null) for (let v = cur + spacing; v <= T.len - 0.1; v += 0.05) { if (nh !== undefined && v >= nh) { nxt = nh; break; } if (free(v)) { nxt = v; break; } }
      if (nxt === null) break;
      cur = hs.includes(nxt) ? nxt : put(nxt);
    }
    placed.sort((a, b) => a - b); let worstGap = { l: 0 };
    const pts = [0].concat(placed, [T.len]);
    for (let i = 0; i + 1 < pts.length; i++) {
      const gap = pts[i + 1] - pts[i], limit = (i === 0 || i + 2 === pts.length) ? T.maxSpan / 2 : T.maxSpan;   // cantilevered ends ≤ half a span
      if (gap > limit + 1e-3 && gap - limit > worstGap.l) worstGap = { l: gap - limit, at: along(T, (pts[i] + pts[i + 1]) / 2), gap };
    }
    T.supports = placed.length;
    if (worstGap.l > 0) warn('{t}: unsupported span {l} m near ({x}, {y}) — add a support / portal or use a ladder tray', { t: T.id, l: worstGap.gap.toFixed(1), x: worstGap.at.x.toFixed(1), y: worstGap.at.y.toFixed(1) });
  }
  // supports under 'under' trays: brackets hung from the host equipment frame (drawn as short hangers)
  for (const T of E.trays) if (T.kind === 'under') for (const s of T.segs) { const n = Math.max(1, Math.ceil(s.L / 1.5)); for (let i = 0; i <= n; i++) { const u = i / n; box(0.03, T.w + 0.06, 0.03, galv, s.x0 + u * (s.x1 - s.x0), s.y0 + u * (s.y1 - s.y0), s.z - 0.02, grp).rotation.z = Math.atan2(s.y1 - s.y0, s.x1 - s.x0); } }

  // ---------------------------------------------------------------- cordsets: device → JB along real carriers, never through the air
  /* carriers: wire duct on the fence bottom rail / inner walls / tunnel-guard walls (interrupted at every opening, door and curtain),
     cable duct in the frame of every flow-path equipment (conveyor, spur, machine, station), floor cable bridges, robot pedestals.
     A device reaches a carrier over its own bracket / post (horizontal hop ≤ hop 0.6 m, any height), carriers meeting within
     0.4 m are clipped together; a JB (or a cabinet) is reached the same way. Shortest route, nearest JB with a free port. */
  const hopMax = cfg.hop || 0.6, joinMax = 0.4, cseg = [];
  const addC = (x0, y0, x1, y1, z, kind, extra) => { const L = Math.hypot(x1 - x0, y1 - y0); if (L > 0.05) cseg.push(Object.assign({ x0, y0, x1, y1, z, kind, L, split: [0, 1], used: [] }, extra || {})); };
  const spans = (a0, a1, openings) => { const out = []; let cur = a0; for (const [a, b] of openings.slice().sort((p, q) => p[0] - q[0])) { if (a > cur) out.push([cur, Math.min(a, a1)]); cur = Math.max(cur, b); } if (cur < a1) out.push([cur, a1]); return out; };
  const zDuct = 0.25;
  if (F) {
    const side = { front: [true, F.y0, F.x0, F.x1], top: [true, F.y1, F.x0, F.x1], left: [false, F.x0, F.y0, F.y1], right: [false, F.x1, F.y0, F.y1] };
    for (const r of saf.runs || []) {
      const [h, fx, a0, a1] = side[r.side]; const op = (r.openings || []).concat((saf.doors || []).filter(d => d.side === r.side).map(d => [d.from, d.to]));
      for (const [a, b] of spans(a0, a1, op)) h ? addC(a, fx, b, fx, zDuct, 'fence') : addC(fx, a, fx, b, zDuct, 'fence');
    }
    for (const d of saf.doors || []) {                                   // the duct continues over a door on its frame header
      const zH = Math.max(F.h || 2, (d.clear || 0) + 0.06) - 0.08;                          // on the header of a tall door (clear height for a load)
      const s = d.side; if (s) { const [h, fx] = side[s]; h ? addC(d.from, fx, d.to, fx, zH, 'fence') : addC(fx, d.from, fx, d.to, zH, 'fence'); }
      else if (d.y !== undefined) addC(d.from, d.y, d.to, d.y, zH, 'fence'); else addC(d.x, d.from, d.x, d.to, zH, 'fence');
    }
    for (const w of saf.walls || []) {
      const on = d => !d.side && (w.y !== undefined ? d.y === w.y && d.from >= w.x0 - 1e-6 && d.to <= w.x1 + 1e-6 : d.x === w.x && d.from >= w.y0 - 1e-6 && d.to <= w.y1 + 1e-6);
      const op = (w.openings || []).concat((saf.doors || []).filter(on).map(d => [d.from, d.to]));
      if (w.y !== undefined) for (const [a, b] of spans(w.x0, w.x1, op)) addC(a, w.y, b, w.y, zDuct, 'fence'); else for (const [a, b] of spans(w.y0, w.y1, op)) addC(w.x, a, w.x, b, zDuct, 'fence');
    }
    for (const h of saf.hoods || []) { const z = (h.zBase || 0) + 0.1;
      if (h.axis === 'x') { addC(h.x0, h.y0, h.x1, h.y0, z, 'fence'); addC(h.x0, h.y1, h.x1, h.y1, z, 'fence'); } else { addC(h.x0, h.y0, h.x0, h.y1, z, 'fence'); addC(h.x1, h.y0, h.x1, h.y1, z, 'fence'); } }
  }
  /* frame duct of a flow keep-out at 0.3 m — or 50 mm below its conveying surface when the keep-out declares one (`deck`: pallet chain
     conveyors at 0.15 m, floor-level stations): a cross member at 0.3 m over a 0.15 m deck runs through the pallets */
  /* frame: 'sides' = a floor area a cart / pallet / vehicle drives into along its long axis (cart dock, pallet spot): ducts only along the
     two long sides, nothing across the path (flow-path sweep: a cross member at 0.3 m ran through the docked carts) */
  for (const k of keep) if (k.flow) { const z = k.deck !== undefined ? Math.max(0.05, Math.min(0.3, k.deck - 0.05)) : 0.3, o = { owner: k };
    const alongX = k.x1 - k.x0 >= k.y1 - k.y0, Lk = alongX ? k.x1 - k.x0 : k.y1 - k.y0, n = k.frame === 'sides' ? 0 : Math.floor(Lk / 2.0);   // cross members every ≤ 2 m
    const sx = k.frame !== 'sides' || alongX, sy = k.frame !== 'sides' || !alongX;              // which perimeter sides get a duct (same order as before)
    if (sx) addC(k.x0, k.y0, k.x1, k.y0, z, 'frame', o); if (sy) addC(k.x1, k.y0, k.x1, k.y1, z, 'frame', o); if (sx) addC(k.x1, k.y1, k.x0, k.y1, z, 'frame', o); if (sy) addC(k.x0, k.y1, k.x0, k.y0, z, 'frame', o);
    for (let i = 1; i <= n; i++) { const t = (alongX ? k.x0 : k.y0) + Lk * i / (n + 1); alongX ? addC(t, k.y0, t, k.y1, z, 'frame', o) : addC(k.x0, t, k.x1, t, z, 'frame', o); } }
  for (const s of segs) if (s.tray.kind === 'floor') addC(s.x0, s.y0, s.x1, s.y1, 0.05, 'bridge', { tray: s.tray });
  const cproj = (s, x, y) => proj(s, x, y);
  // terminals: devices, JBs, cabinets (a device beside a cabinet may be wired into it), robot pedestals (pass-through)
  const hops = {};                                                     // terminal id → [{ c, u, d, px, py }]
  /* no cable across a product / work position: a bracket's legs (up / down at the terminal, along x, then y at carrier height) never run
     through a flow keep-out at working level (frame duct height 0.31 m < z < its top + 0.3 m — the robot places parts and the product
     moves there), and a device inside a flow keep-out (sensor on a nest, a lane, a feeder) leaves it downward through the equipment
     frame (carriers ≤ 0.31 m), never up across the work area to a duct on a guard or a door header */
  const zFrame = 0.31, flowK = keep.filter(k => k.flow);
  const through = (ax, ay, bx, by, k) => Math.max(ax, bx) > k.x0 + 1e-3 && Math.min(ax, bx) < k.x1 - 1e-3 && Math.max(ay, by) > k.y0 + 1e-3 && Math.min(ay, by) < k.y1 - 1e-3;
  const hopBlocked = (x, y, z, c, p) => {
    if (c.z <= zFrame) return false;
    if (z !== undefined && flowK.some(k => inR(x, y, k) && z <= k.h)) return true;
    return flowK.some(k => c.z < k.h + 0.3 && (through(x, y, p.px, y, k) || through(p.px, y, p.px, p.py, k)));
  };
  const hopTo = (id, x, y, rect, maxH, only, z) => {
    const list = [];
    cseg.forEach((c, i) => {
      if (only && c.kind !== only) return;
      const p = cproj(c, x, y); const d = rect ? Math.hypot(Math.max(0, Math.abs(p.px - x) - rect.sx / 2), Math.max(0, Math.abs(p.py - y) - rect.sy / 2)) : p.d;
      const own = id[0] === 'D' && c.owner && inR(x, y, c.owner);        // a device on a machine / conveyor: wired inside it to its frame duct
      if ((d <= maxH || own) && (id[0] === 'R' || !hopBlocked(x, y, z, c, p))) { c.split.push(p.u); list.push({ c: i, u: p.u, d, px: p.px, py: p.py }); }
    });
    hops[id] = list; return list;
  };
  for (const d of devices) hopTo('D:' + d.id, mp(d).x, mp(d).y, null, hopMax, null, d.z);
  for (const j of jbs) hopTo('J:' + j.id, j.x, j.y, null, hopMax);
  // cabinets are not cordset terminals (device → JB → cabinet); only a device mounted beside one (E-stop at a panel) wires straight in
  for (const c of RC.ctxs || []) if (!c.robot.ceiling) hopTo('R:' + c.id, c.base.x, c.base.y, null, attachRobot, 'bridge');   // a pedestal passes cordsets; a ceiling robot does not
  // carrier crossings (clipped junction) and near-touching ends (≤ 0.35 m clip)
  const cedges = [];
  for (let a = 0; a < cseg.length; a++) for (let b = a + 1; b < cseg.length; b++) {
    const A = cseg[a], B = cseg[b];
    const dx1 = A.x1 - A.x0, dy1 = A.y1 - A.y0, dx2 = B.x1 - B.x0, dy2 = B.y1 - B.y0, den = dx1 * dy2 - dy1 * dx2;
    if (Math.abs(den) > 1e-9) { const ua = ((B.x0 - A.x0) * dy2 - (B.y0 - A.y0) * dx2) / den, ub = ((B.x0 - A.x0) * dy1 - (B.y0 - A.y0) * dx1) / den;
      if (ua >= -1e-6 && ua <= 1 + 1e-6 && ub >= -1e-6 && ub <= 1 + 1e-6) { A.split.push(ua); B.split.push(ub); cedges.push([a, ua, b, ub, Math.abs(A.z - B.z)]); continue; } }
    let best = null;
    for (const [P, Q, ip, iq] of [[A, B, a, b], [B, A, b, a]]) for (const [x, y, uq] of [[Q.x0, Q.y0, 0], [Q.x1, Q.y1, 1]]) { const p = cproj(P, x, y); if (p.d <= joinMax && (!best || p.d < best.d)) best = { d: p.d, ip, up: p.u, iq, uq }; }
    if (best) { cseg[best.ip].split.push(best.up); cedges.push([best.ip, best.up, best.iq, best.uq, best.d + Math.abs(A.z - B.z), true]); }
  }
  const ck = (i, u) => i + '@' + u.toFixed(4);
  const cadj = {}; const cpos = {};
  const cedge = (p, q, w, info) => { (cadj[p] = cadj[p] || []).push({ to: q, w, info }); (cadj[q] = cadj[q] || []).push({ to: p, w, info }); };
  cseg.forEach((c, i) => {
    c.us = [...new Set(c.split.map(u => +Math.max(0, Math.min(1, u)).toFixed(4)))].sort((p, q) => p - q);
    for (const u of c.us) cpos[ck(i, u)] = { x: c.x0 + u * (c.x1 - c.x0), y: c.y0 + u * (c.y1 - c.y0), z: c.z };
    for (let k = 0; k + 1 < c.us.length; k++) cedge(ck(i, c.us[k]), ck(i, c.us[k + 1]), (c.us[k + 1] - c.us[k]) * c.L, { seg: i, u0: c.us[k], u1: c.us[k + 1] });
  });
  for (const [a, ua, b, ub, w, clip] of cedges) cedge(ck(a, +Math.max(0, Math.min(1, ua)).toFixed(4)), ck(b, +Math.max(0, Math.min(1, ub)).toFixed(4)), w, { clip: !!clip });
  const tz = id => { const [k, n] = [id[0], id.slice(2)]; return k === 'D' ? devices.find(d => d.id === n).z : k === 'J' ? (E.nodes[n].role === 'jb' ? E.nodes[n].z : 0.3) : 0.25; };
  for (const id in hops) for (const h of hops[id]) cedge(id, ck(h.c, +h.u.toFixed(4)), h.d + Math.abs(tz(id) - cseg[h.c].z), { hop: id, h });
  for (const d of devices) for (const c of cabs) {                       // a device beside a cabinet (E-stop at a controller) wires straight into it
    const dd = Math.hypot(Math.max(0, Math.abs(d.x - c.x) - c.sx / 2), Math.max(0, Math.abs(d.y - c.y) - c.sy / 2));
    if (dd <= hopMax) cedge('D:' + d.id, 'J:' + c.id, dd + 0.5, { direct: true });
  }
  const dijkstra = src => {                                              // binary-heap Dijkstra; never through another device / JB / cabinet
    const dist = { [src]: 0 }, prev = {}, heap = [[0, src]];
    const push = e => { heap.push(e); let i = heap.length - 1; while (i) { const p = (i - 1) >> 1; if (heap[p][0] <= heap[i][0]) break; [heap[p], heap[i]] = [heap[i], heap[p]]; i = p; } };
    const pop = () => { const top = heap[0], last = heap.pop(); if (heap.length) { heap[0] = last; let i = 0; for (;;) { const l = 2 * i + 1, r = l + 1; let m = i; if (l < heap.length && heap[l][0] < heap[m][0]) m = l; if (r < heap.length && heap[r][0] < heap[m][0]) m = r; if (m === i) break; [heap[m], heap[i]] = [heap[i], heap[m]]; i = m; } } return top; };
    while (heap.length) {
      const [du, u] = pop(); if (du > dist[u]) continue;
      if (u !== src && (u[0] === 'D' || u[0] === 'J') && u[1] === ':') continue;
      for (const e of cadj[u] || []) { const v = du + e.w; if (dist[e.to] === undefined || v < dist[e.to]) { dist[e.to] = v; prev[e.to] = { u, info: e.info }; push([v, e.to]); } }
    }
    return { dist, prev };
  };
  const ductM = mat(0x9aa1a7, { roughness: 0.6 }),   /* grey PVC wiring duct 60 × 60 */ wireM = mat(0x16191c, { roughness: 0.8 });
  const bar = (x0, y0, z0, x1, y1, z1, t, m) => {                         // thin straight piece (cable, conduit) between two points
    const L = Math.hypot(x1 - x0, y1 - y0, z1 - z0); if (L < 0.01) return;
    const mesh = new THREE.Mesh(new THREE.BoxGeometry(t, t, L), m); mesh.position.set((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2);
    mesh.quaternion.setFromUnitVectors(new THREE.Vector3(0, 0, 1), new THREE.Vector3(x1 - x0, y1 - y0, z1 - z0).normalize()); grp.add(mesh);
  };
  let maxDropSeen = 0; const hopDrawn = new Set();
  const assign = (d, tgt, dist, prev) => {
    const key = 'J:' + tgt.id; const L = dist[key] + 0.5 + armL(d);       // + 0.5 m slack at both ends, + the bracket arm
    maxDropSeen = Math.max(maxDropSeen, L); if (tgt.role === 'jb') tgt.devices.push(d.id); d.jbId = tgt.id; d.len = L;
    if (L > maxDrop) warn('{d} → {j}: cordset {l} m > {m} m', { d: d.id, j: tgt.id, l: L.toFixed(1), m: maxDrop });
    const pts = []; let u = key;
    while (u) {
      const p = prev[u]; const P = cpos[u] || (u[0] === 'J' ? { x: tgt.x, y: tgt.y, z: tgt.role === 'jb' ? tgt.z : 0.25 } : u[0] === 'D' ? d : null); if (P) pts.push(new THREE.Vector3(P.x, P.y, P.z + 0.02));
      if (!p) break;
      if (p.info.seg !== undefined) cseg[p.info.seg].used.push([p.info.u0, p.info.u1]);
      if (p.info.hop && !hopDrawn.has(p.info.hop + p.info.h.c)) {                  // bracket / post: vertical at the terminal, across at carrier height
        hopDrawn.add(p.info.hop + p.info.h.c); const h = p.info.h, c = cseg[h.c], T = p.info.hop; const n = T[0] === 'D' ? mp(d) : T[0] === 'J' ? tgt : null;
        if (n) { const z0 = T[0] === 'J' && n.role !== 'jb' ? c.z : n.z - (T[0] === 'J' ? 0.1 : 0); const t = T[0] === 'J' ? 0.03 : 0.012, m = T[0] === 'J' ? conduit : wireM; bar(n.x, n.y, z0, n.x, n.y, c.z, t, m); bar(n.x, n.y, c.z, h.px, n.y, c.z, t, m); bar(h.px, n.y, c.z, h.px, h.py, c.z, t, m); }   // up / down the post, then along x, then y
      }
      if (p.info.direct) { const c = tgt; bar(d.x, d.y, d.z, d.x, d.y, 0.25, 0.012, wireM); bar(d.x, d.y, 0.25, c.x, d.y, 0.25, 0.03, conduit); bar(c.x, d.y, 0.25, c.x, c.y, 0.25, 0.03, conduit); }   // along x, then y
      if (p.info.clip) { const a = cpos[u], b = cpos[p.u]; if (a && b) { const zl = Math.min(a.z, b.z);   // clip: across at the LOWER duct, vertical at the higher one
        bar(a.x, a.y, a.z, a.x, a.y, zl, 0.012, wireM); bar(a.x, a.y, zl, b.x, a.y, zl, 0.012, wireM); bar(b.x, a.y, zl, b.x, b.y, zl, 0.012, wireM); bar(b.x, b.y, zl, b.x, b.y, b.z, 0.012, wireM); } }
      // planning line follows the cable: vertical along the post / bracket at the terminal, then horizontal into the duct — never a diagonal
      if (p.info.hop) { const Tk = p.info.hop, c = cseg[p.info.h.c]; const n = Tk[0] === 'D' ? mp(d) : Tk[0] === 'J' ? tgt : (RC.ctxs || []).map(x => ({ x: x.base.x, y: x.base.y, id: 'R:' + x.id })).find(x => x.id === Tk);
        if (n) { const h = p.info.h, a = new THREE.Vector3(n.x, n.y, c.z + 0.02), b = new THREE.Vector3(h.px, n.y, c.z + 0.02); if (u === Tk) pts.push(a, b); else pts.push(b, a); } }
      if (p.info.direct) pts.push(new THREE.Vector3(tgt.x, d.y, 0.27), new THREE.Vector3(d.x, d.y, 0.27));
      if (p.info.clip) { const a = cpos[u], b = cpos[p.u]; if (a && b) { const zl = Math.min(a.z, b.z) + 0.02; pts.push(new THREE.Vector3(a.x, a.y, zl), new THREE.Vector3(b.x, a.y, zl), new THREE.Vector3(b.x, b.y, zl)); } }
      u = p.u;
    }
    if (d.mount) {                                                       // bracket arm: device → mount point(s) at device height (each leg x, then y)
      const P = mpath(d), arm = [];
      for (let i = 1; i < P.length; i++) { const a = P[i - 1], b = P[i]; bar(a.x, a.y, d.z, b.x, a.y, d.z, 0.012, wireM); bar(b.x, a.y, d.z, b.x, b.y, d.z, 0.012, wireM);
        arm.push(new THREE.Vector3(b.x, a.y, d.z + 0.02), new THREE.Vector3(b.x, b.y, d.z + 0.02)); }
      pts.splice(Math.max(0, pts.length - 1), 0, ...arm.reverse());
    }
    if (pts.length > 1) ov.add(new THREE.Line(new THREE.BufferGeometry().setFromPoints(pts), dropM));
    E.cables.push({ from: d.id, to: tgt.id, kind: d.kind === 'safety' ? 'safety' : 'cordset', len: +L.toFixed(1), tray: false });
  };
  const terminals = jbs.concat(cabs);
  const route1 = d => {
    if (!hops['D:' + d.id].length && !(cadj['D:' + d.id] || []).length) { warn('{d}: no cable carrier within {h} m (fence duct, equipment frame, floor bridge)', { d: d.id, h: hopMax }); return; }
    const { dist, prev } = dijkstra('D:' + d.id);
    let tgt = null;
    if (d.jb) { tgt = E.nodes[d.jb]; if (!tgt) { warn('{d}: no junction box', { d: d.id }); return; } if (dist['J:' + tgt.id] === undefined) { warn('{d} → {j}: no cable route', { d: d.id, j: tgt.id }); return; } }
    else {
      const cand = terminals.filter(n => dist['J:' + n.id] !== undefined).sort((a, b) => dist['J:' + a.id] - dist['J:' + b.id]);
      tgt = cand.find(n => (n.role !== 'jb' || n.devices.length < n.ports) && dist['J:' + n.id] + 0.5 <= maxDrop) || cand[0];
      if (!tgt) { warn('{d}: no cable route to a junction box', { d: d.id }); return; }
    }
    assign(d, tgt, dist, prev);
  };
  for (const d of devices) if (d.jb) route1(d);                          // explicit first, then nearest with a free port
  for (const d of devices) if (!d.jb) route1(d);
  cseg.forEach(c => {                                                    // draw the used duct lengths
    if (c.kind === 'bridge' && c.used.length) c.tray.cordsets = true;
    if (c.kind === 'bridge' || !c.used.length) return;
    const iv = c.used.map(([a, b]) => [Math.min(a, b), Math.max(a, b)]).sort((p, q) => p[0] - q[0]); const merged = [];
    for (const v of iv) { const l = merged[merged.length - 1]; if (l && v[0] <= l[1] + 1e-4) l[1] = Math.max(l[1], v[1]); else merged.push(v.slice()); }
    for (const [a, b] of merged) { const g = new THREE.Group(); g.position.set(c.x0 + a * (c.x1 - c.x0), c.y0 + a * (c.y1 - c.y0), c.z); g.rotation.z = Math.atan2(c.y1 - c.y0, c.x1 - c.x0); grp.add(g); box((b - a) * c.L, 0.06, 0.06, ductM, (b - a) * c.L / 2, 0, 0, g); }
  });
  for (const j of jbs) if (!j.devices.length) warn('{j} has no devices', { j: j.id });
  for (const j of jbs) if (j.devices.length > j.ports) warn('{j}: {n} devices > {p} ports', { j: j.id, n: j.devices.length, p: j.ports });
  for (const j of cfg.jbs || []) for (const k of keep) if (inR(j.x, j.y, k)) warn('{j} stands in {n}', { j: j.id, n: k.name || 'equipment' });
  /* a JB (box ± 0.15 m on its post) must not stand in a vertical tray: the vertical tray where a floor bridge meets the trunk is as wide as
     the trunk (0.3–0.8 m across the run, 0.06 m deep along it) — coin_set_assembly: JN2 0.05 m from the R8 bridge riser sat inside the 0.8 m vertical tray */
  for (const j of cfg.jbs || []) for (const v of vtrays) {
    const dx = j.x - v.x, dy = j.y - v.y, ca = Math.cos(v.ang), sa = Math.sin(v.ang), along = dx * ca + dy * sa, across = -dx * sa + dy * ca;
    if (Math.abs(along) < 0.15 + 0.06 + 0.05 && Math.abs(across) < v.w / 2 + 0.15 + 0.05) warn('{j} stands in the vertical tray at ({x}, {y})', { j: j.id, x: v.x.toFixed(2), y: v.y.toFixed(2) });
  }
  E.carriers = cseg;
  const path = (a, b) => {                                              // Dijkstra over the tray graph
    const dist = { ['N:' + a]: 0 }, prev = {}, done = new Set(); const q = ['N:' + a];
    while (q.length) {
      q.sort((p, r) => dist[p] - dist[r]); const u = q.shift(); if (done.has(u)) continue; done.add(u);
      if (u === 'N:' + b) break;
      if (u.startsWith('N:') && u !== 'N:' + a) continue;                 // never route through another cabinet / JB
      for (const e of adj[u] || []) { const v = dist[u] + e.w; if (dist[e.to] === undefined || v < dist[e.to]) { dist[e.to] = v; prev[e.to] = { u, seg: e.seg }; q.push(e.to); } }
    }
    if (dist['N:' + b] === undefined) return null;
    const used = []; const pts = []; let u = 'N:' + b;
    while (u) { if (pos[u]) pts.push(pos[u]); const p = prev[u]; if (!p) break; if (p.seg) used.push(p.seg); u = p.u; }
    return { len: dist['N:' + b], segs: [...new Set(used)], pts: pts.reverse() };
  };
  const route = (a, b, list, kind) => {
    if (!attachTo[a] || !attachTo[b]) return null;
    const p = path(a, b);
    if (!p) { warn('No tray route {a} → {b}', { a: a.replace('robot:', 'robot '), b: b.replace('robot:', 'robot ') }); return null; }
    const len = p.len + 2.0;                                           // attach edges carry the risers; + 2 m service loops
    for (const s of p.segs) for (const [n, d] of list) { s.cables.push(n); s.area += Math.PI * d * d / 4; }
    for (const [n] of list) E.cables.push({ from: a, to: b, kind: n, len: +len.toFixed(1), tray: true, trays: [...new Set(p.segs.map(s => s.tray.id))] });
    if (p.pts.length > 1) ov.add(new THREE.Line(new THREE.BufferGeometry().setFromPoints(p.pts.map(q => new THREE.Vector3(q.x, q.y, q.z + 0.05))), new THREE.LineBasicMaterial({ color: kind === 'robot' ? 0xff6a13 : kind === 'feeder' ? 0xff3b3b : 0x5ec8ff })));
    return len;
  };
  // ---------------------------------------------------------------- cables: JB → cabinet, sub → feed, controller → MCP, controller ↔ robot
  for (const j of jbs) { if (!E.nodes[j.to]) { warn('{j}: cabinet {c} not found', { j: j.id, c: j.to }); continue; } route(j.id, j.to, [['JB bus', DIA.bus], ['JB 24 V', DIA.power24], ['JB power', DIA.jbPower]], 'jb'); }
  for (const c of cabs.filter(n => n.role === 'sub')) route(c.id, c.feed || M.id, [['feeder', DIA.feeder], ['bus', DIA.bus], ['safety', DIA.safety]], 'feeder');
  E.robotCables = [];
  for (const c of cabs.filter(n => n.role === 'ctrl')) {
    route(c.id, M.id, [['feeder', DIA.feeder], ['bus', DIA.bus], ['safety', DIA.safety]], 'feeder');
    for (const r of c.robots) {                                         // one controller cabinet may hold several (OmniCore C30) controllers
      const L = route('robot:' + r, c.id, [['motor power', DIA.motor], ['SMB', DIA.smb], ['customer power / signal', DIA.customer]], 'robot');
      if (!L) continue;
      const std = (c.cables || ROBOT_CABLES).find(v => v >= L);
      E.robotCables.push({ ctrl: c.id, robot: r, need: +L.toFixed(1), std: std || null });
      if (!std) warn('{c} → robot {r}: {l} m > longest manipulator cable {m} m', { c: c.id, r, l: L.toFixed(1), m: (c.cables || ROBOT_CABLES).slice(-1)[0] });
    }
  }
  // tray fill (≤ 40 % of the usable cross-section)
  let worst = { fill: 0, id: '—' };
  for (const T of E.trays) {
    for (const s of T.segs) T.fill = Math.max(T.fill, s.area / (T.w * T.depth));
    if (T.fill > worst.fill) worst = { fill: T.fill, id: T.id };
    if (T.fill > FILL_MAX) warn('{t}: fill {f} % > {m} %', { t: T.id, f: Math.round(T.fill * 100), m: FILL_MAX * 100 });
    if (!T.segs.some(s => s.cables.length) && !T.cordsets) warn('{t} carries no cable', { t: T.id });
  }

  // ---------------------------------------------------------------- service areas: clear of fence lines, walls, equipment, lanes, other cabinets
  const ovl = (a, b) => a.x0 < b.x1 - 1e-3 && a.x1 > b.x0 + 1e-3 && a.y0 < b.y1 - 1e-3 && a.y1 > b.y0 + 1e-3;
  const inside = (x, y) => F && x > F.x0 && x < F.x1 && y > F.y0 && y < F.y1;
  let svcOk = 0;
  for (const c of cabs) {
    const r = c.service; const body = { x0: c.x - c.sx / 2, x1: c.x + c.sx / 2, y0: c.y - c.sy / 2, y1: c.y + c.sy / 2 };
    const hits = [];
    for (const k of fenceLines.concat(keep)) if (ovl(r, k)) hits.push(k.name || 'equipment');
    for (const o of cabs) if (o !== c && ovl(r, { x0: o.x - o.sx / 2, x1: o.x + o.sx / 2, y0: o.y - o.sy / 2, y1: o.y + o.sy / 2 })) hits.push(o.id);
    for (const k of keep) if (ovl(body, k)) hits.push(k.name || 'equipment');
    if (inside(c.x, c.y) && !c.insideOk) hits.push('inside the guarded area');
    if (hits.length) warn('{c}: service area blocked by {h}', { c: c.id, h: [...new Set(hits)].join(', ') }); else svcOk++;
    c.serviceOk = !hits.length;
    const col = hits.length ? 0xff4d4d : 0x35c26b;
    const m = box(r.x1 - r.x0, r.y1 - r.y0, 0.004, new THREE.MeshBasicMaterial({ color: col, transparent: true, opacity: 0.22, depthWrite: false }), (r.x0 + r.x1) / 2, (r.y0 + r.y1) / 2, 0.012, ov); m.castShadow = false;
    const edgePts = [[r.x0, r.y0], [r.x1, r.y0], [r.x1, r.y1], [r.x0, r.y1]].map(p => new THREE.Vector3(p[0], p[1], 0.016));
    ov.add(new THREE.LineLoop(new THREE.BufferGeometry().setFromPoints(edgePts), new THREE.LineBasicMaterial({ color: col })));
    const [fx, fy] = FACE[c.face]; const n = Math.round(c.w / c.leaf);        // door swing arcs
    for (let i = 0; i < n; i++) {
      const o = -c.w / 2 + i * c.leaf; const hx = fx ? c.x + fx * c.sx / 2 : c.x + o, hy = fx ? c.y + o : c.y + fy * c.sy / 2;
      const pts = []; for (let a = 0; a <= 12; a++) { const th = a / 12 * Math.PI / 2; const along = Math.cos(th) * c.leaf, out = Math.sin(th) * c.leaf;
        pts.push(new THREE.Vector3(fx ? hx + fx * out : hx + along, fx ? hy + along : hy + fy * out, 0.018)); }
      ov.add(new THREE.Line(new THREE.BufferGeometry().setFromPoints(pts), new THREE.LineBasicMaterial({ color: 0xe8ecef, transparent: true, opacity: 0.7 })));
    }
  }

  // ---------------------------------------------------------------- stats + HUD + verification helpers
  const sum = k => E.trays.filter(T => T.kind === k).reduce((a, T) => a + T.len, 0);
  E.stats = { main: M ? M.id : '—', subs: cabs.filter(c => c.role === 'sub').length, ctrls: cabs.filter(c => c.role === 'ctrl').length, jbs: jbs.length, devices: devices.length,
              overhead: sum('overhead') + sum('under'), floor: sum('floor'), supports: supports.length, fill: worst, svcOk, svcN: cabs.length, maxDrop: maxDropSeen,
              cableLen: E.cables.reduce((a, c) => a + c.len, 0) };
  E.devices = devices; E.vtrays = vtrays;
  E.setVisible = on => { ov.visible = !!on; return ov.visible; };
  RC.setElecVisible = E.setVisible;
  /* min clearance between every robot's links / tool (world AABBs, conservative) and overhead / under trays, cabinets, JBs */
  const obst = [];
  for (const s of segs) if (s.tray.kind !== 'floor') {
    obst.push({ n: s.tray.id, b: new THREE.Box3(new THREE.Vector3(Math.min(s.x0, s.x1) - s.tray.w / 2, Math.min(s.y0, s.y1) - s.tray.w / 2, s.z - 0.02), new THREE.Vector3(Math.max(s.x0, s.x1) + s.tray.w / 2, Math.max(s.y0, s.y1) + s.tray.w / 2, s.z + s.tray.depth)) });
  }
  for (const c of cabs) obst.push({ n: c.id, b: new THREE.Box3(new THREE.Vector3(c.x - c.sx / 2, c.y - c.sy / 2, 0), new THREE.Vector3(c.x + c.sx / 2, c.y + c.sy / 2, c.h)) });
  for (const u of supports) obst.push({ n: 'tray support', b: new THREE.Box3(new THREE.Vector3(u.x - u.w / 2, u.y - u.w / 2, 0), new THREE.Vector3(u.x + u.w / 2, u.y + u.w / 2, u.z1)) });
  for (const r of risers) obst.push({ n: 'riser', b: new THREE.Box3(new THREE.Vector3(r.x - 0.1, r.y - 0.1, r.z0), new THREE.Vector3(r.x + 0.1, r.y + 0.1, r.z1)) });
  for (const j of jbs) obst.push({ n: j.id, b: new THREE.Box3(new THREE.Vector3(j.x - 0.15, j.y - 0.15, j.z - 0.1), new THREE.Vector3(j.x + 0.15, j.y + 0.15, j.z + 0.1)) });
  E.obstacles = obst;
  const gap = (a, b) => Math.hypot(Math.max(0, a.min.x - b.max.x, b.min.x - a.max.x), Math.max(0, a.min.y - b.max.y, b.min.y - a.max.y), Math.max(0, a.min.z - b.max.z, b.min.z - a.max.z));
  const tmp = new THREE.Box3();
  E.robotClearance = () => {
    let best = { d: 1e9, n: '', robot: '' };
    for (const c of RC.ctxs) {
      const root = c.robot.root; if (!root) continue; root.updateMatrixWorld(true);
      root.traverse(o => {
        if (!o.isMesh) return; if (o.parent && o.parent.name === 'Base' || o.name === 'Base') return;
        tmp.setFromObject(o);
        for (const q of obst) { const d = gap(tmp, q.b); if (d < best.d) best = { d, n: q.n, robot: c.id }; }
      });
    }
    return best;
  };
};

function clipT(s, r) {                                    // parameter interval of segment s inside rectangle r (Liang–Barsky), or null
  let t0 = 0, t1 = 1; const dx = s.x1 - s.x0, dy = s.y1 - s.y0;
  for (const [p, q] of [[-dx, s.x0 - r.x0], [dx, r.x1 - s.x0], [-dy, s.y0 - r.y0], [dy, r.y1 - s.y0]]) {
    if (Math.abs(p) < 1e-12) { if (q < 0) return null; continue; }
    const u = q / p; if (p < 0) { if (u > t1) return null; if (u > t0) t0 = u; } else { if (u < t0) return null; if (u < t1) t1 = u; }
  }
  return t1 - t0 > 1e-6 ? [t0, t1] : null;
}
function segRect(s, r) {                                   // 2D segment ∩ rectangle (Liang–Barsky)
  let t0 = 0, t1 = 1; const dx = s.x1 - s.x0, dy = s.y1 - s.y0;
  for (const [p, q] of [[-dx, s.x0 - r.x0], [dx, r.x1 - s.x0], [-dy, s.y0 - r.y0], [dy, r.y1 - s.y0]]) {
    if (Math.abs(p) < 1e-12) { if (q < 0) return false; continue; }
    const u = q / p; if (p < 0) { if (u > t1) return false; if (u > t0) t0 = u; } else { if (u < t0) return false; if (u < t1) t1 = u; }
  }
  return t1 - t0 > 1e-6;
}

RC.elecHUD = function (setText) {
  const E = RC.elec; if (!E) return; const t = RC.t; const s = E.stats;
  setText('m-el-main', s.main);
  setText('m-el-cab', `${s.subs} / ${s.ctrls}`);
  setText('m-el-jb', `${s.jbs} · ${s.devices}`);
  setText('m-el-tray', t('{o} m / {f} m', { o: s.overhead.toFixed(1), f: s.floor.toFixed(1) }));
  setText('m-el-fill', `${Math.round(s.fill.fill * 100)} % (${s.fill.id})`);
  setText('m-el-svc', `${s.svcOk} / ${s.svcN}`);
  const rc = E.robotCables || []; const worst = rc.reduce((a, r) => !a || r.need > a.need ? r : a, null);
  setText('m-el-rcab', !rc.length ? '—' : rc.length <= 2 ? rc.map(r => `${r.ctrl} ${r.need} → ${r.std || '?'} m`).join(' · ') : t('{n} robots · longest {l} → {s} m', { n: rc.length, l: worst.need, s: worst.std || '?' }));
  setText('m-el-check', E.issues.length ? t('{n} issue(s): ', { n: E.issues.length }) + t(E.issues[0][0], E.issues[0][1]) : t('all connected · clear'));
};

RC.i18n.add({ zh: {
  'Electrical': '電氣', 'Main control panel': '主控制盤', 'Sub-cabinets / robot controllers': '子盤 / 機器人控制櫃', 'Junction boxes · field devices': '接線盒 · 現場元件',
  'Cable trays / floor trunks': '線槽（架空/機下）/ 地面線槽', '{o} m / {f} m': '{o} m / {f} m', 'Max tray fill (≤ 40 %)': '線槽最大填充率（≤ 40 %）',
  'Service areas clear': '維修空間淨空', 'Manipulator cables': '機器人本體電纜', 'Electrical check': '電氣檢查', 'all connected · clear': '全部連接 · 空間淨空',
  '{n} issue(s): ': '{n} 項問題：', '{n} robots · longest {l} → {s} m': '{n} 台機器人 · 最長 {l} → {s} m', 'No main control panel': '缺少主控制盤', '{d}: no junction box': '{d}：無接線盒', '{d} → {j}: cordset {l} m > {m} m': '{d} → {j}：電纜 {l} m > {m} m',
  '{j} has no devices': '{j} 無連接元件', 'Trays {a} and {b} overlap near ({x}, {y})': '線槽 {a} 與 {b} 於 ({x}, {y}) 附近重疊', '{d}: no cable carrier within {h} m (fence duct, equipment frame, floor bridge)': '{d}：{h} m 內無電纜載體（圍籬線槽、設備框架、地面線槽）', '{d} → {j}: no cable route': '{d} → {j}：無電纜路徑', '{d}: no cable route to a junction box': '{d}：無通往接線盒的電纜路徑', '{d} → {j}: cordset crosses {n}': '{d} → {j}：電纜跨越 {n}', '{t}: floor bridge crosses {n}': '{t}：地面線槽跨越 {n}',
  '{t}: unsupported span {l} m near ({x}, {y}) — add a support / portal or use a ladder tray': '{t}：({x}, {y}) 附近無支撐跨距 {l} m — 需加支架 / 門型架或改用梯式線槽', '{j} stands in {n}': '{j} 位於 {n} 內', '{j} stands in the vertical tray at ({x}, {y})': '{j} 位於 ({x}, {y}) 之垂直線槽內', '{j}: {n} devices > {p} ports': '{j}：{n} 個元件 > {p} 個埠', '{t}: overhead tray at z {z} m < 2.2 m headroom': '{t}：架空線槽 z {z} m < 淨高 2.2 m', '{t}: floor trunk crosses vehicle lane {n}': '{t}：地面線槽跨越車道 {n}',
  '{t}: tray at z {z} m over {n} (needs {c} m)': '{t}：線槽 z {z} m 位於 {n} 上方（需 {c} m）', '{n} is not on a cable tray (> {a} m)': '{n} 未連接線槽（> {a} m）',
  'No tray route {a} → {b}': '無線槽路徑 {a} → {b}', '{j}: cabinet {c} not found': '{j}：找不到電櫃 {c}', '{c} → robot {r}: {l} m > longest manipulator cable {m} m': '{c} → 機器人 {r}：{l} m > 最長本體電纜 {m} m',
  '{t}: fill {f} % > {m} %': '{t}：填充率 {f} % > {m} %', '{t} carries no cable': '{t} 無電纜', '{c}: service area blocked by {h}': '{c}：維修空間被 {h} 佔用',
} });
})();

/* ABB parallel delta robot module — IRB 360 FlexPicker (4D: three parallel arms + telescopic A4 shaft). Ceiling-mounted: the
   robot hangs from its mounting face; build option `pedestal` is the z of that face (the frame it hangs from).
   One module, many robots: RC.robots.delta.create(params, id) → independent instance; params = robot_pim.json (.delta, from the
   RobotStudio PIM.xml FwdKinInitData by pim_kinematics.py: base radius R, upper arm L, lower arm l, plate radius r, shoulder depth zs,
   flange below the plate ball plane, CAD-pose arm angle theta0, axis limits, working radius). The GLB is the visual only; its links
   (Base, UpperArm1..3, LowerArm<i>_1 / _2, LowerArmConnectorUpper / Lower<i>, MovingPlate, RevoluteLinkBase / Upper / Lower / Plate)
   are in the mechanism frame at the CAD pose: z DOWN from the mounting face, arm i along azimuth (i − 1) · 120°.
   Instance contract (as serial6 / scara): build(glbBuf, {pedestal, yaw, color, x, y}), ik(tcpBaseFrame) → {q1, q2, q3, reach},
   apply(q), jointText(), withinLimits, toolMount (origin = flange face, tool points −Z, local +X = tool yaw), armZ(rad) (null: the
   arms work above the TCP), setToolOffset(dz), plus armSegs() → world geometry of the upper / lower arms and the plate for sweeps.
   Pose convention: the tool points straight down, the plate stays horizontal (parallelograms); tcp.yaw is the world tool yaw and
   A4 = tcp.yaw − robot yaw (unwrapped: the station keeps tcp.yaw continuous). q1..q3 = geometric upper-arm angles below the horizontal;
   ABB joint value A<i> = q<i> − theta0 (the CAD pose is 0). Reach = IK solvable, A1..A3 inside the PIM limits, the flange inside the
   working radius and no stretched / folded arm (upper ↔ lower arm angle ≥ 8° from collinear). */
(function () {
'use strict';
const RC = window.RobotCell; const { clamp, deg, robotMats } = RC.h;

function create(params, id) {
  const RB = { R: 0.2, L: 0.35, l: 0.8, r: 0.045, zs: 0.275, flange: 0.042, theta0: -0.2301, workR: 0.566, w: 0.0472, tool: 0, top: 0.20, bot: 0.805 };
  const LIM = { A1: [-46.1, 100.2], A4: [-360000, 360000] };
  const P = params && params.delta;
  if (P) { for (const k of ['R', 'L', 'l', 'r', 'zs', 'flange', 'theta0', 'workR']) if (P[k] !== undefined) RB[k] = P[k]; if (P.limitsDeg) Object.assign(LIM, P.limitsDeg); }
  const PHI = [0, 2 * Math.PI / 3, 4 * Math.PI / 3];
  const U = PHI.map(f => ({ x: Math.cos(f), y: Math.sin(f) })), T = PHI.map(f => ({ x: -Math.sin(f), y: Math.cos(f) }));   // radial / tangential (mesh frame)
  const shoulder = i => ({ x: RB.R * U[i].x, y: RB.R * U[i].y, z: RB.zs });
  const elbow = (i, th) => ({ x: (RB.R + RB.L * Math.cos(th)) * U[i].x, y: (RB.R + RB.L * Math.cos(th)) * U[i].y, z: RB.zs + RB.L * Math.sin(th) });
  /* plate ball-plane centre (mesh frame, z down) → upper-arm angles; elbow-out branch */
  function solve(Pm) {
    const th = [], E = [], B = []; let ok = true, gap = Math.PI;
    for (let i = 0; i < 3; i++) {
      const b = { x: Pm.x + RB.r * U[i].x, y: Pm.y + RB.r * U[i].y, z: Pm.z }, S = shoulder(i);
      const dx = b.x - S.x, dy = b.y - S.y, dz = b.z - S.z, a = dx * U[i].x + dy * U[i].y, c = dz;
      const K = dx * dx + dy * dy + dz * dz + RB.L * RB.L - RB.l * RB.l, A = 2 * a * RB.L, Bc = 2 * c * RB.L, n = Math.hypot(A, Bc);
      const k = K / Math.max(n, 1e-9); if (Math.abs(k) > 1) ok = false;
      const t = Math.atan2(Bc, A) - Math.acos(clamp(k, -1, 1));
      th.push(t); const e = elbow(i, t); E.push(e); B.push(b);
      const ux = Math.cos(t), uz = Math.sin(t), lx = (b.x - e.x) * U[i].x + (b.y - e.y) * U[i].y, lz = b.z - e.z, ll = Math.hypot(lx, lz) || 1;
      gap = Math.min(gap, Math.acos(clamp((ux * lx + uz * lz) / ll, -1, 1)), Math.PI - Math.acos(clamp((ux * lx + uz * lz) / ll, -1, 1)));
    }
    return { th, E, B, ok, gap };
  }
  const P0 = { x: 0, y: 0, z: RB.zs + RB.L * Math.sin(RB.theta0) + Math.sqrt(RB.l * RB.l - Math.pow(RB.R + RB.L * Math.cos(RB.theta0) - RB.r, 2)) };   // CAD-pose plate
  const Z0 = solve(P0);
  const v3 = p => new THREE.Vector3(p.x, p.y, p.z);

  const R = {
    id: id || 'r1', model: (params && params.model) || 'delta', RB, LIM, yaw: 0, params: P ? 'PIM' : 'defaults', ceiling: true,
    setToolOffset(dz) { RB.tool = dz; },
    build(glbBuf, o) {
      R.yaw = o.yaw || 0;
      const body = o.color !== undefined ? o.color : 0xf4f4f2;
      const L = R.links = RC.parseGLB(glbBuf, robotMats(body));
      const root = R.root = new THREE.Group(); root.position.set(o.x || 0, o.y || 0, o.pedestal || 0); root.rotation.z = R.yaw; RC.scene.add(root);
      const mg = R.mg = new THREE.Group(); mg.rotation.x = Math.PI; root.add(mg);                      // mechanism frame (z down) → world
      if (L.Base) mg.add(L.Base); if (L.RevoluteLinkBase) mg.add(L.RevoluteLinkBase);
      const piv = (p, mesh) => { const g = new THREE.Group(); g.position.copy(p); const inner = new THREE.Group(); inner.position.copy(p).multiplyScalar(-1); g.add(inner); if (mesh) inner.add(mesh); mg.add(g); return g; };
      R.ua = [], R.bars = [], R.conU = [], R.conL = [];
      for (let i = 0; i < 3; i++) {
        R.ua.push(piv(v3(shoulder(i)), L['UpperArm' + (i + 1)]));
        const t = T[i];
        for (const k of [1, 2]) {                                                // which side of the arm a bar is on: from its mesh centre
          const m = L['LowerArm' + (i + 1) + '_' + k]; if (!m) continue;
          const c = new THREE.Box3().setFromObject(m).getCenter(new THREE.Vector3()), s = Math.sign((c.x - Z0.E[i].x) * t.x + (c.y - Z0.E[i].y) * t.y) || 1;
          const top0 = { x: Z0.E[i].x + s * RB.w * t.x, y: Z0.E[i].y + s * RB.w * t.y, z: Z0.E[i].z };
          R.bars.push({ i, s, g: piv(v3(top0), m) });
        }
        R.conU.push(piv(v3(Z0.E[i]), L['LowerArmConnectorUpper' + (i + 1)])); R.conL.push(piv(v3(Z0.B[i]), L['LowerArmConnectorLower' + (i + 1)]));
      }
      R.plate = piv(v3(P0), L.MovingPlate);
      R.rot = piv(v3({ x: 0, y: 0, z: RB.bot }), L.RevoluteLinkPlate);           // A4 flange shaft on the plate
      R.shU = piv(v3({ x: 0, y: 0, z: RB.top }), L.RevoluteLinkUpper); R.shL = piv(v3({ x: 0, y: 0, z: RB.bot }), L.RevoluteLinkLower);   // telescopic A4 shaft
      const tm = R.toolMount = new THREE.Object3D(); root.add(tm);                // world-aligned (local −Z = down), moved in apply()
      R._sol = Z0; R._P = P0; R.apply({ q1: Z0.th[0], q2: Z0.th[1], q3: Z0.th[2], yaw: R.yaw });
    },
    /* p: TCP in the robot base frame (base axis at x = y = 0, z = 0 on the mounting face, world axes). Tool vertical. */
    ik(p) {
      const c = Math.cos(-R.yaw), s = Math.sin(-R.yaw), x = p.x * c - p.y * s, y = p.x * s + p.y * c, zf = p.z + RB.tool;   // flange, robot frame (z up)
      const Pm = { x, y: -y, z: -zf - RB.flange };
      const sol = solve(Pm); R._sol = sol; R._P = Pm;
      const reach = sol.ok && R.withinLimits(sol.th[0], sol.th[1], sol.th[2]) && Math.hypot(x, y) <= RB.workR + 1e-6 && sol.gap > 8 * Math.PI / 180;
      return { q1: sol.th[0], q2: sol.th[1], q3: sol.th[2], reach };
    },
    withinLimits(a, b, c) { for (const t of [a, b, c]) { const A = deg(t - RB.theta0); if (A < LIM.A1[0] || A > LIM.A1[1]) return false; } return true; },
    apply(q) {
      const sol = R._sol, Pm = R._P;
      for (let i = 0; i < 3; i++) { const d = sol.th[i] - RB.theta0; R.ua[i].quaternion.setFromAxisAngle(new THREE.Vector3(T[i].x, T[i].y, 0), -d); }
      const qL = [];
      for (let i = 0; i < 3; i++) {
        const d0 = v3({ x: Z0.B[i].x - Z0.E[i].x, y: Z0.B[i].y - Z0.E[i].y, z: Z0.B[i].z - Z0.E[i].z }).normalize();
        const d1 = v3({ x: sol.B[i].x - sol.E[i].x, y: sol.B[i].y - sol.E[i].y, z: sol.B[i].z - sol.E[i].z }).normalize();
        qL.push(new THREE.Quaternion().setFromUnitVectors(d0, d1));
        R.conU[i].position.copy(v3(sol.E[i])); R.conU[i].quaternion.copy(qL[i]);
        R.conL[i].position.copy(v3(sol.B[i])); R.conL[i].quaternion.copy(qL[i]);
      }
      for (const b of R.bars) { const e = sol.E[b.i], t = T[b.i]; b.g.position.set(e.x + b.s * RB.w * t.x, e.y + b.s * RB.w * t.y, e.z); b.g.quaternion.copy(qL[b.i]); }
      const dP = v3({ x: Pm.x - P0.x, y: Pm.y - P0.y, z: Pm.z - P0.z });
      R.plate.position.copy(v3(P0)).add(dP);
      const a4 = (q.yaw !== undefined ? q.yaw : R.yaw) - R.yaw;
      R.rot.position.set(dP.x, dP.y, RB.bot + dP.z); R.rot.rotation.set(0, 0, -a4);
      const top = new THREE.Vector3(0, 0, RB.top), bot = new THREE.Vector3(dP.x, dP.y, RB.bot + dP.z), ax = bot.clone().sub(top).normalize();
      const qs = new THREE.Quaternion().setFromUnitVectors(new THREE.Vector3(0, 0, 1), ax);
      R.shU.quaternion.copy(qs); R.shL.position.copy(bot); R.shL.quaternion.copy(qs);
      R.toolMount.position.set(Pm.x, -Pm.y, -(Pm.z + RB.flange)); R.toolMount.rotation.set(0, 0, a4);
      R.q = { th: sol.th.slice(), a4 };
      R.A = { A1: deg(sol.th[0] - RB.theta0), A2: deg(sol.th[1] - RB.theta0), A3: deg(sol.th[2] - RB.theta0), A4: deg(a4) };
    },
    jointText() {
      const A = R.A || { A1: 0, A2: 0, A3: 0, A4: 0 };
      return `A1 ${A.A1.toFixed(0)}°  A2 ${A.A2.toFixed(0)}°  A3 ${A.A3.toFixed(0)}°  A4 ${A.A4.toFixed(0)}°`;
    },
    /* world geometry for sweeps: per arm the upper arm (shoulder → elbow) and the lower-arm pair centre line (elbow → plate ball),
       the plate centre and the flange — points {x, y, z}; the bars of a pair are ±w beside the centre line */
    armSegs() {
      const sol = R._sol, o = R.root.position, cy = Math.cos(R.yaw), sy = Math.sin(R.yaw);
      const W = p => ({ x: o.x + p.x * cy + p.y * sy, y: o.y + p.x * sy - p.y * cy, z: o.z - p.z });   // mesh frame → world (y and z flipped, then yaw)
      const arms = [0, 1, 2].map(i => ({ S: W(shoulder(i)), E: W(sol.E[i]), B: W(sol.B[i]) }));
      return { arms, plate: W(R._P), flange: W({ x: R._P.x, y: R._P.y, z: R._P.z + RB.flange }), w: RB.w };
    },
    armZ() { return null; },
  };
  return R;
}
const factory = { create };
for (const t of ['delta', 'irb360']) RC.robots[t] = factory;
})();

/* ABB IRB 660 (250/3.15) robot module: parallelogram palletizer, 4 axes. Also drives the IRB 460 (110/2.4, model IRB460_*):
   same link set without the balancing cylinder (LinkS4 / LinkS5 absent → skipped), P2 from the PIM.
   One module, many robots: RC.robots.irb660.create(params, id) returns an independent instance (own kinematics, limits,
   mesh, tool offset) — a station may hold several IRB 660s. `params` is that robot's robot_pim.json block.
   Link pivot defaults were measured from the RobotStudio GLB (base frame, arm plane x/z, metres); the PIM overrides them.
   Instance contract (any robot module must provide): build(glbBuf, {pedestal, yaw, color, x, y}), ik(tcpBaseFrame) → {q1,q2,q3,reach},
   apply(q), jointText(q), toolMount (Object3D the gripper hangs from, origin = flange face, tool points -Z), root, links. */
(function () {
'use strict';
const RC = window.RobotCell; const { pivotGroup, clamp, deg, robotMats } = RC.h;

function create(params, id) {
const RB = {
  S: { x: 0.311, z: 0.80 }, L2: 1.288, L3: 1.339,
  E0: { x: 0.311, z: 2.088 }, W0: { x: 1.65, z: 2.088 }, F0: { x: 1.908, z: 1.833 },
  tool: { dx: 0.258, dz: -0.455 },                 // TCP (cup face) relative to the wrist axis; dz includes the 0.2 m gripper
  C0: { x: -0.2, z: 0.80 }, rc: 0.511, T0rel: { x: -0.511, z: -0.01 }, Lr: 1.278,
  P2: { x: -0.117, y: -0.375, z: 0.954 },
  Pc: { x: 0.34, y: 0.27, z: 1.083 }, Pa0: { x: 0.30, y: 0.27, z: 2.10 },
};
function Ry(v, th) { const c = Math.cos(th), s = Math.sin(th); return { x: v.x * c + v.z * s, z: -v.x * s + v.z * c }; }
/* authoritative kinematics from the RobotStudio PIM.xml (this robot's robot_pim.json → window.ROBOT_PARAMS[.byRobot[id]].irb660)
   override the mesh-measured defaults above; joint limits come from the same file (ABB convention: A3 absolute, positive down) */
const LIM = { A1: 180, A2: [-42, 85], A3: [-20, 120], A3minusA2: [-70, 70], A4: 300 };
let wristToFlange = 0.255;
const P = params && params.irb660;
if (P) {
  Object.assign(RB.S, P.S); RB.L2 = P.L2; RB.L3 = P.L3; Object.assign(RB.E0, P.E0); Object.assign(RB.W0, P.W0); Object.assign(RB.F0, P.F0);
  RB.tool.dx = P.toolDx; wristToFlange = P.wristToFlange; RB.tool.dz = -wristToFlange;
  Object.assign(RB.C0, P.C0); RB.rc = P.rc; Object.assign(RB.T0rel, P.T0rel); RB.Lr = P.Lr;
  if (P.Pc && P.Pa0) { RB.Pc.x = P.Pc.x; RB.Pc.z = P.Pc.z; RB.Pa0.x = P.Pa0.x; RB.Pa0.z = P.Pa0.z; }
  if (P.P2) { RB.P2.x = P.P2.x; RB.P2.z = P.P2.z; }
  if (P.limitsDeg) Object.assign(LIM, P.limitsDeg);
}

const R = {
  id: id || 'r1', model: (params && params.model) || 'irb660', RB, yaw: 0,
  LIM, params: P ? 'PIM' : 'mesh',
  setToolOffset(dz) { RB.tool.dz = -wristToFlange - dz; },   // flange face is wristToFlange below the wrist axis; dz = tool thickness
  build(glbBuf, o) {
    R.yaw = o.yaw || 0;
    const body = o.color !== undefined ? o.color : 0xf4f4f2;
    const L = R.links = RC.parseGLB(glbBuf, robotMats(body));
    const root = R.root = new THREE.Group(); root.position.set(o.x || 0, o.y || 0, o.pedestal || 0); root.rotation.z = R.yaw; RC.scene.add(root);   // base at (x, y) for a 2nd robot
    root.add(L.Base);
    const turret = R.turret = new THREE.Group(); root.add(turret); turret.add(L.Link1);
    const P = R.piv = {};
    const mk = (name, piv) => { const g = pivotGroup(piv, turret); if (L[name]) g.userData.inner.add(L[name]); P[name] = g; return g; };
    mk('Link2', RB.S); mk('Link3', RB.E0); mk('LinkS3', RB.E0); mk('LinkS1', { x: 0, z: 0 }); mk('LinkS2', RB.P2);
    mk('Link4', { x: 0, z: 0 }); mk('Link6', RB.F0); mk('LinkD2', RB.S); mk('LinkD1', RB.C0); mk('LinkS5', RB.Pc); mk('LinkS4', RB.Pa0);
    R.toolMount = P.Link6;                            // group origin = flange axis at the flange face
  },
  ik(p) {                                             // p: TCP in the robot base frame (world yaw not applied)
    const q1 = Math.atan2(p.y, p.x);
    const r = Math.hypot(p.x, p.y);
    const wx = r - RB.tool.dx, wz = p.z - RB.tool.dz;
    const dx = wx - RB.S.x, dz = wz - RB.S.z;
    let d = Math.hypot(dx, dz);
    const dmax = RB.L2 + RB.L3 - 0.02, dmin = Math.abs(RB.L2 - RB.L3) + 0.05;
    const reach = !(d > dmax || d < dmin);
    d = clamp(d, dmin, dmax);
    const alpha = Math.atan2(dx, dz);
    const beta = Math.acos(clamp((RB.L2 * RB.L2 + d * d - RB.L3 * RB.L3) / (2 * RB.L2 * d), -1, 1));
    const q2 = alpha - beta;
    const E = { x: RB.S.x + RB.L2 * Math.sin(q2), z: RB.S.z + RB.L2 * Math.cos(q2) };
    const q3 = Math.atan2(wz - E.z, wx - E.x);
    return { q1, q2, q3, reach: reach && R.withinLimits(q1, q2, q3) };
  },
  withinLimits(q1, q2, q3, q4) {                       // ABB joint convention: A2 = q2, A3 = -q3 (positive down)
    const A2 = deg(q2), A3 = -deg(q3);
    if (A2 < LIM.A2[0] || A2 > LIM.A2[1]) return false;
    if (A3 < LIM.A3[0] || A3 > LIM.A3[1]) return false;
    if (A3 - A2 < LIM.A3minusA2[0] || A3 - A2 > LIM.A3minusA2[1]) return false;
    if (q4 !== undefined) { const A4 = deg(Math.atan2(Math.sin(q4), Math.cos(q4))); if (Math.abs(A4) > LIM.A4) return false; }
    return true;
  },
  apply(q) {
    const P = R.piv; const { q1, q2, q3, q4 } = q;
    R.turret.rotation.z = q1 - R.yaw;
    P.Link2.rotation.y = q2;
    const E = { x: RB.S.x + RB.L2 * Math.sin(q2), z: RB.S.z + RB.L2 * Math.cos(q2) };
    const W = { x: E.x + RB.L3 * Math.cos(q3), z: E.z + RB.L3 * Math.sin(q3) };
    P.Link3.position.set(E.x, 0, E.z); P.Link3.rotation.y = -q3;
    P.LinkS3.position.set(E.x, 0, E.z); P.LinkS3.rotation.y = -q3;
    P.LinkS1.position.set(E.x - RB.E0.x, 0, E.z - RB.E0.z);
    P.LinkS2.rotation.y = q2;
    const dW = { x: W.x - RB.W0.x, z: W.z - RB.W0.z };
    P.Link4.position.set(dW.x, 0, dW.z);
    P.Link6.position.set(RB.F0.x + dW.x, 0, RB.F0.z + dW.z); P.Link6.rotation.z = q4;
    // axis-3 crank + rear rod: rod from crank tip C (circle around S) to the upper-arm tail T
    const tr = Ry(RB.T0rel, -q3); const T = { x: E.x + tr.x, z: E.z + tr.z };
    const sx = T.x - RB.S.x, sz = T.z - RB.S.z; const dd = Math.hypot(sx, sz);
    const a = (RB.rc * RB.rc - RB.Lr * RB.Lr + dd * dd) / (2 * dd);
    const h = Math.sqrt(Math.max(0, RB.rc * RB.rc - a * a));
    const ux = sx / dd, uz = sz / dd; const px = RB.S.x + a * ux, pz = RB.S.z + a * uz;
    const c1 = { x: px + h * uz, z: pz - h * ux }, c2 = { x: px - h * uz, z: pz + h * ux };
    const C = c1.x < c2.x ? c1 : c2;
    P.LinkD2.rotation.y = Math.atan2(C.z - RB.S.z, -(C.x - RB.S.x));
    P.LinkD1.position.set(C.x, 0, C.z); P.LinkD1.rotation.y = Math.atan2(T.x - C.x, T.z - C.z);
    // balancing cylinder between the column pivot Pc and the lower-arm point Pa
    const par = Ry({ x: RB.Pa0.x - RB.S.x, z: RB.Pa0.z - RB.S.z }, q2);
    const Pa = { x: RB.S.x + par.x, z: RB.S.z + par.z };
    const phi = Math.atan2(Pa.x - RB.Pc.x, Pa.z - RB.Pc.z) - Math.atan2(RB.Pa0.x - RB.Pc.x, RB.Pa0.z - RB.Pc.z);
    P.LinkS5.rotation.y = phi;
    P.LinkS4.position.set(Pa.x, RB.Pa0.y, Pa.z); P.LinkS4.rotation.y = phi;
    R.E = E; R.W = W;
  },
  jointText(q) {
    const wrap = a => Math.atan2(Math.sin(a), Math.cos(a));
    return `A1 ${deg(wrap(q.q1 - R.yaw)).toFixed(1)}°  A2 ${deg(q.q2).toFixed(1)}°  A3 ${deg(q.q3).toFixed(1)}°  A4 ${deg(wrap(q.q4)).toFixed(1)}°`;
  },
  /* clearance helper used by verification scripts: z of the lower/upper arm centre-line at radial distance `rad`
     in the arm plane for the current pose, or null when outside the arm's span */
  armZ(rad) {
    const E = R.E, W = R.W; if (!E) return null;
    if (rad > RB.S.x && rad < E.x) return { z: RB.S.z + (rad - RB.S.x) / (E.x - RB.S.x) * (E.z - RB.S.z), half: 0.21, part: 'lower' };
    if (rad >= E.x && rad < W.x) return { z: E.z + (rad - E.x) / (W.x - E.x) * (W.z - E.z), half: 0.15, part: 'upper' };
    return null;
  },
};
return R;
}
RC.robots.irb660 = { create };
})();

/* ABB SCARA robot module — IRB 910SC / IRB 920T / IRB 930: A1, A2 revolute about +Z, A3 prismatic (ball-screw spline, negative = down),
   A4 = spline rotation (tool yaw). One module, many robots: RC.robots.scara.create(params, id) returns an independent instance.
   `params` is that robot's robot_pim.json (.scara, extracted from the RobotStudio PIM.xml: link lengths, flange height at
   A3 = 0, stroke, joint limits); the GLB is the visual only (links Base, Link1, Link2, Link3 (none on the IRB 930), Link4 =
   spline shaft, all at the zero pose: both arms along +X). Type aliases irb930 / irb910sc map to this factory.
   Instance contract (as serial6): build(glbBuf, {pedestal, yaw, color, x, y, elbow}), ik(tcpBaseFrame) → {q1, q2, q3, reach},
   apply(q), jointText(q), withinLimits, toolMount (origin = flange face, tool points −Z, local +X = tool yaw), armZ(rad),
   setToolOffset(dz), plus armSegs() → world geometry of link 1, link 2 and the spline shaft for verification sweeps.
   Pose convention: the tool points straight down; tcp.yaw is the world yaw of the tool (A4 counter-rotates A1 + A2).
   q1 = world azimuth of link 1, q2 = A2, q3 = A3 (m). Arm configuration: elbow = +1 (A2 > 0) or −1 (A2 < 0) — a SCARA cannot
   change it on a Cartesian path (it passes the stretched-arm singularity), so a station keeps one and checks A1 over the cycle. */
(function () {
'use strict';
const RC = window.RobotCell; const { pivotGroup, clamp, deg, robotMats } = RC.h;
const wrap = a => Math.atan2(Math.sin(a), Math.cos(a));

function create(params, id) {
  const RB = { L1: 0.56, L2: 0.49, flangeZ0: 0.2742, stroke: [-0.45, 0], shaftLen: 0.7315, tool: 0, elbow: 1,   // IRB 930-22/1.05/0.45 defaults
               link1: { z0: 0.2553, z1: 0.4202, half: 0.118, back: 0.109 }, link2: { z0: 0.237, z1: 0.668, half: 0.105, back: 0.104, fwd: 0.088 }, shaftR: 0.027 };
  const LIM = { A1: [-145, 145], A2: [-145, 145], A4: [-400, 400] };
  const P = params && params.scara;
  if (P) {
    RB.L1 = P.L1; RB.L2 = P.L2; RB.flangeZ0 = P.flangeZ0; RB.stroke = P.stroke.slice(); RB.shaftLen = P.shaftLen;
    if (P.limitsDeg) Object.assign(LIM, P.limitsDeg);
    const L = params.links || {};                                              // arm envelopes from the PIM link bounding boxes
    if (L.Link1 && L.Link1.bbox) { const b = L.Link1.bbox; RB.link1 = { z0: b.Min[2], z1: b.Max[2], half: Math.max(-b.Min[1], b.Max[1]), back: -b.Min[0] }; }
    if (L.Link2 && L.Link2.bbox) { const b = L.Link2.bbox; RB.link2 = { z0: b.Min[2], z1: b.Max[2], half: Math.max(-b.Min[1], b.Max[1]), back: RB.L1 - b.Min[0], fwd: b.Max[0] - RB.L1 - RB.L2 }; }
  }

  const R = {
    id: id || 'r1', model: (params && params.model) || 'scara', RB, LIM, yaw: 0, params: P ? 'PIM' : 'defaults',
    setToolOffset(dz) { RB.tool = dz; },
    setElbow(s) { RB.elbow = s < 0 ? -1 : 1; },
    build(glbBuf, o) {
      R.yaw = o.yaw || 0; if (o.elbow) R.setElbow(o.elbow);
      const body = o.color !== undefined ? o.color : 0xf4f4f2;
      const L = R.links = RC.parseGLB(glbBuf, robotMats(body));
      const root = R.root = new THREE.Group(); root.position.set(o.x || 0, o.y || 0, o.pedestal || 0); root.rotation.z = R.yaw; RC.scene.add(root);
      if (L.Base) root.add(L.Base);
      const j1 = R.j1 = new THREE.Group(); root.add(j1); if (L.Link1) j1.add(L.Link1);            // A1 about the base axis
      const j2 = R.j2 = pivotGroup({ x: RB.L1, z: 0 }, j1); if (L.Link2) j2.userData.inner.add(L.Link2);   // A2 at the elbow
      const j3 = R.j3 = new THREE.Group(); j2.userData.inner.add(j3); if (L.Link3) j3.add(L.Link3);    // A3: spline translation along Z
      const j4 = R.j4 = pivotGroup({ x: RB.L1 + RB.L2, z: 0 }, j3); if (L.Link4) j4.userData.inner.add(L.Link4);   // A4: spline rotation
      const tm = R.toolMount = new THREE.Object3D(); tm.position.set(RB.L1 + RB.L2, 0, RB.flangeZ0); j4.userData.inner.add(tm);
    },
    /* p: TCP in the robot base frame (base axis at x = y = 0, z = 0 on the mounting face). Tool vertical → flange = TCP + tool. */
    ik(p) {
      const q3 = p.z + RB.tool - RB.flangeZ0;
      const r = Math.hypot(p.x, p.y), c2 = (r * r - RB.L1 * RB.L1 - RB.L2 * RB.L2) / (2 * RB.L1 * RB.L2);
      const q2 = RB.elbow * Math.acos(clamp(c2, -1, 1));
      const q1 = Math.atan2(p.y, p.x) - Math.atan2(RB.L2 * Math.sin(q2), RB.L1 + RB.L2 * Math.cos(q2));
      return { q1, q2, q3, reach: Math.abs(c2) <= 1 && R.withinLimits(q1, q2, q3) };
    },
    withinLimits(q1, q2, q3) {
      const A1 = deg(wrap(q1 - R.yaw)), A2 = deg(q2);
      return A1 >= LIM.A1[0] && A1 <= LIM.A1[1] && A2 >= LIM.A2[0] && A2 <= LIM.A2[1] && q3 >= RB.stroke[0] - 1e-6 && q3 <= RB.stroke[1] + 1e-6;
    },
    apply(q) {
      const a1 = wrap(q.q1 - R.yaw);
      R.j1.rotation.z = a1; R.j2.rotation.z = q.q2; R.j3.position.z = q.q3;
      /* spline turn relative to link 2 → tool yaw = tcp.yaw (world); built from the continuous A1 (never an atan2 branch), so A4
         is as continuous as tcp.yaw and never winds up: a station keeps A4 bounded by choosing the yaw representation (toolYaw) */
      const t4 = (q.yaw || 0) - (a1 + R.yaw) - q.q2;
      R.j4.rotation.z = t4;
      R.q = { a1, a2: q.q2, a3: q.q3, t4 };
      R.A = { A1: deg(a1), A2: deg(q.q2), A3: q.q3 * 1000, A4: -deg(t4) };   // A4: ABB sign (joint axis points down, DH twist −π)
    },
    /* the representation of a world tool yaw (target + 2πk) that gives |A4| ≤ 180° at TCP p (base frame) — pass it as tcp.yaw */
    toolYaw(yaw, p) {
      const k = R.ik(p); const base = wrap(k.q1 - R.yaw) + R.yaw + k.q2;
      return base + wrap(yaw - base);
    },
    jointText() {
      const A = R.A || { A1: 0, A2: 0, A3: 0, A4: 0 };
      return `A1 ${A.A1.toFixed(0)}°  A2 ${A.A2.toFixed(0)}°  A3 ${A.A3.toFixed(0)} mm  A4 ${A.A4.toFixed(0)}°`;
    },
    /* world geometry of the arm for sweeps: link 1 / link 2 as horizontal bars (segment a → b, half-width, z0..z1) and the
       spline shaft as a vertical cylinder (x, y, r, z0..z1). Link 2 reaches `fwd` past the spline axis (motor housing). */
    armSegs() {
      if (!R.q) return null;
      const o = R.root.position, cy = Math.cos(R.yaw), sy = Math.sin(R.yaw);
      const W = (x, y) => ({ x: o.x + x * cy - y * sy, y: o.y + x * sy + y * cy });
      const a1 = R.q.a1, a12 = a1 + R.q.a2, u1 = [Math.cos(a1), Math.sin(a1)], u2 = [Math.cos(a12), Math.sin(a12)];
      const E = [RB.L1 * u1[0], RB.L1 * u1[1]], Fl = [E[0] + RB.L2 * u2[0], E[1] + RB.L2 * u2[1]];
      const l1 = RB.link1, l2 = RB.link2, fz = o.z + RB.flangeZ0 + R.q.a3, f = W(Fl[0], Fl[1]);
      return {
        link1: { a: W(-l1.back * u1[0], -l1.back * u1[1]), b: W(E[0], E[1]), half: l1.half, z0: o.z + l1.z0, z1: o.z + l1.z1 },
        link2: { a: W(E[0] - l2.back * u2[0], E[1] - l2.back * u2[1]), b: W(Fl[0] + l2.fwd * u2[0], Fl[1] + l2.fwd * u2[1]), half: l2.half, z0: o.z + l2.z0, z1: o.z + l2.z1 },
        shaft: { x: f.x, y: f.y, r: RB.shaftR, z0: fz, z1: fz + RB.shaftLen },
        elbow: W(E[0], E[1]), flange: Object.assign(f, { z: fz }),
      };
    },
    /* contract helper (serial6 / irb660 style): centre-line z of the arm (base frame) at radial distance rad in the arm plane —
       a SCARA's arms are horizontal, so this is the link-2 band whenever rad is inside the reach */
    armZ(rad) {
      if (rad > RB.L1 + RB.L2 + RB.link2.fwd) return null;
      const l = rad < RB.L1 ? RB.link1 : RB.link2;
      return { z: (l.z0 + l.z1) / 2, half: (l.z1 - l.z0) / 2, part: rad < RB.L1 ? 'link 1' : 'link 2' };
    },
  };
  return R;
}
const factory = { create };
for (const t of ['scara', 'irb930', 'irb910sc', 'irb920']) RC.robots[t] = factory;
})();

/* ABB 6-axis articulated arm (spherical wrist) robot module — IRB 6700 / 4600 / 1300 / 2600 / 6640 style arms.
   One module, many robots: RC.robots.serial6.create(params, id) returns an independent instance (own link lengths, joint
   limits, mesh, tool offset), so a station may hold several arms of the same or different models. `params` is that
   robot's robot_pim.json (window.ROBOT_PARAMS.byRobot[<glb key>] → .serial6, extracted from the RobotStudio PIM.xml);
   the GLB mesh is the visual only (links Base, Link1..Link6, LinkD1/LinkD2 = balancing cylinder when the arm has one).
   Type aliases irb6700 / irb4600 / irb1300 / irb2600 / irb6640 all map to this factory.
   Instance contract: build(glbBuf, {pedestal, yaw, color, x, y}), ik(tcpBaseFrame) → {q1,q2,q3,reach}, apply(q), jointText(q),
   withinLimits, toolMount (origin = flange face, tool points -Z), armZ(rad), root, links, setToolOffset(dz).
   Pose convention: the tool always points straight down (pick & place); A4 = 0, A5 keeps the tool vertical, A6 gives the
   world yaw (core passes q4 = yaw + π − A1). q2 = A2, q3 = A3 (ABB signs).
   setToolMode('side'): the tool axis is HORIZONTAL along the TCP yaw (flange facing sideways, e.g. a fork that A6 turns
   over): the TCP is on the A6 axis, the wrist centre lies flangeLen + tool behind it, A4 / A5 point the flange along the
   yaw (the branch closest to the previous A4 is kept), A6 = roll about the tool axis with roll 0 = the tool mount's local
   +X up (core passes q.yaw, q.roll = tcp.roll).
   setToolMode('tilt'): the tool axis leans tcp.tilt (rad) from vertical-down towards the azimuth tcp.yaw (weld torch, scanner); the wrist
   centre is TCP − (flange + tool) · axis, A4 / A5 point the flange along it (wristDir), A6 keeps the down-mode roll carried along by the
   tilt — so tilt 0 is exactly the down mode and a tool can lean in and out of vertical without a jump (core passes q.tilt). */
(function () {
'use strict';
const RC = window.RobotCell; const { pivotGroup, clamp, deg, robotMats } = RC.h;
const wrap = a => Math.atan2(Math.sin(a), Math.cos(a));
function Ry(v, th) { const c = Math.cos(th), s = Math.sin(th); return { x: v.x * c + v.z * s, z: -v.x * s + v.z * c }; }

function create(params, id) {
  const RB = {                                        // defaults = IRB 6700-150/3.20 zero pose (metres, arm plane x/z)
    S: { x: 0.32, z: 0.78 }, L2: 1.28, a3: 0.2, d4: 1.5925, E0: { x: 0.32, z: 2.06 }, W0: { x: 1.9125, z: 2.26 }, F0: { x: 2.1125, z: 2.26 },
    flangeLen: 0.2, wristOff: 0, tool: 0,                         // tool = TCP distance below the flange face (gripper length), set by setToolOffset
    balancer: null, meshOffsets: {},
  };
  const LIM = { A1: [-170, 170], A2: [-65, 85], A3: [-180, 70], A4: [-300, 300], A5: [-130, 130], A6: [-360, 360] };
  const P = params && params.serial6;
  if (P) {
    Object.assign(RB.S, P.S); RB.L2 = P.L2; RB.a3 = P.a3; RB.d4 = P.d4; Object.assign(RB.E0, P.E0); Object.assign(RB.W0, P.W0); Object.assign(RB.F0, P.F0);
    RB.flangeLen = P.flangeLen; RB.wristOff = P.wristOff || 0; if (P.balancer) RB.balancer = P.balancer; if (P.meshOffsets) RB.meshOffsets = P.meshOffsets;
    if (P.limitsDeg) Object.assign(LIM, P.limitsDeg);
  } else RB.balancer = { C: { x: -0.349, z: 0.638 }, A0: { x: 0.1113, z: 0.7356 } };
  RB.L3 = Math.hypot(RB.d4, RB.a3); RB.eOff = Math.atan2(RB.a3, RB.d4);   // elbow → wrist-centre distance and its angle above the forearm axis

  const R = {
    id: id || 'r1', model: (params && params.model) || 'serial6', RB, LIM, yaw: 0, params: P ? 'PIM' : 'defaults',
    setToolOffset(dz) { RB.tool = dz; },
    toolMode: 'down', setToolMode(m) { R.toolMode = m === 'side' || m === 'tilt' ? m : 'down'; },
    /* tool axis (flange → TCP) as a world unit vector: down = −Z; side = horizontal along yaw; tilt = leaned tcp.tilt (rad) from
       vertical-down towards the azimuth yaw (tilt 0 = down mode, same A4 / A5 / A6) — torches, scanners, angled grippers */
    toolDir(yaw, tilt) {
      if (R.toolMode === 'side') return { x: Math.cos(yaw), y: Math.sin(yaw), z: 0 };
      if (R.toolMode === 'tilt') { const s = Math.sin(tilt || 0); return { x: s * Math.cos(yaw), y: s * Math.sin(yaw), z: -Math.cos(tilt || 0) }; }
      return { x: 0, y: 0, z: -1 };
    },
    /* A4 / A5 that point the flange along the world unit vector u (arm-plane frame: x radial, y tangential, z up); forearm pitch
       φ = q3 − q2. A5 bends the flange from the forearm axis f towards n = (sin φ, 0, −cos φ), A4 rolls that bend about f:
       u = cos A5 · f + sin A5 · (cos A4 · n + sin A4 · y). Two branches (A4, A5) / (A4 + π, −A5): keep the one nearest prev. */
    wristDir(q1, q2, q3, u, prevA4) {
      const ph = q3 - q2, c = Math.cos(q1), s = Math.sin(q1);
      const ux = u.x * c + u.y * s, uy = -u.x * s + u.y * c, uz = u.z;
      const a5 = Math.acos(clamp(ux * Math.cos(ph) + uz * Math.sin(ph), -1, 1));
      if (Math.abs(Math.sin(a5)) < 1e-4) return { A4: prevA4 || 0, A5: a5 };                 // wrist singular: keep A4, A6 rolls
      const cn = ux * Math.sin(ph) - uz * Math.cos(ph);
      const s1 = { A4: Math.atan2(uy, cn), A5: a5 }, s2 = { A4: Math.atan2(-uy, -cn), A5: -a5 };
      const p = prevA4 || 0, d = a => Math.abs(wrap(a - p));
      return d(s1.A4) <= d(s2.A4) ? s1 : s2;
    },
    wristSide(q1, q2, q3, yaw, prevA4) { return R.wristDir(q1, q2, q3, { x: Math.cos(yaw), y: Math.sin(yaw), z: 0 }, prevA4); },
    build(glbBuf, o) {
      R.yaw = o.yaw || 0; R.a6Raw = !!o.a6Raw;                                 // a6Raw: down-mode A6 = π − q4 without wrapping (station keeps tcp.yaw continuous; robots with A6 ±270°)
      const body = o.color !== undefined ? o.color : 0xf4f4f2;
      const L = R.links = RC.parseGLB(glbBuf, robotMats(body), RB.meshOffsets);
      const root = R.root = new THREE.Group(); root.position.set(o.x || 0, o.y || 0, o.pedestal || 0); root.rotation.z = R.yaw; RC.scene.add(root);   // base at (x, y)
      root.add(L.Base);
      const turret = R.turret = new THREE.Group(); root.add(turret); turret.add(L.Link1);
      const Pv = R.piv = {};
      const mk = (name, piv, parent) => { const g = pivotGroup(piv, parent); if (L[name]) g.userData.inner.add(L[name]); Pv[name] = g; return g; };
      const g2 = mk('Link2', RB.S, turret);                                   // A2 about Y at the shoulder
      const g3 = mk('Link3', RB.E0, g2.userData.inner);                       // A3 about Y at the elbow
      const g4 = mk('Link4', { x: RB.E0.x, z: RB.W0.z }, g3.userData.inner);  // A4 roll about the forearm axis
      const g5 = mk('Link5', RB.W0, g4.userData.inner);                       // A5 about Y at the wrist centre
      const g6 = mk('Link6', { x: RB.W0.x, z: RB.W0.z + RB.wristOff }, g5.userData.inner);   // A6 roll about the tool axis (offset wrist: wristOff above A4)
      if (RB.balancer) { mk('LinkD1', RB.balancer.C, turret); mk('LinkD2', RB.balancer.A0, turret); }
      const tm = R.toolMount = new THREE.Object3D(); tm.position.set(RB.F0.x, 0, RB.F0.z); tm.rotation.y = -Math.PI / 2;   // local -Z = tool axis (+X native)
      g6.userData.inner.add(tm);
    },
    /* p: TCP in the robot base frame (base at origin, pedestal removed). Tool vertical → wrist centre = TCP + tool + flange above. */
    ik(p, tcp) {
      const side = R.toolMode !== 'down', yaw = tcp ? tcp.yaw : 0, back = RB.tool + RB.flangeLen;
      const u = R.toolDir(yaw, tcp && tcp.tilt);
      const wx = p.x - back * u.x, wy = p.y - back * u.y;                     // wrist centre = TCP − (tool + flange) · tool axis
      const q1 = Math.atan2(wy, wx);
      /* offset wrist (GoFa): in down mode (A4 0) A5 turns the zero-pose offset (+z) to +radial, so the A6 axis stands wristOff radially
         OUTSIDE the wrist centre on the same ray. Side / tilt modes assume a spherical wrist. */
      const r = Math.hypot(wx, wy) - (side ? 0 : RB.wristOff), wz = p.z - back * u.z;
      const dx = r - RB.S.x, dz = wz - RB.S.z;
      let d = Math.hypot(dx, dz);
      const dmax = RB.L2 + RB.L3 - 0.02, dmin = Math.abs(RB.L2 - RB.L3) + 0.05;
      const reach = !(d > dmax || d < dmin);
      d = clamp(d, dmin, dmax);
      const alpha = Math.atan2(dx, dz);
      const beta = Math.acos(clamp((RB.L2 * RB.L2 + d * d - RB.L3 * RB.L3) / (2 * RB.L2 * d), -1, 1));
      const q2 = alpha - beta;                                                 // elbow-up solution, A2 from vertical, + forward
      const E = { x: RB.S.x + RB.L2 * Math.sin(q2), z: RB.S.z + RB.L2 * Math.cos(q2) };
      const phiEW = Math.atan2(wz - E.z, r - E.x);                             // elbow → wrist centre, from horizontal, + up
      const q3 = phiEW - RB.eOff + q2;                                         // forearm axis angle (phiEW − eOff) is A3 − A2
      const a5 = side ? R.wristDir(q1, q2, q3, u, R._A4).A5 : undefined;
      return { q1, q2, q3, reach: reach && R.withinLimits(q1, q2, q3, a5) };
    },
    a5(q2, q3) { return q3 - q2 + Math.PI / 2; },                             // wrist pitch that keeps the tool vertical
    withinLimits(q1, q2, q3, a5) {
      const A1 = deg(wrap(q1 - R.yaw)), A2 = deg(q2), A3 = deg(q3), A5 = deg(a5 !== undefined ? a5 : R.a5(q2, q3));
      return A1 >= LIM.A1[0] && A1 <= LIM.A1[1] && A2 >= LIM.A2[0] && A2 <= LIM.A2[1] && A3 >= LIM.A3[0] && A3 <= LIM.A3[1] && A5 >= LIM.A5[0] && A5 <= LIM.A5[1];
    },
    apply(q) {
      const Pv = R.piv; const { q1, q2, q3, q4 } = q;
      R.turret.rotation.z = q1 - R.yaw;
      Pv.Link2.rotation.y = q2;
      Pv.Link3.rotation.y = -q3;
      let A4 = 0, A5, A6;
      if (R.toolMode === 'side') {
        const w = R.wristSide(q1, q2, q3, q.yaw, R._A4); A4 = w.A4; A5 = w.A5; R._A4 = A4;
        Pv.Link4.rotation.x = A4; Pv.Link5.rotation.y = A5; Pv.Link6.rotation.x = 0;
        /* A6: turn the tool mount's local +X to world up (the tool axis is horizontal, so up ⊥ axis), then add the roll;
           unwrapped to the previous A6 so a 180° roll turns the short way the program asked for */
        R.toolMount.updateWorldMatrix(true, false); const e = R.toolMount.matrixWorld.elements;
        const a = Math.atan2(e[6], e[2]);                                      // world z in the (local X, local Y) basis
        A6 = -a + (q.roll || 0); if (R._A6 !== undefined) A6 += 2 * Math.PI * Math.round((R._A6 - A6) / (2 * Math.PI)); R._A6 = A6;
        Pv.Link6.rotation.x = A6;
      } else if (R.toolMode === 'tilt') {
        /* roll reference: the tool's local X in the down-mode pose (A4 0, A5 vertical, A6 = π − q4), carried along by the tilt
           rotation (axis ⊥ yaw, horizontal) — so tilt 0 is exactly the down mode and the roll stays continuous as the tool leans */
        const t = q.tilt || 0;
        Pv.Link4.rotation.x = 0; Pv.Link5.rotation.y = R.a5(q2, q3); Pv.Link6.rotation.x = wrap(Math.PI - q4);
        R.toolMount.updateWorldMatrix(true, false); let e = R.toolMount.matrixWorld.elements;
        const X = new THREE.Vector3(e[0], e[1], e[2]).normalize();
        if (Math.abs(t) > 1e-6) X.applyAxisAngle(new THREE.Vector3(Math.sin(q.yaw), -Math.cos(q.yaw), 0), t);
        const w = R.wristDir(q1, q2, q3, R.toolDir(q.yaw, t), R._A4); A4 = w.A4; A5 = w.A5; R._A4 = A4;
        Pv.Link4.rotation.x = A4; Pv.Link5.rotation.y = A5; Pv.Link6.rotation.x = 0;
        R.toolMount.updateWorldMatrix(true, false); e = R.toolMount.matrixWorld.elements;
        const a = Math.atan2(X.x * e[4] + X.y * e[5] + X.z * e[6], X.x * e[0] + X.y * e[1] + X.z * e[2]);
        A6 = -a + (q.roll || 0); if (R._A6 !== undefined) A6 += 2 * Math.PI * Math.round((R._A6 - A6) / (2 * Math.PI)); R._A6 = A6;
        Pv.Link6.rotation.x = A6;
      } else {
        Pv.Link4.rotation.x = 0;
        A5 = R.a5(q2, q3); Pv.Link5.rotation.y = A5;
        A6 = R.a6Raw ? Math.PI - q4 : wrap(Math.PI - q4); Pv.Link6.rotation.x = A6;                     // tool yaw: gripper local X has world azimuth A1 − A6
      }
      if (RB.balancer) {                                                       // balancing cylinder: body pivots at C on Link1, rod follows the attachment point A (on Link2)
        const B = RB.balancer; const rel = Ry({ x: B.A0.x - RB.S.x, z: B.A0.z - RB.S.z }, q2);
        const A = { x: RB.S.x + rel.x, z: RB.S.z + rel.z };
        const th0 = Math.atan2(B.A0.z - B.C.z, B.A0.x - B.C.x), th = Math.atan2(A.z - B.C.z, A.x - B.C.x);
        Pv.LinkD1.rotation.y = th0 - th;
        Pv.LinkD2.position.set(A.x, 0, A.z); Pv.LinkD2.rotation.y = th0 - th;
      }
      R.E = { x: RB.S.x + RB.L2 * Math.sin(q2), z: RB.S.z + RB.L2 * Math.cos(q2) };
      const phi = q3 - q2;                                                     // forearm axis angle from horizontal
      R.W = { x: R.E.x + RB.L3 * Math.cos(phi + RB.eOff), z: R.E.z + RB.L3 * Math.sin(phi + RB.eOff) };
      R.A = { A1: deg(wrap(q1 - R.yaw)), A2: deg(q2), A3: deg(q3), A4: deg(A4), A5: deg(A5), A6: deg(A6) };
    },
    jointText(q) {
      const A = R.A || { A1: 0, A2: 0, A3: 0, A4: 0, A5: 0, A6: 0 };
      return `A1 ${A.A1.toFixed(0)}°  A2 ${A.A2.toFixed(0)}°  A3 ${A.A3.toFixed(0)}°  A4 ${A.A4.toFixed(0)}°  A5 ${A.A5.toFixed(0)}°  A6 ${A.A6.toFixed(0)}°`;
    },
    /* clearance helper for verification scripts: z of the arm centre-line at radial distance `rad` (arm plane, current pose),
       or null outside the arm's span. Half-thicknesses scale with the upper-arm length (IRB 6700: ≈ 0.25 / 0.28 m). */
    armZ(rad) {
      const E = R.E, W = R.W; if (!E) return null; const k = RB.L2 / 1.28;
      if (rad > RB.S.x && rad < E.x) return { z: RB.S.z + (rad - RB.S.x) / (E.x - RB.S.x) * (E.z - RB.S.z), half: 0.25 * k, part: 'upper arm' };
      if (rad >= E.x && rad < W.x) return { z: E.z + (rad - E.x) / (W.x - E.x) * (W.z - E.z), half: 0.28 * k, part: 'forearm' };
      return null;
    },
  };
  return R;
}
const factory = { create };
for (const t of ['serial6', 'irb6700', 'irb4600', 'irb1300', 'irb2600', 'irb6640']) RC.robots[t] = factory;
})();

/* Station: logistics application — goods-to-robot order fulfilment, after the ABB Robotics video "ABB Robotics | Logistics"
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
const NSKU = 40, BINS_PER_SKU = 3;
const SKUS = []; for (let i = 0; i < NSKU; i++) SKUS.push({ i, code: 'SKU-' + String(1001 + i), size: ['S', 'M', 'L'][i % 5 === 4 ? 2 : i % 3 === 2 ? 1 : 0], lab: LABELS[i % LABELS.length] });
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
  it.free = false; it.sku = sku; it.lab.material = M.labs[LABELS.indexOf(SKUS[sku].lab)]; it.g.visible = true; it.g.rotation.set(0, 0, 0);
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
  if (L.id === 'pick' && (z.k === 'P1' || z.k === 'P2') && b.station === z.k) { allocate(b); ST[z.k].valid = false; if (!b.picks.length) b.done = true; b.arrT = S.t; S.stat.presented++; }
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

// ------------------------------------------------------------------ WMS: orders (rolling waves on the put wall), demand, allocation
function zipf(n) { const w = []; let s = 0; for (let i = 0; i < n; i++) { const v = S.mode === 'flat' ? 1 : 1 / Math.pow(i + 1, 0.9); w.push(v); s += v; } return w.map(v => v / s); }
let W = null;
function drawSku(excl) { for (let t = 0; t < 50; t++) { let r = Math.random(), i = 0; while (i < W.length - 1 && r > W[i]) { r -= W[i]; i++; } if (!excl.includes(i)) return i; } return excl.length ? (excl[0] + 1) % NSKU : 0; }
function newOrder() {
  const nl = 1 + Math.floor(Math.pow(Math.random(), 1.6) * S.linesMax), lines = [], used = [];
  for (let k = 0; k < nl; k++) { const sku = drawSku(used); used.push(sku); lines.push({ sku, qty: 1 + Math.floor(Math.pow(Math.random(), 2.2) * S.qtyMax), alloc: 0, picked: 0, sorted: 0 }); }
  return { id: ++ORDERS.seq, lines, lane: null, tA: 0, items: lines.reduce((a, l) => a + l.qty, 0) };
}
function activateOrders() {
  while (ORDERS.queue.length < 12) ORDERS.queue.push(newOrder());
  for (const ln of LANES) if (ln.state === 'ready' && !ln.order) { const o = ORDERS.queue.shift(); o.lane = ln; o.tA = S.t; ln.order = o; ln.state = 'active'; ORDERS.active.push(o); ORDERS.queue.push(newOrder()); }
}
const openQty = sku => ORDERS.active.reduce((a, o) => a + o.lines.reduce((c, l) => c + (l.sku === sku ? l.qty - l.alloc : 0), 0), 0);
/* bins of a SKU already on their way to a picking stop (or carried by R1 towards it) cover the open quantity */
function covered(sku) {
  let c = 0; const add = b => { if (b && b.sku === sku && !b.allocated) c += b.n; };
  for (const z of LOOPS.pick.zones) add(z.bin);
  const r = ctx('r1'); if (r.task && r.task.kind === 'retrieve') add(r.task.b);
  return c;
}
function nextNeed() {
  for (const o of ORDERS.active) for (const l of o.lines) { if (l.qty - l.alloc <= 0) continue; const q = openQty(l.sku); if (q > covered(l.sku) && BINS.some(b => b.sku === l.sku && b.where === 'rack' && b.n > 0 && !b.res)) return l.sku; }
  return -1;
}
function allocate(b) {
  b.picks = []; b.allocated = true;
  for (const o of ORDERS.active) for (const l of o.lines) { if (l.sku !== b.sku) continue; while (l.qty - l.alloc > 0 && b.picks.length < b.n) { l.alloc++; b.picks.push({ o, l }); } }
  b.pi = 0;
}
/* decision point before P1, taken when the bin can leave it (P1 free): P2 unless a bin is already at / heading to P2, else P1 — a bin
   for P1 never waits behind one for P2; no open demand → straight back (choosing on arrival sent most bins to P1 and left R3 idle) */
function decide(b) {
  if (openQty(b.sku) <= 0) { b.station = null; return true; }
  const P = LOOPS.pick, free = k => { const z = P.zones[zoneOf(P, k)]; return !z.bin && !z.res; }, z4 = P.zones[zoneOf(P, 'P1') + 1];
  const busy2 = P.zones.some(z => z.bin && z.bin !== b && z.bin.station === 'P2' && !z.bin.done);
  if (!free('P1')) return false;
  b.station = busy2 ? 'P1' : 'P2'; return true;
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
function chooseR1() {
  const c = ctx('r1'), cur = Math.atan2(c.tcp.y, c.tcp.x);
  const hin = headOf('pick', 'HIN'), rin = headOf('rst', 'HIN'), hout = headOf('pick', 'H'), rout = headOf('rst', 'H');
  const arrived = h => h.z.bin && !h.z.bin.mv;
  const outFree = h => !h.z.bin && !h.z.res;
  if (arrived(hin)) { const b = hin.z.bin; if (b.n <= 0 && outFree(rout) && rstCount() < 4) return { kind: 'toRst', b, src: hin, dst: rout }; const s = freeSlot(cur); if (s) return { kind: 'store', b, src: hin, slot: s }; }
  if (arrived(rin)) { const s = freeSlot(cur); if (s) return { kind: 'storeRst', b: rin.z.bin, src: rin, slot: s }; }
  if (outFree(hout) && pickLoopCount() < 7) { const sku = nextNeed(); if (sku >= 0) { const b = pickBinFor(sku, cur); if (b) return { kind: 'retrieve', b, slot: b.slot, dst: hout }; } }
  if (outFree(rout) && rstCount() < 3) { const b = BINS.find(q => q.where === 'rack' && q.n <= 0 && !q.res); if (b) return { kind: 'rackToRst', b, slot: b.slot, dst: rout }; }
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
    setStep(2, 'R1 · waiting for a task', 'no return, no restock bin, no open demand', {});
    yield* waitUntil(() => !!(t = chooseR1()), 'no bin to move');
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
    yield* waitUntil(() => { b = binAt(P); return b && !b.done && b.pi < b.picks.length && P.valid; }, 'no bin / 3D imaging');
    const t0 = S.t, it = b.items[b.items.length - 1], pk = b.picks[b.pi];
    it.g.updateWorldMatrix(true, false); const wp = new THREE.Vector3().setFromMatrixPosition(it.g.matrixWorld), h = SIZE[it.sz].h;
    setStep(4, '{r} · pick {s} for order {o} from bin {b}', '4-cup vacuum · top layer first · {n} left in the bin', { r: R, s: SKUS[b.sku].code, o: pk.o.id, b: b.id, n: b.items.length - 1 });
    yield* go(wp.x, wp.y, Z.pCarry);
    yield* move({ x: wp.x, y: wp.y, z: wp.z + h }, { surface: wp.z + h });
    c.vac = true; yield* dwell(0.12);
    b.items.pop(); b.n--; grab(c, it.g); c.item = it; it.order = pk.o; it.line = pk.l; pk.l.picked++; b.pi++; P.valid = false; S.stat.picked++;
    yield* move({ x: wp.x, y: wp.y, z: Z.pCarry });
    yield* go(P.x, BL.y, Z.pCarry);
    if (b.pi >= b.picks.length) b.done = true;                               // tool has left the bin: the stop may release it
    setStep(5, '{r} · place on the item belt', 'belt holds while the gripper is down · free gap ≥ 70 mm', { r: R });
    yield* waitUntil(() => !zStop() && beltFree(P.x, SIZE[it.sz].l), 'no free gap on the belt');
    PLACE[id] = P.x;
    yield* move({ x: P.x, y: BL.y, z: BL.z + h }, { surface: BL.z + h });
    c.vac = false; yield* dwell(0.06); c.held = null; c.item = null;
    it.x = P.x; it.state = 'belt'; it.g.position.set(it.x, BL.y, BL.z); it.g.rotation.set(0, 0, 0); BELT.push(it);
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
    if (!halt && BELT.some(it => it.target === sid && it.x >= X - 1e-4)) halt = 'sort';
    if (!halt && c.tcp.z < Z.sCarry - 0.02 && Math.hypot(c.tcp.x - X, c.tcp.y - B.y) < 0.25) halt = 'sort'; }
  if (!halt && EXC.n >= K.ex.cap && BELT.some(it => it.x > B.endX - 0.3)) halt = 'exception tote full';
  G.beltHalt = halt; if (halt) { S.stat.haltT += dt; }
  else for (const it of BELT) { let nx = it.x + B.v * dt; if (it.target === 's1' || it.target === 's2') { const X = K.rob[it.target].pickX; if (it.x <= X && nx > X) nx = X; } it.x = nx; }
  G.scanBeam.visible = false;
  for (let i = BELT.length - 1; i >= 0; i--) { const it = BELT[i];
    if (!it.read && it.x >= B.scanX) { it.read = true; G.scanBeam.visible = true; S.stat.scans++;
      if (Math.random() < S.noRead || !it.order || !it.order.lane) { it.target = 'ex'; S.stat.noRead++; } else it.target = it.order.lane.sid; }
    if (it.x >= B.x1 - 0.02) { BELT.splice(i, 1); toException(it); continue; }
    it.g.position.set(it.x, B.y, B.z); }
  S.interlocks.belt = !Object.keys(PLACE).length || !!halt;
}
function toException(it) {
  EXC.n++; S.stat.exc++; if (it.line) { it.line.alloc--; it.line.picked--; } freeItem(it);
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
    yield* waitUntil(() => !!(it = BELT.find(q => q.target === id && Math.abs(q.x - X) < 1e-3)), 'no item for this sorter');
    const t0 = S.t, h = SIZE[it.sz].h, ln = it.order.lane;
    setStep(6, '{r} · {s} → order {o} (lane {l})', 'bellows cup · bar code read at the arch', { r: Rn, s: SKUS[it.sku].code, o: it.order.id, l: ln.id });
    yield* move({ x: X, y: BL.y, z: BL.z + h }, { surface: BL.z + h });
    c.vac = true; yield* dwell(0.08); BELT.splice(BELT.indexOf(it), 1); grab(c, it.g); c.item = it; it.state = 'held';
    yield* move({ x: X, y: BL.y, z: Z.sCarry });
    yield* go(ln.x, PW.yIn, Z.sCarry);
    yield* waitUntil(() => ln.state === 'active' && ln.u === 0, 'case lane moving');
    const zd = Z.caseRim + 0.03 + h; c.inLane = ln;
    yield* move({ x: ln.x, y: PW.yIn, z: zd }, { surface: zd });
    c.vac = false; yield* dwell(0.05); c.held = null; c.item = null; ln.items.push(it); showLane(ln);
    it.line.sorted++; S.stat.sorted++; S.out.items.push(S.t); if (S.out.items.length > 400) S.out.items.shift();
    yield* move({ x: ln.x, y: PW.yIn, z: Z.sCarry }); c.inLane = null;
    if (it.order.lines.every(l => l.sorted >= l.qty)) completeOrder(it.order);
    yield* go(X, BL.y, Z.sCarry);
    c.n = (c.n || 0) + 1; c.lastCycle = S.t - t0; c.cyc.push(c.lastCycle); if (c.cyc.length > 30) c.cyc.shift();
  }
}
function completeOrder(o) {
  const ln = o.lane; ORDERS.active.splice(ORDERS.active.indexOf(o), 1); ORDERS.done++; ORDERS.doneT.push(S.t); if (ORDERS.doneT.length > 200) ORDERS.doneT.shift();
  S.out.leadT.push(S.t - o.tA); if (S.out.leadT.length > 100) S.out.leadT.shift();
  ln.state = 'out'; ln.order = null;
}
/* put-wall lanes: a finished case runs out (only with the sorter's tool clear of the lane), the operator swaps it, an empty case runs in */
const toolOverLane = ln => { const c = ctx(ln.sid); return c.tcp.z < Z.sCarry - 0.03 && Math.abs(c.tcp.x - ln.x) < 0.25 && c.tcp.y > PW.yIn - 0.25; };
function updateLanes(dt) {
  for (const ln of LANES) {
    if (ln.state === 'out' && !zStop() && !toolOverLane(ln)) { ln.u = Math.min(1, ln.u + PW.v * dt / (PW.yOut - PW.yIn)); if (ln.u >= 1) { ln.state = 'atOuter'; ln.outT = S.t; } }
    else if (ln.state === 'in' && !zStop()) { ln.u = Math.max(0, ln.u - PW.v * dt / (PW.yOut - PW.yIn)); if (ln.u <= 0) ln.state = 'ready'; }
    showLane(ln);
    setLamp(ln.lamp, ln.state === 'atOuter' ? 0xff6a13 : ln.state === 'active' ? 0x22ff66 : ln.state === 'out' || ln.state === 'in' ? 0x2fa8ff : 0x444444);
  }
  S.interlocks.lanes = LANES.every(ln => !(ln.state === 'out' || ln.state === 'in') || !toolOverLane(ln));
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
    if (op.t >= S.refillT) { const add = dimOf(b.sku).cap - b.n; for (let k = b.n; k < dimOf(b.sku).cap; k++) { const it = takeItem(b.sku), p = gridPos(b.sku, k); b.g.add(it.g); it.g.position.set(p.x, p.y, p.z); b.items.push(it); }
      b.n = dimOf(b.sku).cap; b.refilled = true; op.t = 0; op.n++; S.stat.refills++; S.stat.restocked += add; } }
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
  const stepTo = p => { const dx = p.x - op.x, dy = p.y - op.y, L = Math.hypot(dx, dy), st = v * dt; if (L <= st) { op.x = p.x; op.y = p.y; op.walk += L; return true; } op.x += dx / L * st; op.y += dy / L * st; op.walk += st; op.face = Math.atan2(dy, dx); return false; };
  const T_ = op.task;
  if (T_) { op.home = null;
    if (T_.leg < T_.pts.length) { if (stepTo(T_.pts[T_.leg])) T_.leg++; }
    else { op.face = T_.face; T_.t += dt; op.busy += dt;
      if (T_.kind === 'case' && T_.t > 0.5 * T_.dur && T_.ln.items.length) { for (const it of T_.ln.items) freeItem(it); T_.ln.items = []; S.stat.cases++; }
      if (T_.t >= T_.dur) {
        if (T_.kind === 'case') { T_.ln.state = 'in'; T_.ln.claimOp = false; }
        else { S.stat.exRestocked += EXC.n; EXC.n = 0; EXC.claim = false; EXC.lastT = S.t; G.exFill.visible = false; }
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

// ------------------------------------------------------------------ station definition
const rob = (id, type, prog, extra) => Object.assign({ id, type, color: 0xf4f4f2, motion: motion[id], pedestal: K.rob[id].ped, yaw: K.rob[id].yaw, base: K.rob[id].base, glb: type, program: prog }, extra || {});
RC.defineStation({
  id: 'logistics_application', K,
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
    // bins: 3 per SKU, random slots
    let n = 0; for (let s = 0; s < NSKU; s++) for (let k = 0; k < BINS_PER_SKU; k++) BINS.push({ id: 'B' + String(++n).padStart(3, '0'), sku: s, n: 0, slot: null, where: 'rack', g: null, items: [] });
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
    W = zipf(NSKU);
    for (const c of RC.ctxs) { c.held = null; c.item = null; c.task = null; c.waitAcc = 0; c.busyAcc = 0; c.n = 0; c.cyc = []; c.waiting = ''; c.vac = false; c.inLane = null; }
    for (const it of ITEMPOOL) freeItem(it);
    BELT.length = 0; for (const k in PLACE) delete PLACE[k];
    for (const L of Object.values(LOOPS)) { for (const z of L.zones) { z.bin = null; z.res = null; } L.moves = 0; }
    for (const s of SLOTS) { s.bin = null; s.res = false; }
    const free = SLOTS.slice(); for (let i = free.length - 1; i > 0; i--) { const j = Math.floor(Math.random() * (i + 1)); [free[i], free[j]] = [free[j], free[i]]; }
    BINS.forEach((b, i) => { const cap = dimOf(b.sku).cap; Object.assign(b, { n: Math.round(cap * rand(0.25, 1)), slot: free[i], where: 'rack', res: false, mv: null, station: null, done: false, picks: [], allocated: false, placed: false, refilled: false, zi: -1, loop: null });
      free[i].bin = b; if (b.g) b.g.visible = false; b.items = []; });
    for (const s of SLOTS) setInst(s);
    for (const P of Object.values(ST)) Object.assign(P, { valid: false, imgT: 0, images: 0 });
    ORDERS.queue = []; ORDERS.active = []; ORDERS.seq = 0; ORDERS.done = 0; ORDERS.doneT = [];
    for (const ln of LANES) { Object.assign(ln, { u: 0, state: 'ready', order: null, items: [], claimOp: false, outT: 0 }); showLane(ln); }
    EXC.n = 0; EXC.claim = false; EXC.lastT = 0; G.exFill.visible = false;
    S.zone = freeZone(); S.zone.open = !!S.toggles.door; S.estop = !!S.toggles.estop;
    S.stat = { retrieved: 0, stored: 0, toRst: 0, presented: 0, picked: 0, sorted: 0, scans: 0, noRead: 0, exc: 0, exRestocked: 0, refills: 0, restocked: 0, cases: 0, haltT: 0 };
    S.out = { items: [], leadT: [] };
    Object.assign(G.op1, { t: 0, busy: 0, n: 0, task: null });
    Object.assign(G.op2, { x: WK.idle.x, y: WK.idle.y, face: 0, task: null, home: null, walk: 0, busy: 0, n: 0 });
    for (const k in motion) motion[k].vMax = V0[k] * S.speedPct / 100;
    S.interlocks = { zone: true, estop: !S.estop, image: true, binStop: true, belt: true, lanes: true, reachAll: true };
    activateOrders();
  },
  update(dt) {
    for (const c of RC.ctxs) { if (c.waiting) c.waitAcc += dt; else c.busyAcc += dt; }
    updateZone(dt); activateOrders(); updateLoop(LOOPS.pick, dt); updateLoop(LOOPS.rst, dt); updateCams(dt); updateBelt(dt); updateLanes(dt); updateOp1(dt); updateOp2(dt);
    for (const k of ['P1', 'P2']) { const P = ST[k], b = binAt(P); G.blades[k].position.z = b && !b.done ? K.zc - 0.03 + 0.05 : K.zc - 0.08; }
    S.interlocks.reachAll = RC.ctxs.every(c => c.reach);
  },
  onPose(tcp, c) {
    const h = c.held; if (!h) return;
    const cy = Math.cos(tcp.yaw), sy = Math.sin(tcp.yaw);
    h.m.position.set(tcp.x + h.lx * cy - h.ly * sy, tcp.y + h.lx * sy + h.ly * cy, tcp.z - h.dz); h.m.rotation.set(0, 0, tcp.yaw + h.rz, 'ZYX');
  },
  setMode(m) { S.mode = m === 'flat' ? 'flat' : 'zipf'; },
  setToggle(id, on) { if (id === 'door') S.zone.open = on; else if (id === 'estop') S.estop = on; },
  setNumber(id, v) {
    if (id === 'speed' && v > 0) { S.speedPct = clamp(v, 10, 100); for (const k in motion) motion[k].vMax = V0[k] * S.speedPct / 100; }
    else if (id === 'no-read' && v >= 0) S.noRead = clamp(v, 0, 20) / 100;
    else if (id === 'refill-t' && v > 0) S.refillT = clamp(v, 5, 300);
    else if (id === 'lines-max' && v > 0) S.linesMax = Math.round(clamp(v, 1, 6));
    else if (id === 'qty-max' && v > 0) S.qtyMax = Math.round(clamp(v, 1, 5));
  },
  hud() {
    const out = {}, s = S.stat, r = rate(S.out.items, 900), ro = rate(ORDERS.doneT, 1800);
    out['m-rate'] = r ? ['{r} items / h sorted · {o} orders / h', { r: r.toFixed(0), o: ro ? ro.toFixed(0) : '—' }] : '— (measuring)';
    out['m-orders'] = ['{d} done · {a} active · lead time {l} s', { d: ORDERS.done, a: ORDERS.active.length, l: S.out.leadT.length ? avg(S.out.leadT).toFixed(0) : '—' }];
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
    out['m-exc'] = ['{e} no-read → exception tote ({n} in the tote) · {s} scans', { e: s.exc, n: EXC.n, s: s.scans }];
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
