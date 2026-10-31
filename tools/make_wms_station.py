"""Derive the browser edition's WMS-driven station (src/Logistics.Wms.Web/wwwroot/twin/wms-station.js) from the cell simulation's own
station (simulation/canvas/project/app.js, from '/* Station: ...'). Every edit must match exactly once, so a changed simulation fails
loudly instead of producing a half-patched station.

Usage: python3 tools/make_wms_station.py simulation/canvas/project/app.js src/Logistics.Wms.Web/wwwroot/twin/wms-station.js
"""
import sys

src, dst = sys.argv[1], sys.argv[2]
app = open(src, encoding='utf-8').read()
mark = '/* Station: logistics application'
s = app[app.index(mark):]


def rep(old, new, count=1):
    global s
    n = s.count(old)
    if n != count:
        raise SystemExit(f'expected {count} match(es), found {n}: {old[:90]!r}')
    s = s.replace(old, new)


# ---------------------------------------------------------------- header
rep('''/* Station: logistics application — goods-to-robot order fulfilment''', '''/* WMS-driven variant of the station below (derived from simulation/canvas/project/app.js): the robots, conveyors, operators and
   safety are unchanged, but every decision comes from the Logistics WMS in the parent page and every event is booked there —
   R1 tasks (with the occupancy of its transfer heads), routing at the decision point, picks allocated at the stops, sortation
   targets from the bar-code arch, put-wall orders and lane steps, restock, exception tote. Robots the WMS reports as not available
   (fault, manual, maintenance) stand still. The WMS clock follows the simulation time (it is passed with every call).

   Station: logistics application — goods-to-robot order fulfilment''')

# ---------------------------------------------------------------- WMS bridge helpers
rep('''const S = RC.sim;
const TAU = 2 * Math.PI, D2R = Math.PI / 180;''', '''const S = RC.sim;
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
const robotWhy = id => { const r = WMS.robots[id.toUpperCase()]; return !r ? '' : r.state === 'Fault' ? 'WMS: fault ' + (r.faultCode || '') : 'WMS: ' + r.mode + ' mode'; };''')

# ---------------------------------------------------------------- SKU catalogue from the WMS
rep('''const NSKU = 40, BINS_PER_SKU = 3;
const SKUS = []; for (let i = 0; i < NSKU; i++) SKUS.push({ i, code: 'SKU-' + String(1001 + i), size: ['S', 'M', 'L'][i % 5 === 4 ? 2 : i % 3 === 2 ? 1 : 0], lab: LABELS[i % LABELS.length] });''',
    '''const SKUS = [];                                                             // filled from the WMS (code, size), see loadSkus()
function loadSkus() { SKUS.length = 0; (WMS.init.skus || []).forEach((k, i) => SKUS.push({ i, code: k.code, size: k.size, lab: LABELS[i % LABELS.length] })); }''')

# ---------------------------------------------------------------- orders / demand / allocation → WMS
a = s.index('// ------------------------------------------------------------------ WMS: orders (rolling waves on the put wall), demand, allocation')
b = s.index('// ------------------------------------------------------------------ build: cylinder shelf')
s = s[:a] + '''// ------------------------------------------------------------------ WMS: orders, waves, demand and allocation live in the WMS
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

''' + s[b:]

# ---------------------------------------------------------------- loop arrival hook
rep('''  if (L.id === 'pick' && (z.k === 'P1' || z.k === 'P2') && b.station === z.k) { allocate(b); ST[z.k].valid = false; if (!b.picks.length) b.done = true; b.arrT = S.t; S.stat.presented++; }''',
    '''  if (L.id === 'pick' && (z.k === 'P1' || z.k === 'P2') && b.station === z.k) { arrive(b, z.k); ST[z.k].valid = false; b.arrT = S.t; S.stat.presented++; }''')

# ---------------------------------------------------------------- R1: tasks from the WMS
a = s.index('function chooseR1() {')
b = s.index('function* progR1() {')
s = s[:a] + '''/* what R1's controller sees at its transfer heads, sent with every task request */
function r1Heads() {
  const hin = headOf('pick', 'HIN'), rin = headOf('rst', 'HIN'), hout = headOf('pick', 'H'), rout = headOf('rst', 'H');
  const at = h => h.z.bin && !h.z.bin.mv ? h.z.bin.id : null, free = h => !h.z.bin && !h.z.res;
  return [at(hin), at(rin), free(hout), free(rout)];
}
const slotOf = code => { const m = /C(\\d+)-L(\\d+)/.exec(code || ''); return m ? SLOTS.find(q => q.k === +m[1] - 1 && q.j === +m[2] - 1) : null; };
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
''' + s[b:]

rep('''  while (true) {
    let t = null;
    setStep(2, 'R1 · waiting for a task', 'no return, no restock bin, no open demand', {});
    yield* waitUntil(() => !!(t = chooseR1()), 'no bin to move');
    c.task = t; const b = t.b; b.res = true; if (t.slot) t.slot.res = true; if (t.dst) t.dst.z.res = 'r1';
    const t0 = S.t;''', '''  while (true) {
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
    const t0 = S.t;''')
rep('''    b.res = false; c.task = null; c.n = (c.n || 0) + 1; c.lastCycle = S.t - t0; c.cyc.push(c.lastCycle); if (c.cyc.length > 40) c.cyc.shift();
  }
}''', '''    b.res = false; c.task = null; c.n = (c.n || 0) + 1; c.lastCycle = S.t - t0; c.cyc.push(c.lastCycle); if (c.cyc.length > 40) c.cyc.shift();
    const done = wmsCall('R1Complete', t.v.id); yield* waitUntil(() => done.done, 'WMS');
  }
}''')

# ---------------------------------------------------------------- R2 / R3: picks allocated by the WMS
rep('''    yield* waitUntil(() => { b = binAt(P); return b && !b.done && b.pi < b.picks.length && P.valid; }, 'no bin / 3D imaging');
    const t0 = S.t, it = b.items[b.items.length - 1], pk = b.picks[b.pi];''', '''    yield* waitUntil(() => robotOk(id), robotWhy(id) || 'robot not available');
    yield* waitUntil(() => { b = binAt(P); return robotOk(id) && b && !b.done && b.picks && b.pi < b.picks.length && P.valid && b.items.length > 0; }, 'no bin / 3D imaging');
    const t0 = S.t, it = b.items[b.items.length - 1], pk = b.picks[b.pi];
    wmsCall('RobotBegin', R, 'pick:' + pk.id, 'pick ' + SKUS[b.sku].code + ' from bin ' + b.id + ' for order ' + pk.orderId);''')
rep('''{ r: R, s: SKUS[b.sku].code, o: pk.o.id, b: b.id, n: b.items.length - 1 });''', '''{ r: R, s: SKUS[b.sku].code, o: pk.orderId, b: b.id, n: b.items.length - 1 });''')
rep('''    b.items.pop(); b.n--; grab(c, it.g); c.item = it; it.order = pk.o; it.line = pk.l; pk.l.picked++; b.pi++; P.valid = false; S.stat.picked++;''',
    '''    b.items.pop(); b.n--; grab(c, it.g); c.item = it; it.pick = pk.id; it.orderId = pk.orderId; b.pi++; P.valid = false; S.stat.picked++;''')
rep('''    it.x = P.x; it.state = 'belt'; it.g.position.set(it.x, BL.y, BL.z); it.g.rotation.set(0, 0, 0); BELT.push(it);''',
    '''    it.x = P.x; it.state = 'belt'; it.g.position.set(it.x, BL.y, BL.z); it.g.rotation.set(0, 0, 0); BELT.push(it);
    wmsCall('PickDone', it.pick);                                           // booked out of the bin, tracked by its pick id''')

# ---------------------------------------------------------------- belt: the arch asks the WMS for the target
rep('''    if (!halt && BELT.some(it => it.target === sid && it.x >= X - 1e-4)) halt = 'sort';''',
    '''    if (!halt && BELT.some(it => it.target === sid && it.x >= X - 1e-4 && robotOk(sid))) halt = 'sort';''')
rep('''    if (!it.read && it.x >= B.scanX) { it.read = true; G.scanBeam.visible = true; S.stat.scans++;
      if (Math.random() < S.noRead || !it.order || !it.order.lane) { it.target = 'ex'; S.stat.noRead++; } else it.target = it.order.lane.sid; }''',
    '''    if (!it.read && it.x >= B.scanX) { it.read = true; G.scanBeam.visible = true; S.stat.scans++;
      const nr = Math.random() < S.noRead; if (nr) S.stat.noRead++;
      it.target = 'wait'; it.scanReq = wmsCall('Scan', it.pick, nr ? null : SKUS[it.sku].code); }
    if (it.scanReq && it.scanReq.done) { const r = it.scanReq, v = r.ok ? r.value : null; it.scanReq = null;
      if (!v || v.target === 'Exception') { it.target = 'ex'; it.reason = v ? v.reason : 'refused'; } else { it.target = v.target.toLowerCase(); it.laneId = v.lane; } }
    if ((it.target === 's1' || it.target === 's2') && !robotOk(it.target)) { it.target = 'ex'; it.reason = 'sorter down'; wmsCall('Missed', it.pick); }''')
rep('''  EXC.n++; S.stat.exc++; if (it.line) { it.line.alloc--; it.line.picked--; } freeItem(it);''',
    '''  EXC.n++; S.stat.exc++; freeItem(it);                                     // booked by the WMS at the arch''')

# ---------------------------------------------------------------- S1 / S2: targets from the WMS
rep('''    yield* waitUntil(() => !!(it = BELT.find(q => q.target === id && Math.abs(q.x - X) < 1e-3)), 'no item for this sorter');
    const t0 = S.t, h = SIZE[it.sz].h, ln = it.order.lane;''', '''    yield* waitUntil(() => robotOk(id), robotWhy(id) || 'robot not available');
    yield* waitUntil(() => !!(it = BELT.find(q => q.target === id && Math.abs(q.x - X) < 1e-3)) && robotOk(id), 'no item for this sorter');
    const t0 = S.t, h = SIZE[it.sz].h, ln = LANES.find(l => l.id === it.laneId);
    if (!ln) { it.target = 'ex'; wmsCall('Missed', it.pick); continue; }
    wmsCall('RobotBegin', Rn, 'sort:' + it.pick, 'put ' + SKUS[it.sku].code + ' (carton #' + it.pick + ') into lane ' + ln.id);''')
rep('''{ r: Rn, s: SKUS[it.sku].code, o: it.order.id, l: ln.id });''', '''{ r: Rn, s: SKUS[it.sku].code, o: it.orderId, l: ln.id });''')
rep('''    it.line.sorted++; S.stat.sorted++; S.out.items.push(S.t); if (S.out.items.length > 400) S.out.items.shift();
    yield* move({ x: ln.x, y: PW.yIn, z: Z.sCarry }); c.inLane = null;
    if (it.order.lines.every(l => l.sorted >= l.qty)) completeOrder(it.order);''', '''    S.stat.sorted++; S.out.items.push(S.t); if (S.out.items.length > 400) S.out.items.shift();
    const sr = wmsCall('Sort', it.pick, ln.id);
    yield* move({ x: ln.x, y: PW.yIn, z: Z.sCarry }); c.inLane = null;
    yield* waitUntil(() => sr.done, 'WMS');
    if (sr.ok && sr.value === true) completeOrder(ln);''')
rep('''function completeOrder(o) {
  const ln = o.lane; ORDERS.active.splice(ORDERS.active.indexOf(o), 1); ORDERS.done++; ORDERS.doneT.push(S.t); if (ORDERS.doneT.length > 200) ORDERS.doneT.shift();
  S.out.leadT.push(S.t - o.tA); if (S.out.leadT.length > 100) S.out.leadT.shift();
  ln.state = 'out'; ln.order = null;
}''', '''/* the WMS completed the lane's order: the case runs out to the packing operator */
function completeOrder(ln) {
  ORDERS.done++; ORDERS.doneT.push(S.t); if (ORDERS.doneT.length > 200) ORDERS.doneT.shift();
  ln.state = 'out'; ln.order = null;
}''')

# ---------------------------------------------------------------- put-wall lanes: steps booked in the WMS, orders from the WMS
rep('''    if (ln.state === 'out' && !zStop() && !toolOverLane(ln)) { ln.u = Math.min(1, ln.u + PW.v * dt / (PW.yOut - PW.yIn)); if (ln.u >= 1) { ln.state = 'atOuter'; ln.outT = S.t; } }
    else if (ln.state === 'in' && !zStop()) { ln.u = Math.max(0, ln.u - PW.v * dt / (PW.yOut - PW.yIn)); if (ln.u <= 0) ln.state = 'ready'; }''',
    '''    if (ln.state === 'out' && !zStop() && !toolOverLane(ln)) { ln.u = Math.min(1, ln.u + PW.v * dt / (PW.yOut - PW.yIn)); if (ln.u >= 1) { ln.state = 'atOuter'; ln.outT = S.t; ln.req = wmsCall('Lane', ln.id, 'CaseAtOperator'); } }
    else if (ln.state === 'in' && !zStop()) { ln.u = Math.max(0, ln.u - PW.v * dt / (PW.yOut - PW.yIn)); if (ln.u <= 0) { ln.state = 'ready'; ln.req = wmsCall('Lane', ln.id, 'EmptyCaseInPosition'); } }
    if (ln.req && ln.req.done) { const r = ln.req; ln.req = null; if (r.ok && r.value) applyLane(ln, r.value); }''')
rep('''const pwMuted = sid =>''', '''/* a WMS lane state onto a lane that stands still (moving cases and the operator's swap are driven from here) */
function applyLane(ln, w) {
  if (ln.req || !(ln.state === 'ready' || ln.state === 'active')) return;
  if (w.state === 'Active') { ln.state = 'active'; ln.order = { id: w.orderId }; }
  else if (w.state === 'Ready') { ln.state = 'ready'; ln.order = null; }
  else if (w.state === 'Outbound') { ln.state = 'out'; ln.order = null; }
}
const pwMuted = sid =>''')
rep('''        if (T_.kind === 'case') { T_.ln.state = 'in'; T_.ln.claimOp = false; }
        else { S.stat.exRestocked += EXC.n; EXC.n = 0; EXC.claim = false; EXC.lastT = S.t; G.exFill.visible = false; }''',
    '''        if (T_.kind === 'case') { T_.ln.state = 'in'; T_.ln.claimOp = false; T_.ln.req = wmsCall('Lane', T_.ln.id, 'CaseReplaced'); }
        else { S.stat.exRestocked += EXC.n; EXC.n = 0; EXC.claim = false; EXC.lastT = S.t; G.exFill.visible = false; wmsCall('ToteEmptied'); }''')

# ---------------------------------------------------------------- restock operator: refill booked in the WMS
rep('''    if (op.t >= S.refillT) { const add = dimOf(b.sku).cap - b.n; for (let k = b.n; k < dimOf(b.sku).cap; k++) { const it = takeItem(b.sku), p = gridPos(b.sku, k); b.g.add(it.g); it.g.position.set(p.x, p.y, p.z); b.items.push(it); }
      b.n = dimOf(b.sku).cap; b.refilled = true; op.t = 0; op.n++; S.stat.refills++; S.stat.restocked += add; } }''',
    '''    if (op.t >= S.refillT && !b.rstReq && (b.rstT || 0) <= S.t) b.rstReq = wmsCall('Restock', b.id);
    if (b.rstReq && b.rstReq.done) { const r = b.rstReq; b.rstReq = null;
      if (!r.ok || !r.value) b.rstT = S.t + 5;
      else { if (r.value.sku >= 0 && r.value.sku !== b.sku) { for (const it of b.items) freeItem(it); b.items = []; b.n = 0; b.sku = r.value.sku; }
        const q = r.value.qty, add = q - b.n; for (let k = b.n; k < q; k++) { const it = takeItem(b.sku), p = gridPos(b.sku, k); b.g.add(it.g); it.g.position.set(p.x, p.y, p.z); b.items.push(it); }
        b.n = q; b.refilled = true; op.t = 0; op.n++; S.stat.refills++; S.stat.restocked += add; } } }''')

# ---------------------------------------------------------------- build: bins from the WMS
rep('''    // bins: 3 per SKU, random slots
    let n = 0; for (let s = 0; s < NSKU; s++) for (let k = 0; k < BINS_PER_SKU; k++) BINS.push({ id: 'B' + String(++n).padStart(3, '0'), sku: s, n: 0, slot: null, where: 'rack', g: null, items: [] });''',
    '''    // bins: the WMS's bins (placed in reset)
    loadSkus();''')

# ---------------------------------------------------------------- reset: the cell as the WMS has it
a = s.index('''    W = zipf(NSKU);''')
b = s.index('''    for (const P of Object.values(ST)) Object.assign(P, { valid: false, imgT: 0, images: 0 });''')
s = s[:a] + '''    for (const c of RC.ctxs) { c.held = null; c.item = null; c.task = null; c.waitAcc = 0; c.busyAcc = 0; c.n = 0; c.cyc = []; c.waiting = ''; c.vac = false; c.inLane = null; }
    for (const it of ITEMPOOL) freeItem(it);
    BELT.length = 0; for (const k in PLACE) delete PLACE[k];
    for (const L of Object.values(LOOPS)) { for (const z of L.zones) { z.bin = null; z.res = null; } L.moves = 0; }
    for (const s of SLOTS) { s.bin = null; s.res = false; }
    loadWmsState();
    for (const s of SLOTS) setInst(s);
''' + s[b:]
rep('''    ORDERS.queue = []; ORDERS.active = []; ORDERS.seq = 0; ORDERS.done = 0; ORDERS.doneT = [];
    for (const ln of LANES) { Object.assign(ln, { u: 0, state: 'ready', order: null, items: [], claimOp: false, outT: 0 }); showLane(ln); }
    EXC.n = 0; EXC.claim = false; EXC.lastT = 0; G.exFill.visible = false;''', '''    ORDERS.queue = []; ORDERS.active = []; ORDERS.seq = 0; ORDERS.done = 0; ORDERS.doneT = [];
    const laneState = { Ready: ['ready', 0], Active: ['active', 0], Outbound: ['out', 0], AtOperator: ['atOuter', 1], Inbound: ['in', 1] };
    for (const ln of LANES) { const w = (WMS.init.lanes || []).find(q => q.id === ln.id), st = laneState[w ? w.state : 'Ready'] || ['ready', 0];
      Object.assign(ln, { u: st[1], state: st[0], order: w && w.orderId ? { id: w.orderId } : null, items: [], claimOp: false, outT: 0, req: null }); showLane(ln); }
    EXC.n = WMS.init.tote || 0; EXC.claim = false; EXC.lastT = 0; G.exFill.visible = EXC.n > 0;''')
rep('''    S.interlocks = { zone: true, estop: !S.estop, image: true, binStop: true, belt: true, lanes: true, reachAll: true };
    activateOrders();
  },''', '''    S.interlocks = { zone: true, estop: !S.estop, image: true, binStop: true, belt: true, lanes: true, reachAll: true };
    applyPoll(WMS.init);
  },''')
rep('''    updateZone(dt); activateOrders(); updateLoop(LOOPS.pick, dt);''', '''    updateZone(dt); syncWms(); resolveArrivals(); updateLoop(LOOPS.pick, dt);''')

# helpers: placing the WMS state, polling
rep('''// ------------------------------------------------------------------ station definition''', '''// ------------------------------------------------------------------ WMS state → cell, polling
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
function applyPoll(p) {
  if (!p) return;
  WMS.robots = {}; for (const r of p.robots || []) WMS.robots[r.id] = r;
  WMS.orders = p.orders || WMS.orders; WMS.opts = p.options || WMS.opts; WMS.tote = p.tote;
  if (WMS.opts) { S.noRead = WMS.opts.noReadRate; S.mode = WMS.opts.pareto ? 'zipf' : 'flat'; }
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

// ------------------------------------------------------------------ station definition''')

# ---------------------------------------------------------------- metrics from the WMS
rep('''    out['m-orders'] = ['{d} done · {a} active · lead time {l} s', { d: ORDERS.done, a: ORDERS.active.length, l: S.out.leadT.length ? avg(S.out.leadT).toFixed(0) : '—' }];''',
    '''    const wo = WMS.orders;
    out['m-orders'] = wo ? ['WMS: {d} done · {a} active · {q} queued · lead time {l} s', { d: wo.completed, a: wo.active, q: wo.queued, l: wo.leadSeconds ? wo.leadSeconds.toFixed(0) : '—' }] : '—';
    const rs = Object.values(WMS.robots);
    out['m-wms'] = rs.length ? rs.map(r => r.id + ' ' + (r.state === 'Fault' ? '⚠ ' + (r.faultCode || 'fault') : r.mode !== 'Auto' ? r.mode.toLowerCase() : r.state === 'Working' ? 'working' : 'idle')).join(' · ') : '—';
    out['m-wmscalls'] = ['{n} calls · {e} refused', { n: WMS.calls, e: WMS.errors.length }];''')
rep('''    out['m-exc'] = ['{e} no-read → exception tote ({n} in the tote) · {s} scans', { e: s.exc, n: EXC.n, s: s.scans }];''',
    '''    out['m-exc'] = ['{e} to the exception tote ({n} in the tote) · {s} scans', { e: s.exc, n: EXC.n, s: s.scans }];''')
rep('''  setMode(m) { S.mode = m === 'flat' ? 'flat' : 'zipf'; },''', '''  setMode(m) { S.mode = m === 'flat' ? 'flat' : 'zipf'; wmsCall('SetOption', 'pareto', S.mode === 'zipf' ? 1 : 0); },''')
rep('''    else if (id === 'no-read' && v >= 0) S.noRead = clamp(v, 0, 20) / 100;''', '''    else if (id === 'no-read' && v >= 0) { S.noRead = clamp(v, 0, 20) / 100; WMS.bridge && WMS.bridge.invoke('SetOption', 'no-read', v); }''')
rep('''    else if (id === 'lines-max' && v > 0) S.linesMax = Math.round(clamp(v, 1, 6));
    else if (id === 'qty-max' && v > 0) S.qtyMax = Math.round(clamp(v, 1, 5));''', '''    else if (id === 'lines-max' && v > 0) { S.linesMax = Math.round(clamp(v, 1, 6)); WMS.bridge && WMS.bridge.invoke('SetOption', 'lines-max', S.linesMax); }
    else if (id === 'qty-max' && v > 0) { S.qtyMax = Math.round(clamp(v, 1, 5)); WMS.bridge && WMS.bridge.invoke('SetOption', 'qty-max', S.qtyMax); }''')
rep('''    "Pareto demand (top 20 % SKUs ≈ 55 % lines)": "柏拉圖需求（前 20 % SKU ≈ 55 % 行數）",''', '''    "Pareto demand (top 20 % SKUs ≈ 55 % lines)": "柏拉圖需求（前 20 % SKU ≈ 55 % 行數）",
    "WMS: {d} done · {a} active · {q} queued · lead time {l} s": "WMS：完成 {d} · 進行中 {a} · 佇列 {q} · 前置時間 {l} 秒",
    "{n} calls · {e} refused": "{n} 次呼叫 · {e} 次拒絕",
    "{e} to the exception tote ({n} in the tote) · {s} scans": "{e} 件至例外料盒（盒內 {n}）· 掃描 {s} 次",
    "WMS robots": "WMS 機器人",
    "WMS link": "WMS 連線",
    "R1 · waiting for a task": "R1 · 等待任務",
    "asking the WMS for the next bin move": "向 WMS 索取下一個料箱搬移",
    "bin at the transfer head": "料箱到達輸送頭",
    "transfer head free": "輸送頭淨空",
    "robot not available": "機器人無法使用",
    "R1 not available": "R1 無法使用",''')

# robustness: the engine's first frame can report a negative dt (rAF timestamp before the performance.now() taken at start); the
# packing operator standing on its target then computed 0 / 0 and its position stayed NaN for good (the put wall stalled)
rep('''  const stepTo = p => { const dx = p.x - op.x, dy = p.y - op.y, L = Math.hypot(dx, dy), st = v * dt;''',
    '''  const stepTo = p => { const dx = p.x - op.x, dy = p.y - op.y, L = Math.hypot(dx, dy), st = Math.max(0, v * dt);''')
rep('''  update(dt) {
    for (const c of RC.ctxs) {''', '''  update(dt) {
    if (!(dt >= 0)) return;                                                  // see stepTo in updateOp2
    for (const c of RC.ctxs) {''')

# ---------------------------------------------------------------- the cell's orders: lane labels, orders panel, focus on an order
rep('''  M.labs = LABELS.map(c => mat(c, { roughness: 0.6 }));''', '''  M.labs = LABELS.map(c => mat(c, { roughness: 0.6 }));
  M.cartonFocus = mat(0xff8a3d, { roughness: 0.55, emissive: 0x7a2a00, emissiveIntensity: 0.7 });   // cartons of the order in focus''')
rep('''  it.free = false; it.sku = sku; it.lab.material = M.labs[LABELS.indexOf(SKUS[sku].lab)];''',
    '''  it.free = false; it.sku = sku; it.g.children[0].material = M.carton; it.lab.material = M.labs[LABELS.indexOf(SKUS[sku].lab)];''')
rep('''function applyPoll(p) {
  if (!p) return;''', '''/* label over each put-wall lane: lane, order reference, sorted / ordered cartons (orange: the order in focus) */
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
  for (const w of WMS.lanes || []) { const ln = LANES.find(l => l.id === w.id); if (ln) laneLabel(ln, w); }''')
rep('''  if (WMS.opts) { S.noRead = WMS.opts.noReadRate; S.mode = WMS.opts.pareto ? 'zipf' : 'flat'; }''',
    '''  if (WMS.opts) { S.noRead = WMS.opts.noReadRate; S.mode = WMS.opts.pareto ? 'zipf' : 'flat'; }
  renderOrders();''')
rep('''    updateZone(dt); syncWms(); resolveArrivals();''', '''    updateZone(dt); syncWms(); resolveArrivals(); tintFocus();''')
rep('''    out['m-orders'] = wo ? ['WMS: {d} done · {a} active · {q} queued · lead time {l} s', { d: wo.completed, a: wo.active, q: wo.queued, l: wo.leadSeconds ? wo.leadSeconds.toFixed(0) : '—' }] : '—';''',
    '''    out['m-orders'] = wo ? ['WMS: {d} done · {a} active · {q} queued · lead time {l} s', { d: wo.completed, a: wo.active, q: wo.queued, l: wo.leadSeconds ? wo.leadSeconds.toFixed(0) : '—' }] : '—';
    out['m-ordrate'] = wo ? ['{n} orders in the last hour', { n: wo.lastHour }] : '—';''')
rep('''    "WMS robots": "WMS 機器人",''', '''    "WMS robots": "WMS 機器人",
    "{n} orders in the last hour": "最近一小時 {n} 張訂單",
    "Orders, last hour": "訂單（最近一小時）",''')

# ---------------------------------------------------------------- the belt never runs ahead of the WMS
# a carton waits for its routing (WMS reply) before the first sorter; one that passed its sorter anyway is reported as missed
rep('''  if (!halt && EXC.n >= K.ex.cap && BELT.some(it => it.x > B.endX - 0.3)) halt = 'exception tote full';''',
    '''  if (!halt && BELT.some(it => it.target === 'wait' && it.x >= K.rob.s1.pickX - 0.05)) halt = 'scan';
  if (!halt && EXC.n >= K.ex.cap && BELT.some(it => it.x > B.endX - 0.3)) halt = 'exception tote full';''')
rep('''    if ((it.target === 's1' || it.target === 's2') && !robotOk(it.target)) { it.target = 'ex'; it.reason = 'sorter down'; wmsCall('Missed', it.pick); }''',
    '''    if ((it.target === 's1' || it.target === 's2') && !robotOk(it.target)) { it.target = 'ex'; it.reason = 'sorter down'; wmsCall('Missed', it.pick); }
    if ((it.target === 's1' || it.target === 's2') && it.x > K.rob[it.target].pickX + 0.01) { it.target = 'ex'; it.reason = 'missed'; wmsCall('Missed', it.pick); }''')
rep('''    "holds: exception tote full": "保持：例外料盒已滿",''', '''    "holds: exception tote full": "保持：例外料盒已滿",
    "holds: scan": "保持：等待 WMS 分揀指示",''')

# the station's own id, so the engine's remembered panel state does not collide with robot_cell.html
rep('''  id: 'logistics_application', K,''', '''  id: 'logistics_application_wms', K,''')

open(dst, 'w', encoding='utf-8').write(s)
print('ok', len(s))
