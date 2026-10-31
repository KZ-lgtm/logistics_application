/* Boot of the live 3D cell inside the WMS (adapted from the boot script of simulation/robot_cell.html): wait for the WMS bridge of
   the parent page, load the WMS state, start the cell engine with the WMS-driven station (wms-station.js), wire the controls. */
(async function () {
  const $ = id => document.getElementById(id); const RC = window.RobotCell; const UI = window.STATION_UI;
  const loading = $('loading');
  let bridge = null;
  for (let i = 0; i < 100 && !bridge; i++) {                               // the parent registers the bridge right after rendering
    try { bridge = window.parent !== window && window.parent.wmsTwinBridge; } catch (e) { bridge = null; }
    if (!bridge) await new Promise(r => setTimeout(r, 100));
  }
  if (!bridge) { loading.textContent = 'This view runs inside the WMS: open "Live 3D cell" there.'; return; }
  RC.wms.bridge = bridge;
  const init = async () => { const r = await bridge.invoke('Init', RC.sim.t || 0); if (!r || !r.ok) throw new Error((r && r.error) || 'no answer'); RC.wms.init = r.data; };
  try { await init(); } catch (e) { loading.textContent = 'The WMS could not be read: ' + e.message; return; }
  loading.textContent = 'Loading robot models…';
  await new Promise(r => setTimeout(r, 30));                                // let the text paint before the heavy start

  /* the engine takes dt = rAF timestamp − previous timestamp, starting from performance.now(): a first timestamp from before that
     gives a negative dt, so hand it monotonic timestamps */
  const raf = window.requestAnimationFrame.bind(window);
  let lastFrame = performance.now();
  window.requestAnimationFrame = cb => raf(ts => { lastFrame = performance.now(); cb(Math.max(ts, lastFrame)); });
  RC.start({ canvas: $('view'), glbB64: window.ROBOT_GLB_B64, glbs: window.ROBOT_GLBS, ui: UI });

  /* The cell keeps working while the WMS shows another page: nothing is drawn then, and when the browser stops animation frames for
     the hidden view the simulation is stepped from a timer instead (the WMS keeps getting its events). */
  let hidden = false;
  const render = RC.renderer.render.bind(RC.renderer);
  RC.renderer.render = (scene, camera) => { if (!hidden) render(scene, camera); };
  /* stepped in slices with a pause in between: the WMS's replies (routing, picks) arrive between slices, as they do between frames */
  (async () => {
    let lastTick = performance.now();
    for (;;) {
      await new Promise(r => setTimeout(r, 100));
      const now = performance.now(), dt = Math.min(1, (now - lastTick) / 1000); lastTick = now;
      if (!RC.sim.running || now - lastFrame < 300) continue;
      let left = dt * RC.sim.speed;
      while (left > 1e-6) {
        const slice = Math.min(0.25, left); left -= slice;
        const n = Math.max(1, Math.ceil(slice / 0.02)); for (let i = 0; i < n; i++) RC.step(slice / n);
        if (left > 1e-6) await new Promise(r => setTimeout(r, 0));
      }
    }
  })();
  window.wmsTwinVisible = v => { hidden = !v; };
  window.wmsTwinFocus = id => RC.wms.focusOrder(id || null);
  if (RC.wms.init.focus) window.wmsTwinFocus(RC.wms.init.focus);          // "Watch in 3D" opened the cell
  const pending = window.frameElement && window.frameElement.pending;     // calls made before this page was ready
  if (pending) { if ('wmsTwinVisible' in pending) window.wmsTwinVisible(pending.wmsTwinVisible); if ('wmsTwinFocus' in pending) window.wmsTwinFocus(pending.wmsTwinFocus); }
  $('wms-orders').addEventListener('click', e => { const li = e.target.closest('li[data-order]'); const id = li && parseInt(li.dataset.order, 10); if (id) RC.wms.focusOrder(id === RC.wms.focus ? null : id); });
  loading.remove();
  const o = RC.wms.opts || {};
  $('n-no-read').value = ((o.noReadRate || 0) * 100).toFixed(1).replace(/\.0$/, '');
  if (o.maxLines) $('n-lines-max').value = Math.min(6, o.maxLines);
  if (o.maxQuantity) $('n-qty-max').value = Math.min(5, o.maxQuantity);
  const modeUI = pareto => { $('b-mode-zipf').classList.toggle('on', pareto); $('b-mode-flat').classList.toggle('on', !pareto); };
  modeUI(o.pareto !== false); $('b-gen').classList.toggle('on', !!o.generateOrders);

  $('b-run').addEventListener('click', () => { const on = RC.toggleRun(); $('b-run').setAttribute('data-i18n', on ? 'Pause' : 'Run'); $('b-run').textContent = RC.t(on ? 'Pause' : 'Run'); $('b-run').classList.toggle('on', on); });
  $('b-reset').addEventListener('click', async () => {                      // rebuild from the WMS (e.g. after changes made elsewhere)
    $('b-reset').disabled = true;
    try { await init(); RC.reset(); } catch (e) { console.warn(e); } finally { $('b-reset').disabled = false; }
  });
  $('b-gen').addEventListener('click', async () => {
    const on = !$('b-gen').classList.contains('on'); const r = await bridge.invoke('SetOption', 'generate', on ? 1 : 0);
    if (r && r.ok) $('b-gen').classList.toggle('on', on);
  });
  $('speed').addEventListener('input', e => { const v = parseFloat(e.target.value); RC.setSpeed(v); $('speed-v').textContent = v.toFixed(2).replace(/\.?0+$/, '') + '×'; });
  for (const n of UI.controls.numbers || []) $('n-' + n.id).addEventListener('change', e => RC.setNumber(n.id, parseFloat(e.target.value)));
  /* demand mode goes to the WMS's order generator; RC.setMode would rebuild the cell */
  for (const m of UI.controls.modes || []) $('b-mode-' + m.id).addEventListener('click', () => { RC.station.setMode(m.id); modeUI(m.id !== 'flat'); });
  for (const g of UI.controls.toggles || []) $('b-tog-' + g.id).addEventListener('click', () => { const on = RC.setToggle(g.id, !$('b-tog-' + g.id).classList.contains('on')); $('b-tog-' + g.id).classList.toggle('on', on); });
  for (const v of UI.controls.views || []) $('b-view-' + v.id).addEventListener('click', () => { RC.setView(v.id); for (const q of UI.controls.views) $('b-view-' + q.id).classList.toggle('on', q.id === v.id); });
  if (UI.controls.scanFields && $('b-scan')) {
    $('b-scan').addEventListener('click', () => { const on = RC.setScanVisible(!RC.scanGroup.visible); $('b-scan').classList.toggle('on', on); });
    $('scan-op').addEventListener('input', e => RC.setScanOpacity(parseFloat(e.target.value)));
  }
  if (UI.controls.electrical && $('b-elec')) $('b-elec').addEventListener('click', () => { const on = RC.setElecVisible ? RC.setElecVisible(!RC.elec.overlay.visible) : false; $('b-elec').classList.toggle('on', on); });
  const langUI = () => { const zh = RC.lang === 'zh'; $('b-zh').classList.toggle('on', zh); $('b-en').classList.toggle('on', !zh); };
  $('b-en').addEventListener('click', () => { RC.setLang('en'); langUI(); });
  $('b-zh').addEventListener('click', () => { RC.setLang('zh'); langUI(); });
  langUI();
  // panel collapse / hide (remembered per browser)
  const panels = ['hdr', 'metrics', 'ctrl']; let pst = {};
  try { pst = JSON.parse(localStorage.getItem('wms-twin-panels') || '{}') || {}; } catch (e) { pst = {}; }
  const applyPanels = () => {
    panels.forEach(id => {
      $(id).classList.toggle('collapsed', !!pst[id + '_c']); $(id).classList.toggle('hidden', !!pst[id + '_h']);
      const chev = document.querySelector('[data-collapse="' + id + '"]'); const key = pst[id + '_c'] ? 'Expand' : 'Collapse';
      chev.setAttribute('aria-label', RC.t(key)); chev.title = RC.t(key);
      document.querySelector('[data-show="' + id + '"]').style.display = pst[id + '_h'] ? '' : 'none';
    });
    try { localStorage.setItem('wms-twin-panels', JSON.stringify(pst)); } catch (e) {}
  };
  document.querySelectorAll('[data-collapse]').forEach(b => b.addEventListener('click', () => { pst[b.dataset.collapse + '_c'] = !pst[b.dataset.collapse + '_c']; applyPanels(); }));
  document.querySelectorAll('[data-hide]').forEach(b => b.addEventListener('click', () => { pst[b.dataset.hide + '_h'] = true; applyPanels(); }));
  document.querySelectorAll('[data-show]').forEach(b => b.addEventListener('click', () => { pst[b.dataset.show + '_h'] = false; applyPanels(); }));
  applyPanels();
})();
