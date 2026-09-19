'use strict';
// S10 teach page (采集助手): polls /phone/teach/status, draws a top view, sends actions.
// Records only; never commands the robot.
(() => {
  const $ = (id) => document.getElementById(id);
  let st = null, csrf = '', busy = false, formOpen = false, selectedWp = 'WP01', follow = false, lastResultSeq = 0, autoAdvanced = 0;
  const WPS = Array.from({length: 30}, (_, i) => 'WP' + String(i + 1).padStart(2, '0'));

  // ---------- helpers
  const fmtTime = (s) => s == null ? '—' : (s >= 3600 ? Math.floor(s / 3600) + ':' : '') +
    String(Math.floor(s % 3600 / 60)).padStart(2, '0') + ':' + String(Math.floor(s % 60)).padStart(2, '0');
  const fmtSize = (b) => b == null ? '—' : b >= 1e9 ? (b / 1e9).toFixed(2) + ' GB' : b >= 1e6 ? (b / 1e6).toFixed(1) + ' MB' : (b / 1e3).toFixed(0) + ' KB';
  const say = (text, tone) => { const m = $('msg'); m.textContent = text || ''; if (tone) m.dataset.tone = tone; else delete m.dataset.tone; };
  function chip(el, text, s) { el.textContent = text; if (s) el.dataset.s = s; else delete el.dataset.s; }

  async function get(path) {
    const r = await fetch(path, {cache: 'no-store', credentials: 'same-origin'});
    if (r.status === 401) { location.href = '/'; throw new Error('请先登录'); }
    const data = await r.json().catch(() => ({}));
    if (!r.ok) throw new Error(data.detail || ('HTTP ' + r.status));
    return data;
  }
  async function act(body, okText) {
    if (busy) return;
    busy = true; render();
    try {
      const r = await fetch('/phone/teach/submit', {method: 'POST', credentials: 'same-origin',
        headers: {'Content-Type': 'application/json', 'X-CSRF-Token': csrf}, body: JSON.stringify(body)});
      const data = await r.json().catch(() => ({}));
      if (!r.ok) throw new Error(data.detail || ('HTTP ' + r.status));
      if (okText) say(okText, 'good'); else say('');
      await poll();
      return data;
    } catch (e) { say(e.message, 'bad'); }
    finally { busy = false; render(); }
  }

  // ---------- WP grid
  const grid = $('wpGrid');
  for (const w of WPS) {
    const b = document.createElement('button'); b.type = 'button'; b.textContent = w.slice(2); b.dataset.wp = w;
    b.setAttribute('aria-label', w); b.addEventListener('click', () => { selectedWp = w; render(); });
    grid.appendChild(b);
  }

  // ---------- tabs
  function showTab(id) {
    for (const t of document.querySelectorAll('.tabs button')) t.setAttribute('aria-selected', String(t.dataset.tab === id));
    for (const p of document.querySelectorAll('.tabpanel')) p.hidden = p.id !== id;
    try { localStorage.setItem('teachTab', id); } catch (e) { /* storage may be blocked */ }
  }
  for (const t of document.querySelectorAll('.tabs button')) t.addEventListener('click', () => showTab(t.dataset.tab));
  try { const saved = localStorage.getItem('teachTab'); if (saved && $(saved)) showTab(saved); } catch (e) { /* ignore */ }

  // ---------- map drawing
  const cv = $('map'), ctx = cv.getContext('2d');
  function draw() {
    const dpr = window.devicePixelRatio || 1, w = cv.clientWidth, h = cv.clientHeight;
    if (cv.width !== Math.round(w * dpr) || cv.height !== Math.round(h * dpr)) { cv.width = Math.round(w * dpr); cv.height = Math.round(h * dpr); }
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0); ctx.clearRect(0, 0, w, h);
    if (!st) return;
    const trail = st.trail || [], marks = (st.marks || []).filter(m => m.pose), pose = st.pose;
    const pts = trail.map(p => [p[0], p[1]]).concat(marks.map(m => [m.pose[0], m.pose[1]]));
    if (pose) pts.push([pose.x, pose.y]);
    if (st.loop) pts.push([st.loop.start[0], st.loop.start[1]]);
    if (!pts.length) { ctx.fillStyle = '#6b7c73'; ctx.font = '14px system-ui'; ctx.fillText('等待新 SLAM 位姿…', 14, 26); return; }
    let x0, x1, y0, y1;
    if (follow && pose) { x0 = pose.x - 10; x1 = pose.x + 10; y0 = pose.y - 10; y1 = pose.y + 10; }
    else {
      x0 = Math.min(...pts.map(p => p[0])); x1 = Math.max(...pts.map(p => p[0]));
      y0 = Math.min(...pts.map(p => p[1])); y1 = Math.max(...pts.map(p => p[1]));
      const pad = Math.max(2, 0.08 * Math.max(x1 - x0, y1 - y0)); x0 -= pad; x1 += pad; y0 -= pad; y1 += pad;
    }
    const s = Math.min(w / (x1 - x0), h / (y1 - y0)), ox = (w - s * (x1 - x0)) / 2, oy = (h - s * (y1 - y0)) / 2;
    const X = (x) => ox + (x - x0) * s, Y = (y) => h - (oy + (y - y0) * s);
    // grid
    const span = Math.max(x1 - x0, y1 - y0), step = span > 60 ? 10 : span > 25 ? 5 : span > 8 ? 2 : 1;
    ctx.strokeStyle = '#e4eae4'; ctx.lineWidth = 1; ctx.beginPath();
    for (let gx = Math.ceil(x0 / step) * step; gx <= x1; gx += step) { ctx.moveTo(X(gx), 0); ctx.lineTo(X(gx), h); }
    for (let gy = Math.ceil(y0 / step) * step; gy <= y1; gy += step) { ctx.moveTo(0, Y(gy)); ctx.lineTo(w, Y(gy)); }
    ctx.stroke();
    ctx.fillStyle = '#7b8b82'; ctx.font = '11px system-ui'; ctx.fillText('网格 ' + step + ' m', 8, h - 8);
    // trail
    if (trail.length > 1) {
      ctx.strokeStyle = st.recording && st.recording.mode === 'path' ? '#d9822b' : '#285f9e'; ctx.lineWidth = 2.5; ctx.lineJoin = 'round';
      ctx.beginPath(); ctx.moveTo(X(trail[0][0]), Y(trail[0][1]));
      for (const p of trail) ctx.lineTo(X(p[0]), Y(p[1]));
      ctx.stroke();
    }
    // loop start
    if (st.loop) {
      const sx = X(st.loop.start[0]), sy = Y(st.loop.start[1]);
      ctx.fillStyle = '#e0a100'; ctx.beginPath(); ctx.arc(sx, sy, 9, 0, 7); ctx.fill();
      ctx.fillStyle = '#5b4300'; ctx.font = 'bold 12px system-ui'; ctx.fillText('起点', sx + 11, sy - 8);
    }
    // marks
    for (const m of marks) {
      const mx = X(m.pose[0]), my = Y(m.pose[1]), ok = !m.result || m.result.passed;
      if (m.kind === 'WP') {
        ctx.fillStyle = ok ? (m.result && m.result.warn ? '#c28a1c' : '#166b54') : '#a3382c';
        ctx.beginPath(); ctx.arc(mx, my, 7, 0, 7); ctx.fill();
        ctx.fillStyle = '#10261c'; ctx.font = 'bold 12px system-ui'; ctx.fillText(m.wp_id.slice(2), mx + 9, my + 4);
      } else if (m.kind === 'SWIN' || m.kind === 'SWOUT') {
        ctx.fillStyle = ok ? '#6b3fa0' : '#a3382c'; ctx.beginPath();
        const d = m.kind === 'SWIN' ? -1 : 1;
        ctx.moveTo(mx, my + 8 * d); ctx.lineTo(mx - 7, my - 6 * d); ctx.lineTo(mx + 7, my - 6 * d); ctx.closePath(); ctx.fill();
      } else if (m.kind === 'WP_PASS') {
        ctx.strokeStyle = '#166b54'; ctx.lineWidth = 2; ctx.beginPath(); ctx.arc(mx, my, 6, 0, 7); ctx.stroke();
      }
    }
    // robot
    if (pose) {
      const rx = X(pose.x), ry = Y(pose.y), a = pose.yaw_deg * Math.PI / 180;
      ctx.save(); ctx.translate(rx, ry); ctx.rotate(-a);
      ctx.fillStyle = '#a3382c'; ctx.beginPath(); ctx.moveTo(14, 0); ctx.lineTo(-9, 8); ctx.lineTo(-5, 0); ctx.lineTo(-9, -8); ctx.closePath(); ctx.fill();
      ctx.restore();
    }
  }
  window.addEventListener('resize', draw);
  $('btnFit').addEventListener('click', () => { follow = false; $('btnFollow').setAttribute('aria-pressed', 'false'); draw(); });
  $('btnFollow').addEventListener('click', () => { follow = !follow; $('btnFollow').setAttribute('aria-pressed', String(follow)); draw(); });
  $('xnavLink').href = location.protocol + '//' + location.hostname + ':' + (location.port === '18080' ? '18000' : '8000') + '/';

  // ---------- render
  function render() {
    const s = st, hasSession = !!(s && s.session), rec = s && s.recording, job = s && s.job;
    const rosOk = s && s.ros && s.ros.ok;
    $('fakeTag').hidden = !(s && s.fake);
    if (s) {
      const t = s.topics;
      const rate = (x, lo) => x.age == null || x.age > 2 ? 'bad' : x.hz >= lo ? 'good' : 'warn';
      chip($('cLidar'), '点云 ' + (t.lidar.age == null || t.lidar.age > 2 ? '无数据' : t.lidar.hz + ' Hz'), rate(t.lidar, 8));
      chip($('cImu'), 'IMU ' + (t.imu.age == null || t.imu.age > 2 ? '无数据' : t.imu.hz + ' Hz'), rate(t.imu, 150));
      chip($('cPose'), '定位 ' + (t.pose.age == null || t.pose.age > 2 ? '无数据' : t.pose.hz + ' Hz'), rate(t.pose, 5));
      chip($('cDisk'), '磁盘剩余 ' + s.disk.free_gb + ' GB', s.disk.free_gb < 5 ? 'bad' : s.disk.free_gb < s.disk.mapping_min_gb ? 'warn' : 'good');
      chip($('cRec'), rec ? '● 录制中 ' + ({mapping: '建图采集', survey: '标点', path: '示教'}[rec.mode]) + ' ' + fmtTime(rec.elapsed_s) : '未录制', rec ? 'rec' : '');
      const conn = $('conn');
      if (!rosOk) { conn.textContent = s.ros.error || 'ROS 未连接'; conn.dataset.tone = 'bad'; }
      else if (s.pose_error) { conn.textContent = s.pose_error; conn.dataset.tone = 'bad'; }
      else if (t.pose.age == null || t.pose.age > 2) { conn.textContent = '没有新 SLAM 位姿（' + s.config.pose_topic + '）：确认 x_nav 已启动。建图采集仍可只录点云和 IMU。'; conn.dataset.tone = ''; }
      else { conn.textContent = '已连接 · 位姿话题 ' + s.config.pose_topic + (s.pose_frame ? '（坐标系 ' + s.pose_frame + '）' : ''); conn.dataset.tone = 'good'; }
      const p = s.pose;
      $('poseText').textContent = p ? `x ${p.x.toFixed(2)}  y ${p.y.toFixed(2)}  z ${p.z.toFixed(2)} m  朝向 ${p.yaw_deg}°` : '';
      $('sessionInfo').innerHTML = hasSession
        ? `当前会话：<b>${escapeHtml(s.session.id)}</b>${s.session.map_name ? ' · 地图 ' + escapeHtml(s.session.map_name) : ''}`
        : '还没有会话：先新建一个。';
    }
    const recMode = rec ? rec.mode : null, stopping = rec && rec.state === 'stopping';
    // mapping tab
    $('btnMapStart').hidden = recMode === 'mapping'; $('btnMapStop').hidden = recMode !== 'mapping';
    $('btnMapStart').disabled = busy || !hasSession || !!rec || !rosOk;
    $('btnMapStop').disabled = busy || stopping;
    $('mTime').textContent = recMode === 'mapping' ? fmtTime(rec.elapsed_s) : '—';
    $('mSize').textContent = recMode === 'mapping' ? fmtSize(rec.bytes) : '—';
    $('mLoop').textContent = s && s.loop ? s.loop.dist.toFixed(2) + ' m' : '—';
    // survey tab
    $('btnSurveyStop').hidden = recMode !== 'survey';
    $('btnSurveyStop').disabled = busy || stopping;
    const poseFresh = s && s.topics.pose.age != null && s.topics.pose.age < 1;
    const canMark = !busy && hasSession && !job && poseFresh && (!rec || recMode === 'survey');
    $('btnMarkWp').textContent = '标 ' + selectedWp + '（停稳 3 秒）';
    $('btnMarkWp').disabled = !canMark; $('btnSwin').disabled = !canMark; $('btnSwout').disabled = !canMark;
    $('btnRedo').disabled = busy || !hasSession;
    const board = (s && s.board) || {};
    for (const b of grid.children) {
      b.dataset.s = board[b.dataset.wp] || 'none';
      b.setAttribute('aria-pressed', String(b.dataset.wp === selectedWp));
    }
    $('jobBox').hidden = !job;
    if (job) {
      $('jobBar').style.width = Math.round(job.progress * 100) + '%';
      $('jobText').textContent = `正在采样 ${job.kind === 'WP' ? job.wp_id : job.kind}：保持不动（${job.samples} 个位姿）`;
    }
    const r = s && s.last_result;
    const box = $('result');
    if (r && r.result) {
      box.hidden = false;
      const name = r.kind === 'WP' ? r.wp_id : (r.kind === 'SWIN' ? '▲ SWIN' : '▼ SWOUT');
      box.dataset.ok = String(!!r.result.passed); box.dataset.warn = String(!!r.result.warn);
      box.textContent = r.result.passed
        ? `✓ ${name} 已保存：离散 ${(r.result.std_xy * 100).toFixed(1)} cm / ${r.result.yaw_std_deg.toFixed(1)}°${r.result.warn ? '（偏大，建议重标）' : ''}`
        : `✗ ${name} 不合格，请停稳后重标：${(r.result.reasons || []).join('；')}`;
      if (r.seq !== lastResultSeq) {
        lastResultSeq = r.seq;
        if (r.kind === 'WP' && r.result.passed && autoAdvanced !== r.seq) {
          autoAdvanced = r.seq;
          const next = WPS.find((w, i) => i > WPS.indexOf(r.wp_id) && board[w] !== 'pass');
          if (next) selectedWp = next;
        }
      }
    } else box.hidden = true;
    const pairs = (s && s.pairs) || [];
    const open = pairs.length && pairs[pairs.length - 1].swout == null;
    $('pairInfo').textContent = `已标 ${pairs.filter(p => p.swout != null).length} 对切换点` + (open ? '；有一个 ▲SWIN 还没配 ▼SWOUT' : '');
    // path tab
    $('btnPathStart').hidden = recMode === 'path'; $('btnPathStop').hidden = recMode !== 'path';
    $('btnPathStart').disabled = busy || !hasSession || !!rec || !poseFresh;
    $('btnPathStop').disabled = busy || stopping;
    if (s && s.path) {
      $('pLen').textContent = s.path.length_m.toFixed(1) + ' m'; $('pTime').textContent = fmtTime(s.path.elapsed_s);
      $('pSpeed').textContent = s.path.elapsed_s > 1 ? (s.path.length_m / s.path.elapsed_s).toFixed(2) + ' m/s' : '—';
    }
    $('btnNew').disabled = busy || !!rec;
    $('sessionForm').hidden = hasSession && !formOpen; $('btnShowForm').hidden = !hasSession || formOpen;
    $('btnShowForm').disabled = busy || !!rec;
    draw();
  }
  function escapeHtml(t) { return String(t).replace(/[&<>"']/g, c => ({'&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;'}[c])); }

  // ---------- actions
  $('btnNew').addEventListener('click', () => act({action: 'session_new', label: $('sLabel').value.trim() || 'field', map_name: ''}, '已新建会话').then((r) => { if (r) formOpen = false; render(); }));
  $('btnShowForm').addEventListener('click', () => { formOpen = true; render(); });
  $('btnMapStart').addEventListener('click', () => act({action: 'record_start', mode: 'mapping'}, '开始采集：先原地静止 5 秒，再出发'));
  $('btnMapStop').addEventListener('click', () => { if (confirm('确定结束采集？（请先停回起点脚印框）')) act({action: 'record_stop'}, '采集已结束：去 x_nav 保存地图并记下地图名'); });
  $('btnSurveyStop').addEventListener('click', () => act({action: 'record_stop'}, '标点数据已保存'));
  // Marking starts the survey recording by itself, so there is one button less to forget.
  async function mark(body) {
    if (st && !st.recording) { const r = await act({action: 'record_start', mode: 'survey'}); if (!r) return; }
    return act(body);
  }
  $('btnMarkWp').addEventListener('click', () => mark({action: 'mark', kind: 'WP', wp_id: selectedWp}));
  $('btnSwin').addEventListener('click', () => mark({action: 'mark', kind: 'SWIN'}));
  $('btnSwout').addEventListener('click', () => mark({action: 'mark', kind: 'SWOUT'}));
  $('btnRedo').addEventListener('click', () => { if (confirm('作废上一个标记？')) act({action: 'redo'}, '已作废上一个标记'); });
  $('btnPathStart').addEventListener('click', () => act({action: 'record_start', mode: 'path'}, '开始示教：连续走完 WP01 → WP30'));
  $('btnPathStop').addEventListener('click', () => { if (confirm('确定结束示教？')) act({action: 'record_stop'}, '示教路径已保存'); });

  // ---------- polling
  async function poll() {
    try {
      st = await get('/phone/teach/status');
      csrf = st.csrf || csrf;
    } catch (e) {
      st = null;
      const c = $('conn'); c.textContent = e.message; c.dataset.tone = 'bad';
    }
    render();
  }
  poll();
  setInterval(() => { if (!document.hidden) poll(); }, 1000);
})();
