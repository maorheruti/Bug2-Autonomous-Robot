// UiObsMap.h — Obstacle Map live page served at /obsmap_ui
#pragma once

const char UI_OBSMAP_HTML[] PROGMEM = R"OBSHTML(
<!DOCTYPE html>
<html>
<head>
  <meta charset="utf-8">
  <title>Obstacle Map – Live</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>
    body{margin:0;background:#111;color:#eee;font-family:system-ui,sans-serif;}
    canvas{display:block;margin:8px auto;border:1px solid #333;background:#000;}
    h2{text-align:center;margin:8px 0 4px;}
    .bar{display:flex;justify-content:center;gap:10px;padding:4px 0;}
    .bar button{background:#2f7dff;border:none;padding:6px 14px;border-radius:10px;
      color:#fff;cursor:pointer;font-weight:600;font-size:0.85rem;}
    .bar button.stop{background:#ff3b3b;}
    .bar button.ghost{background:#3a3a3a;}
    .info{text-align:center;font-size:0.85rem;color:#aaa;margin-bottom:4px;}
    .badge{display:inline-block;padding:2px 8px;border-radius:999px;background:#2b2b2b;
      font-size:0.8rem;margin:0 4px;}
    .vtx-list{max-height:180px;overflow-y:auto;font-size:0.75rem;font-family:monospace;
      padding:8px 16px;background:#1a1a1a;border-top:1px solid #333;}
  </style>
</head>
<body>
  <h2>Obstacle Map — Live</h2>
  <div class="bar">
    <button id="btnRefresh">Refresh Now</button>
    <button id="btnClear" class="stop">Clear Map</button>
    <button id="btnAuto" class="ghost">Auto: OFF</button>
    <span class="badge" id="wsBadge">ws: —</span>
  </div>
  <div class="info" id="info">Loading…</div>
  <canvas id="obsCanvas" width="800" height="540"></canvas>
  <div class="vtx-list" id="vtxList"></div>

<script>
(function(){
  const BASE = window.location.origin;
  const canvas = document.getElementById('obsCanvas');
  const ctx = canvas.getContext('2d');
  const infoEl = document.getElementById('info');
  const vtxEl  = document.getElementById('vtxList');
  const wsBadge = document.getElementById('wsBadge');
  const btnAuto = document.getElementById('btnAuto');

  let autoMode = false;
  let autoTimer = null;
  let latestRobot = null;
  let latestTarget = null;

  // --- Board bounds (same as main dashboard) ---
  const board = { xmin:-1.6, xmax:4.0, ymin:-1.6, ymax:1.8 };

  function w2c(x, y) {
    return {
      x: (x - board.xmin) / (board.xmax - board.xmin) * canvas.width,
      y: (y - board.ymin) / (board.ymax - board.ymin) * canvas.height
    };
  }

  // --- Drawing ---
  function drawGrid() {
    const W = canvas.width, H = canvas.height;
    ctx.fillStyle = '#000';
    ctx.fillRect(0, 0, W, H);
    ctx.strokeStyle = '#222'; ctx.lineWidth = 1;
    ctx.beginPath();
    for(let gx = Math.ceil(board.xmin); gx <= board.xmax; gx += 1){
      const p1 = w2c(gx, board.ymin), p2 = w2c(gx, board.ymax);
      ctx.moveTo(p1.x, p1.y); ctx.lineTo(p2.x, p2.y);
    }
    for(let gy = Math.ceil(board.ymin); gy <= board.ymax; gy += 1){
      const p1 = w2c(board.xmin, gy), p2 = w2c(board.xmax, gy);
      ctx.moveTo(p1.x, p1.y); ctx.lineTo(p2.x, p2.y);
    }
    ctx.stroke();

    // Axis labels
    ctx.fillStyle = '#666'; ctx.font = '11px monospace';
    ctx.textBaseline = 'top';
    for(let gx = Math.ceil(board.xmin); gx <= board.xmax; gx += 1){
      const p = w2c(gx, board.ymin);
      ctx.fillText(gx.toFixed(0), p.x+2, H - 14);
    }
    ctx.textBaseline = 'middle';
    for(let gy = Math.ceil(board.ymin); gy <= board.ymax; gy += 1){
      const p = w2c(board.xmin, gy);
      ctx.fillText(gy.toFixed(0), 4, p.y);
    }
  }

  function drawRobot() {
    if (!latestRobot || latestRobot.x === undefined) return;
    const r = latestRobot;
    const pc = w2c(r.x, r.y);
    const yaw = r.yaw || 0;
    const len = 12, hw = 6;
    const cosY = Math.cos(yaw), sinY = Math.sin(yaw);

    // Convert world offsets to canvas pixels
    const sx = canvas.width / (board.xmax - board.xmin);
    const sy = canvas.height / (board.ymax - board.ymin);

    const tip = { x: pc.x + len*cosY*sx/100, y: pc.y + len*sinY*sy/100 };
    const bl  = { x: pc.x - len*cosY*sx/100 - hw*sinY*sx/100,
                  y: pc.y - len*sinY*sy/100 + hw*cosY*sy/100 };
    const br  = { x: pc.x - len*cosY*sx/100 + hw*sinY*sx/100,
                  y: pc.y - len*sinY*sy/100 - hw*cosY*sy/100 };

    ctx.fillStyle = 'rgba(77,208,225,0.7)';
    ctx.beginPath();
    ctx.moveTo(tip.x, tip.y);
    ctx.lineTo(bl.x, bl.y);
    ctx.lineTo(br.x, br.y);
    ctx.closePath();
    ctx.fill();

    // Small label
    ctx.fillStyle = '#4dd0e1';
    ctx.font = '10px monospace';
    ctx.fillText('R(' + r.x.toFixed(1) + ',' + r.y.toFixed(1) + ')', pc.x + 10, pc.y - 8);
  }

  function drawTarget() {
    if (!latestTarget || latestTarget.x === undefined) return;
    const t = latestTarget;
    const pc = w2c(t.x, t.y);
    ctx.fillStyle = '#ffcc00';
    ctx.beginPath();
    ctx.arc(pc.x, pc.y, 6, 0, Math.PI*2);
    ctx.fill();
    ctx.fillStyle = '#ffcc00';
    ctx.font = '10px monospace';
    ctx.fillText('T(' + t.x.toFixed(1) + ',' + t.y.toFixed(1) + ')', pc.x + 8, pc.y - 8);
  }

  // Compute hull boundary indices for a segment (outermost vertices)
  function segEndpoints(seg) {
    if (seg.length <= 2) return seg.map((_, i) => i);
    // Return only first and last (polyline endpoints)
    return [0, seg.length - 1];
  }

  function renderObsMap(data) {
    drawGrid();

    if (!data) { drawRobot(); drawTarget(); return; }

    const iso = data.iso || [];
    const segs = data.seg || [];

    // Draw isolated points (orange dots, small)
    ctx.fillStyle = '#e67e22';
    iso.forEach(([x,y]) => {
      const p = w2c(x, y);
      ctx.beginPath();
      ctx.arc(p.x, p.y, 3, 0, Math.PI*2);
      ctx.fill();
    });

    // Draw segments
    let segInfo = [];
    segs.forEach((seg, si) => {
      if (seg.length < 2) return;

      // Draw lines (green)
      ctx.strokeStyle = '#2ecc71';
      ctx.lineWidth = 2;
      ctx.beginPath();
      const p0 = w2c(seg[0][0], seg[0][1]);
      ctx.moveTo(p0.x, p0.y);
      for(let i = 1; i < seg.length; i++){
        const p = w2c(seg[i][0], seg[i][1]);
        ctx.lineTo(p.x, p.y);
      }
      ctx.stroke();

      // Only label the two endpoints (first + last vertex)
      const endIdx = segEndpoints(seg);
      endIdx.forEach(vi => {
        const [x, y] = seg[vi];
        const p = w2c(x, y);
        ctx.fillStyle = '#4dd0e1';
        ctx.beginPath();
        ctx.arc(p.x, p.y, 4, 0, Math.PI*2);
        ctx.fill();
        ctx.fillStyle = '#fff';
        ctx.font = '10px monospace';
        ctx.fillText('(' + x.toFixed(1) + ',' + y.toFixed(1) + ')', p.x+5, p.y-5);
      });

      // Inner vertices: small dot only, no label
      for(let i = 0; i < seg.length; i++){
        if (endIdx.includes(i)) continue;
        const p = w2c(seg[i][0], seg[i][1]);
        ctx.fillStyle = '#1a7a42';
        ctx.beginPath();
        ctx.arc(p.x, p.y, 2, 0, Math.PI*2);
        ctx.fill();
      }

      const first = seg[0], last = seg[seg.length-1];
      segInfo.push('S' + si + ' (' + seg.length + 'pts): (' +
        first[0].toFixed(1) + ',' + first[1].toFixed(1) + ') → (' +
        last[0].toFixed(1) + ',' + last[1].toFixed(1) + ')');
    });

    // Draw robot + target on top
    drawRobot();
    drawTarget();

    // Info bar
    const totalPts = iso.length + segs.reduce((a, s) => a + s.length, 0);
    infoEl.textContent = 'Isolated: ' + iso.length + '  Segments: ' + segs.length +
                          '  Vertices: ' + totalPts;

    // Vertex list
    let html = '';
    if (segInfo.length > 0) {
      html += '<b>Segments (endpoints):</b><br>';
      segInfo.forEach(s => { html += s + '<br>'; });
    }
    if (iso.length > 0) {
      html += '<b>Isolated (' + iso.length + '):</b> ';
      const show = iso.slice(0, 20);
      html += show.map(([x,y]) => '(' + x.toFixed(1) + ',' + y.toFixed(1) + ')').join(' ');
      if (iso.length > 20) html += ' …(' + (iso.length - 20) + ' more)';
      html += '<br>';
    }
    vtxEl.innerHTML = html;
  }

  // --- Fetch obstacle data ---
  let lastObsData = null;

  async function fetchObs() {
    try {
      const resp = await fetch(BASE + '/obsmap');
      lastObsData = await resp.json();
      renderObsMap(lastObsData);
    } catch(e) {
      infoEl.textContent = 'Fetch error: ' + e;
    }
  }

  // --- WebSocket for live robot/target ---
  let ws;
  function connectWS() {
    const proto = (location.protocol === 'https:') ? 'wss' : 'ws';
    ws = new WebSocket(proto + '://' + location.hostname + ':81/');
    wsBadge.textContent = 'ws: connecting';
    wsBadge.style.background = '#555';

    ws.onopen = () => {
      wsBadge.textContent = 'ws: connected';
      wsBadge.style.background = '#1e8e3e';
    };
    ws.onclose = () => {
      wsBadge.textContent = 'ws: disconnected';
      wsBadge.style.background = '#555';
      setTimeout(connectWS, 2000);
    };
    ws.onmessage = ev => {
      try {
        const j = JSON.parse(ev.data);
        if (j.robot) latestRobot = j.robot;
        if (j.target) latestTarget = j.target;
        // Re-render with latest obs data + new robot/target
        renderObsMap(lastObsData);
      } catch(e) {}
    };
  }
  connectWS();

  // --- Auto-refresh obstacle data ---
  btnAuto.onclick = () => {
    autoMode = !autoMode;
    btnAuto.textContent = 'Auto: ' + (autoMode ? 'ON' : 'OFF');
    btnAuto.style.background = autoMode ? '#1e8e3e' : '#3a3a3a';
    if (autoMode) {
      autoTimer = setInterval(fetchObs, 2000);
    } else {
      if (autoTimer) clearInterval(autoTimer);
      autoTimer = null;
    }
  };

  document.getElementById('btnRefresh').onclick = fetchObs;
  document.getElementById('btnClear').onclick = async () => {
    await fetch(BASE + '/obsmap_clear');
    lastObsData = null;
    renderObsMap(null);
    infoEl.textContent = 'Map cleared.';
    vtxEl.innerHTML = '';
  };

  // Resize canvas
  function resize() {
    const rect = canvas.getBoundingClientRect();
    canvas.width = Math.min(rect.width, 900);
    canvas.height = Math.round(canvas.width * 0.65);
    renderObsMap(lastObsData);
  }
  window.addEventListener('resize', resize);

  // Initial fetch
  fetchObs();
})();
</script>
</body>
</html>
)OBSHTML";
