// UiDashboard.h
#pragma once
const char UI_DASHBOARD_HTML[] PROGMEM = R"HTML(
<!DOCTYPE html>
<html>
<head>
  <meta charset="utf-8">
  <title>Bug2 / NetESP Dashboard</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>
    body{
      font-family: system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
      margin: 16px;
      background: #111;
      color: #eee;
    }
    .row{ display:flex; flex-wrap:wrap; gap:12px; align-items:flex-start; }
    .card{
      background:#1c1c1c;
      border-radius:16px;
      padding:12px 16px;
      box-shadow:0 2px 12px rgba(0,0,0,.4);
      margin-bottom:12px;
    }
    h1{ margin-top:0; font-size:1.4rem;}
    h3{ margin:0 0 8px 0; font-size:1.1rem;}
    .badge{
      display:inline-block;
      padding:0.2rem 0.5rem;
      border-radius:999px;
      background:#2b2b2b;
      font-size:0.8rem;
    }
    .btn{
      background:#2f7dff;
      border:none;
      padding:0.5rem 0.9rem;
      border-radius:12px;
      color:#fff;
      cursor:pointer;
      font-weight:600;
      font-size:0.9rem;
    }
    .btn.stop{ background:#ff3b3b; }
    .btn.ghost{ background:#3a3a3a; }
    .mono{ font-family: ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, "Liberation Mono", "Courier New", monospace; }
    small{ opacity:.7; }
    canvas.map{
      width:100%;
      height:260px;
      background:#000;
      border-radius:12px;
      border:1px solid #333;
      display:block;
    }
    .map-container{
      flex:1;
      min-width:260px;
    }
    table{
      width:100%;
      border-collapse:collapse;
      font-size:0.85rem;
    }
    th,td{
      padding:4px 6px;
      border-bottom:1px solid #222;
    }
    th{ text-align:left; color:#aaa; font-weight:500;}
    #log{
      max-height:120px;
      overflow-y:auto;
      font-size:0.78rem;
      font-family: ui-monospace, monospace;
      background:#050505;
      border-radius:8px;
      padding:6px;
      border:1px solid #222;
    }
    #log div{ margin-bottom:2px; }
    #dump{
      width:100%;
      min-height:110px;
      max-height:220px;
      resize:vertical;
      background:#050505;
      border:1px solid #222;
      border-radius:8px;
      padding:8px;
      color:#ddd;
      font-size:0.78rem;
      font-family: ui-monospace, monospace;
      white-space:pre;
      overflow:auto;
      display:none;
    }
    select, input{ background:#111; color:#eee; border:1px solid #333; border-radius:8px; padding:6px 8px; }
  </style>
</head>
<body>
  <h1>Bug2 / NetESP – Live Dashboard</h1>

  <div class="card">
    <div class="row">
      <button id="btnStart" class="btn">Start</button>
      <button id="btnStop"  class="btn stop">Stop</button>
      <span id="stateBadge" class="badge">disconnected</span>
      <span id="reasonBadge" class="badge mono"></span>
    </div>
    <small>Control prefers WebSocket (port 81). HTTP is used only as a fallback.</small>
  </div>

  <div class="card">
    <h3>Navigate (slow)</h3>
    <div class="row">
      <label class="mono">x <input id="navX" type="number" step="0.01" value="0.00" style="width:110px;"></label>
      <label class="mono">y <input id="navY" type="number" step="0.01" value="0.00" style="width:110px;"></label>
      <button id="btnNavGo" class="btn">Nav Go</button>
      <button id="btnNavStop" class="btn stop">Nav Stop</button>
    </div>
    <small>Nav drives slowly to (x,y) using pure pursuit.</small>
  </div>

  <div class="card">
    <h3>Bug2</h3>
    <div class="row">
      <label class="mono">x <input id="bugX" type="number" step="0.01" value="0.00" style="width:110px;"></label>
      <label class="mono">y <input id="bugY" type="number" step="0.01" value="0.00" style="width:110px;"></label>
      <button id="btnBug2Go" class="btn" style="background:#7b61ff;">Bug2 Go</button>
      <button id="btnBug2Stop" class="btn stop">Bug2 Stop</button>

      <span class="badge mono">pick →</span>
      <select id="pickTarget">
        <option value="nav">nav</option>
          <option value="wf">wf</option>
          <option value="bug2">bug2</option>
      </select>
      <button id="btnPick" class="btn" style="background:#444;">Pick from map</button>
      <span class="badge mono" id="pickBadge">pick: off</span>
    </div>
    <small>Workflow: (optional) Nav to a good start pose → Bug2 Go to a goal.</small>
  </div>

    <div class="card">
      <h3>WF Mode</h3>
      <div class="row">
        <label class="mono">x <input id="wfX" type="number" step="0.01" value="0.00" style="width:110px;"></label>
        <label class="mono">y <input id="wfY" type="number" step="0.01" value="0.00" style="width:110px;"></label>
        <button id="btnWfGo" class="btn" style="background:#3aa56b;">WF Go</button>
        <button id="btnWfStop" class="btn stop">WF Stop</button>
      </div>
      <small>WF-only follow testing (no leave-to-goal).</small>
    </div>

  <div class="card">
    <h3>Field Views</h3>
    <div style="margin-bottom:8px;">
      <label style="display:inline;">
        <input type="checkbox" id="chkCalRays"> Show calibration rays (FoV + zones)
      </label>
    </div>
    <div class="row">
      <div class="map-container">
        <strong>Live pose</strong>
        <canvas id="mapPose" class="map"></canvas>
        <small>Robot triangle + target dot.</small>
      </div>
      <div class="map-container">
        <strong>Trajectory & obstacles</strong>
        <canvas id="mapTrail" class="map"></canvas>
        <small>Cyan path = robot trajectory. Red squares = accumulated obstacle hits.</small>
      </div>
    </div>
  </div>

  <div class="card">
    <h3>Telemetry</h3>
    <table class="mono">
      <tr><th colspan="2">Robot</th><th colspan="2">Target</th></tr>
      <tr>
        <td>x: <span id="rx">—</span></td>
        <td>y: <span id="ry">—</span></td>
        <td>x: <span id="tx">—</span></td>
        <td>y: <span id="ty">—</span></td>
      </tr>
      <tr>
        <td>yaw: <span id="ryaw">—</span></td>
        <td>servo: <span id="servo">—</span></td>
        <td>yaw: <span id="tyaw">—</span></td>
        <td>started: <span id="started">—</span></td>
      </tr>
    </table>

    <table class="mono" style="margin-top:8px;">
      <tr><th colspan="2">Motors</th><th colspan="3">Sensors (mm)</th></tr>
      <tr>
        <td>A: <span id="mA">—</span></td>
        <td>B: <span id="mB">—</span></td>
        <td>L: <span id="dL">—</span></td>
        <td>M: <span id="dM">—</span></td>
        <td>R: <span id="dR">—</span></td>
      </tr>
    </table>
  </div>

  <div class="card">
    <h3>Sensor Matrix Debug</h3>
    <div style="display:flex; gap:16px; flex-wrap:wrap; margin-bottom:12px;">
      <div style="flex:1; min-width:140px;">
        <div style="margin-bottom:4px;">
          <strong>LEFT</strong> <small>(5CX, +70°)</small><br>
          <label style="display:inline; margin-right:4px; font-size:0.8rem;"><input type="checkbox" id="chkL0" checked> Z0</label>
          <label style="display:inline; margin-right:4px; font-size:0.8rem;"><input type="checkbox" id="chkL1" checked> Z1</label>
          <label style="display:inline; margin-right:4px; font-size:0.8rem;"><input type="checkbox" id="chkL2" checked> Z2</label>
          <label style="display:inline; font-size:0.8rem;"><input type="checkbox" id="chkL3" checked> Z3</label>
        </div>
        <canvas id="matrixL" width="128" height="128" style="border:1px solid #333; image-rendering:pixelated; width:128px; height:128px;"></canvas>
        <div id="matrixLInfo" class="mono" style="font-size:0.72rem; color:#aaa; margin-top:4px;">LEFT: hover a cell</div>
      </div>
      <div style="flex:1; min-width:140px;">
        <div style="margin-bottom:4px;">
          <strong>MID</strong> <small>(7CX, 0°)</small><br>
          <label style="display:inline; margin-right:4px; font-size:0.8rem;"><input type="checkbox" id="chkM0" checked> Z0</label>
          <label style="display:inline; margin-right:4px; font-size:0.8rem;"><input type="checkbox" id="chkM1" checked> Z1</label>
          <label style="display:inline; margin-right:4px; font-size:0.8rem;"><input type="checkbox" id="chkM2" checked> Z2</label>
          <label style="display:inline; font-size:0.8rem;"><input type="checkbox" id="chkM3" checked> Z3</label>
        </div>
        <canvas id="matrixM" width="128" height="128" style="border:1px solid #333; image-rendering:pixelated; width:128px; height:128px;"></canvas>
        <div id="matrixMInfo" class="mono" style="font-size:0.72rem; color:#aaa; margin-top:4px;">MID: hover a cell</div>
      </div>
      <div style="flex:1; min-width:140px;">
        <div style="margin-bottom:4px;">
          <strong>RIGHT</strong> <small>(5CX, -70°)</small><br>
          <label style="display:inline; margin-right:4px; font-size:0.8rem;"><input type="checkbox" id="chkR0" checked> Z0</label>
          <label style="display:inline; margin-right:4px; font-size:0.8rem;"><input type="checkbox" id="chkR1" checked> Z1</label>
          <label style="display:inline; margin-right:4px; font-size:0.8rem;"><input type="checkbox" id="chkR2" checked> Z2</label>
          <label style="display:inline; font-size:0.8rem;"><input type="checkbox" id="chkR3" checked> Z3</label>
        </div>
        <canvas id="matrixR" width="128" height="128" style="border:1px solid #333; image-rendering:pixelated; width:128px; height:128px;"></canvas>
        <div id="matrixRInfo" class="mono" style="font-size:0.72rem; color:#aaa; margin-top:4px;">RIGHT: hover a cell</div>
      </div>  
    </div>
    <div style="margin-top:4px;">
      <label style="display:inline; margin-right:12px; font-size:0.8rem;">
        <input type="checkbox" id="chkRaw"> Show raw (include masked rows)
      </label>
    </div>
    <small>Each 8×8 matrix: columns → zones (2 cols/zone). Hover over cells to see mm values. Green=close, Red=far, Gray=invalid, <span style="color:#5a3a0a;">Amber=masked (floor)</span>.</small>
  </div>

  <div class="card">
    <h3>Decision Log</h3>
    <div class="row" style="margin-bottom:8px;">
      <button id="btnDumpHit" class="btn ghost">Dump around last HIT</button>
      <button id="btnDump30" class="btn ghost">Dump last 30s</button>
      <button id="btnDumpHide" class="btn ghost">Hide dump</button>
      <button id="btnClearMarks" class="btn ghost">Clear marks</button>
      <a href="/obsmap_ui" target="_blank" class="btn" style="background:#e67e22;text-decoration:none;">Obstacle Map</a>
      <button id="btnObsClear" class="btn ghost">Clear Obs Map</button>
      <span class="badge mono" id="hitBadge">hit: —</span>
      <span class="badge mono" id="obsBadge">obs: —</span>
    </div>
    <div id="dump" class="mono"></div>
    <div id="log"></div>
  </div>

  <div class="card">
    <details>
      <summary><strong>Wi-Fi</strong> (expand)</summary>
      <div style="margin-top:10px;">
        <div class="row">
          <label class="mono">ssid <input id="wifiSsid" type="text" value="" style="width:170px;"></label>
          <label class="mono">pass <input id="wifiPass" type="password" value="" style="width:170px;"></label>
          <button id="btnWifiSet" class="btn">Apply Wi-Fi</button>
          <span id="wifiBadge" class="badge mono">wifi: --</span>
        </div>
        <div class="row" style="margin-top:8px;">
          <button id="btnWifiScan" class="btn ghost">Scan SSIDs</button>
          <select id="wifiScanList" style="min-width:210px;">
            <option value="">(scan first)</option>
          </select>
          <button id="btnWifiPick" class="btn ghost" title="Copy selected SSID to field">→ SSID</button>
        </div>
        <div style="margin-top:8px;">
          <strong>Saved Networks</strong>
          <div id="wifiSaved" class="mono" style="margin-top:6px; font-size:0.82rem;"></div>
        </div>
        <small>Saves to NVS and reconnects. Device keeps using last saved credentials until changed.</small>
      </div>
    </details>
  </div>

  <div class="card">
    <details>
      <summary><strong>WF Tune</strong> (expand)</summary>
      <div style="margin-top:10px;">
        <div class="row">
          <label class="mono">target <input id="wfTarget" type="number" step="1" style="width:90px;"></label>
          <label class="mono">distk <input id="wfDistK" type="number" step="0.1" style="width:90px;"></label>
          <label class="mono">yawk <input id="wfYawK" type="number" step="0.1" style="width:90px;"></label>
          <label class="mono">beark <input id="wfBearK" type="number" step="0.01" style="width:90px;"></label>
          <label class="mono">oppk <input id="wfOppK" type="number" step="0.1" style="width:90px;"></label>
          <label class="mono">frontk <input id="wfFrontK" type="number" step="0.1" style="width:90px;"></label>
        </div>
        <div class="row" style="margin-top:8px;">
          <label class="mono">zemerg <input id="wfZEmerg" type="number" step="1" style="width:90px;"></label>
          <label class="mono">zclose <input id="wfZClose" type="number" step="1" style="width:90px;"></label>
          <label class="mono">zmid <input id="wfZMid" type="number" step="1" style="width:90px;"></label>
          <label class="mono">drev <input id="wfDrev" type="number" step="1" style="width:90px;"></label>
          <label class="mono">dturn <input id="wfDturn" type="number" step="1" style="width:90px;"></label>
          <label class="mono">dwell <input id="wfDwell" type="number" step="1" style="width:90px;"></label>
        </div>
        <div class="row" style="margin-top:8px;">
          <label class="mono">fblkE <input id="wfFblkEc" type="number" step="0.01" style="width:90px;"></label>
          <label class="mono">fblkX <input id="wfFblkXc" type="number" step="0.01" style="width:90px;"></label>
          <label class="mono">fblkEmm <input id="wfFblkEmm" type="number" step="1" style="width:90px;"></label>
          <label class="mono">fblkXmm <input id="wfFblkXmm" type="number" step="1" style="width:90px;"></label>
        </div>
        <div class="row" style="margin-top:8px;">
          <label class="mono">dEf <input id="wfDendEfc" type="number" step="0.01" style="width:90px;"></label>
          <label class="mono">dEo <input id="wfDendEoc" type="number" step="0.01" style="width:90px;"></label>
          <label class="mono">dEfoll <input id="wfDendEfol" type="number" step="0.01" style="width:90px;"></label>
          <label class="mono">dXf <input id="wfDendXfc" type="number" step="0.01" style="width:90px;"></label>
          <label class="mono">dXo <input id="wfDendXoc" type="number" step="0.01" style="width:90px;"></label>
          <button id="btnWfTuneLoad" class="btn ghost">Load</button>
          <button id="btnWfTuneApply" class="btn">Apply</button>
          <button id="btnWfTuneReset" class="btn stop">Reset</button>
          <span id="wfTuneBadge" class="badge mono">wf: --</span>
        </div>
        <small>Live tuning via <span class="mono">/wf_tune</span> (no reflash).</small>
      </div>
    </details>
  </div>

<script>
(function(){
  const el = id => document.getElementById(id);

  const stateBadge  = el('stateBadge');
  const reasonBadge = el('reasonBadge');
  const logEl       = el('log');
  const dumpEl      = el('dump');
  const hitBadge    = el('hitBadge');

  const canvasPose  = el('mapPose');
  const canvasTrail = el('mapTrail');
  const ctxPose     = canvasPose.getContext('2d');
  const ctxTrail    = canvasTrail.getContext('2d');
  const chkCalRays  = el('chkCalRays');
  
  // Matrix debug panel
  const canvasMatL = el('matrixL');
  const canvasMatM = el('matrixM');
  const canvasMatR = el('matrixR');
  const ctxMatL = canvasMatL.getContext('2d');
  const ctxMatM = canvasMatM.getContext('2d');
  const ctxMatR = canvasMatR.getContext('2d');
  const matrixLInfo = el('matrixLInfo');
  const matrixMInfo = el('matrixMInfo');
  const matrixRInfo = el('matrixRInfo');
  
  const chkL = [el('chkL0'), el('chkL1'), el('chkL2'), el('chkL3')];
  const chkM = [el('chkM0'), el('chkM1'), el('chkM2'), el('chkM3')];
  const chkR = [el('chkR0'), el('chkR1'), el('chkR2'), el('chkR3')];
  const chkRaw = el('chkRaw');

  // Floor-rejection row masks (row 0 = floor)
  const maskRowsL = 2, maskRowsM = 1, maskRowsR = 1;

  const navX = el('navX'), navY = el('navY');
  const bugX = el('bugX'), bugY = el('bugY');
  const wfX = el('wfX'), wfY = el('wfY');
  const wifiSsid = el('wifiSsid'), wifiPass = el('wifiPass');
  const wifiBadge = el('wifiBadge');
  const wifiScanList = el('wifiScanList');
  const wifiSaved = el('wifiSaved');
  const wfTarget = el('wfTarget');
  const wfDistK = el('wfDistK');
  const wfYawK = el('wfYawK');
  const wfBearK = el('wfBearK');
  const wfOppK = el('wfOppK');
  const wfFrontK = el('wfFrontK');
  const wfZEmerg = el('wfZEmerg');
  const wfZClose = el('wfZClose');
  const wfZMid = el('wfZMid');
  const wfDrev = el('wfDrev');
  const wfDturn = el('wfDturn');
  const wfDwell = el('wfDwell');
  const wfFblkEc = el('wfFblkEc');
  const wfFblkXc = el('wfFblkXc');
  const wfFblkEmm = el('wfFblkEmm');
  const wfFblkXmm = el('wfFblkXmm');
  const wfDendEfc = el('wfDendEfc');
  const wfDendEoc = el('wfDendEoc');
  const wfDendEfol = el('wfDendEfol');
  const wfDendXfc = el('wfDendXfc');
  const wfDendXoc = el('wfDendXoc');
  const wfTuneBadge = el('wfTuneBadge');

  let wifiSsidDirty = false;

  const pickTarget = el('pickTarget');
  const pickBadge  = el('pickBadge');
  let pickMode = false;

  let robotPath = [];
  let obstaclePoints = [];
  
  // Matrix data storage (decoded from hex)
  let matrixL = new Int16Array(64);
  let matrixM = new Int16Array(64);
  let matrixR = new Int16Array(64);
  let statusL = new Uint8Array(64);
  let statusM = new Uint8Array(64);
  let statusR = new Uint8Array(64);

  // ---- Telemetry ring buffer (for debugging) ----
  const TELE_CAP = 1400; // ~70s if 20Hz
  let tele = [];
  let lastHitIdx = -1;

  function pushTele(frame){
    tele.push(frame);
    if (tele.length > TELE_CAP){
      const over = tele.length - TELE_CAP;
      tele.splice(0, over);
      if (lastHitIdx >= 0) lastHitIdx = Math.max(-1, lastHitIdx - over);
    }
  }
  
  // Decode hex matrix string into Int16Array
  function decodeHexMatrix(hexStr, target){
    if (!hexStr || hexStr.length !== 256) return false; // 64 values × 4 hex chars
    for(let i = 0; i < 64; i++){
      const hex = hexStr.substr(i*4, 4);
      const val = parseInt(hex, 16);
      // Convert from unsigned to signed 16-bit
      target[i] = val > 32767 ? val - 65536 : val;
    }
    return true;
  }

  function decodeHexStatus(hexStr, target){
    if (!hexStr || hexStr.length !== 128) return false; // 64 values × 2 hex chars
    for(let i = 0; i < 64; i++){
      const hex = hexStr.substr(i*2, 2);
      target[i] = parseInt(hex, 16) & 0xff;
    }
    return true;
  }

  function isValidCell(dist, status){
    return (dist > 0 && dist < 8191 && (status === 5 || status === 9));
  }
  
  // Draw 8×8 matrix on canvas with zone highlighting
  function drawMatrix(canvas, ctx, matrix, statuses, zoneVisibility, maskRows, showRaw){
    const sz = 16; // 8×8 pixels per cell (128 / 8 = 16)
    
    ctx.fillStyle = '#000';
    ctx.fillRect(0, 0, 128, 128);
    
    // Draw 8×8 grid with distance-based coloring
    for(let row = 0; row < 8; row++){
      for(let col = 0; col < 8; col++){
        const idx = row * 8 + col;
        const dist = matrix[idx];
        const status = statuses[idx];
        const zone = Math.floor(col / 2); // columns 0-1 → zone 0, 2-3 → zone 1, etc.
        const masked = (row < maskRows) && !showRaw;
        
        // Determine if zone is visible (checked)
        const visible = (zone < zoneVisibility.length) && zoneVisibility[zone];
        
        if(!visible){
          // Hide unchecked zones
          ctx.fillStyle = '#111';
        } else if(masked){
          // Masked row (floor rejection) — dark amber
          ctx.fillStyle = '#2a1a0a';
        } else if(!isValidCell(dist, status)){
          // Invalid/no data
          ctx.fillStyle = '#333';
        } else {
          // Valid: green close, red far
          ctx.fillStyle = (dist <= 1200) ? '#00d26a' : '#ff3b3b';
        }
        
        const x = col * sz;
        const y = row * sz;
        ctx.fillRect(x, y, sz, sz);
        
        ctx.strokeStyle = '#1a1a1a';
        ctx.lineWidth = 1;
        ctx.strokeRect(x, y, sz, sz);
      }
    }

    // Zone boundary lines (every 2 columns)
    ctx.strokeStyle = '#b0b0b0';
    ctx.lineWidth = 2;
    for(let x = 2; x < 8; x += 2){
      const px = x * sz;
      ctx.beginPath();
      ctx.moveTo(px, 0);
      ctx.lineTo(px, 128);
      ctx.stroke();
    }
  }

  function renderMatrixPanels(){
    const raw = chkRaw.checked;
    drawMatrix(canvasMatL, ctxMatL, matrixL, statusL, chkL.map(c => c.checked), maskRowsL, raw);
    drawMatrix(canvasMatM, ctxMatM, matrixM, statusM, chkM.map(c => c.checked), maskRowsM, raw);
    drawMatrix(canvasMatR, ctxMatR, matrixR, statusR, chkR.map(c => c.checked), maskRowsR, raw);
  }

  function attachMatrixHover(canvas, matrix, statuses, infoEl, name, maskRows){
    canvas.addEventListener('mousemove', (ev) => {
      const rect = canvas.getBoundingClientRect();
      const x = (ev.clientX - rect.left) * (canvas.width / rect.width);
      const y = (ev.clientY - rect.top) * (canvas.height / rect.height);
      const col = Math.max(0, Math.min(7, Math.floor(x / 16)));
      const row = Math.max(0, Math.min(7, Math.floor(y / 16)));
      const zone = Math.floor(col / 2);
      const idx = row * 8 + col;
      const dist = matrix[idx];
      const status = statuses[idx];
      const masked = (row < maskRows) && !chkRaw.checked;
      if (masked){
        infoEl.textContent = `${name} r${row} c${col} z${zone}: masked (floor) d=${dist} s=${status}`;
      } else if (isValidCell(dist, status)){
        infoEl.textContent = `${name} r${row} c${col} z${zone}: ${dist} mm (status ${status})`;
      } else {
        infoEl.textContent = `${name} r${row} c${col} z${zone}: invalid (d=${dist}, s=${status})`;
      }
    });

    canvas.addEventListener('mouseleave', () => {
      infoEl.textContent = `${name}: hover a cell`;
    });
  }

  function fmtLine(f, t0ms){
    const dt = (f.ms - t0ms) / 1000.0;
    const r = f.robot || {};
    const b2 = (f.b2 !== undefined) ? ` b2=${f.b2}` : '';
    const hasB2Dbg = !!f.b2dbg;
    const b2dbg = hasB2Dbg ? ` b2dbg=${f.b2dbg}` : '';
    const dL = Array.isArray(f.dL) ? `[${f.dL.join(',')}]` : f.dL;
    const dM = Array.isArray(f.dM) ? `[${f.dM.join(',')}]` : f.dM;
    const dR = Array.isArray(f.dR) ? `[${f.dR.join(',')}]` : f.dR;
    const tail = hasB2Dbg
      ? `servo=${f.servo ?? '—'} mA=${f.mA} mB=${f.mB}`
      : `servo=${f.servo ?? '—'} mA=${f.mA} mB=${f.mB} dL=${dL} dM=${dM} dR=${dR}`;
    return `[+${dt.toFixed(3)}s] reason=${f.reason || '—'} started=${f.started?1:0}${b2}${b2dbg} ` +
      `rx=${(r.x??NaN).toFixed(3)} ry=${(r.y??NaN).toFixed(3)} yaw=${(r.yaw??NaN).toFixed(3)} ` +
      tail;
  }

  function showDump(lines){
    dumpEl.style.display = 'block';
    dumpEl.textContent = lines.join('\\n');
  }

  function dumpLastSeconds(sec){
    const now = Date.now();
    const t0 = now - sec*1000;
    const slice = tele.filter(f => f.ms >= t0);
    if (slice.length === 0){
      showDump([`(no frames in last ${sec}s)`]);
      return;
    }
    const base = slice[0].ms;
    showDump(slice.map(f => fmtLine(f, base)));
  }

  function dumpAroundHit(){
    if (lastHitIdx < 0 || lastHitIdx >= tele.length){
      showDump(['(no HIT detected yet)']);
      return;
    }
    const before = 120;
    const after  = 160;
    const a = Math.max(0, lastHitIdx - before);
    const b = Math.min(tele.length, lastHitIdx + after);
    const slice = tele.slice(a, b);
    const base = slice[0].ms;
    showDump(slice.map(f => fmtLine(f, base)));
  }

  function clearMarks(){
    robotPath = [];
    obstaclePoints = [];
    // Redraw with empty history (no page refresh needed)
    drawPose(lastFrame || {});
    drawTrail(lastFrame || {});
  }

  function resizeCanvases(){
    const rect1 = canvasPose.getBoundingClientRect();
    const rect2 = canvasTrail.getBoundingClientRect();
    canvasPose.width  = rect1.width;
    canvasPose.height = rect1.height;
    canvasTrail.width  = rect2.width;
    canvasTrail.height = rect2.height;
  }

  window.addEventListener('resize', resizeCanvases);
  resizeCanvases();

  const board = { xmin:-1.6, xmax:4.0, ymin:-1.6, ymax:1.8 };

  function worldToCanvas(canvas, x, y){
    const w = canvas.width;
    const h = canvas.height;
    const sx = (x - board.xmin) / (board.xmax - board.xmin);
    const sy = (y - board.ymin) / (board.ymax - board.ymin);
    return { x: sx * w, y: sy * h };
  }

  function canvasToWorld(canvas, px, py){
    const sx = px / canvas.width;
    const sy = py / canvas.height;
    const x = board.xmin + sx * (board.xmax - board.xmin);
    const y = board.ymin + sy * (board.ymax - board.ymin);
    return {x, y};
  }

  function drawGrid(ctx, canvas){
    const w = canvas.width, h = canvas.height;
    ctx.fillStyle = '#000';
    ctx.fillRect(0,0,w,h);

    ctx.strokeStyle = '#222';
    ctx.lineWidth = 1;
    ctx.beginPath();
    for(let gx=Math.ceil(board.xmin); gx<=board.xmax; gx+=1){
      const p1 = worldToCanvas(canvas, gx, board.ymin);
      const p2 = worldToCanvas(canvas, gx, board.ymax);
      ctx.moveTo(p1.x, p1.y);
      ctx.lineTo(p2.x, p2.y);
    }
    for(let gy=Math.ceil(board.ymin); gy<=board.ymax; gy+=1){
      const p1 = worldToCanvas(canvas, board.xmin, gy);
      const p2 = worldToCanvas(canvas, board.xmax, gy);
      ctx.moveTo(p1.x, p1.y);
      ctx.lineTo(p2.x, p2.y);
    }
    ctx.stroke();
  }

  // Axis marks on left (Y) and bottom (X)
  function drawAxisLabels(ctx, canvas){
    ctx.save();
    ctx.fillStyle = '#aaa';
    ctx.font = '12px ui-monospace, monospace';
    ctx.textBaseline = 'middle';

    // Y labels at integer meters
    for(let gy=Math.ceil(board.ymin); gy<=board.ymax; gy+=1){
      const p = worldToCanvas(canvas, board.xmin, gy);
      ctx.fillText(String(gy), 6, p.y);
    }

    // X labels at integer meters
    ctx.textBaseline = 'alphabetic';
    for(let gx=Math.ceil(board.xmin); gx<=board.xmax; gx+=1){
      const p = worldToCanvas(canvas, gx, board.ymin);
      ctx.fillText(String(gx), p.x + 2, canvas.height - 6);
    }

    ctx.restore();
  }

  // Robot dimensions (meters)
  const ROBOT_L = 0.32;  // 32 cm length (with upper board)
  const ROBOT_W = 0.20;  // 20 cm width  (with upper board)

  function drawRobotRect(ctx, canvas, rx, ry, yaw){
    const cosY = Math.cos(yaw), sinY = Math.sin(yaw);
    const hL = ROBOT_L / 2, hW = ROBOT_W / 2;

    // 4 corners of rectangle centered on (rx, ry), rotated by yaw
    // Robot frame: +x = forward (heading), +y = left
    const corners = [
      { x: rx + hL*cosY - hW*sinY, y: ry + hL*sinY + hW*cosY },  // front-left
      { x: rx + hL*cosY + hW*sinY, y: ry + hL*sinY - hW*cosY },  // front-right
      { x: rx - hL*cosY + hW*sinY, y: ry - hL*sinY - hW*cosY },  // rear-right
      { x: rx - hL*cosY - hW*sinY, y: ry - hL*sinY + hW*cosY },  // rear-left
    ];
    const cc = corners.map(c => worldToCanvas(canvas, c.x, c.y));

    // Draw rectangle outline
    ctx.strokeStyle = 'rgba(77,208,225,0.5)';
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.moveTo(cc[0].x, cc[0].y);
    for (let i = 1; i < 4; i++) ctx.lineTo(cc[i].x, cc[i].y);
    ctx.closePath();
    ctx.stroke();

    // Heading arrow inside rectangle: tip at front edge center, base set back
    const arrowLen = 0.12;   // arrow length (m)
    const arrowHW  = 0.05;   // arrow half-width at base (m)
    const pFront = worldToCanvas(canvas, rx + hL*cosY, ry + hL*sinY);
    const bx = rx + (hL - arrowLen)*cosY, by = ry + (hL - arrowLen)*sinY;
    const pAL = worldToCanvas(canvas, bx - arrowHW*sinY, by + arrowHW*cosY);
    const pAR = worldToCanvas(canvas, bx + arrowHW*sinY, by - arrowHW*cosY);

    ctx.fillStyle = '#4dd0e1';
    ctx.beginPath();
    ctx.moveTo(pFront.x, pFront.y);
    ctx.lineTo(pAL.x, pAL.y);
    ctx.lineTo(pAR.x, pAR.y);
    ctx.closePath();
    ctx.fill();
  }

  function drawCalibrationRays(ctx, canvas, rx, ry, yaw){
    if (!chkCalRays || !chkCalRays.checked) return;
    const sensors = [
      { offX:0.13 - 0.035355, offY:+0.035355, baseDeg:+70, fovDeg:45, color:'rgba(0,210,106,0.45)' }, // LEFT
      { offX:0.13,            offY:0.0,       baseDeg:0,   fovDeg:60, color:'rgba(255,204,0,0.45)' }, // MID
      { offX:0.13 - 0.035355, offY:-0.035355, baseDeg:-70, fovDeg:45, color:'rgba(255,82,82,0.45)' }  // RIGHT
    ];
    const rayLen = 1.0;
    const degToRad = Math.PI / 180.0;

    for (const s of sensors){
      const sx = rx + (s.offX * Math.cos(yaw) - s.offY * Math.sin(yaw));
      const sy = ry + (s.offX * Math.sin(yaw) + s.offY * Math.cos(yaw));
      const p0 = worldToCanvas(canvas, sx, sy);

      // Draw FoV boundaries.
      const leftEdgeDeg = s.baseDeg - (s.fovDeg / 2);
      const rightEdgeDeg = s.baseDeg + (s.fovDeg / 2);
      for (const d of [leftEdgeDeg, rightEdgeDeg]){
        // Match the same sign convention used by obstacle projection (angCorr = -angleOffsetRad).
        const a = yaw - d * degToRad;
        const ex = sx + rayLen * Math.cos(a);
        const ey = sy + rayLen * Math.sin(a);
        const p1 = worldToCanvas(canvas, ex, ey);

        ctx.strokeStyle = s.color;
        ctx.lineWidth = 1.0;
        ctx.setLineDash([4, 4]);
        ctx.beginPath();
        ctx.moveTo(p0.x, p0.y);
        ctx.lineTo(p1.x, p1.y);
        ctx.stroke();
      }

      // Draw 4 zone-center rays (same partitioning as addObstacleFromZones()).
      const stepDeg = s.fovDeg / 4.0;
      const startDeg = s.baseDeg - (s.fovDeg / 2.0) + (stepDeg / 2.0);
      ctx.setLineDash([]);
      for (let i = 0; i < 4; i++){
        const d = startDeg + i * stepDeg;
        const a = yaw - d * degToRad;
        const ex = sx + rayLen * Math.cos(a);
        const ey = sy + rayLen * Math.sin(a);
        const p1 = worldToCanvas(canvas, ex, ey);

        ctx.strokeStyle = s.color;
        ctx.lineWidth = 1.4;
        ctx.beginPath();
        ctx.moveTo(p0.x, p0.y);
        ctx.lineTo(p1.x, p1.y);
        ctx.stroke();
      }
    }
  }

  function drawPose(frame){
    drawGrid(ctxPose, canvasPose);
    drawAxisLabels(ctxPose, canvasPose);
    if (!frame) return;
    const r = frame.robot || {};
    const t = frame.target || {};

    if (t.x !== undefined && t.y !== undefined){
      const tp = worldToCanvas(canvasPose, t.x, t.y);
      ctxPose.fillStyle = '#ffcc00';
      ctxPose.beginPath();
      ctxPose.arc(tp.x, tp.y, 6, 0, Math.PI*2);
      ctxPose.fill();
    }

    if (r.x !== undefined && r.y !== undefined){
      const yaw = r.yaw || 0;
      drawCalibrationRays(ctxPose, canvasPose, r.x, r.y, yaw);
      drawRobotRect(ctxPose, canvasPose, r.x, r.y, yaw);
    }
  }

  function drawTrail(frame){
    drawGrid(ctxTrail, canvasTrail);
    drawAxisLabels(ctxTrail, canvasTrail);

    if (robotPath.length > 1){
      ctxTrail.strokeStyle = '#4dd0e1';
      ctxTrail.lineWidth = 1.5;
      ctxTrail.beginPath();
      const p0 = worldToCanvas(canvasTrail, robotPath[0].x, robotPath[0].y);
      ctxTrail.moveTo(p0.x, p0.y);
      for (let i = 1; i < robotPath.length; i++){
        const p = worldToCanvas(canvasTrail, robotPath[i].x, robotPath[i].y);
        ctxTrail.lineTo(p.x, p.y);
      }
      ctxTrail.stroke();
    }

    if (obstaclePoints.length > 0){
      ctxTrail.fillStyle = '#ff5252';
      const size = 4;
      for (const o of obstaclePoints){
        const p = worldToCanvas(canvasTrail, o.x, o.y);
        ctxTrail.fillRect(p.x - size/2, p.y - size/2, size, size);
      }
    }

    if (!frame) return;

    const r = frame.robot || {};
    const t = frame.target || {};

    if (t.x !== undefined && t.y !== undefined){
      const tp = worldToCanvas(canvasTrail, t.x, t.y);
      ctxTrail.fillStyle = '#ffcc00';
      ctxTrail.beginPath();
      ctxTrail.arc(tp.x, tp.y, 5, 0, Math.PI*2);
      ctxTrail.fill();
    }

    if (r.x !== undefined && r.y !== undefined){
      const yaw = r.yaw || 0;
      drawCalibrationRays(ctxTrail, canvasTrail, r.x, r.y, yaw);
      drawRobotRect(ctxTrail, canvasTrail, r.x, r.y, yaw);
    }
  }

  let lastFrame = null;
  
  // Fetch matrix data from /matrix endpoint
  async function fetchMatrixData(){
    try {
      const resp = await fetch('/matrix');
      if(!resp.ok) return;
      const j = await resp.json();
      if(j.dL_matrix) decodeHexMatrix(j.dL_matrix, matrixL);
      if(j.dM_matrix) decodeHexMatrix(j.dM_matrix, matrixM);
      if(j.dR_matrix) decodeHexMatrix(j.dR_matrix, matrixR);
      if(j.sL_matrix) decodeHexStatus(j.sL_matrix, statusL);
      if(j.sM_matrix) decodeHexStatus(j.sM_matrix, statusM);
      if(j.sR_matrix) decodeHexStatus(j.sR_matrix, statusR);
      renderMatrixPanels();
    } catch(e){}
  }
  
  // Poll matrix data periodically (every 200ms)
  attachMatrixHover(canvasMatL, matrixL, statusL, matrixLInfo, 'LEFT', maskRowsL);
  attachMatrixHover(canvasMatM, matrixM, statusM, matrixMInfo, 'MID', maskRowsM);
  attachMatrixHover(canvasMatR, matrixR, statusR, matrixRInfo, 'RIGHT', maskRowsR);
  renderMatrixPanels();
  setInterval(fetchMatrixData, 200);
  
  function updateTelemetry(j){
    const r = j.robot || {};
    const t = j.target || {};

    el('rx').textContent   = (r.x  !== undefined) ? r.x.toFixed(3) : '—';
    el('ry').textContent   = (r.y  !== undefined) ? r.y.toFixed(3) : '—';
    el('ryaw').textContent = (r.yaw!== undefined) ? r.yaw.toFixed(3) : '—';

    el('tx').textContent   = (t.x  !== undefined) ? t.x.toFixed(3) : '—';
    el('ty').textContent   = (t.y  !== undefined) ? t.y.toFixed(3) : '—';
    el('tyaw').textContent = (t.yaw!== undefined) ? t.yaw.toFixed(3) : '—';

    el('servo').textContent   = j.servo ?? '—';
    el('started').textContent = j.started ? 'true' : 'false';

    const mA = (j.dirA || '?') + String(j.pwmA ?? 0).padStart(3,'0');
    const mB = (j.dirB || '?') + String(j.pwmB ?? 0).padStart(3,'0');
    el('mA').textContent = mA;
    el('mB').textContent = mB;

    function fmtZones(v){
      if (!Array.isArray(v)) return (v !== undefined) ? String(v) : '—';
      return '[' + v.map(x => (x ?? '—')).join(',') + ']';
    }
    el('dL').textContent = fmtZones(j.dL);
    el('dM').textContent = fmtZones(j.dM);
    el('dR').textContent = fmtZones(j.dR);

    stateBadge.textContent = j.started ? 'running' : 'stopped';
    stateBadge.style.background = j.started ? '#1e8e3e' : '#555';
    if (j.reason) reasonBadge.textContent = j.reason;

    // Keep last ~70s of telemetry frames for debugging/paste
    const f = {
      ms: Date.now(),
      started: !!j.started,
      reason: j.reason || '',
      servo: j.servo,
      dL: j.dL, dM: j.dM, dR: j.dR,
      mA, mB,
      robot: { x:r.x, y:r.y, yaw:r.yaw },
      target: { x:t.x, y:t.y, yaw:t.yaw }
    };
    pushTele(f);

    // HIT detection = first time entering wall-follow reason (best-effort)
    const prevReason = lastFrame ? (lastFrame.reason || '') : '';
    const nowReason  = j.reason || '';
    if (nowReason.startsWith('bug2_wf_') && !prevReason.startsWith('bug2_wf_')){
      lastHitIdx = tele.length - 1;
      const stamp = new Date().toLocaleTimeString();
      hitBadge.textContent = `hit: ${stamp} (idx=${lastHitIdx})`;
      hitBadge.style.background = '#7b61ff';
    }

    if (r.x !== undefined && r.y !== undefined){
      const last = robotPath[robotPath.length - 1];
      const dx = !last ? Infinity : (r.x - last.x);
      const dy = !last ? Infinity : (r.y - last.y);
      if (!last || Math.hypot(dx, dy) > 0.01){
        robotPath.push({x: r.x, y: r.y});
        if (robotPath.length > 2000) robotPath.shift();
      }
    }

    function addObstacleFromSensor(dist_mm, angleOffsetRad, offX, offY){
      if (r.x === undefined || r.y === undefined || r.yaw === undefined) return;
      if (dist_mm === undefined || dist_mm <= 0 || dist_mm >= 8000) return;
      const dist_m = dist_mm / 1000.0;
      // Projection frame correction: UI map was mirrored laterally vs physical setup.
      // Keep firmware/data unchanged and only correct UI projection signs here.
      const offYcorr = -offY;
      const angCorr = -angleOffsetRad;
      // Match stored-map geometry: robot frame +x forward, +y left
      const baseX = r.x + (offX * Math.cos(r.yaw) - offYcorr * Math.sin(r.yaw));
      const baseY = r.y + (offX * Math.sin(r.yaw) + offYcorr * Math.cos(r.yaw));
      const ox = baseX + dist_m * Math.cos(r.yaw + angCorr);
      const oy = baseY + dist_m * Math.sin(r.yaw + angCorr);

      const last = obstaclePoints[obstaclePoints.length - 1];
      const dx = !last ? Infinity : (ox - last.x);
      const dy = !last ? Infinity : (oy - last.y);
      if (!last || Math.hypot(dx, dy) > 0.02){
        obstaclePoints.push({x: ox, y: oy});
      }
    }

    // MID Z0/Z3 calibration constants (must match obstacle_map.cpp)
    const MID_Z0_DIST_SCALE = 1.06;
    const MID_Z3_DIST_SCALE = 1.06;
    const MID_Z0_ANGLE_BIAS = -2.0 * Math.PI / 180.0; // less right
    const MID_Z3_ANGLE_BIAS = +2.0 * Math.PI / 180.0; // less left

    function addObstacleFromZones(zoneArr, baseAngleRad, offX, offY, fovDeg){
      if (!Array.isArray(zoneArr)) {
        addObstacleFromSensor(zoneArr, baseAngleRad, offX, offY);
        return;
      }
      const fovRad = (fovDeg * Math.PI) / 180.0;
      const step = fovRad / 4.0;
      const start = baseAngleRad - (fovRad / 2.0) + (step / 2.0);
      const isMid = (Math.abs(baseAngleRad) < 0.01);
      for (let i = 0; i < 4; i++) {
        let d = zoneArr[i];
        let a = start + i * step;
        if (isMid) {
          if (i === 0) { d *= MID_Z0_DIST_SCALE; a += MID_Z0_ANGLE_BIAS; }
          if (i === 3) { d *= MID_Z3_DIST_SCALE; a += MID_Z3_ANGLE_BIAS; }
        }
        addObstacleFromSensor(d, a, offX, offY);
      }
    }

    addObstacleFromZones(j.dL,  +70.0 * Math.PI/180.0, 0.13 - 0.035355, +0.035355, 45.0);
    addObstacleFromZones(j.dM,  0,                    0.13,            0.00,      60.0);
    addObstacleFromZones(j.dR, -70.0 * Math.PI/180.0, 0.13 - 0.035355, -0.035355, 45.0);

    const stamp = new Date().toLocaleTimeString();
    if (!lastFrame || lastFrame.reason !== j.reason || lastFrame.b2dbg !== j.b2dbg){
      const div = document.createElement('div');
      const dbgInfo = j.b2dbg ? ` ${j.b2dbg}` : '';
      const tailInfo = j.b2dbg ? `(servo=${j.servo}, mA=${mA}, mB=${mB})` : `(servo=${j.servo}, mA=${mA}, mB=${mB}, dL=${fmtZones(j.dL)}, dM=${fmtZones(j.dM)}, dR=${fmtZones(j.dR)})`;
      div.textContent = `[${stamp}] ${j.reason || '—'} ${tailInfo}${dbgInfo}`;
      logEl.appendChild(div);
      logEl.scrollTop = logEl.scrollHeight;
    }

    lastFrame = j;
    drawPose(j);
    drawTrail(j);
  }

  // Dump buttons
  el('btnDumpHit').onclick = dumpAroundHit;
  el('btnDump30').onclick  = () => dumpLastSeconds(30);
  el('btnDumpHide').onclick = () => { dumpEl.style.display = 'none'; };
  el('btnClearMarks').onclick = clearMarks;

  // --- Obstacle map badge update ---
  const obsBadge = el('obsBadge');

  el('btnObsClear').onclick = async () => {
    await fetch('/obsmap_clear');
    if (obsBadge) obsBadge.textContent = 'obs: cleared';
  };

  
  // Matrix zone toggle listeners
  for(let i = 0; i < 4; i++){
    chkL[i].onchange = () => {
      renderMatrixPanels();
    };
    chkM[i].onchange = () => {
      renderMatrixPanels();
    };
    chkR[i].onchange = () => {
      renderMatrixPanels();
    };
  }
  chkRaw.onchange = () => { renderMatrixPanels(); };
  if (chkCalRays){
    chkCalRays.onchange = () => {
      drawPose(lastFrame || {});
      drawTrail(lastFrame || {});
    };
  }

  let ws;
  function connectWS(){
    const proto = (location.protocol === 'https:') ? 'wss' : 'ws';
    ws = new WebSocket(proto + '://' + location.hostname + ':81/');
    stateBadge.textContent = 'connecting…';
    stateBadge.style.background = '#555';

    ws.onopen = () => {
      stateBadge.textContent = 'connected (stopped?)';
      stateBadge.style.background = '#555';
    };
    ws.onclose = () => {
      stateBadge.textContent = 'disconnected';
      stateBadge.style.background = '#555';
      setTimeout(connectWS, 1500);
    };
    ws.onmessage = ev => {
      try{ updateTelemetry(JSON.parse(ev.data)); }catch(e){}
    };
  }
  connectWS();
  wifiSsid.addEventListener('input', () => { wifiSsidDirty = true; });
  wifiSsid.addEventListener('focus', () => { wifiSsidDirty = true; });

  async function refreshWifiInfo(){
    try{
      const resp = await fetch('/wifi_info');
      if (!resp.ok) return;
      const j = await resp.json();
      const editing = (document.activeElement === wifiSsid);
      if (wifiSsid && typeof j.ssid === 'string' && !editing && !wifiSsidDirty){
        wifiSsid.value = j.ssid;
      }
      if (wifiBadge){
        const mode = j.ap ? 'AP' : (j.connected ? 'STA' : 'DOWN');
        wifiBadge.textContent = `wifi: ${mode} ${j.ssid || ''} ${j.ip || ''}`.trim();
        wifiBadge.style.background = j.connected ? '#1e8e3e' : (j.ap ? '#e67e22' : '#555');
      }
    }catch(e){}
  }

  async function refreshWifiProfiles(){
    try{
      const resp = await fetch('/wifi_profiles');
      if (!resp.ok) return;
      const j = await resp.json();
      const items = Array.isArray(j.items) ? j.items : [];
      if (!wifiSaved) return;
      if (!items.length){
        wifiSaved.innerHTML = '<div style="opacity:.7;">(no saved networks)</div>';
        return;
      }
      const rows = items.map(it => {
        const ssid = String(it.ssid || '');
        const pref = !!it.preferred;
        const safe = ssid.replace(/"/g, '&quot;');
        return `<div style="display:flex;gap:8px;align-items:center;margin-bottom:4px;">` +
               `<span style="min-width:180px;">${ssid}${pref?' *':''}</span>` +
               `<button class="btn ghost" data-wifi-use="${safe}" style="padding:0.25rem 0.6rem;">Use</button>` +
               `<button class="btn stop" data-wifi-forget="${safe}" style="padding:0.25rem 0.6rem;">Forget</button>` +
               `</div>`;
      });
      wifiSaved.innerHTML = rows.join('');
    }catch(e){}
  }

  async function refreshWifiScan(){
    try{
      if (wifiBadge){
        wifiBadge.textContent = 'wifi: scanning...';
        wifiBadge.style.background = '#555';
      }
      const resp = await fetch('/wifi_scan');
      if (!resp.ok) return;
      const j = await resp.json();
      const items = Array.isArray(j.items) ? j.items : [];
      if (wifiScanList){
        wifiScanList.innerHTML = '';
        if (!items.length){
          const o = document.createElement('option');
          o.value = '';
          o.textContent = '(none found)';
          wifiScanList.appendChild(o);
        } else {
          items.sort((a,b)=> (b.rssi||-200) - (a.rssi||-200));
          for (const it of items){
            const ssid = String(it.ssid || '');
            const rssi = Number(it.rssi || 0);
            const o = document.createElement('option');
            o.value = ssid;
            o.textContent = `${ssid} (${rssi} dBm)`;
            wifiScanList.appendChild(o);
          }
        }
      }
      await refreshWifiInfo();
    }catch(e){}
  }

  async function wifiUseSaved(ssid){
    if (!ssid) return;
    if (wifiBadge){
      wifiBadge.textContent = `wifi: switching to ${ssid}...`;
      wifiBadge.style.background = '#e67e22';
    }
    try{
      const body = new URLSearchParams({ ssid });
      await fetch('/wifi_use', {
        method: 'POST',
        headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
        body
      });
      setTimeout(async ()=>{ await refreshWifiInfo(); await refreshWifiProfiles(); }, 3500);
    }catch(e){}
  }

  async function wifiForgetSaved(ssid){
    if (!ssid) return;
    try{
      const body = new URLSearchParams({ ssid });
      await fetch('/wifi_forget', {
        method: 'POST',
        headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
        body
      });
      await refreshWifiProfiles();
      await refreshWifiInfo();
    }catch(e){}
  }

  refreshWifiInfo();
  refreshWifiProfiles();
  setInterval(refreshWifiInfo, 8000);
  setInterval(refreshWifiProfiles, 12000);

  async function refreshWfTune(){
    try{
      const resp = await fetch('/wf_tune');
      if (!resp.ok) return;
      const j = await resp.json();
      if (wfTarget) wfTarget.value = Number(j.target ?? 0).toFixed(1);
      if (wfDistK) wfDistK.value = Number(j.distk ?? 0).toFixed(2);
      if (wfYawK) wfYawK.value = Number(j.yawk ?? 0).toFixed(2);
      if (wfBearK) wfBearK.value = Number(j.beark ?? 0).toFixed(3);
      if (wfOppK) wfOppK.value = Number(j.oppk ?? 0).toFixed(2);
      if (wfFrontK) wfFrontK.value = Number(j.frontk ?? 0).toFixed(2);
      if (wfZEmerg) wfZEmerg.value = Number(j.zemerg ?? 0);
      if (wfZClose) wfZClose.value = Number(j.zclose ?? 0);
      if (wfZMid) wfZMid.value = Number(j.zmid ?? 0);
      if (wfDrev) wfDrev.value = Number(j.drev ?? 0);
      if (wfDturn) wfDturn.value = Number(j.dturn ?? 0);
      if (wfDwell) wfDwell.value = Number(j.dwell ?? 0);
      if (wfFblkEc) wfFblkEc.value = Number(j.fblk_ec ?? 0).toFixed(2);
      if (wfFblkXc) wfFblkXc.value = Number(j.fblk_xc ?? 0).toFixed(2);
      if (wfFblkEmm) wfFblkEmm.value = Number(j.fblk_emm ?? 0);
      if (wfFblkXmm) wfFblkXmm.value = Number(j.fblk_xmm ?? 0);
      if (wfDendEfc) wfDendEfc.value = Number(j.dend_efc ?? 0).toFixed(2);
      if (wfDendEoc) wfDendEoc.value = Number(j.dend_eoc ?? 0).toFixed(2);
      if (wfDendEfol) wfDendEfol.value = Number(j.dend_efol ?? 0).toFixed(2);
      if (wfDendXfc) wfDendXfc.value = Number(j.dend_xfc ?? 0).toFixed(2);
      if (wfDendXoc) wfDendXoc.value = Number(j.dend_xoc ?? 0).toFixed(2);
      if (wfTuneBadge){
        wfTuneBadge.textContent = 'wf: loaded';
        wfTuneBadge.style.background = '#1e8e3e';
      }
    }catch(e){
      if (wfTuneBadge){
        wfTuneBadge.textContent = 'wf: load failed';
        wfTuneBadge.style.background = '#b63b3b';
      }
    }
  }

  async function applyWfTune(){
    try{
      const p = new URLSearchParams({
        target: String(wfTarget?.value || ''),
        distk: String(wfDistK?.value || ''),
        yawk: String(wfYawK?.value || ''),
        beark: String(wfBearK?.value || ''),
        oppk: String(wfOppK?.value || ''),
        frontk: String(wfFrontK?.value || ''),
        zemerg: String(wfZEmerg?.value || ''),
        zclose: String(wfZClose?.value || ''),
        zmid: String(wfZMid?.value || ''),
        drev: String(wfDrev?.value || ''),
        dturn: String(wfDturn?.value || ''),
        dwell: String(wfDwell?.value || ''),
        fblk_ec: String(wfFblkEc?.value || ''),
        fblk_xc: String(wfFblkXc?.value || ''),
        fblk_emm: String(wfFblkEmm?.value || ''),
        fblk_xmm: String(wfFblkXmm?.value || ''),
        dend_efc: String(wfDendEfc?.value || ''),
        dend_eoc: String(wfDendEoc?.value || ''),
        dend_efol: String(wfDendEfol?.value || ''),
        dend_xfc: String(wfDendXfc?.value || ''),
        dend_xoc: String(wfDendXoc?.value || '')
      });
      const resp = await fetch('/wf_tune', {
        method: 'POST',
        headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
        body: p
      });
      if (!resp.ok) throw new Error('apply failed');
      await refreshWfTune();
      if (wfTuneBadge){
        wfTuneBadge.textContent = 'wf: applied';
        wfTuneBadge.style.background = '#1e8e3e';
      }
    }catch(e){
      if (wfTuneBadge){
        wfTuneBadge.textContent = 'wf: apply failed';
        wfTuneBadge.style.background = '#b63b3b';
      }
    }
  }

  async function resetWfTune(){
    try{
      const resp = await fetch('/wf_tune?reset=1');
      if (!resp.ok) throw new Error('reset failed');
      await refreshWfTune();
      if (wfTuneBadge){
        wfTuneBadge.textContent = 'wf: reset';
        wfTuneBadge.style.background = '#e67e22';
      }
    }catch(e){
      if (wfTuneBadge){
        wfTuneBadge.textContent = 'wf: reset failed';
        wfTuneBadge.style.background = '#b63b3b';
      }
    }
  }
  refreshWfTune();

  // ---- Control helpers (avoid double-send HTTP+WS) ----
  function wsSendOrHttp(textCmd, httpPath){
    if (ws && ws.readyState === WebSocket.OPEN){
      ws.send(textCmd);
    } else if (httpPath){
      fetch(httpPath).catch(()=>{});
    }
  }

  el('btnStart').onclick = () => wsSendOrHttp('start', '/start');
  el('btnStop').onclick  = () => wsSendOrHttp('stop',  '/stop');
  el('btnWfTuneLoad').onclick = () => { refreshWfTune(); };
  el('btnWfTuneApply').onclick = () => { applyWfTune(); };
  el('btnWfTuneReset').onclick = () => { resetWfTune(); };
  el('btnWifiScan').onclick = () => { refreshWifiScan(); };
  el('btnWifiPick').onclick = () => {
    if (!wifiScanList) return;
    const v = wifiScanList.value || '';
    if (!v) return;
    wifiSsid.value = v;
    wifiSsidDirty = true;
  };

  if (wifiSaved){
    wifiSaved.addEventListener('click', (ev) => {
      const t = ev.target;
      if (!(t instanceof HTMLElement)) return;
      const use = t.getAttribute('data-wifi-use');
      if (use){
        wifiUseSaved(use);
        return;
      }
      const forget = t.getAttribute('data-wifi-forget');
      if (forget){
        wifiForgetSaved(forget);
      }
    });
  }

  el('btnWifiSet').onclick = async () => {
    const ssid = (wifiSsid.value || '').trim();
    const pass = wifiPass.value || '';
    if (!ssid){
      if (wifiBadge){
        wifiBadge.textContent = 'wifi: ssid required';
        wifiBadge.style.background = '#b63b3b';
      }
      return;
    }
    if (wifiBadge){
      wifiBadge.textContent = 'wifi: applying...';
      wifiBadge.style.background = '#555';
    }
    try{
      const body = new URLSearchParams({ ssid, pass });
      const resp = await fetch('/wifi_set', {
        method: 'POST',
        headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
        body
      });
      if (!resp.ok){
        if (wifiBadge){
          wifiBadge.textContent = 'wifi: apply failed';
          wifiBadge.style.background = '#b63b3b';
        }
        return;
      }
      if (wifiBadge){
        wifiBadge.textContent = 'wifi: reconnecting...';
        wifiBadge.style.background = '#e67e22';
      }
      wifiSsidDirty = false;
      setTimeout(async ()=>{
        await refreshWifiInfo();
        await refreshWifiProfiles();
      }, 3500);
    }catch(e){
      if (wifiBadge){
        wifiBadge.textContent = 'wifi: reconnecting...';
        wifiBadge.style.background = '#e67e22';
      }
    }
  };

  el('btnNavGo').onclick = () => {
    const x = Number(navX.value);
    const y = Number(navY.value);
    wsSendOrHttp(`nav ${x} ${y}`, `/nav?x=${encodeURIComponent(x)}&y=${encodeURIComponent(y)}`);
  };
  el('btnNavStop').onclick = () => wsSendOrHttp('navstop', '/navstop');

  el('btnBug2Go').onclick = () => {
    const x = Number(bugX.value);
    const y = Number(bugY.value);
    wsSendOrHttp(`bug2 ${x} ${y}`, `/bug2?x=${encodeURIComponent(x)}&y=${encodeURIComponent(y)}`);
  };
  el('btnBug2Stop').onclick = () => wsSendOrHttp('bug2stop', '/bug2stop');

  el('btnWfGo').onclick = () => {
    const x = Number(wfX.value);
    const y = Number(wfY.value);
    wsSendOrHttp(`bug2wf ${x} ${y}`, `/bug2wf?x=${encodeURIComponent(x)}&y=${encodeURIComponent(y)}`);
  };
  el('btnWfStop').onclick = () => wsSendOrHttp('bug2stop', '/bug2stop');

  el('btnPick').onclick = () => {
    pickMode = !pickMode;
    pickBadge.textContent = 'pick: ' + (pickMode ? 'ON' : 'off');
    pickBadge.style.background = pickMode ? '#1e8e3e' : '#2b2b2b';
  };

  canvasTrail.addEventListener('click', (ev) => {
    if (!pickMode) return;
    const rect = canvasTrail.getBoundingClientRect();
    const px = (ev.clientX - rect.left) * (canvasTrail.width / rect.width);
    const py = (ev.clientY - rect.top)  * (canvasTrail.height / rect.height);
    const w = canvasToWorld(canvasTrail, px, py);

    if (pickTarget.value === 'bug2'){
      bugX.value = w.x.toFixed(2);
      bugY.value = w.y.toFixed(2);
    } else if (pickTarget.value === 'wf'){
      wfX.value = w.x.toFixed(2);
      wfY.value = w.y.toFixed(2);
    } else {
      navX.value = w.x.toFixed(2);
      navY.value = w.y.toFixed(2);
    }
  });

})();
</script>

</body>
</html>
)HTML";
