const WS_URL = `ws://${location.hostname || "localhost"}:8765`;
const TRAIL_LEN = 400;

const state = {
  connected: false,
  target: null,
  missile: null,
  engagement: null,
  aircraft: null,
  targetTrail: [],
  missileTrail: [],
  lastStep: 0,
  interceptLogged: false,
};

const els = {};
const canvas = document.getElementById("tactical-map");
const ctx = canvas.getContext("2d");

function $(id) {
  if (!els[id]) els[id] = document.getElementById(id);
  return els[id];
}

function fmt(v, d = 1) {
  return v == null || Number.isNaN(v) ? "—" : v.toFixed(d);
}

function speed(v) {
  if (!v) return 0;
  return Math.hypot(v[0], v[1], v[2]);
}

function heading(vn, ve) {
  if (vn == null || ve == null) return "—";
  const deg = ((Math.atan2(ve, vn) * 180) / Math.PI + 360) % 360;
  return `${deg.toFixed(0)}°`;
}

function log(msg, level = "") {
  const el = $("event-log");
  const ts = new Date().toISOString().slice(11, 19);
  const row = document.createElement("div");
  row.className = `log-entry ${level}`;
  row.innerHTML = `<span class="ts">${ts}</span>${msg}`;
  el.prepend(row);
  while (el.children.length > 80) el.removeChild(el.lastChild);
}

function resizeCanvas() {
  const rect = canvas.parentElement.getBoundingClientRect();
  canvas.width = rect.width * devicePixelRatio;
  canvas.height = rect.height * devicePixelRatio;
  canvas.style.width = `${rect.width}px`;
  canvas.style.height = `${rect.height}px`;
  ctx.setTransform(devicePixelRatio, 0, 0, devicePixelRatio, 0, 0);
}

function worldToScreen(x, y, bounds, w, h) {
  const pad = 48;
  const spanX = bounds.maxX - bounds.minX || 1;
  const spanY = bounds.maxY - bounds.minY || 1;
  const sx = pad + ((x - bounds.minX) / spanX) * (w - pad * 2);
  const sy = pad + ((y - bounds.minY) / spanY) * (h - pad * 2);
  return { x: sx, y: h - sy };
}

function drawMap() {
  const w = canvas.clientWidth;
  const h = canvas.clientHeight;
  ctx.clearRect(0, 0, w, h);

  const points = [...state.targetTrail, ...state.missileTrail];
  if (state.target?.position) points.push(state.target.position);
  if (state.missile?.position) points.push(state.missile.position);

  if (points.length === 0) {
    ctx.fillStyle = "#4a6178";
    ctx.font = "11px JetBrains Mono";
    ctx.textAlign = "center";
    ctx.fillText("AWAITING TELEMETRY", w / 2, h / 2);
    return;
  }

  const xs = points.map((p) => p[0]);
  const ys = points.map((p) => p[1]);
  const bounds = {
    minX: Math.min(...xs) - 500,
    maxX: Math.max(...xs) + 500,
    minY: Math.min(...ys) - 500,
    maxY: Math.max(...ys) + 500,
  };

  ctx.strokeStyle = "rgba(76, 154, 255, 0.08)";
  ctx.lineWidth = 1;
  for (let i = 0; i < 20; i++) {
    const gx = bounds.minX + (i / 20) * (bounds.maxX - bounds.minX);
    const p1 = worldToScreen(gx, bounds.minY, bounds, w, h);
    const p2 = worldToScreen(gx, bounds.maxY, bounds, w, h);
    ctx.beginPath();
    ctx.moveTo(p1.x, p1.y);
    ctx.lineTo(p2.x, p2.y);
    ctx.stroke();
    const gy = bounds.minY + (i / 20) * (bounds.maxY - bounds.minY);
    const p3 = worldToScreen(bounds.minX, gy, bounds, w, h);
    const p4 = worldToScreen(bounds.maxX, gy, bounds, w, h);
    ctx.beginPath();
    ctx.moveTo(p3.x, p3.y);
    ctx.lineTo(p4.x, p4.y);
    ctx.stroke();
  }

  function drawTrail(trail, color) {
    if (trail.length < 2) return;
    ctx.strokeStyle = color;
    ctx.lineWidth = 1.5;
    ctx.globalAlpha = 0.5;
    ctx.beginPath();
    trail.forEach((p, i) => {
      const s = worldToScreen(p[0], p[1], bounds, w, h);
      if (i === 0) ctx.moveTo(s.x, s.y);
      else ctx.lineTo(s.x, s.y);
    });
    ctx.stroke();
    ctx.globalAlpha = 1;
  }

  drawTrail(state.targetTrail, "rgba(245, 158, 11, 0.6)");
  drawTrail(state.missileTrail, "rgba(239, 68, 68, 0.6)");

  if (state.target?.position && state.missile?.position) {
    const t = worldToScreen(state.target.position[0], state.target.position[1], bounds, w, h);
    const m = worldToScreen(state.missile.position[0], state.missile.position[1], bounds, w, h);
    ctx.strokeStyle = "rgba(34, 211, 238, 0.4)";
    ctx.setLineDash([4, 6]);
    ctx.beginPath();
    ctx.moveTo(m.x, m.y);
    ctx.lineTo(t.x, t.y);
    ctx.stroke();
    ctx.setLineDash([]);
  }

  function drawEntity(pos, color, label) {
    const s = worldToScreen(pos[0], pos[1], bounds, w, h);
    ctx.fillStyle = color;
    ctx.shadowColor = color;
    ctx.shadowBlur = 12;
    ctx.beginPath();
    ctx.arc(s.x, s.y, 6, 0, Math.PI * 2);
    ctx.fill();
    ctx.shadowBlur = 0;
    ctx.fillStyle = "#e8eef4";
    ctx.font = "600 9px JetBrains Mono";
    ctx.fillText(label, s.x + 10, s.y + 3);
  }

  if (state.missile?.position) drawEntity(state.missile.position, "#ef4444", "MSL");
  if (state.target?.position) drawEntity(state.target.position, "#f59e0b", "TGT");
}

function updateUI() {
  const e = state.engagement;
  const t = state.target;
  const m = state.missile;
  const a = state.aircraft;

  $("step-count").textContent = e?.step_count ?? 0;
  $("range-val").textContent = fmt(e?.range_m, 0);
  $("miss-val").textContent = fmt(e?.miss_distance_m, 1);
  $("target-speed").textContent = fmt(t ? speed(t.velocity) : null, 1);
  $("missile-speed").textContent = fmt(m ? speed(m.velocity) : null, 1);

  const tag = $("engagement-tag");
  if (e?.intercept) {
    tag.textContent = "INTERCEPT";
    tag.className = "panel-tag intercept";
  } else if (e?.complete) {
    tag.textContent = "COMPLETE";
    tag.className = "panel-tag complete";
  } else {
    tag.textContent = "ACTIVE";
    tag.className = "panel-tag";
  }

  $("intercept-dot").className = "status-dot" + (e?.intercept ? " hit" : "");
  $("missile-dot").className = "status-dot" + (m?.active ? " active" : m?.hit ? " hit" : "");
  $("target-dot").className = "status-dot active";

  if (t?.position) {
    $("tgt-n").textContent = fmt(t.position[0], 0);
    $("tgt-e").textContent = fmt(t.position[1], 0);
    $("tgt-alt").textContent = fmt(-t.position[2], 0) + " m";
    $("tgt-vn").textContent = fmt(t.velocity?.[0], 1);
    $("tgt-ve").textContent = fmt(t.velocity?.[1], 1);
    $("tgt-hdg").textContent = heading(t.velocity?.[0], t.velocity?.[1]);
  }

  if (m?.position) {
    $("msl-n").textContent = fmt(m.position[0], 0);
    $("msl-e").textContent = fmt(m.position[1], 0);
    $("msl-alt").textContent = fmt(-m.position[2], 0) + " m";
    $("msl-vn").textContent = fmt(m.velocity?.[0], 1);
    $("msl-ve").textContent = fmt(m.velocity?.[1], 1);
    $("msl-phase").textContent = m.hit ? "KILL" : m.active ? "TERMINAL" : "INACTIVE";
  }

  if (a) {
    $("ac-alt").textContent = fmt(a.altitude_m, 0) + " m";
    $("ac-spd").textContent = fmt(a.speed_mps, 1) + " m/s";
    $("ac-thr").textContent = fmt(a.throttle * 100, 0) + "%";
    $("ac-safe").textContent = a.safe_mode === 0 ? "NORMAL" : a.safe_mode === 1 ? "HOLD" : "LATCHED";
  }

  if (e?.intercept && !state.interceptLogged) {
    state.interceptLogged = true;
    log(`INTERCEPT · miss ${fmt(e.miss_distance_m, 2)} m`, "success");
  }

  drawMap();
}

function applyPayload(data) {
  if (data.target) {
    state.target = data.target;
    state.targetTrail.push([...data.target.position]);
    if (state.targetTrail.length > TRAIL_LEN) state.targetTrail.shift();
  }
  if (data.missile) {
    state.missile = data.missile;
    state.missileTrail.push([...data.missile.position]);
    if (state.missileTrail.length > TRAIL_LEN) state.missileTrail.shift();
  }
  if (data.engagement) {
    if (data.engagement.step_count > state.lastStep && state.lastStep === 0) {
      log("Telemetry stream active", "info");
    }
    state.lastStep = data.engagement.step_count;
    state.engagement = data.engagement;
  }
  if (data.aircraft) state.aircraft = data.aircraft;
  updateUI();
}

function setConnected(on) {
  state.connected = on;
  const chip = $("conn-chip");
  chip.textContent = on ? "CONNECTED" : "DISCONNECTED";
  chip.className = "chip" + (on ? " connected" : "");
  $("footer-status").textContent = on ? "Receiving live telemetry" : "Waiting for bridge…";
}

function connect() {
  const ws = new WebSocket(WS_URL);
  ws.onopen = () => {
    setConnected(true);
    log("WebSocket connected to ROS bridge", "info");
  };
  ws.onclose = () => {
    setConnected(false);
    log("Connection lost — retrying in 2s", "warn");
    setTimeout(connect, 2000);
  };
  ws.onerror = () => ws.close();
  ws.onmessage = (ev) => {
    try {
      applyPayload(JSON.parse(ev.data));
    } catch (_) { /* ignore */ }
  };
}

function tickClock() {
  $("clock").textContent = new Date().toISOString().slice(11, 19) + " UTC";
}

window.addEventListener("resize", () => {
  resizeCanvas();
  drawMap();
});

resizeCanvas();
tickClock();
setInterval(tickClock, 1000);
connect();
requestAnimationFrame(function loop() {
  drawMap();
  requestAnimationFrame(loop);
});
