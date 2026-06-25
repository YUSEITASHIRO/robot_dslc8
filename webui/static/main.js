/**
 * main.js — G1 Web UI クライアント
 * WebSocket でブリッジサーバーと通信し、各パネルをリアルタイム更新する。
 */

const WS_URL = `ws://${location.host}/ws`;
const MAX_LOG_LINES    = 300;
const MAX_TRANSCRIPT   = 100;
const MAX_MOTION_HIST  = 30;

let ws = null;
let reconnectTimer = null;

// ─── パネル要素 ──────────────────────────────────────────────────────────────
const els = {
  statusDot:    document.getElementById("status-dot"),
  statusText:   document.getElementById("status-text"),
  camUserImg:   document.getElementById("cam-user"),
  camRobotImg:  document.getElementById("cam-robot"),
  camUserFps:   document.getElementById("cam-user-fps"),
  camRobotFps:  document.getElementById("cam-robot-fps"),
  camUserVis:   document.getElementById("cam-user-vis"),
  logSim:       document.getElementById("log-sim"),
  logDial:      document.getElementById("log-dial"),
  transcript:   document.getElementById("transcript"),
  motionCur:    document.getElementById("motion-current"),
  motionHist:   document.getElementById("motion-history"),
  phaseBadge:   document.getElementById("phase-badge"),
  keyStatus:    document.getElementById("key-status"),
};

// ─── WebSocket 接続 ──────────────────────────────────────────────────────────
function connect() {
  ws = new WebSocket(WS_URL);

  ws.onopen = () => {
    setStatus(true);
    clearTimeout(reconnectTimer);
  };

  ws.onclose = () => {
    setStatus(false);
    reconnectTimer = setTimeout(connect, 2000);
  };

  ws.onerror = () => ws.close();

  ws.onmessage = (ev) => {
    let msg;
    try { msg = JSON.parse(ev.data); } catch { return; }
    dispatch(msg);
  };
}

function setStatus(online) {
  els.statusDot.className  = online ? "dot online" : "dot offline";
  els.statusText.textContent = online ? "Connected" : "Reconnecting…";
}

// ─── メッセージディスパッチ ───────────────────────────────────────────────────
function dispatch(msg) {
  switch (msg.type) {
    case "camera":     handleCamera(msg);     break;
    case "log":        handleLog(msg);        break;
    case "transcript": handleTranscript(msg); break;
    case "motion":     handleMotion(msg);     break;
    case "phase":      handlePhase(msg);      break;
  }
}

// ─── カメラパネル ─────────────────────────────────────────────────────────────
function handleCamera(msg) {
  const src = `data:image/jpeg;base64,${msg.data}`;
  if (msg.cam === "user_eye") {
    els.camUserImg.src  = src;
    els.camUserFps.textContent = `${msg.fps} fps`;
    if (msg.robot_visible !== undefined) {
      els.camUserVis.textContent = msg.robot_visible ? "🟢 Robot IN VIEW" : "🟠 Robot NOT IN VIEW";
      els.camUserVis.className   = msg.robot_visible ? "vis-badge visible" : "vis-badge hidden";
    }
  } else if (msg.cam === "ego_view") {
    els.camRobotImg.src  = src;
    els.camRobotFps.textContent = `${msg.fps} fps`;
  }
}

// ─── ターミナルログパネル ─────────────────────────────────────────────────────
const _logLines = { sim: [], dialogue: [] };

function handleLog(msg) {
  const key  = msg.source === "sim" ? "sim" : "dialogue";
  const el   = key === "sim" ? els.logSim : els.logDial;
  const arr  = _logLines[key];

  const line = document.createElement("div");
  line.className = "log-line";
  line.textContent = msg.text;
  _colorLogLine(line, msg.text);

  arr.push(line);
  if (arr.length > MAX_LOG_LINES) {
    el.removeChild(arr.shift());
  }
  el.appendChild(line);
  el.scrollTop = el.scrollHeight;
}

function _colorLogLine(el, text) {
  if (text.startsWith("▶") || text.includes("[started")) el.classList.add("log-start");
  else if (text.includes("Error") || text.includes("error")) el.classList.add("log-error");
  else if (text.includes("Warning") || text.includes("warn")) el.classList.add("log-warn");
  else if (text.includes("[Motion]")) el.classList.add("log-motion");
  else if (text.includes("[Phase]"))  el.classList.add("log-phase");
  else if (text.includes("[Vision]") || text.includes("[Camera]")) el.classList.add("log-vision");
}

// ─── 音声対話パネル ──────────────────────────────────────────────────────────
function handleTranscript(msg) {
  const el    = els.transcript;
  const items = el.querySelectorAll(".bubble");

  const bubble = document.createElement("div");
  bubble.className = `bubble ${msg.role}`;

  const label = document.createElement("span");
  label.className = "bubble-label";
  label.textContent = msg.role === "user" ? "部長" : "ロボット";

  const text = document.createElement("span");
  text.className = "bubble-text";
  text.textContent = msg.text;

  const time = document.createElement("span");
  time.className = "bubble-time";
  time.textContent = _timeStr();

  bubble.appendChild(label);
  bubble.appendChild(text);
  bubble.appendChild(time);

  if (items.length >= MAX_TRANSCRIPT) {
    el.removeChild(el.querySelector(".bubble"));
  }
  el.appendChild(bubble);
  el.scrollTop = el.scrollHeight;
}

// ─── 動作パネル ───────────────────────────────────────────────────────────────
const _motionHistory = [];

function handleMotion(msg) {
  _motionHistory.unshift({ name: msg.name, ts: msg.ts });
  if (_motionHistory.length > MAX_MOTION_HIST) _motionHistory.pop();

  els.motionCur.textContent = `▶ ${msg.name}`;
  els.motionCur.classList.add("flash");
  setTimeout(() => els.motionCur.classList.remove("flash"), 800);

  // 履歴リスト再描画
  els.motionHist.innerHTML = "";
  _motionHistory.forEach((m, i) => {
    const row = document.createElement("div");
    row.className = i === 0 ? "motion-row active" : "motion-row";
    row.innerHTML = `<span class="motion-name">${m.name}</span><span class="motion-ts">${_timeStr(m.ts)}</span>`;
    els.motionHist.appendChild(row);
  });
}

// ─── フェーズバッジ ──────────────────────────────────────────────────────────
const _PHASE_COLORS = {
  GREETING:    "#4caf50",
  TOURING:     "#2196f3",
  NEGOTIATING: "#ff9800",
  CLOSING:     "#9c27b0",
};

function handlePhase(msg) {
  const phase = msg.phase.toUpperCase();
  els.phaseBadge.textContent = phase;
  els.phaseBadge.style.background = _PHASE_COLORS[phase] || "#607d8b";
}

// ─── キーボード制御 ──────────────────────────────────────────────────────────
const _KEY_LABELS = {
  ArrowUp:    "↑ 前進",
  ArrowDown:  "↓ 後退",
  ArrowLeft:  "← 左転向",
  ArrowRight: "→ 右転向",
  " ":        "SPACE 180°反転",
};

let _lastKeyTs = 0;

document.addEventListener("keydown", (e) => {
  if (!_KEY_LABELS[e.key]) return;
  e.preventDefault();

  const now = Date.now();
  if (now - _lastKeyTs < 80) return;  // デバウンス 80ms
  _lastKeyTs = now;

  sendKey(e.key);
  els.keyStatus.textContent = _KEY_LABELS[e.key];
  els.keyStatus.classList.add("key-flash");
  setTimeout(() => els.keyStatus.classList.remove("key-flash"), 300);
});

// タッチ/クリックボタン対応
document.querySelectorAll("[data-key]").forEach(btn => {
  btn.addEventListener("click", () => {
    const key = btn.dataset.key;
    sendKey(key);
    els.keyStatus.textContent = _KEY_LABELS[key] || key;
    els.keyStatus.classList.add("key-flash");
    setTimeout(() => els.keyStatus.classList.remove("key-flash"), 300);
  });
});

function sendKey(key) {
  if (ws && ws.readyState === WebSocket.OPEN) {
    ws.send(JSON.stringify({ type: "key", key }));
  }
}

// ─── ユーティリティ ───────────────────────────────────────────────────────────
function _timeStr(ts) {
  const d = ts ? new Date(ts * 1000) : new Date();
  return d.toTimeString().slice(0, 8);
}

// ─── 起動 ────────────────────────────────────────────────────────────────────
connect();
