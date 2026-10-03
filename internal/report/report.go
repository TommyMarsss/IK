// Package report renders a solved trajectory as a single self-contained
// static HTML file (data + vanilla JS inlined, no server, no third-party
// assets).
package report

import (
	"encoding/json"
	"html/template"
	"os"
	"strings"
)

// Frame is one time-step of a solved trajectory.
type Frame struct {
	Index  int        `json:"index"`
	Target [2]float64 `json:"target"`
	HasPhi bool       `json:"hasPhi"`
	Phi    float64    `json:"phi"`
	// Joints holds the commanded joint configuration. On failed frames it
	// holds the last successful configuration (state is not advanced).
	Joints []float64 `json:"joints"`
	// Points are the FK joint positions for Joints (base + n end points).
	Points [][2]float64 `json:"points"`
	// ActualEE is the FK end-effector position; on a failed frame it differs
	// from Target (the arm stays put).
	ActualEE [2]float64 `json:"actualEE"`
	// ActualPhi is the achieved end-effector orientation.
	ActualPhi float64 `json:"actualPhi"`
	Status    string  `json:"status"`
	Message   string  `json:"message"`
	Branch    string  `json:"branch"`
	MinSigma  float64 `json:"minSigma"`
	PosError  float64 `json:"posError"`
	OriError  float64 `json:"oriError"`
	// Diagnostic configuration of a limit-violating frame (off-limit), drawn
	// as a ghost, never commanded.
	DiagPoints [][2]float64 `json:"diagPoints,omitempty"`
	Violations []int        `json:"violations,omitempty"`
}

// Data is the complete report payload.
type Data struct {
	Title   string       `json:"title"`
	NLinks  int          `json:"nLinks"`
	Lengths []float64    `json:"lengths"`
	Limits  [][2]float64 `json:"limits"`
	Frames  []Frame      `json:"frames"`
	// ReachInner/ReachOuter bound the position workspace annulus.
	ReachInner float64 `json:"reachInner"`
	ReachOuter float64 `json:"reachOuter"`
}

// htmlTmpl keeps all presentation in one place; the JSON payload is injected
// into a <script> tag and rendered with the Canvas 2D API by vanilla JS.
const htmlTmpl = `<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>{{.Title}}</title>
<style>
  :root {
    --bg: #0f1420; --panel: #181f2e; --ink: #e8edf6; --muted: #93a0b8;
    --ok: #34d399; --singular: #fbbf24; --limit: #f87171; --unreach: #9ca3af;
    --noconv: #c084fc; --accent: #60a5fa; --ghost: #f87171;
  }
  * { box-sizing: border-box; }
  body {
    margin: 0; background: var(--bg); color: var(--ink);
    font: 14px/1.5 -apple-system, BlinkMacSystemFont, "Segoe UI", "PingFang SC",
          "Hiragino Sans GB", "Microsoft YaHei", sans-serif;
  }
  header { padding: 18px 24px 8px; }
  h1 { font-size: 19px; margin: 0 0 4px; }
  .sub { color: var(--muted); font-size: 13px; }
  .layout { display: flex; flex-wrap: wrap; gap: 16px; padding: 16px 24px 24px; }
  .stage { flex: 1 1 620px; min-width: 320px; }
  canvas {
    width: 100%; background: var(--panel); border-radius: 12px;
    border: 1px solid #263045; display: block;
  }
  .side { flex: 0 1 360px; min-width: 280px; display: flex; flex-direction: column; gap: 12px; }
  .card { background: var(--panel); border: 1px solid #263045; border-radius: 12px; padding: 14px 16px; }
  .card h2 { font-size: 13px; text-transform: uppercase; letter-spacing: .08em; color: var(--muted); margin: 0 0 10px; }
  .controls { display: flex; align-items: center; gap: 10px; flex-wrap: wrap; }
  button {
    background: #243049; color: var(--ink); border: 1px solid #33415c;
    border-radius: 8px; padding: 6px 14px; font-size: 13px; cursor: pointer;
  }
  button:hover { background: #2c3a57; }
  input[type=range] { flex: 1; min-width: 140px; accent-color: var(--accent); }
  .badge {
    display: inline-block; padding: 2px 10px; border-radius: 999px;
    font-size: 12px; font-weight: 600;
  }
  .b-ok { background: rgba(52,211,153,.15); color: var(--ok); }
  .b-singular { background: rgba(251,191,36,.15); color: var(--singular); }
  .b-limit_unreachable { background: rgba(248,113,113,.18); color: var(--limit); }
  .b-unreachable { background: rgba(156,163,175,.18); color: var(--unreach); }
  .b-no_convergence { background: rgba(192,132,252,.18); color: var(--noconv); }
  table { width: 100%; border-collapse: collapse; font-variant-numeric: tabular-nums; }
  td { padding: 3px 0; vertical-align: top; }
  td:first-child { color: var(--muted); width: 110px; }
  .msg { font-size: 13px; color: var(--muted); }
  .legend { display: flex; flex-wrap: wrap; gap: 12px; font-size: 12px; color: var(--muted); margin-top: 10px; }
  .dot { display: inline-block; width: 10px; height: 10px; border-radius: 50%; margin-right: 5px; vertical-align: -1px; }
  .framelist { max-height: 220px; overflow-y: auto; display: flex; flex-wrap: wrap; gap: 5px; }
  .chip {
    width: 26px; height: 22px; border-radius: 6px; font-size: 11px;
    display: flex; align-items: center; justify-content: center;
    cursor: pointer; border: 1px solid #33415c; color: var(--muted); background: #1d2638;
  }
  .chip.ok { border-color: rgba(52,211,153,.5); color: var(--ok); }
  .chip.singular { border-color: rgba(251,191,36,.7); background: rgba(251,191,36,.12); color: var(--singular); }
  .chip.limit_unreachable { border-color: rgba(248,113,113,.8); background: rgba(248,113,113,.15); color: var(--limit); }
  .chip.unreachable { border-color: rgba(156,163,175,.6); color: var(--unreach); }
  .chip.no_convergence { border-color: rgba(192,132,252,.7); color: var(--noconv); }
  .chip.active { outline: 2px solid var(--accent); }
  code { background: #0d1320; padding: 1px 5px; border-radius: 4px; font-size: 12px; }
</style>
</head>
<body>
<header>
  <h1 id="title"></h1>
  <div class="sub">平面多连杆机械臂逆运动学报告 · 逐帧回放 · 数据与逻辑全部内嵌，无第三方依赖</div>
</header>
<div class="layout">
  <div class="stage">
    <canvas id="cv" width="900" height="720"></canvas>
    <div class="legend">
      <span><span class="dot" style="background:var(--ok)"></span>正常</span>
      <span><span class="dot" style="background:var(--singular)"></span>奇异位形（DLS 阻尼介入）</span>
      <span><span class="dot" style="background:var(--limit)"></span>工作空间内但违反关节限位</span>
      <span><span class="dot" style="background:var(--unreach)"></span>几何不可达</span>
      <span><span class="dot" style="background:var(--ghost);opacity:.5"></span>违反限位的几何候选（虚影，仅诊断）</span>
    </div>
  </div>
  <div class="side">
    <div class="card">
      <h2>回放控制</h2>
      <div class="controls">
        <button id="play">▶ 播放</button>
        <button id="prev">◀</button>
        <button id="next">▶|</button>
        <label style="color:var(--muted)">速度</label>
        <input id="speed" type="range" min="1" max="30" value="8">
      </div>
      <div class="controls" style="margin-top:10px">
        <input id="scrub" type="range" min="0" value="0" style="width:100%">
      </div>
      <div id="framepos" class="sub" style="margin-top:6px"></div>
    </div>
    <div class="card">
      <h2>当前帧</h2>
      <div style="margin-bottom:8px"><span id="badge" class="badge"></span></div>
      <table>
        <tr><td>目标点</td><td id="target"></td></tr>
        <tr><td>实际末端</td><td id="actual"></td></tr>
        <tr><td>目标/实际朝向</td><td id="phi"></td></tr>
        <tr><td>关节角 (rad)</td><td id="joints"></td></tr>
        <tr><td>构型分支</td><td id="branch"></td></tr>
        <tr><td>σ<sub>min</sub>(J)</td><td id="sigma"></td></tr>
        <tr><td>位置残差</td><td id="perr"></td></tr>
      </table>
      <div class="msg" id="msg" style="margin-top:8px"></div>
    </div>
    <div class="card">
      <h2>全部帧（点击跳转）</h2>
      <div class="framelist" id="chips"></div>
    </div>
  </div>
</div>
<script id="payload" type="application/json">{{.Payload}}</script>
<script>
"use strict";
const data = JSON.parse(document.getElementById("payload").textContent);
document.getElementById("title").textContent = data.title;

const STATUS_LABEL = {
  ok: "正常求解",
  singular: "奇异位形（退化处理）",
  limit_unreachable: "限位不可达",
  unreachable: "几何不可达",
  no_convergence: "未收敛"
};

const cv = document.getElementById("cv");
const ctx = cv.getContext("2d");

// ---- world -> screen mapping (square workspace, keep aspect) ----
function makeView() {
  const R = data.reachOuter;
  const pad = R * 0.25;
  const lo = -R - pad, hi = R + pad;
  const dpr = window.devicePixelRatio || 1;
  const cssW = cv.clientWidth || 900;
  const cssH = Math.round(cssW * 0.8);
  cv.style.height = cssH + "px";
  cv.width = Math.round(cssW * dpr);
  cv.height = Math.round(cssH * dpr);
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  const scale = Math.min(cssW, cssH) / (hi - lo);
  return {
    cssW, cssH, lo, scale,
    x: (wx) => (wx - lo) * scale,
    y: (wy) => cssH - (wy - lo) * scale
  };
}

function disk(vx, vy, r, color) {
  ctx.beginPath();
  ctx.arc(v.x(vx), v.y(vy), r, 0, Math.PI * 2);
  ctx.fillStyle = color;
  ctx.fill();
}

function line(p1, p2, color, width, dash) {
  ctx.save();
  ctx.beginPath();
  ctx.setLineDash(dash || []);
  ctx.moveTo(v.x(p1[0]), v.y(p1[1]));
  ctx.lineTo(v.x(p2[0]), v.y(p2[1]));
  ctx.strokeStyle = color;
  ctx.lineWidth = width || 2;
  ctx.stroke();
  ctx.restore();
}

let v;
function drawGrid() {
  ctx.fillStyle = getCss("--panel");
  ctx.fillRect(0, 0, v.cssW, v.cssH);
  // workspace annulus
  ctx.beginPath();
  ctx.arc(v.x(0), v.y(0), data.reachOuter * v.scale, 0, Math.PI * 2);
  if (data.reachInner > 1e-9) {
    ctx.arc(v.x(0), v.y(0), data.reachInner * v.scale, 0, Math.PI * 2, true);
  }
  ctx.fillStyle = "rgba(96,165,250,0.06)";
  ctx.fill("evenodd");
  ctx.beginPath();
  ctx.arc(v.x(0), v.y(0), data.reachOuter * v.scale, 0, Math.PI * 2);
  ctx.strokeStyle = "rgba(96,165,250,0.35)";
  ctx.setLineDash([5, 5]);
  ctx.lineWidth = 1;
  ctx.stroke();
  ctx.setLineDash([]);
  // axes
  line([v.lo, 0], [-v.lo, 0], "rgba(255,255,255,0.10)", 1);
  line([0, v.lo], [0, -v.lo], "rgba(255,255,255,0.10)", 1);
  // base
  disk(0, 0, 6, "#cbd5e1");
}

function drawArm(points, color, width, jointColor) {
  for (let i = 0; i + 1 < points.length; i++) {
    line(points[i], points[i + 1], color, width);
  }
  points.forEach((p, i) => disk(p[0], p[1], i === points.length - 1 ? 6 : 4, jointColor));
}

function drawOrientationArrow(ee, phi, color) {
  const s = data.reachOuter * 0.12;
  const ex = ee[0] + s * Math.cos(phi), ey = ee[1] + s * Math.sin(phi);
  line(ee, [ex, ey], color, 2);
  // arrow head
  const a = 0.45, b = s * 0.35;
  line([ex, ey], [ex - b * Math.cos(phi - a), ey - b * Math.sin(phi - a)], color, 2);
  line([ex, ey], [ex - b * Math.cos(phi + a), ey - b * Math.sin(phi + a)], color, 2);
}

function getCss(name) {
  return getComputedStyle(document.documentElement).getPropertyValue(name).trim();
}

function statusColor(st) {
  return {
    ok: getCss("--ok"), singular: getCss("--singular"),
    limit_unreachable: getCss("--limit"), unreachable: getCss("--unreach"),
    no_convergence: getCss("--noconv")
  }[st] || "#fff";
}

let cur = 0, playing = false, timer = null;

function render(i) {
  cur = i;
  const f = data.frames[i];
  v = makeView();
  drawGrid();

  // target trajectory ghost (all targets faint)
  data.frames.forEach((g, k) => {
    const col = k < i ? "rgba(96,165,250,0.35)" : "rgba(96,165,250,0.12)";
    disk(g.target[0], g.target[1], 2.2, col);
  });

  // target marker for this frame: red X if geometrically unreachable
  if (f.status === "unreachable") {
    const s = data.reachOuter * 0.03;
    line([f.target[0]-s, f.target[1]-s], [f.target[0]+s, f.target[1]+s], getCss("--limit"), 2);
    line([f.target[0]-s, f.target[1]+s], [f.target[0]+s, f.target[1]-s], getCss("--limit"), 2);
  } else {
    ctx.beginPath();
    ctx.arc(v.x(f.target[0]), v.y(f.target[1]), 7, 0, Math.PI*2);
    ctx.strokeStyle = getCss("--accent");
    ctx.lineWidth = 2;
    ctx.stroke();
    if (f.hasPhi) drawOrientationArrow(f.target, f.phi, getCss("--accent"));
  }

  // ghost: limit-violating geometric candidate
  if (f.diagPoints && f.diagPoints.length) {
    drawArm(f.diagPoints, "rgba(248,113,113,0.45)", 2, "rgba(248,113,113,0.5)");
  }

  // commanded arm
  const col = statusColor(f.status);
  const armCol = f.status === "ok" ? getCss("--ink") : col;
  drawArm(f.points, armCol, 5, col);
  drawOrientationArrow(f.actualEE, f.actualPhi, col);

  // info panel
  const badge = document.getElementById("badge");
  badge.textContent = STATUS_LABEL[f.status] || f.status;
  badge.className = "badge b-" + f.status;
  const fmt = (x) => (x >= 0 ? " " : "") + x.toFixed(4);
  document.getElementById("target").textContent =
    "(" + fmt(f.target[0]) + ", " + fmt(f.target[1]) + ")" +
    (f.hasPhi ? "   φ=" + f.phi.toFixed(4) : "  (仅位置)");
  document.getElementById("actual").textContent =
    "(" + fmt(f.actualEE[0]) + ", " + fmt(f.actualEE[1]) + ")";
  document.getElementById("phi").textContent =
    f.hasPhi ? (f.phi.toFixed(4) + " / " + f.actualPhi.toFixed(4)) : "—";
  document.getElementById("joints").textContent =
    f.joints.map((q) => q.toFixed(4)).join(",  ");
  document.getElementById("branch").textContent = f.branch || "—";
  document.getElementById("sigma").textContent =
    f.minSigma.toExponential(3) +
    (f.status === "singular" ? "   ⚠ 低于奇异阈值" : "");
  document.getElementById("perr").textContent = f.posError.toExponential(3) +
    (f.hasPhi ? "   朝向残差 " + f.oriError.toExponential(3) : "");
  document.getElementById("msg").textContent = f.message || "";
  document.getElementById("framepos").textContent =
    "帧 " + (i + 1) + " / " + data.frames.length;
  document.getElementById("scrub").value = i;
  document.querySelectorAll(".chip").forEach((c, k) =>
    c.classList.toggle("active", k === i));
}

// chips
const chips = document.getElementById("chips");
data.frames.forEach((f, i) => {
  const d = document.createElement("div");
  d.className = "chip " + f.status;
  d.textContent = i + 1;
  d.title = (STATUS_LABEL[f.status] || f.status) + "：" + (f.message || "");
  d.addEventListener("click", () => { stop(); render(i); });
  chips.appendChild(d);
});

// controls
const scrub = document.getElementById("scrub");
scrub.max = data.frames.length - 1;
function stop() {
  playing = false;
  document.getElementById("play").textContent = "▶ 播放";
  if (timer) { clearInterval(timer); timer = null; }
}
function play() {
  playing = true;
  document.getElementById("play").textContent = "⏸ 暂停";
  const period = () => 1050 - document.getElementById("speed").valueAsNumber * 34;
  timer = setInterval(() => {
    let n = cur + 1;
    if (n >= data.frames.length) { stop(); return; }
    render(n);
  }, period());
}
document.getElementById("play").addEventListener("click", () => playing ? stop() : play());
document.getElementById("next").addEventListener("click", () => { stop(); render(Math.min(cur + 1, data.frames.length - 1)); });
document.getElementById("prev").addEventListener("click", () => { stop(); render(Math.max(cur - 1, 0)); });
scrub.addEventListener("input", () => { stop(); render(scrub.valueAsNumber); });
window.addEventListener("resize", () => render(cur));

render(0);
</script>
</body>
</html>`

// Write renders the report to path as one standalone HTML file.
func Write(path string, d *Data) error {
	payload, err := json.Marshal(d)
	if err != nil {
		return err
	}
	// Escape HTML-significant bytes so the JSON can be embedded verbatim in a
	// <script> block (none of our JSON keys contain these characters, so a
	// byte-level replacement is safe).
	payload = []byte(bytesReplacer.Replace(string(payload)))
	tpl, err := template.New("report").Parse(htmlTmpl)
	if err != nil {
		return err
	}
	f, err := os.Create(path)
	if err != nil {
		return err
	}
	defer f.Close()
	return tpl.Execute(f, map[string]any{
		"Title":   d.Title,
		"Payload": template.JS(payload),
	})
}

var bytesReplacer = strings.NewReplacer(
	"<", `<`,
	">", `>`,
	"&", `&`,
	" ", ` `,
	" ", ` `,
)
