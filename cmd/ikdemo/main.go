// ikdemo 生成一段目标轨迹，逐帧求解逆运动学，并把结果连同回放逻辑
// 打包成一个单一的静态 HTML 文件（数据与逻辑全部内嵌，无第三方依赖）。
//
// 用法：
//
//	go run ./cmd/ikdemo [输出文件]   // 默认 ik_replay.html
package main

import (
	"encoding/json"
	"fmt"
	"math"
	"os"
	"strings"

	"github.com/TommyMarsss/IK/ik"
)

// buildTrajectory 构造三段轨迹：
//  1. 右侧圆弧（正常求解区）；
//  2. 圆弧延伸到左侧，超出关节 1 限位（限位不可达区）；
//  3. 沿固定方向径向外推，穿越完全伸直的奇异位形后超出工作空间
//     （接近奇异区 + 几何不可达区）。
func buildTrajectory() []ik.Pose {
	var targets []ik.Pose
	// 段 1+2：半径 1.8 的圆弧，角度 -1.5 → 3.1 rad，末端姿态沿径向朝外。
	const n1 = 180
	for i := 0; i <= n1; i++ {
		ang := -1.5 + (3.1-(-1.5))*float64(i)/n1
		targets = append(targets, ik.Pose{
			X:   1.8 * math.Cos(ang),
			Y:   1.8 * math.Sin(ang),
			Phi: ang,
		})
	}
	// 段 3：沿 0.4 rad 方向从 r=2.6 缓慢推到 r=3.2（最大可达半径为 3.0），
	// 采样加密以便在完全伸直的奇异带内落进多帧。
	const n2 = 120
	for i := 1; i <= n2; i++ {
		r := 2.6 + (3.2-2.6)*float64(i)/n2
		targets = append(targets, ik.Pose{
			X:   r * math.Cos(0.4),
			Y:   r * math.Sin(0.4),
			Phi: 0.4,
		})
	}
	return targets
}

func main() {
	out := "ik_replay.html"
	if len(os.Args) > 1 {
		out = os.Args[1]
	}

	arm := ik.NewArm(
		[]float64{1.2, 1.0, 0.8},
		[]ik.JointLimits{
			{Min: -1.7, Max: 1.7},         // 关节 1 限位：左侧目标将限位不可达
			{Min: -math.Pi, Max: math.Pi}, // 关节 2 不限
			{Min: -math.Pi, Max: math.Pi}, // 关节 3 不限
		},
	)
	// 演示用更宽的奇异带（可操作度 < 0.5，约对应肘关节距完全伸直 25° 以内），
	// 让轨迹在穿越奇异区时能落进若干帧，便于观察高亮效果。
	arm.SingularityThreshold = 0.5

	targets := buildTrajectory()
	frames := arm.SolveTrajectory(targets, nil)

	counts := map[ik.FrameStatus]int{}
	for _, f := range frames {
		counts[f.Status]++
	}
	fmt.Printf("共 %d 帧：正常 %d，接近奇异 %d，限位不可达 %d，几何不可达 %d\n",
		len(frames), counts[ik.StatusOK], counts[ik.StatusNearSingular],
		counts[ik.StatusLimitUnreachable], counts[ik.StatusUnreachable])

	frameJSON, err := json.Marshal(frames)
	if err != nil {
		fmt.Fprintln(os.Stderr, "序列化帧数据失败:", err)
		os.Exit(1)
	}
	linkJSON, _ := json.Marshal(arm.Lengths)

	html := strings.Replace(pageTemplate, "/*__FRAMES__*/null", string(frameJSON), 1)
	html = strings.Replace(html, "/*__LINKS__*/null", string(linkJSON), 1)

	if err := os.WriteFile(out, []byte(html), 0o644); err != nil {
		fmt.Fprintln(os.Stderr, "写入文件失败:", err)
		os.Exit(1)
	}
	fmt.Println("已生成", out, "—— 用浏览器打开即可回放")
}

// pageTemplate 单文件回放页面：原生 HTML/CSS/JavaScript + Canvas 2D，无任何外部依赖。
const pageTemplate = `<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<title>平面三连杆机械臂 IK 轨迹回放</title>
<style>
  body { font-family: -apple-system, "Helvetica Neue", "PingFang SC", Arial, sans-serif;
         margin: 24px; background: #fafafa; color: #222; }
  h1 { font-size: 20px; margin-bottom: 4px; }
  .sub { color: #666; font-size: 13px; margin-bottom: 16px; }
  #wrap { display: flex; gap: 24px; flex-wrap: wrap; align-items: flex-start; }
  #view { background: #fff; border: 1px solid #ccc; border-radius: 6px; display: block; }
  #timeline { display: block; margin-top: 8px; cursor: pointer;
              border: 1px solid #ccc; border-radius: 4px; background: #fff; }
  #panel { width: 320px; }
  #status { padding: 10px 14px; border-radius: 6px; color: #fff;
            font-weight: 600; margin-bottom: 12px; transition: background .1s; }
  #controls { display: flex; align-items: center; gap: 10px; margin-bottom: 12px; }
  button { padding: 6px 16px; font-size: 14px; cursor: pointer; border: 1px solid #999;
           border-radius: 4px; background: #fff; }
  button:hover { background: #eee; }
  input[type=range] { flex: 1; }
  table { border-collapse: collapse; width: 100%; font-size: 13px; margin-bottom: 14px; }
  td, th { padding: 3px 6px; border-bottom: 1px solid #eee; text-align: right; }
  td:first-child, th:first-child { text-align: left; }
  .legend div { margin: 4px 0; font-size: 13px; }
  .legend .sw { display: inline-block; width: 12px; height: 12px; border-radius: 3px;
                margin-right: 6px; vertical-align: -1px; }
  h3 { font-size: 14px; margin: 14px 0 6px; }
</style>
</head>
<body>
<h1>平面三连杆机械臂逆运动学轨迹回放</h1>
<div class="sub">数据与逻辑全部内嵌于本文件 · 高亮显示奇异位形与限位/几何不可达帧</div>
<div id="wrap">
  <div>
    <canvas id="view" width="620" height="620"></canvas>
    <canvas id="timeline" width="620" height="26"></canvas>
  </div>
  <div id="panel">
    <div id="status"></div>
    <div id="controls">
      <button id="play">播放</button>
      <input type="range" id="slider" min="0" value="0">
    </div>
    <table id="info"></table>
    <h3>图例</h3>
    <div class="legend">
      <div><span class="sw" style="background:#2e7d32"></span>正常求解</div>
      <div><span class="sw" style="background:#ef6c00"></span>接近奇异位形（可操作度低于阈值）</div>
      <div><span class="sw" style="background:#c62828"></span>限位不可达（几何可达但超出关节限位）</div>
      <div><span class="sw" style="background:#616161"></span>几何不可达（目标在工作空间外）</div>
    </div>
    <h3>说明</h3>
    <div style="font-size:13px;color:#555;line-height:1.6">
      失败帧中机械臂保持上一帧构型（灰色虚线），目标以红色十字标出。
      下方时间轴逐帧着色，点击可跳转。虚线圆为最大可达半径。
    </div>
  </div>
</div>
<script>
"use strict";
const FRAMES = /*__FRAMES__*/null;
const LINKS = /*__LINKS__*/null;
const STATUS_TEXT = ["正常", "接近奇异位形", "限位不可达", "几何不可达"];
const COLORS = ["#2e7d32", "#ef6c00", "#c62828", "#616161"];

const view = document.getElementById("view");
const ctx = view.getContext("2d");
const maxReach = LINKS.reduce((a, b) => a + b, 0);
const scale = (view.width / 2 - 36) / maxReach;
const cx = view.width / 2, cy = view.height / 2;
const px = x => cx + x * scale;
const py = y => cy - y * scale;

const slider = document.getElementById("slider");
slider.max = FRAMES.length - 1;
const playBtn = document.getElementById("play");
const statusBox = document.getElementById("status");
const infoTable = document.getElementById("info");
const tl = document.getElementById("timeline");
const tlCtx = tl.getContext("2d");

let idx = 0, playing = false, timer = null;

function drawScene() {
  const f = FRAMES[idx];
  ctx.clearRect(0, 0, view.width, view.height);

  // 状态背景高亮
  if (f.status !== 0) {
    ctx.fillStyle = COLORS[f.status] + "18";
    ctx.fillRect(0, 0, view.width, view.height);
  }

  // 网格与坐标轴
  ctx.strokeStyle = "#eee"; ctx.lineWidth = 1;
  for (let g = -3; g <= 3; g++) {
    ctx.beginPath(); ctx.moveTo(px(g), py(-3)); ctx.lineTo(px(g), py(3)); ctx.stroke();
    ctx.beginPath(); ctx.moveTo(px(-3), py(g)); ctx.lineTo(px(3), py(g)); ctx.stroke();
  }
  ctx.strokeStyle = "#ccc";
  ctx.beginPath(); ctx.moveTo(px(-3.2), py(0)); ctx.lineTo(px(3.2), py(0)); ctx.stroke();
  ctx.beginPath(); ctx.moveTo(px(0), py(-3.2)); ctx.lineTo(px(0), py(3.2)); ctx.stroke();

  // 最大可达半径
  ctx.setLineDash([4, 4]); ctx.strokeStyle = "#bbb";
  ctx.beginPath(); ctx.arc(cx, cy, maxReach * scale, 0, 2 * Math.PI); ctx.stroke();
  ctx.setLineDash([]);

  // 目标轨迹
  ctx.strokeStyle = "#90caf9"; ctx.lineWidth = 1.5; ctx.beginPath();
  FRAMES.forEach((fr, i) => {
    const X = px(fr.target.X ?? fr.target.x), Y = py(fr.target.Y ?? fr.target.y);
    i === 0 ? ctx.moveTo(X, Y) : ctx.lineTo(X, Y);
  });
  ctx.stroke();

  // 机械臂
  const failed = f.status === 2 || f.status === 3;
  const joints = f.joints.map(p => [px(p.X ?? p.x), py(p.Y ?? p.y)]);
  ctx.lineWidth = 5; ctx.lineCap = "round";
  ctx.strokeStyle = failed ? "#9e9e9e" : "#1565c0";
  if (failed) ctx.setLineDash([8, 5]);
  ctx.beginPath();
  joints.forEach((j, i) => i === 0 ? ctx.moveTo(j[0], j[1]) : ctx.lineTo(j[0], j[1]));
  ctx.stroke();
  ctx.setLineDash([]);
  joints.forEach((j, i) => {
    ctx.beginPath(); ctx.arc(j[0], j[1], i === 0 ? 7 : 5, 0, 2 * Math.PI);
    ctx.fillStyle = i === 0 ? "#333" : (failed ? "#9e9e9e" : "#1e88e5");
    ctx.fill();
  });

  // 末端姿态指示
  const ee = joints[joints.length - 1];
  const phi = f.q.reduce((a, b) => a + b, 0);
  ctx.strokeStyle = failed ? "#9e9e9e" : "#43a047"; ctx.lineWidth = 3;
  ctx.beginPath(); ctx.moveTo(ee[0], ee[1]);
  ctx.lineTo(ee[0] + Math.cos(phi) * 0.35 * scale, ee[1] - Math.sin(phi) * 0.35 * scale);
  ctx.stroke();

  // 当前目标十字
  const tx = px(f.target.X ?? f.target.x), ty = py(f.target.Y ?? f.target.y);
  ctx.strokeStyle = failed ? "#c62828" : "#e53935"; ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.moveTo(tx - 8, ty - 8); ctx.lineTo(tx + 8, ty + 8);
  ctx.moveTo(tx + 8, ty - 8); ctx.lineTo(tx - 8, ty + 8);
  ctx.stroke();

  // 状态与信息面板
  statusBox.textContent = "第 " + (idx + 1) + " / " + FRAMES.length + " 帧 — " + STATUS_TEXT[f.status];
  statusBox.style.background = COLORS[f.status];
  const deg = r => (r * 180 / Math.PI).toFixed(2) + "°";
  let rows = "";
  f.q.forEach((q, i) => { rows += "<tr><td>关节 " + (i + 1) + "</td><td>" + deg(q) + "</td></tr>"; });
  rows += "<tr><td>可操作度</td><td>" + f.manip.toFixed(4) + "</td></tr>";
  rows += "<tr><td>目标 x</td><td>" + (f.target.X ?? f.target.x).toFixed(3) + "</td></tr>";
  rows += "<tr><td>目标 y</td><td>" + (f.target.Y ?? f.target.y).toFixed(3) + "</td></tr>";
  infoTable.innerHTML = rows;

  drawTimeline();
}

function drawTimeline() {
  tlCtx.clearRect(0, 0, tl.width, tl.height);
  const w = tl.width / FRAMES.length;
  FRAMES.forEach((fr, i) => {
    tlCtx.fillStyle = COLORS[fr.status];
    tlCtx.fillRect(i * w, 4, Math.max(1, w - 0.5), tl.height - 8);
  });
  tlCtx.fillStyle = "#000";
  tlCtx.fillRect(idx * w - 1, 0, 2, tl.height);
}

function seek(i) {
  idx = Math.max(0, Math.min(FRAMES.length - 1, i));
  slider.value = idx;
  drawScene();
}

slider.addEventListener("input", () => seek(+slider.value));
tl.addEventListener("click", e => {
  const r = tl.getBoundingClientRect();
  seek(Math.floor((e.clientX - r.left) / r.width * FRAMES.length));
});
playBtn.addEventListener("click", () => {
  playing = !playing;
  playBtn.textContent = playing ? "暂停" : "播放";
  if (playing) {
    timer = setInterval(() => {
      if (idx >= FRAMES.length - 1) { playBtn.click(); return; }
      seek(idx + 1);
    }, 50);
  } else {
    clearInterval(timer);
  }
});
document.addEventListener("keydown", e => {
  if (e.key === "ArrowRight") seek(idx + 1);
  if (e.key === "ArrowLeft") seek(idx - 1);
});

drawScene();
</script>
</body>
</html>
`
