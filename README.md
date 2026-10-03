# 平面多连杆机械臂逆运动学求解器（Go 标准库）

纯 Go（仅标准库）实现的平面 2 连杆 / 3 连杆旋转关节机械臂逆运动学（IK），
用于检验**多解选择策略**、**奇异位形附近的数值鲁棒性**与**关节限位处理**。
求解一段目标轨迹后，生成一个**完全内嵌、无任何第三方依赖、无需服务器**的
单一静态 HTML 报告，浏览器直接打开即可逐帧回放。

## 目录结构

```
pkg/ik/                IK 核心库
  arm.go               运动学模型：FK、雅可比、工作空间
  analytic2.go         2R 几何解析解（枚举肘部上/下两个构型）
  analytic3.go         3R 位姿解析解（给定末端位置+朝向）
  matrix.go            自研小型矩阵运算：乘法/转置/高斯消元/
                       Jacobi 对称特征值（用于奇异值）
  solver.go            状态化求解器：构型选择 + DLS 阻尼最小二乘 + 限位
  *_test.go            自动化测试
internal/report/       单一静态 HTML 报告生成（原生 Canvas + JavaScript）
cmd/demo/              演示轨迹：正常跟踪 / 完全伸直奇异 /
                       限位不可达 / 几何不可达
report.html            演示输出（已被 .gitignore 忽略，可随时再生成）
```

## 快速开始

```bash
go test ./...                 # 运行全部自动化测试
go run ./cmd/demo             # 生成 ./report.html
go run ./cmd/demo out.html    # 自定义输出路径
```

然后用浏览器直接打开 `report.html`（`file://` 即可，不需要任何服务）：
播放/暂停、拖动进度条、点击帧编号跳转；奇异帧黄色高亮，限位失败帧红色
（并以红色虚影显示"如果放松限位本可以到达"的几何候选），几何不可达帧灰色，
目标点画红色叉号。

## 运动学模型

n 个旋转关节、n 根连杆，连杆长度 $L_0,\dots,L_{n-1}$。角度约定：

- $q_0$：第一根连杆相对 +x 轴的**绝对**角；
- $q_i\ (i\ge1)$：第 i 根连杆相对第 i-1 根的**相对**关节角（逆时针为正）；
- 末端朝向 $\varphi=\sum_i q_i$。

正运动学：

$$
\phi_k=\sum_{j=0}^{k}q_j,\qquad
P_k=\sum_{j=0}^{k-1}L_j(\cos\phi_j,\sin\phi_j)
$$

位置雅可比（列 j 为末端线速度对 q_j 的偏导）：

$$
J_v=\begin{bmatrix}-\sum_{k\ge j}L_k\sin\phi_k\\[2pt] \sum_{k\ge j}L_k\cos\phi_k\end{bmatrix}
$$

3 连杆带朝向时追加第三行 $[1\,1\,1]$（$\dot\varphi=\sum\dot q_j$）。

### 2R 解析解（枚举全部几何解）

目标 $(x,y)$，极坐标 $r=\sqrt{x^2+y^2}$、$\theta=\operatorname{atan2}(y,x)$。
由肘部三角形余弦定理，相对肘关节角的弯曲量为

$$
\beta=\operatorname{acos}\!\bigl((r^2-L_0^2-L_1^2)/(2L_0L_1)\bigr)
\in[0,\pi],
$$

底角 $\alpha=\operatorname{acos}((r^2+L_0^2-L_1^2)/(2L_0r))$。两个构型为

- 肘部朝上：$q_0=\theta+\alpha,\ q_1=-\beta$
- 肘部朝下：$q_0=\theta-\alpha,\ q_1=+\beta$

$q_1=0$ 为完全伸直、$|q_1|=\pi$ 为完全折叠。求解器**总是返回两个构型**，
并分别标注是否满足关节限位。

### 3R 位姿解析解

给定末端位置与朝向 $(x,y,\varphi)$，腕部点
$W=(x-L_2\cos\varphi,\ y-L_2\sin\varphi)$，前两连杆对 W 做 2R 解析解
（同样两个肘部构型），再由 $q_2=\varphi-q_0-q_1$ 闭合朝向。
仅给位置（冗余情形）时，策略是**继承当前末端绝对朝向**，化为确定问题，
同样枚举两个构型。

### 工作空间

位置工作空间为圆环，半径

$$
r_{\max}=\sum_i L_i,\qquad r_{\min}=\max(0,\,2L_{\max}-\sum_i L_i).
$$

等长 2 连杆时 $r_{\min}=0$。工作空间判定与限位无关——这是区分两类失败
的前提（见下）。

## 多解选择与构型连续性

对同一目标存在多个可行几何解时，选择策略是**与当前关节角最近**：

1. 代价 = 各关节最短角度差（绕回 $[-\pi,\pi]$）之和；
2. 取代价最小的可行解；
3. **分支迟滞（hysteresis，默认 0.2 rad）**：上一帧跟踪的分支只要其代价
   不比最优代价差超过该裕度，就继续保持。完全伸直/完全折叠处两个构型
   重合，纯最近邻策略会在此处来回翻转，迟滞裕度消除这种翻转。

求解器是**有状态**的（`Reset` 设定初值，`Solve` 逐点推进）。
求解失败时状态不前进（机械臂停在上一个有效构型），孤立的不可达帧不会
破坏后续轨迹的连续性。

## 奇异位形判据与退化处理

**判据**：计算雅可比的奇异值 $\sigma_i$（对 $J^\top J$ 做自研 Jacobi
旋转求特征值再开方，不依赖任何线性代数库）。当

$$
\sigma_{\min}(J) < \sigma_0 \quad(\text{默认 } \sigma_0=10^{-2})
$$

时判定进入奇异区域。平面 2R 在**完全伸直**（$q_1=0$，无法沿臂方向伸缩）
和**完全折叠**（等长臂、$r=0$）处秩亏；3R 各连杆共线时位姿雅可比同样秩亏。

**退化处理 —— 阻尼最小二乘（DLS / Levenberg–Marquardt）**：

$$
\Delta q=J^\top(JJ^\top+\lambda^2 I)^{-1}e
$$

- 被求逆矩阵因 $\lambda^2 I$ 而恒正定可逆，**数学上不存在除零**；
  代码中高斯消元仍带部分主元与防御性下限，极端情况下返回零步长而非 NaN。
- $\lambda$ 在奇异区域内随 $\sigma_{\min}$ 从 0 平滑增大到上限（默认 0.05），
  不可行的笛卡尔方向上的关节速度被连续衰减为有限值，而不是发散。
- 每次迭代的关节步长另有上限钳制（默认 0.5 rad）。
- 可达但处于奇异区域的解返回状态 `singular`（求解本身仍精确），
  与正常解 `ok` 区分。
- `Solver.JointVelocity` 提供 resolved-rate 速度求解，行为一致：
  例如完全伸直时，垂直臂方向（整体转动，可行）的速度正常通过，
  沿臂方向（继续伸长，不可行）的指令被衰减。

## 关节限位与两类失败的严格区分

每个几何候选都独立检查全部关节限位。状态分四类，**互不混淆**：

| 状态 | 含义 |
|---|---|
| `ok` | 找到满足限位的解 |
| `singular` | 解满足限位，但位于奇异区域（DLS 已退化处理） |
| `limit_unreachable` | 几何解**存在**（点在工作空间圆环内），但每个构型都违反至少一个关节限位："目标在工作空间内但当前限位下不可达"；返回离当前姿态最近的违规候选（仅供诊断/虚影显示，不得下发）及违规关节编号 |
| `unreachable` | 不存在任何几何解（圆环之外），无候选 |

此外数值精化未能收敛时返回 `no_convergence`。求解器保证：限位失败时
绝不返回一个违反限位的角度来冒充成功。

## HTML 报告

`internal/report` 把全部帧数据（关节角、FK 关节位置、状态、σ_min、残差、
违规候选等）序列化为 JSON 内嵌进 `<script type="application/json">` 块，
由原生 Canvas 2D + 原生 JavaScript 渲染（已对 `<`/`>`/`&` 做 Unicode
转义，可安全内嵌）。无网络请求、无 CDN、无框架/图形库，单文件可离线分发。

## API 速览

```go
arm := ik.NewArm2(1.0, 1.0,
    ik.Limit{Min: -math.Pi, Max: math.Pi}, // 肩关节
    ik.Limit{Min: -2.9, Max: 2.9},         // 肘关节（相对角）
)
solver := ik.NewSolver(arm, nil)
solver.Reset([]float64{0.3, -0.4})

sol := solver.Solve(ik.Pose{X: 1.4, Y: 0.3})
// sol.Status: ok | singular | limit_unreachable | unreachable | no_convergence
// sol.Q, sol.Branch ("elbow_up"/"elbow_down"), sol.MinSigma, ...

// 3 连杆带朝向：
arm3 := ik.NewArm3(1, 0.9, 0.6, lim0, lim1, lim2)
solver3.Solve(ik.Pose{X: 1.6, Y: 0.7, Phi: 0.4, HasOrient: true})
```

## 测试覆盖

`go test ./...` 覆盖：

- `TestAnalytic2BothBranches` / `TestAnalytic2BranchGeometry` /
  `TestAnalytic3PoseBothBranches`：可达点的两个构型都被枚举，且各自 FK
  精确回到目标；肘部上/下标签与几何一致；3R 位姿两解同时满足位置与朝向；
- `TestContinuityNoBranchFlip` / `TestHysteresisKeepsBranch`：密集连续轨迹
  （穿过完全伸直奇异点）分支标签不变、相邻帧关节角不发生量级 π 的跳变；
  迟滞策略在两分支几乎重合时保持既有分支；
- `TestSingularityAtFullExtension` / `TestNearSingularStability` /
  `TestFullAlignSingular3`：精确奇异点 σ_min≈0 且被标记；可行/不可行方向的
  笛卡尔速度都得到有限输出、不可行方向被衰减；穿越奇异区域全程无 NaN/Inf、
  步长有界、定位精确；
- `TestLimitVsGeometricUnreachable`：完全折叠点报告 `limit_unreachable`
  （含违规关节编号与诊断构型），圆环外点报告 `unreachable`，失败后状态
  不前进、后续目标仍可正常求解；`TestOneBranchBlockedByLimit` 验证只有
  单分支可行时自动选择可行分支；
- `TestWriteStandalone`：报告文件 JSON 载荷可往返解析、不含任何外部资源
  引用、包含回放控件与异常状态标记。
