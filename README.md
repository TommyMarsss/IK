# IK — 平面多连杆机械臂逆运动学求解器

纯 Go 标准库实现的平面 2-link / 3-link 机械臂逆运动学（IK）求解器，面向
**多解选择策略**与**奇异位形鲁棒性**的验证。附带单文件 HTML 轨迹回放工具
（原生 HTML/CSS/JavaScript，无任何第三方依赖）。

## 运动学模型

- 平面旋转关节串联臂，基座固定于原点，连杆长度 `L[0..n-1]`（n = 2 或 3）。
- 关节角 `q[i]` 为相对转角，第 k 根连杆的绝对角为 `q[0]+…+q[k]`。
- 正运动学：末端位置为各连杆向量累加；3-link 时末端姿态角 `phi = Σq` 可独立指定。
- 逆运动学为**解析解**：
  - **2-link**：由余弦定理得 `cos(q2) = (d² − L1² − L2²) / (2·L1·L2)`，
    `q2 = ±acos(·)` 对应**肘下 / 肘上**两种构型，`q1` 由几何关系唯一确定。
  - **3-link**：由目标姿态 `phi` 反推腕心 `w = p − L3·(cos phi, sin phi)`，
    转化为 2-link 问题，再解 `q3 = phi − q1 − q2`，同样枚举两种构型。
- 角度统一归一化到 `(-π, π]`。

## 多解枚举与选择策略

`Arm.Solve(target)` 返回**全部**几何可行且满足限位的解（通常 2 个构型）。
轨迹跟踪时由 `SelectClosest(sols, current)` 选取与上一帧关节角
**角距离平方和最小**的解。由于轨迹连续、相邻帧目标位移小，该贪心策略
会始终停留在同一解分支上（例如全程保持肘下构型），相邻采样点之间
不会发生构型跳变。`TestSelectionContinuity` 对此有自动化验证
（200 帧轨迹上逐帧关节角变化有界、`q2` 符号全程不变）。

## 奇异判据与退化处理

- **判据**：可操作度 `w = sqrt(det(J·Jᵀ))`，其中 J 为末端位置的 2×n 雅可比。
  对 2-link，`w = |L1·L2·sin(q2)|`——完全伸直（q2=0）或完全折叠（q2=±π）时
  w = 0，雅可比不可逆。`w < SingularityThreshold`（默认 0.15）即判定为
  接近奇异位形（`Arm.IsSingular`），轨迹帧标记为 `StatusNearSingular`。
- **退化处理**：速度级求解 `Arm.VelocityIK` 采用**阻尼最小二乘**（DLS）伪逆
  `q̇ = Jᵀ(JJᵀ + λ²I)⁻¹ẋ`，阻尼系数 λ 随可操作度跌入阈值以下按
  Nakamura–Hanafusa 形式 `λ = λmax·√(1 − (w/w_th)²)` 平滑增大：
  - 远离奇异时 λ = 0，退化为精确伪逆（`J·q̇ = ẋ` 严格成立）；
  - 奇异附近 `JJᵀ + λ²I` 严格正定，2×2 解析求逆**无除零、无病态矩阵**，
    关节速度保持有界（`TestSingularityStability` 验证 q2=0 处无 NaN/Inf、不发散）。
- 位置级解析解在边界处对 `cos(q2)` 做数值钳制（|c| 略超 1 视为恰好到达边界），
  不会因浮点误差把边界目标误判为不可达。

## 关节限位与失败分类

求解失败明确区分为两类，可通过 `errors.Is` 区分，绝不混淆：

| 情形 | 错误 / 帧状态 | 含义 |
|---|---|---|
| 几何不可达 | `ErrUnreachable` / `StatusUnreachable` | 目标在工作空间外，不存在几何解 |
| 限位不可达 | `ErrLimitUnreachable` / `StatusLimitUnreachable` | 几何解存在，但全部违反关节限位 |

限位过滤在几何枚举之后进行：越限的解被直接丢弃，**绝不会返回违反限位的解**；
几何解全部越限时返回 `ErrLimitUnreachable` 而非静默给出越限构型。
轨迹求解中失败帧保持上一帧构型（机械臂停在原地），保证回放连续。

## 代码结构

```
ik/ik.go           运动学模型、解析 IK、多解选择、雅可比、DLS 速度解、奇异判据
ik/trajectory.go   轨迹逐帧求解与帧状态分类
ik/ik_test.go      自动化测试
cmd/ikdemo/main.go 演示：生成轨迹 → 求解 → 输出单文件 HTML 回放页
```

## 使用

```bash
go test ./...        # 运行全部测试
go run ./cmd/ikdemo  # 生成 ik_replay.html（默认文件名，可传参指定）
```

用浏览器打开生成的 HTML：播放/暂停、拖动进度条、点击时间轴逐帧回放；
奇异帧（橙）、限位不可达帧（红）、几何不可达帧（灰）在时间轴与画布上高亮。
数据与逻辑全部内嵌，不启动任何服务、不加载任何外部资源。

## 测试覆盖

- `TestTwoLinkBothConfigurations` / `TestThreeLinkBothConfigurations`：
  可达点的两种构型解均枚举正确，正运动学回代位置（与姿态）误差 < 1e-9。
- `TestSelectionContinuity`：连续轨迹下选解不跳变（构型分支一致、关节角增量有界）。
- `TestSingularityStability`：奇异位形附近（含 q2=0 完全伸直）速度解有界、无 NaN/Inf；
  `TestVelocityIKExactAwayFromSingularity`：远离奇异时 DLS 退化为精确解。
- `TestLimitVsGeometricUnreachable` / `TestTrajectoryStatusClassification`：
  限位不可达与几何不可达被正确区分报告，且越限解不会被返回。
