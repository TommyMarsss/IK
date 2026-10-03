// Package ik 实现平面多连杆机械臂（2-link / 3-link）的正逆运动学。
//
// 模型：所有关节均为旋转关节，q[i] 为第 i 个关节的相对转角，
// 第 k 根连杆的绝对角为 q[0]+...+q[k]。基座固定于原点。
//
// 逆运动学采用解析法枚举全部几何解（2-link 的肘上/肘下两种构型，
// 3-link 在给定末端姿态 (x, y, phi) 时转化为腕心的 2-link 问题），
// 再按关节限位过滤，并提供"与当前关节角最接近"的多解选择策略。
package ik

import (
	"errors"
	"math"
)

// Point 是平面上的点。
type Point struct{ X, Y float64 }

// Pose 是末端位姿：位置 (X, Y) 与姿态角 Phi（弧度）。
// 对 2-link 机械臂 Phi 被忽略（末端姿态由 q 唯一确定且不可独立指定）。
type Pose struct{ X, Y, Phi float64 }

// JointLimits 是关节角限位（弧度，闭区间）。
type JointLimits struct{ Min, Max float64 }

// Unlimited 表示不施加额外限位（角度仍归一化到 (-pi, pi]）。
var Unlimited = JointLimits{Min: -math.Pi, Max: math.Pi}

// 求解失败的两类错误，必须可区分：
var (
	// ErrUnreachable 目标在工作空间之外，几何上无解。
	ErrUnreachable = errors.New("ik: target outside reachable workspace")
	// ErrLimitUnreachable 几何解存在，但全部违反关节限位。
	ErrLimitUnreachable = errors.New("ik: target in workspace but unreachable under joint limits")
	// ErrUnsupportedLinks 仅支持 2-link 与 3-link。
	ErrUnsupportedLinks = errors.New("ik: only 2-link and 3-link arms are supported")
)

const (
	// 默认奇异判据：可操作度（雅可比 J J^T 行列式的平方根）低于该值视为接近奇异。
	defaultSingularityThreshold = 0.15
	// 阻尼最小二乘的最大阻尼系数（奇异处）。
	defaultDampingMax = 0.3
)

// Arm 是平面旋转关节多连杆机械臂（2 或 3 根连杆）。
type Arm struct {
	Lengths []float64     // 连杆长度
	Limits  []JointLimits // 各关节限位
	// SingularityThreshold 奇异判据阈值：可操作度低于此值判定为接近奇异位形。
	SingularityThreshold float64
	// DampingMax 阻尼最小二乘在奇异处的最大阻尼系数。
	DampingMax float64
}

// NewArm 构造机械臂；limits 传 nil 表示所有关节不限位。
func NewArm(lengths []float64, limits []JointLimits) *Arm {
	if limits == nil {
		limits = make([]JointLimits, len(lengths))
		for i := range limits {
			limits[i] = Unlimited
		}
	}
	return &Arm{
		Lengths:              lengths,
		Limits:               limits,
		SingularityThreshold: defaultSingularityThreshold,
		DampingMax:           defaultDampingMax,
	}
}

// NormalizeAngle 将角度归一化到 (-pi, pi]。
func NormalizeAngle(a float64) float64 {
	a = math.Mod(a+math.Pi, 2*math.Pi)
	if a < 0 {
		a += 2 * math.Pi
	}
	a -= math.Pi
	if a <= -math.Pi { // -pi 归入 pi，保持 (-pi, pi] 约定
		a += 2 * math.Pi
	}
	return a
}

// ForwardKinematics 正运动学：返回各关节位置（下标 0 为基座，最后为末端）
// 以及末端绝对姿态角。
func (a *Arm) ForwardKinematics(q []float64) (joints []Point, phi float64) {
	joints = make([]Point, len(a.Lengths)+1)
	theta := 0.0
	p := Point{}
	for i, l := range a.Lengths {
		theta += q[i]
		p.X += l * math.Cos(theta)
		p.Y += l * math.Sin(theta)
		joints[i+1] = p
	}
	return joints, theta
}

// Solve 枚举全部几何解并按关节限位过滤，返回全部可行解。
// 几何无解返回 ErrUnreachable；几何有解但全部越限返回 ErrLimitUnreachable，
// 二者可通过 errors.Is 区分，且越限时绝不返回违反限位的解。
func (a *Arm) Solve(target Pose) ([][]float64, error) {
	geoms, err := a.solveGeometric(target)
	if err != nil {
		return nil, err
	}
	var feas [][]float64
	for _, g := range geoms {
		nq := normalizeAll(g)
		if a.withinLimits(nq) {
			feas = append(feas, nq)
		}
	}
	if len(feas) == 0 {
		return nil, ErrLimitUnreachable
	}
	return feas, nil
}

// solveGeometric 枚举全部几何解（忽略限位）。
func (a *Arm) solveGeometric(target Pose) ([][]float64, error) {
	switch len(a.Lengths) {
	case 2:
		sols, ok := ik2(a.Lengths[0], a.Lengths[1], Point{target.X, target.Y})
		if !ok {
			return nil, ErrUnreachable
		}
		out := make([][]float64, 0, len(sols))
		for _, s := range sols {
			out = append(out, []float64{s[0], s[1]})
		}
		return out, nil
	case 3:
		sols, ok := ik3(a.Lengths[0], a.Lengths[1], a.Lengths[2], target)
		if !ok {
			return nil, ErrUnreachable
		}
		out := make([][]float64, 0, len(sols))
		for _, s := range sols {
			out = append(out, []float64{s[0], s[1], s[2]})
		}
		return out, nil
	default:
		return nil, ErrUnsupportedLinks
	}
}

// ik2 求解 2-link 逆运动学，返回肘下/肘上两个解（q2 一负一正）。
// 目标不可达时 ok=false。完全伸直/完全折叠时两解退化为同一解。
func ik2(l1, l2 float64, t Point) (sols [][2]float64, ok bool) {
	const tol = 1e-9
	d2 := t.X*t.X + t.Y*t.Y
	c := (d2 - l1*l1 - l2*l2) / (2 * l1 * l2)
	if c > 1+tol || c < -1-tol {
		return nil, false
	}
	// 数值钳制：|c| 略超 1 时视为恰好在工作空间边界（奇异位形）。
	if c > 1 {
		c = 1
	} else if c < -1 {
		c = -1
	}
	q2 := math.Acos(c)
	base := math.Atan2(t.Y, t.X)
	mk := func(elbow float64) [2]float64 {
		q1 := base - math.Atan2(l2*math.Sin(elbow), l1+l2*math.Cos(elbow))
		return [2]float64{q1, elbow}
	}
	return [][2]float64{mk(q2), mk(-q2)}, true
}

// ik3 求解 3-link 逆运动学：由目标姿态 phi 反推腕心位置，
// 转化为 2-link 问题，再解出 q3。
func ik3(l1, l2, l3 float64, t Pose) (sols [][3]float64, ok bool) {
	wrist := Point{
		X: t.X - l3*math.Cos(t.Phi),
		Y: t.Y - l3*math.Sin(t.Phi),
	}
	s2, ok := ik2(l1, l2, wrist)
	if !ok {
		return nil, false
	}
	for _, s := range s2 {
		q3 := t.Phi - s[0] - s[1]
		sols = append(sols, [3]float64{s[0], s[1], q3})
	}
	return sols, true
}

func normalizeAll(q []float64) []float64 {
	out := make([]float64, len(q))
	for i, v := range q {
		out[i] = NormalizeAngle(v)
	}
	return out
}

func (a *Arm) withinLimits(q []float64) bool {
	const tol = 1e-9
	for i, v := range q {
		if v < a.Limits[i].Min-tol || v > a.Limits[i].Max+tol {
			return false
		}
	}
	return true
}

// SelectClosest 多解选择策略：在可行解中选取与 current 关节角
// 角距离平方和最小的解。沿连续轨迹逐帧调用可保证构型不跳变
// （始终停留在同一解分支上，如始终保持肘下构型）。
func SelectClosest(sols [][]float64, current []float64) []float64 {
	best := -1
	bestCost := math.Inf(1)
	for i, s := range sols {
		cost := 0.0
		for j := range s {
			d := NormalizeAngle(s[j] - current[j])
			cost += d * d
		}
		if cost < bestCost {
			bestCost, best = cost, i
		}
	}
	return append([]float64(nil), sols[best]...)
}

// Jacobian 返回末端位置的 2×n 雅可比矩阵（行 0 为 dx/dq，行 1 为 dy/dq）。
func (a *Arm) Jacobian(q []float64) [2][]float64 {
	n := len(a.Lengths)
	theta := make([]float64, n)
	acc := 0.0
	for i := range theta {
		acc += q[i]
		theta[i] = acc
	}
	var J [2][]float64
	J[0] = make([]float64, n)
	J[1] = make([]float64, n)
	for i := 0; i < n; i++ {
		sx, sy := 0.0, 0.0
		for k := i; k < n; k++ {
			sx += -a.Lengths[k] * math.Sin(theta[k])
			sy += a.Lengths[k] * math.Cos(theta[k])
		}
		J[0][i] = sx
		J[1][i] = sy
	}
	return J
}

// Manipulability 可操作度 w = sqrt(det(J J^T))，w → 0 时雅可比接近不可逆。
// 对 2-link 而言 w = |l1*l2*sin(q2)|，即完全伸直或完全折叠时为 0。
func (a *Arm) Manipulability(q []float64) float64 {
	J := a.Jacobian(q)
	var aa, bb, cc float64
	for i := range J[0] {
		aa += J[0][i] * J[0][i]
		bb += J[0][i] * J[1][i]
		cc += J[1][i] * J[1][i]
	}
	det := aa*cc - bb*bb
	if det < 0 {
		det = 0
	}
	return math.Sqrt(det)
}

// IsSingular 奇异判据：可操作度低于阈值即判定为接近奇异位形。
func (a *Arm) IsSingular(q []float64) bool {
	return a.Manipulability(q) < a.SingularityThreshold
}

// VelocityIK 用阻尼最小二乘（DLS）伪逆将末端速度映射为关节速度：
//
//	qdot = J^T (J J^T + λ²I)^{-1} xdot
//
// 阻尼系数 λ 随可操作度低于阈值而平滑增大（Nakamura-Hanafusa 形式），
// 因此在奇异位形附近关节速度保持有界，不会发散，也不会出现除零。
// 远离奇异时 λ=0，退化为精确伪逆。
func (a *Arm) VelocityIK(q, xdot []float64) []float64 {
	J := a.Jacobian(q)
	w := a.Manipulability(q)
	lambda := 0.0
	if w < a.SingularityThreshold {
		r := w / a.SingularityThreshold
		lambda = a.DampingMax * math.Sqrt(1-r*r)
	}
	var aa, bb, cc float64
	for i := range J[0] {
		aa += J[0][i] * J[0][i]
		bb += J[0][i] * J[1][i]
		cc += J[1][i] * J[1][i]
	}
	l2 := lambda * lambda
	// (J J^T + λ²I) 为 2×2 对称半正定矩阵，λ>0 时严格正定，解析求逆无除零风险。
	det := (aa+l2)*(cc+l2) - bb*bb
	if det < 1e-15 {
		det = 1e-15 // 防御性兜底（λ=0 且恰好奇异时）
	}
	ix := ((cc+l2)*xdot[0] - bb*xdot[1]) / det
	iy := (-bb*xdot[0] + (aa+l2)*xdot[1]) / det
	qdot := make([]float64, len(a.Lengths))
	for i := range qdot {
		qdot[i] = J[0][i]*ix + J[1][i]*iy
	}
	return qdot
}
