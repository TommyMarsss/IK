package ik

import (
	"errors"
	"math"
	"testing"
)

const eps = 1e-9

func fkPos(a *Arm, q []float64) (Point, float64) {
	joints, phi := a.ForwardKinematics(q)
	return joints[len(joints)-1], phi
}

// 可达目标点必须枚举出肘上/肘下两种构型，且每个解经正运动学验证都精确到达目标。
func TestTwoLinkBothConfigurations(t *testing.T) {
	arm := NewArm([]float64{1, 1}, nil)
	target := Pose{X: 1, Y: 1}

	sols, err := arm.Solve(target)
	if err != nil {
		t.Fatalf("Solve failed: %v", err)
	}
	if len(sols) != 2 {
		t.Fatalf("expected 2 configurations, got %d", len(sols))
	}
	// 两解的肘关节角应互为相反数（肘上/肘下）。
	if math.Abs(sols[0][1]+sols[1][1]) > eps {
		t.Fatalf("elbow angles not symmetric: %v, %v", sols[0][1], sols[1][1])
	}
	if math.Abs(sols[0][1]) < 1e-6 {
		t.Fatalf("expected two distinct configurations, both q2≈0")
	}
	for i, q := range sols {
		p, _ := fkPos(arm, q)
		if d := math.Hypot(p.X-target.X, p.Y-target.Y); d > 1e-9 {
			t.Fatalf("solution %d FK error %g", i, d)
		}
	}
}

// 3-link 在给定位姿下同样应枚举两种构型，位置与姿态都要吻合。
func TestThreeLinkBothConfigurations(t *testing.T) {
	arm := NewArm([]float64{1.2, 1.0, 0.8}, nil)
	target := Pose{X: 1.5, Y: 0.8, Phi: 0.6}

	sols, err := arm.Solve(target)
	if err != nil {
		t.Fatalf("Solve failed: %v", err)
	}
	if len(sols) != 2 {
		t.Fatalf("expected 2 configurations, got %d", len(sols))
	}
	for i, q := range sols {
		p, phi := fkPos(arm, q)
		if d := math.Hypot(p.X-target.X, p.Y-target.Y); d > 1e-9 {
			t.Fatalf("solution %d position error %g", i, d)
		}
		if d := math.Abs(NormalizeAngle(phi - target.Phi)); d > 1e-9 {
			t.Fatalf("solution %d orientation error %g", i, d)
		}
	}
}

// 沿连续轨迹用 SelectClosest 逐帧选解，关节角不得发生跳变，
// 且应始终停留在同一构型分支（q2 符号不变）。
func TestSelectionContinuity(t *testing.T) {
	arm := NewArm([]float64{1, 1}, nil)
	const n = 200
	targets := make([]Pose, n)
	for i := range targets {
		y := -0.6 + 1.2*float64(i)/float64(n-1)
		targets[i] = Pose{X: 1.4, Y: y}
	}

	frames := arm.SolveTrajectory(targets, nil)
	for i, f := range frames {
		if f.Status != StatusOK {
			t.Fatalf("frame %d unexpectedly failed: %v", i, f.Status)
		}
	}
	// 构型分支一致：q2 符号全程不变。
	sign := math.Signbit(frames[0].Q[1])
	for i, f := range frames {
		if math.Signbit(f.Q[1]) != sign {
			t.Fatalf("configuration switched at frame %d (q2 sign flip)", i)
		}
	}
	// 相邻帧关节角变化有界（无跳变）。
	for i := 1; i < n; i++ {
		for j := range frames[i].Q {
			d := math.Abs(NormalizeAngle(frames[i].Q[j] - frames[i-1].Q[j]))
			if d > 0.1 {
				t.Fatalf("joint %d jumped %g rad between frames %d and %d", j, d, i-1, i)
			}
		}
	}
}

// 奇异位形附近：可操作度判据触发，且阻尼最小二乘的关节速度有界、无 NaN/Inf。
func TestSingularityStability(t *testing.T) {
	arm := NewArm([]float64{1, 1}, nil)
	xdot := []float64{1.0, 0.5}

	// 完全伸直（q2=0）与接近伸直两种情形。
	for _, q := range [][]float64{{0.3, 0.0}, {0.3, 1e-4}, {0.3, 0.01}} {
		if !arm.IsSingular(q) {
			t.Fatalf("q=%v should be singular (manip=%g)", q, arm.Manipulability(q))
		}
		qdot := arm.VelocityIK(q, xdot)
		for i, v := range qdot {
			if math.IsNaN(v) || math.IsInf(v, 0) {
				t.Fatalf("q=%v: qdot[%d] is not finite: %v", q, i, v)
			}
			if math.Abs(v) > 100 {
				t.Fatalf("q=%v: qdot[%d]=%v diverged near singularity", q, i, v)
			}
		}
	}
}

// 远离奇异时阻尼为零，VelocityIK 应退化为精确解：J*qdot == xdot。
func TestVelocityIKExactAwayFromSingularity(t *testing.T) {
	arm := NewArm([]float64{1, 1}, nil)
	q := []float64{0.5, 1.0}
	xdot := []float64{0.3, -0.2}

	qdot := arm.VelocityIK(q, xdot)
	J := arm.Jacobian(q)
	for r := 0; r < 2; r++ {
		got := J[r][0]*qdot[0] + J[r][1]*qdot[1]
		if math.Abs(got-xdot[r]) > 1e-9 {
			t.Fatalf("row %d: J*qdot=%g, want %g", r, got, xdot[r])
		}
	}
}

// 限位不可达与几何不可达必须被区分报告，且限位失败时不得返回越限解。
func TestLimitVsGeometricUnreachable(t *testing.T) {
	arm := NewArm([]float64{1, 1}, []JointLimits{
		{Min: -0.1, Max: 0.1}, // 关节 1 几乎锁死
		Unlimited,
	})

	// (1.5, 0.5)：几何可达（d≈1.58 < 2），但两个构型的 q1≈-0.34 / 0.98 均越限。
	_, err := arm.Solve(Pose{X: 1.5, Y: 0.5})
	if !errors.Is(err, ErrLimitUnreachable) {
		t.Fatalf("expected ErrLimitUnreachable, got %v", err)
	}
	if errors.Is(err, ErrUnreachable) {
		t.Fatalf("limit-unreachable must not be conflated with geometric-unreachable")
	}

	// (5, 0)：超出工作空间，几何无解。
	_, err = arm.Solve(Pose{X: 5, Y: 0})
	if !errors.Is(err, ErrUnreachable) {
		t.Fatalf("expected ErrUnreachable, got %v", err)
	}
	if errors.Is(err, ErrLimitUnreachable) {
		t.Fatalf("geometric-unreachable must not be reported as limit-unreachable")
	}

	// 放宽限位后同一点应可解，且解不越限。
	arm.Limits[0] = Unlimited
	sols, err := arm.Solve(Pose{X: 1.5, Y: 0.5})
	if err != nil {
		t.Fatalf("Solve failed after relaxing limits: %v", err)
	}
	for _, q := range sols {
		if !arm.withinLimits(q) {
			t.Fatalf("returned solution violates limits: %v", q)
		}
	}
}

// 轨迹层面：限位失败帧与几何不可达帧的状态标记必须正确区分。
func TestTrajectoryStatusClassification(t *testing.T) {
	arm := NewArm([]float64{1, 1}, []JointLimits{{Min: -0.15, Max: 0.15}, Unlimited})
	targets := []Pose{
		{X: 1.5, Y: 0.5}, // 限位不可达（两构型 q1≈-0.34 / 0.98）
		{X: 5, Y: 0},     // 几何不可达
		{X: 1.99, Y: 0},  // 正常（q1≈±0.10 在限位内）
	}
	frames := arm.SolveTrajectory(targets, nil)
	want := []FrameStatus{StatusLimitUnreachable, StatusUnreachable, StatusOK}
	for i, f := range frames {
		if f.Status != want[i] {
			t.Fatalf("frame %d: status %v, want %v", i, f.Status, want[i])
		}
	}
	// 失败帧保持上一帧构型（不跳变、不输出越限角）。
	if frames[0].Q[0] != frames[1].Q[0] || frames[0].Q[1] != frames[1].Q[1] {
		t.Fatalf("failed frames should hold previous configuration")
	}
}

func TestNormalizeAngle(t *testing.T) {
	cases := map[float64]float64{
		0:              0,
		3 * math.Pi:    math.Pi,
		-3 * math.Pi:   math.Pi,
		math.Pi / 2:    math.Pi / 2,
		2*math.Pi + 1:  1,
		-2*math.Pi - 1: -1,
	}
	for in, want := range cases {
		if got := NormalizeAngle(in); math.Abs(got-want) > 1e-12 {
			t.Fatalf("NormalizeAngle(%v) = %v, want %v", in, got, want)
		}
	}
}
