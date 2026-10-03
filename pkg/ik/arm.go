// Package ik implements forward/inverse kinematics for a planar revolute
// manipulator (2-link and 3-link) using only the Go standard library.
//
// Angle convention:
//   - q[0] is the absolute angle of link 1 measured from the +x axis;
//   - q[i] (i >= 1) is the RELATIVE joint angle, i.e. the angle between
//     link i and link i-1 (positive = counter-clockwise);
//   - end-effector orientation is phi = q[0]+q[1]+...+q[n-1].
//
// All angles are stored/returned in radians and normalised into [-pi, pi].
package ik

import "math"

// Vec2 is a 2D point in the manipulator plane (x forward, y up).
type Vec2 struct {
	X, Y float64
}

// Limit is a hard joint angle limit in radians. Angles are compared in the
// [-pi, pi] representation.
type Limit struct {
	Min, Max float64
}

// Contains reports whether a (already wrapped) angle satisfies the limit.
func (l Limit) Contains(a float64) bool {
	return a >= l.Min-angularTol && a <= l.Max+angularTol
}

// Width returns the allowed angular interval.
func (l Limit) Width() float64 { return l.Max - l.Min }

// Pose is a Cartesian target; HasOrient requests a full position+orientation
// pose (only meaningful for the 3-link arm).
type Pose struct {
	X, Y float64
	Phi  float64 // desired end-effector orientation when HasOrient is true

	HasOrient bool
}

// Arm is a planar revolute arm with n links of lengths L[i] and per-joint
// limits.
type Arm struct {
	L      []float64
	Limits []Limit
}

// NewArm2 builds a 2-link arm.
func NewArm2(l1, l2 float64, shoulder, elbow Limit) *Arm {
	return &Arm{
		L:      []float64{l1, l2},
		Limits: []Limit{shoulder, elbow},
	}
}

// NewArm3 builds a 3-link arm.
func NewArm3(l1, l2, l3 float64, l1lim, l2lim, l3lim Limit) *Arm {
	return &Arm{
		L:      []float64{l1, l2, l3},
		Limits: []Limit{l1lim, l2lim, l3lim},
	}
}

// N returns the number of joints/links.
func (a *Arm) N() int { return len(a.L) }

// Reach returns the outer (fully stretched) and inner (fully folded) radii of
// the position workspace.
func (a *Arm) Reach() (outer, inner float64) {
	outer = 0.0
	for _, l := range a.L {
		outer += l
	}
	// Inner radius = max(0, longest link - sum of the others).
	max := 0.0
	sum := 0.0
	for _, l := range a.L {
		sum += l
		if l > max {
			max = l
		}
	}
	inner = math.Max(0, 2*max-sum)
	return outer, inner
}

// FK evaluates the forward kinematics. It returns the n+1 joint positions
// (index 0 = base, index n = end effector) and the end-effector orientation.
func (a *Arm) FK(q []float64) ([]Vec2, float64) {
	n := a.N()
	if len(q) != n {
		panic("ik: FK called with wrong number of joint angles")
	}
	pts := make([]Vec2, n+1)
	phi := 0.0
	x, y := 0.0, 0.0
	for i := 0; i < n; i++ {
		phi += q[i]
		x += a.L[i] * math.Cos(phi)
		y += a.L[i] * math.Sin(phi)
		pts[i+1] = Vec2{X: x, Y: y}
	}
	return pts, phi
}

// Jacobian evaluates the kinematic Jacobian at q.
//
// With orient == false it returns the 2 x n position Jacobian
//
//	Jv = [ -sum_{k>=j} Lk sin(phi_{k+1}) ]
//	     [  sum_{k>=j} Lk cos(phi_{k+1}) ]
//
// With orient == true (n == 3 only) a third row [1 1 1] is appended, giving
// the 3 x n pose Jacobian [dx/dq; dy/dq; dphi/dq].
func (a *Arm) Jacobian(q []float64, orient bool) *Matrix {
	n := a.N()
	rows := 2
	if orient {
		rows = 3
	}
	J := NewMatrix(rows, n)

	// Cumulative link direction angles phi[k+1] = q[0]+...+q[k].
	phi := 0.0
	phis := make([]float64, n)
	for i := 0; i < n; i++ {
		phi += q[i]
		phis[i] = phi
	}

	for j := 0; j < n; j++ {
		dx, dy := 0.0, 0.0
		for k := j; k < n; k++ {
			dx -= a.L[k] * math.Sin(phis[k])
			dy += a.L[k] * math.Cos(phis[k])
		}
		J.Set(0, j, dx)
		J.Set(1, j, dy)
		if orient {
			J.Set(2, j, 1)
		}
	}
	return J
}

// wrapPi maps an angle into [-pi, pi].
func wrapPi(a float64) float64 {
	for a > math.Pi {
		a -= 2 * math.Pi
	}
	for a < -math.Pi {
		a += 2 * math.Pi
	}
	return a
}

// angDist is the shortest signed angular distance between two angles.
func angDist(a, b float64) float64 {
	return wrapPi(a - b)
}
