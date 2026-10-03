package ik

import (
	"math"
	"testing"
)

// TestAnalytic3PositionOnly: a position-only target on the redundant 3-link
// arm is resolved by inheriting the current tool orientation, still yielding
// two branches that reach the target position with that orientation.
func TestAnalytic3PositionOnly(t *testing.T) {
	arm := NewArm3(1, 0.9, 0.6,
		Limit{Min: -math.Pi, Max: math.Pi},
		Limit{Min: -math.Pi, Max: math.Pi},
		Limit{Min: -math.Pi, Max: math.Pi},
	)
	solver := NewSolver(arm, nil)
	q0 := []float64{0.5, -0.4, 0.2}
	solver.Reset(q0)
	_, wantPhi := arm.FK(q0)

	p := Pose{X: 1.5, Y: 0.6}
	sol := solver.Solve(p)
	if sol.Status != StatusOK && sol.Status != StatusSingular {
		t.Fatalf("status = %s (%s)", sol.Status, sol.Message)
	}
	pts, phi := arm.FK(sol.Q)
	ee := pts[3]
	approxEq(t, "ee.x", ee.X, p.X, 1e-6)
	approxEq(t, "ee.y", ee.Y, p.Y, 1e-6)
	approxEq(t, "inherited phi", phi, wantPhi, 1e-6)
	if sol.Branch != BranchElbowUp && sol.Branch != BranchElbowDown {
		t.Errorf("unexpected branch label %q", sol.Branch)
	}
}

// TestMatrixSingularValuesKnown: check the Jacobi eigendecomposition against
// a matrix with known singular values.
func TestMatrixSingularValuesKnown(t *testing.T) {
	// diag(3, 1) already has singular values {3, 1}.
	m := NewMatrix(2, 2)
	m.Set(0, 0, 3)
	m.Set(1, 1, 1)
	got := m.SingularValues()
	approxEq(t, "sigma1", got[0], 3, 1e-12)
	approxEq(t, "sigma2", got[1], 1, 1e-12)

	// A = U diag(2, 0.5) V^T with known rotations -> same singular values.
	u := rot2(0.7)
	v := rot2(-0.3)
	d := NewMatrix(2, 2)
	d.Set(0, 0, 2)
	d.Set(1, 1, 0.5)
	a := u.Mul(d).Mul(v.Transpose())
	got2 := a.SingularValues()
	approxEq(t, "rot sigma1", got2[0], 2, 1e-10)
	approxEq(t, "rot sigma2", got2[1], 0.5, 1e-10)
}

func rot2(a float64) *Matrix {
	m := NewMatrix(2, 2)
	m.Set(0, 0, math.Cos(a))
	m.Set(0, 1, -math.Sin(a))
	m.Set(1, 0, math.Sin(a))
	m.Set(1, 1, math.Cos(a))
	return m
}
