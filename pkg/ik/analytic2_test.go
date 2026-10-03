package ik

import (
	"math"
	"testing"
)

const testEps = 1e-9

func approxEq(t *testing.T, name string, got, want, eps float64) {
	t.Helper()
	if math.IsNaN(got) || math.IsInf(got, 0) {
		t.Errorf("%s = %v, want finite %.6f", name, got, want)
		return
	}
	if math.Abs(got-want) > eps {
		t.Errorf("%s = %.10f, want %.10f (diff %.2e)", name, got, want, math.Abs(got-want))
	}
}

// TestAnalytic2BothBranches: every reachable target must yield exactly two
// geometric solutions, BOTH of which reproduce the target under FK, with
// opposite elbow signs.
func TestAnalytic2BothBranches(t *testing.T) {
	arm := NewArm2(1.0, 0.8,
		Limit{Min: -math.Pi, Max: math.Pi},
		Limit{Min: -math.Pi, Max: math.Pi},
	)
	cases := []Vec2{
		{X: 1.2, Y: 0.3},
		{X: 0.5, Y: 1.0},
		{X: -0.9, Y: 0.6},
		{X: 1.7999, Y: 0.0}, // near full extension
		{X: 0.21, Y: 0.0},   // deep fold (inner radius = 0.2)
	}
	for _, target := range cases {
		sols, ok := arm.Analytic2(target.X, target.Y)
		if !ok {
			t.Fatalf("(%v,%v) reported unreachable", target.X, target.Y)
		}
		if len(sols) != 2 {
			t.Fatalf("(%v,%v): got %d solutions, want 2", target.X, target.Y, len(sols))
		}
		if sols[0].Branch == sols[1].Branch {
			t.Fatalf("branch labels not distinct: %s %s", sols[0].Branch, sols[1].Branch)
		}
		for _, s := range sols {
			pts, _ := arm.FK(s.Q)
			ee := pts[2]
			approxEq(t, "ee.x", ee.X, target.X, 1e-8)
			approxEq(t, "ee.y", ee.Y, target.Y, 1e-8)
			if !s.WithinLimits {
				t.Errorf("(%v,%v) branch %s unexpectedly violates limits", target.X, target.Y, s.Branch)
			}
		}
		// The two configurations must differ in the elbow sign.
		if s := math.Signbit(sols[0].Q[1]); s == math.Signbit(sols[1].Q[1]) {
			t.Errorf("(%v,%v): elbow angles %v and %v do not form opposite branches",
				target.X, target.Y, sols[0].Q[1], sols[1].Q[1])
		}
	}
}

// TestAnalytic2BranchGeometry: for a target in the first quadrant the
// "elbow_up" solution really has its elbow joint above the base->target
// chord, and vice versa.
func TestAnalytic2BranchGeometry(t *testing.T) {
	arm := NewArm2(1, 1,
		Limit{Min: -math.Pi, Max: math.Pi},
		Limit{Min: -math.Pi, Max: math.Pi},
	)
	tx, ty := 1.2, 0.5
	sols, _ := arm.Analytic2(tx, ty)
	byBranch := map[string]AnalyticSolution{}
	for _, s := range sols {
		byBranch[s.Branch] = s
	}
	// Signed cross product (chord dir) x (elbow pos); >0 means elbow on the
	// left (CCW/"above") side of the base->target directed chord.
	cross := func(elbow Vec2) float64 { return tx*elbow.Y - ty*elbow.X }
	up, okUp := byBranch[BranchElbowUp]
	down, okDown := byBranch[BranchElbowDown]
	if !okUp || !okDown {
		t.Fatal("missing branch labels")
	}
	upPts, _ := arm.FK(up.Q)
	downPts, _ := arm.FK(down.Q)
	if cross(upPts[1]) <= 0 {
		t.Errorf("elbow_up elbow %v is not above the chord", upPts[1])
	}
	if cross(downPts[1]) >= 0 {
		t.Errorf("elbow_down elbow %v is not below the chord", downPts[1])
	}
}

// TestAnalytic2WorkspaceBoundary: exactly stretched (r=l1+l2) is reachable;
// anything beyond is geometrically unreachable regardless of limits.
func TestAnalytic2WorkspaceBoundary(t *testing.T) {
	arm := NewArm2(1, 1,
		Limit{Min: -math.Pi, Max: math.Pi},
		Limit{Min: -math.Pi, Max: math.Pi},
	)
	if _, ok := arm.Analytic2(2.0, 0); !ok {
		t.Error("full extension r=2 should be reachable")
	}
	if _, ok := arm.Analytic2(2.001, 0); ok {
		t.Error("r=2.001 must be geometrically unreachable")
	}
	armN := NewArm2(1, 0.5,
		Limit{Min: -math.Pi, Max: math.Pi},
		Limit{Min: -math.Pi, Max: math.Pi},
	)
	if _, ok := armN.Analytic2(0.49, 0); ok {
		t.Error("r=0.49 inside the annular hole (inner radius 0.5) must be unreachable")
	}
	if _, ok := armN.Analytic2(0.5, 0); !ok {
		t.Error("r=0.5 at inner boundary should be reachable")
	}
}
