package ik

import (
	"math"
	"testing"
)

// TestLimitVsGeometricUnreachable distinguishes the two failure modes:
//   - a target INSIDE the annulus whose every geometric configuration needs
//     an elbow angle beyond the limit => StatusLimitUnreachable;
//   - a target OUTSIDE the annulus => StatusUnreachable.
func TestLimitVsGeometricUnreachable(t *testing.T) {
	arm := NewArm2(1, 1,
		Limit{Min: -math.Pi, Max: math.Pi}, // shoulder free
		Limit{Min: -2.9, Max: 2.9},         // elbow cannot fold past ~166°
	)
	solver := NewSolver(arm, nil)
	solver.Reset([]float64{0.0, -2.0})

	// Full fold at the base (r = 0) needs |q1| = pi > 2.9: geometrically
	// reachable, forbidden by the elbow limit.
	solFold := solver.Solve(Pose{X: 0, Y: 0})
	if solFold.Status != StatusLimitUnreachable {
		t.Fatalf("fold target: status = %s, want %s", solFold.Status, StatusLimitUnreachable)
	}
	if len(solFold.Violations) == 0 {
		t.Error("limit failure must report which joints violate the limit")
	}
	hasElbow := false
	for _, j := range solFold.Violations {
		if j == 1 {
			hasElbow = true
		}
	}
	if !hasElbow {
		t.Errorf("expected elbow (joint 1) among violations, got %v", solFold.Violations)
	}
	if solFold.DiagnosticQ == nil {
		t.Error("limit failure must carry a diagnostic configuration")
	}
	// The diagnostic configuration must actually hit the geometric target...
	dpts, _ := arm.FK(solFold.DiagnosticQ)
	if d := math.Hypot(dpts[2].X, dpts[2].Y); d > 1e-8 {
		t.Errorf("diagnostic config does not reach the fold target (r=%.2e)", d)
	}
	// ...and really violate the reported limit.
	for _, j := range solFold.Violations {
		if arm.Limits[j].Contains(solFold.DiagnosticQ[j]) {
			t.Errorf("joint %d reported violating but %.4f is inside %+v", j, solFold.DiagnosticQ[j], arm.Limits[j])
		}
	}
	// State must not have advanced: LastQ is still the reset pose.
	if q := solver.LastQ(); angularCost(q, []float64{0, -2.0}) > 0 {
		t.Errorf("state advanced on failure: %v", q)
	}

	// Outside outer radius: geometrically unreachable, no candidates at all.
	solOut := solver.Solve(Pose{X: 2.5, Y: 0})
	if solOut.Status != StatusUnreachable {
		t.Fatalf("outside target: status = %s, want %s", solOut.Status, StatusUnreachable)
	}
	if len(solOut.AllCandidates) != 0 {
		t.Errorf("geometric unreachable must carry no candidates, got %d", len(solOut.AllCandidates))
	}

	// A normal reachable target right after the failures must still solve
	// (state continuity preserved across the failed frames).
	solOK := solver.Solve(Pose{X: 1.4, Y: 0.3})
	if solOK.Status != StatusOK {
		t.Fatalf("recovery target: status = %s (%s)", solOK.Status, solOK.Message)
	}
}

// TestOneBranchBlockedByLimit: one elbow branch satisfies the limits while
// the other does not; the solver must select the feasible branch instead of
// declaring failure.
func TestOneBranchBlockedByLimit(t *testing.T) {
	arm := NewArm2(1, 1,
		Limit{Min: -math.Pi, Max: math.Pi},
		Limit{Min: -3.0, Max: 0.6}, // positive (fold-up) elbow deeply blocked
	)
	solver := NewSolver(arm, nil)
	solver.Reset([]float64{0.2, 0.0})

	// Chosen target where the elbow_down branch needs a positive bend.
	p := Pose{X: 1.2, Y: 0.8}
	cands, ok := arm.Analytic2(p.X, p.Y)
	if !ok {
		t.Fatal("target should be geometrically reachable")
	}
	nFeasible := 0
	for _, c := range cands {
		if c.WithinLimits {
			nFeasible++
		}
	}
	if nFeasible != 1 {
		t.Fatalf("test setup: want exactly 1 feasible branch, got %d", nFeasible)
	}
	sol := solver.Solve(p)
	if sol.Status != StatusOK {
		t.Fatalf("status = %s, want ok (%s)", sol.Status, sol.Message)
	}
	if !arm.Limits[1].Contains(sol.Q[1]) {
		t.Errorf("returned solution violates elbow limit: q1=%.4f", sol.Q[1])
	}
}

// TestAnalytic3PoseBothBranches: full-pose 3-link IK returns two branches,
// both reaching the requested position AND orientation.
func TestAnalytic3PoseBothBranches(t *testing.T) {
	arm := NewArm3(1, 0.9, 0.6,
		Limit{Min: -math.Pi, Max: math.Pi},
		Limit{Min: -math.Pi, Max: math.Pi},
		Limit{Min: -math.Pi, Max: math.Pi},
	)
	x, y, phi := 1.6, 0.7, 0.4
	sols, ok := arm.Analytic3(x, y, phi)
	if !ok {
		t.Fatal("pose target unreachable")
	}
	if len(sols) != 2 {
		t.Fatalf("got %d pose solutions, want 2", len(sols))
	}
	for _, s := range sols {
		pts, gotPhi := arm.FK(s.Q)
		ee := pts[3]
		approxEq(t, "ee.x", ee.X, x, 1e-8)
		approxEq(t, "ee.y", ee.Y, y, 1e-8)
		approxEq(t, "phi", gotPhi, phi, 1e-8)
	}
}

// TestAnalytic3SolverRoundTrip: pose targets produced by FK must be solved
// back to the same end-effector pose.
func TestAnalytic3SolverRoundTrip(t *testing.T) {
	arm := NewArm3(1, 0.9, 0.6,
		Limit{Min: -math.Pi, Max: math.Pi},
		Limit{Min: -math.Pi, Max: math.Pi},
		Limit{Min: -math.Pi, Max: math.Pi},
	)
	solver := NewSolver(arm, nil)
	solver.Reset([]float64{0.4, -0.5, 0.3})
	pts, phi := arm.FK(solver.LastQ())
	ee := pts[3]
	sol := solver.Solve(Pose{X: ee.X, Y: ee.Y, Phi: phi, HasOrient: true})
	if sol.Status != StatusOK && sol.Status != StatusSingular {
		t.Fatalf("status = %s (%s)", sol.Status, sol.Message)
	}
	p2, phi2 := arm.FK(sol.Q)
	approxEq(t, "ee.x", p2[3].X, ee.X, 1e-6)
	approxEq(t, "ee.y", p2[3].Y, ee.Y, 1e-6)
	approxEq(t, "phi", phi2, phi, 1e-6)
}
