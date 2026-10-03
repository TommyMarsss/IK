package ik

import (
	"math"
	"testing"
)

// TestContinuityNoBranchFlip: along a dense continuous arc (including points
// arbitrarily close to the full-extension singular pose where both branches
// coincide), the tracked branch label must stay constant and consecutive
// joint solutions must be close together.
func TestContinuityNoBranchFlip(t *testing.T) {
	arm := NewArm2(1, 1,
		Limit{Min: -math.Pi, Max: math.Pi},
		Limit{Min: -math.Pi, Max: math.Pi},
	)
	solver := NewSolver(arm, nil)
	solver.Reset([]float64{0.3, -0.4})

	// Targets approach full extension r=2 at theta=0.5, touch it and leave.
	// Sampling is dense so that genuine within-branch motion between frames
	// stays well below the order-pi jump a branch flip would produce.
	var targets []Pose
	rs := []float64{1.4, 1.5, 1.6, 1.7, 1.8, 1.9, 1.95, 1.99, 1.999, 1.9999}
	for _, r := range rs {
		targets = append(targets, Pose{X: r * math.Cos(0.5), Y: r * math.Sin(0.5)})
	}
	targets = append(targets, Pose{X: 2 * math.Cos(0.5), Y: 2 * math.Sin(0.5)})
	for i := len(rs) - 1; i >= 0; i-- {
		r := rs[i]
		targets = append(targets, Pose{X: r * math.Cos(0.5), Y: r * math.Sin(0.5)})
	}

	branches := []string{}
	var prevQ []float64
	for i, p := range targets {
		sol := solver.Solve(p)
		if sol.Status != StatusOK && sol.Status != StatusSingular {
			t.Fatalf("frame %d: unexpected status %s (%s)", i, sol.Status, sol.Message)
		}
		branches = append(branches, sol.Branch)
		if prevQ != nil {
			if jump := angularCost(sol.Q, prevQ); jump > 0.6 {
				t.Errorf("frame %d: joint jump %.3f rad indicates a configuration flip (%v -> %v)",
					i, jump, prevQ, sol.Q)
			}
		}
		prevQ = append(prevQ[:0:0], sol.Q...)
	}
	for i := 1; i < len(branches); i++ {
		if branches[i] != branches[0] {
			t.Errorf("branch flipped at frame %d: %s -> %s (all=%v)", i, branches[i-1], branches[i], branches)
		}
	}
}

// TestHysteresisKeepsBranch: near full extension the two branches are close.
// When the current configuration sits exactly on the OTHER branch, plain
// nearest-angle selection switches, but the tracked branch is retained as
// long as its cost lies inside the hysteresis band.
func TestHysteresisKeepsBranch(t *testing.T) {
	arm := NewArm2(1, 1,
		Limit{Min: -math.Pi, Max: math.Pi},
		Limit{Min: -math.Pi, Max: math.Pi},
	)
	solver := NewSolver(arm, nil)
	solver.Reset([]float64{0.3, -0.4})

	// r = 1.9999: the two branches nearly coincide (inter-branch angular
	// gap ~= 0.06 rad total), well inside the default hysteresis band.
	p := Pose{X: 1.9999 * math.Cos(0.5), Y: 1.9999 * math.Sin(0.5)}
	sol := solver.Solve(p)
	tracked := sol.Branch

	cands, _ := arm.Analytic2(p.X, p.Y)
	var other AnalyticSolution
	for _, c := range cands {
		if c.Branch != tracked {
			other = c
		}
	}

	// Current pose coincides with the other branch: it is strictly nearest.
	chosen := selectCandidate(cands, other.Q, tracked, 0.5)
	if chosen.Branch != tracked {
		t.Fatalf("hysteresis failed: lost tracked branch %s in favour of %s", tracked, chosen.Branch)
	}
	chosen = selectCandidate(cands, other.Q, tracked, 0)
	if chosen.Branch != other.Branch {
		t.Fatalf("zero hysteresis must allow switching to the strictly nearest branch")
	}
}
