package ik

import (
	"math"
	"testing"
)

// TestSingularityAtFullExtension: at exact full extension the 2R position
// Jacobian is rank-1 (rank drop from 2). The solver must (a) flag the
// singular status, (b) return the exact geometric answer, and (c) produce a
// FINITE, bounded resolved-rate joint velocity for an infeasible Cartesian
// command instead of blowing up or dividing by zero.
func TestSingularityAtFullExtension(t *testing.T) {
	arm := NewArm2(1, 1,
		Limit{Min: -math.Pi, Max: math.Pi},
		Limit{Min: -math.Pi, Max: math.Pi},
	)
	qExt := []float64{0.5, 0.0} // arm fully stretched along angle 0.5
	J := arm.Jacobian(qExt, false)
	sig := J.SingularValues()
	if sig[1] > 1e-12 {
		t.Fatalf("full extension not singular: singular values = %v", sig)
	}
	if sig[0] <= 1.99 {
		t.Fatalf("largest singular value unexpected at full extension: %v", sig)
	}

	solver := NewSolver(arm, nil)
	solver.Reset(qExt)
	sol := solver.Solve(Pose{X: 2 * math.Cos(0.5), Y: 2 * math.Sin(0.5)})
	if sol.Status != StatusSingular {
		t.Fatalf("status = %s, want %s", sol.Status, StatusSingular)
	}
	if math.Abs(sol.PosError) > 1e-9 {
		t.Errorf("singular frame position error = %.2e", sol.PosError)
	}
	for i, q := range sol.Q {
		if math.IsNaN(q) || math.IsInf(q, 0) {
			t.Errorf("joint %d non-finite: %v", i, q)
		}
	}

	// At full extension the Jacobian's null space is the RADIAL direction
	// (stretching any further is impossible) while the TRANSVERSE direction
	// is feasible (rigid rotation of the whole arm). DLS must return finite
	// output for both, attenuate the infeasible radial command toward zero,
	// and still transmit the feasible transverse one.
	radial := []float64{math.Cos(0.5), math.Sin(0.5)}
	trans := []float64{-math.Sin(0.5), math.Cos(0.5)}
	vr := solver.JointVelocity(qExt, radial)
	vt := solver.JointVelocity(qExt, trans)
	for _, x := range append(append([]float64{}, vr.DQ...), vt.DQ...) {
		if math.IsNaN(x) || math.IsInf(x, 0) {
			t.Fatalf("resolved-rate output non-finite near singularity: %v / %v", vr.DQ, vt.DQ)
		}
	}
	if !vr.Singular || !vt.Singular {
		t.Error("both velocity requests should be flagged singular")
	}
	if n := vecNorm(vr.DQ); n > 0.1 {
		t.Errorf("infeasible radial velocity not attenuated: ||dq|| = %.4f", n)
	}
	if n := vecNorm(vt.DQ); n < 0.1 {
		t.Errorf("feasible transverse motion over-damped: ||dq|| = %.4f", n)
	}
}

// TestNearSingularStability: sweep targets through the singular region and
// require bounded joint steps, finite values and accurate positioning at
// every sample, including ones microscopically inside r = l1+l2.
func TestNearSingularStability(t *testing.T) {
	arm := NewArm2(1, 1,
		Limit{Min: -math.Pi, Max: math.Pi},
		Limit{Min: -math.Pi, Max: math.Pi},
	)
	solver := NewSolver(arm, nil)
	solver.Reset([]float64{0.3, -0.4})

	for _, r := range []float64{1.9, 1.99, 1.999, 1.9999, 1.99999, 2.0, 1.99999, 1.9999, 1.99, 1.9} {
		p := Pose{X: r * math.Cos(0.5), Y: r * math.Sin(0.5)}
		sol := solver.Solve(p)
		if sol.Status != StatusOK && sol.Status != StatusSingular {
			t.Fatalf("r=%v: unexpected status %s", r, sol.Status)
		}
		for i, q := range sol.Q {
			if math.IsNaN(q) || math.IsInf(q, 0) {
				t.Fatalf("r=%v: joint %d non-finite", r, i)
			}
		}
		if sol.MaxStepNorm > 0.6 {
			t.Errorf("r=%v: max Newton step %.4f exceeds clamp", r, sol.MaxStepNorm)
		}
		if sol.PosError > 1e-6 {
			t.Errorf("r=%v: position residual %.2e", r, sol.PosError)
		}
		// Near/at the singular threshold the DLS lambda must be engaged.
		if r > 1.99 {
			J := arm.Jacobian(sol.Q, false)
			sMin := J.SingularValues()[1]
			if sMin > DefaultSingularThreshold && sol.Status == StatusSingular {
				t.Errorf("r=%v: marked singular with sigma=%.4f above threshold", r, sMin)
			}
		}
	}
}

// TestFullAlignSingular3: a 3-link arm with all links aligned (fully
// extended) is singular for the pose Jacobian as well; pose IK must remain
// finite and flag it.
func TestFullAlignSingular3(t *testing.T) {
	arm := NewArm3(1, 1, 0.5,
		Limit{Min: -math.Pi, Max: math.Pi},
		Limit{Min: -math.Pi, Max: math.Pi},
		Limit{Min: -math.Pi, Max: math.Pi},
	)
	// q = [phi, 0, 0]: links 2 and 3 both point along link 1 -> folded/aligned.
	q := []float64{0.7, 0.0, 0.0}
	pts, phi := arm.FK(q)
	ee := pts[3]
	solver := NewSolver(arm, nil)
	solver.Reset(q)
	sol := solver.Solve(Pose{X: ee.X, Y: ee.Y, Phi: phi, HasOrient: true})
	for i, qv := range sol.Q {
		if math.IsNaN(qv) || math.IsInf(qv, 0) {
			t.Fatalf("joint %d non-finite at fold: %v", i, qv)
		}
	}
	// The analytic seed reaches the pose to acos/atan2 roundoff, which is
	// inside the solver's 1e-7 convergence tolerance; accept that floor.
	if sol.PosError > 1e-6 || sol.OriError > 1e-6 {
		t.Errorf("fold solution inaccurate: pos %.2e ori %.2e", sol.PosError, sol.OriError)
	}
	J := arm.Jacobian(sol.Q, true)
	sMin := J.SingularValues()[2]
	if sMin > 1e-9 {
		t.Errorf("aligned 3-link should be singular, got sigma_min=%.2e", sMin)
	}
}
