package ik

import "math"

// Analytic3 solves the full-pose inverse kinematics of a 3-link planar arm
// for target position (x, y) and end-effector orientation phi.
//
// The last link points from the wrist W to the end effector, so
//
//	W = (x - l3*cos(phi), y - l3*sin(phi))
//
// and the first two links are solved for W with the standard 2-link geometry
// (two elbow branches); q2 = phi - q0 - q1 closes the orientation. Both
// branches are returned with their limit status, exactly as in Analytic2.
func (a *Arm) Analytic3(x, y, phi float64) (sols []AnalyticSolution, ok bool) {
	if a.N() != 3 {
		panic("ik: Analytic3 requires a 3-link arm")
	}
	wx := x - a.L[2]*math.Cos(phi)
	wy := y - a.L[2]*math.Sin(phi)

	// Reuse the 2-link geometry for the (l1, l2) sub-arm; limits are shared.
	sub := &Arm{L: a.L[:2], Limits: a.Limits[:2]}
	subSols, reachable := sub.Analytic2(wx, wy)
	if !reachable {
		return nil, false
	}
	for _, s := range subSols {
		q := []float64{s.Q[0], s.Q[1], wrapPi(phi - s.Q[0] - s.Q[1])}
		sol := AnalyticSolution{Q: q, Branch: s.Branch, WithinLimits: true}
		for i := range q {
			if !a.Limits[i].Contains(q[i]) {
				sol.WithinLimits = false
				sol.LimitViolations = append(sol.LimitViolations, i)
			}
		}
		sols = append(sols, sol)
	}
	return sols, true
}
