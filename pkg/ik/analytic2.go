package ik

import "math"

// Branch labels the two geometric configurations of a 2-link planar arm.
const (
	BranchElbowUp   = "elbow_up"
	BranchElbowDown = "elbow_down"
)

// Analytic2 solves the 2-link position inverse kinematics geometrically and
// returns BOTH configurations (elbow up / elbow down). The returned angles
// follow the package convention: q[0] absolute base angle, q[1] relative
// elbow angle. Solutions that do not satisfy the joint limits are still
// returned; callers inspect WithinLimits / check limits themselves so that
// "reachable but violates limits" can be distinguished from "unreachable".
//
// ok is false when the target is outside the annular position workspace
// (geometrically unreachable by any configuration, limits aside).
func (a *Arm) Analytic2(x, y float64) (sols []AnalyticSolution, ok bool) {
	if a.N() != 2 {
		panic("ik: Analytic2 requires a 2-link arm")
	}
	l1, l2 := a.L[0], a.L[1]
	r2 := x*x + y*y
	r := math.Sqrt(r2)
	outer, inner := a.Reach()

	if r > outer+geometricTol {
		return nil, false
	}
	if r < inner-geometricTol {
		return nil, false
	}
	// Clamp cos(alpha) into [-1, 1] against roundoff exactly on the boundary.
	cosAlpha := (r2 + l1*l1 - l2*l2) / (2 * l1 * r)
	cosAlpha = math.Max(-1, math.Min(1, cosAlpha))
	alpha := math.Acos(cosAlpha) // angle of the elbow triangle at the base
	theta := math.Atan2(y, x)    // polar angle of the target

	if r < geometricTol {
		// Target coincides with the base: only possible when l1 == l2
		// (fully folded). Infinitely many configurations; return the two
		// canonical folds around base angle 0.
		sols = append(sols, a.makeAnalytic2(0, math.Pi, BranchElbowUp))
		sols = append(sols, a.makeAnalytic2(0, -math.Pi, BranchElbowDown))
		return sols, true
	}

	// gamma is the elbow-triangle interior angle at the elbow; the RELATIVE
	// joint angle q1 is pi-gamma (0 when fully stretched, +/-pi when fully
	// folded). Law of cosines, computed via acos((r²-l1²-l2²)/2 l1 l2).
	cosBend := (r2 - l1*l1 - l2*l2) / (2 * l1 * l2)
	cosBend = math.Max(-1, math.Min(1, cosBend))
	bend := math.Acos(cosBend)

	// With y up and CCW positive: q0 = theta+alpha, q1 = -bend places the
	// elbow above the base->target chord (elbow up); the mirrored solution
	// places it below (elbow down).
	qUp := []float64{wrapPi(theta + alpha), -bend}
	qDown := []float64{wrapPi(theta - alpha), bend}
	sols = append(sols, a.makeAnalytic2(qUp[0], qUp[1], BranchElbowUp))
	sols = append(sols, a.makeAnalytic2(qDown[0], qDown[1], BranchElbowDown))
	return sols, true
}

// AnalyticSolution is one geometric IK solution.
type AnalyticSolution struct {
	Q            []float64
	Branch       string
	WithinLimits bool
	// LimitViolations lists the joint indices whose limit is broken.
	LimitViolations []int
}

func (a *Arm) makeAnalytic2(q0, q1 float64, branch string) AnalyticSolution {
	q := []float64{wrapPi(q0), wrapPi(q1)}
	s := AnalyticSolution{Q: q, Branch: branch, WithinLimits: true}
	for i := range q {
		if !a.Limits[i].Contains(q[i]) {
			s.WithinLimits = false
			s.LimitViolations = append(s.LimitViolations, i)
		}
	}
	return s
}
