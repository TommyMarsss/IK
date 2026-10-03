// Package ik — numerical solver.
//
// Solve strategy (per target, stateful across a trajectory):
//
//  1. Enumerate EVERY geometric solution analytically (two elbow branches;
//     for the 3-link position-only case the tool orientation is inherited
//     from the current configuration, which also gives two branches).
//  2. Workspace membership is decided by the geometry alone: if no geometric
//     solution exists the target is UNREACHABLE (outside the workspace).
//  3. Each geometric solution is checked against the joint limits. When
//     solutions exist but none satisfies the limits the result is
//     LIMIT_UNREACHABLE — explicitly distinct from UNREACHABLE.
//  4. Among the limit-feasible solutions the unique choice is the one nearest
//     the current joint angles (minimum sum of wrapped angular distances),
//     with a branch hysteresis margin: the previously tracked branch is kept
//     whenever its cost is within Hysteresis of the best cost. This removes
//     flips at the fully folded/extended poses where both branches coincide.
//  5. The chosen seed is refined with damped-least-squares (DLS) Newton
//     iteration. DLS solves  dq = Jᵀ(JJᵀ + λ²I)⁻¹ e ; the damped matrix is
//     always invertible, so near a singularity the joint step degrades
//     smoothly (shrinking in the infeasible Cartesian direction) instead of
//     exploding or dividing by zero. λ ramps up inside the singular region
//     defined by SingularThreshold on the smallest Jacobian singular value.
package ik

import (
	"fmt"
	"math"
)

// Solve statuses.
const (
	// StatusOK: target reached with a limit-feasible, non-singular solution.
	StatusOK = "ok"
	// StatusSingular: target reached, but the configuration lies in the
	// singular region; the DLS solution is deliberately degraded.
	StatusSingular = "singular"
	// StatusLimitUnreachable: geometric solutions exist but all violate joint
	// limits. "In workspace, unreachable under current limits."
	StatusLimitUnreachable = "limit_unreachable"
	// StatusUnreachable: no geometric solution exists (outside workspace).
	StatusUnreachable = "unreachable"
	// StatusNoConvergence: geometry is feasible but the numerical refinement
	// did not converge in MaxIter iterations.
	StatusNoConvergence = "no_convergence"
)

// Options configures the Solver. Zero fields fall back to package defaults.
type Options struct {
	MaxIter           int
	PosTol            float64
	OriTol            float64
	SingularThreshold float64
	Damping           float64
	Hysteresis        float64
	MaxStep           float64 // per-iteration joint-step clamp, radians
}

func (o *Options) withDefaults() Options {
	d := Options{
		MaxIter:           DefaultMaxIter,
		PosTol:            DefaultPosTol,
		OriTol:            DefaultOriTol,
		SingularThreshold: DefaultSingularThreshold,
		Damping:           DefaultDamping,
		Hysteresis:        DefaultHysteresis,
		MaxStep:           0.5,
	}
	if o == nil {
		return d
	}
	if o.MaxIter > 0 {
		d.MaxIter = o.MaxIter
	}
	if o.PosTol > 0 {
		d.PosTol = o.PosTol
	}
	if o.OriTol > 0 {
		d.OriTol = o.OriTol
	}
	if o.SingularThreshold > 0 {
		d.SingularThreshold = o.SingularThreshold
	}
	if o.Damping > 0 {
		d.Damping = o.Damping
	}
	if o.Hysteresis > 0 {
		d.Hysteresis = o.Hysteresis
	}
	if o.MaxStep > 0 {
		d.MaxStep = o.MaxStep
	}
	return d
}

// Solution is the outcome of one target solve.
type Solution struct {
	Q        []float64
	Branch   string
	Status   string
	Message  string
	MinSigma float64 // smallest singular value observed at the solution
	PosError float64
	OriError float64
	// MaxStepNorm is the largest joint-space step taken during refinement;
	// bounded by construction under DLS.
	MaxStepNorm float64
	Iterations  int
	// DiagnosticQ, when Status is limit_unreachable, holds the geometric
	// candidate nearest the current pose for visualisation ONLY; it violates
	// the limits and must not be commanded. Violations names its joints.
	DiagnosticQ   []float64
	Violations    []int
	AllCandidates []AnalyticSolution
}

// Solver is a stateful IK solver: state carries the selected branch across a
// trajectory so consecutive targets do not flip configurations.
type Solver struct {
	arm  *Arm
	opts Options

	hasState   bool
	lastQ      []float64
	lastBranch string
}

// NewSolver builds a solver for arm.
func NewSolver(arm *Arm, opts *Options) *Solver {
	return &Solver{arm: arm, opts: opts.withDefaults()}
}

// Reset fixes the initial configuration and clears branch history.
func (s *Solver) Reset(q []float64) {
	if len(q) != s.arm.N() {
		panic("ik: Reset called with wrong number of joint angles")
	}
	s.lastQ = append([]float64(nil), q...)
	s.lastBranch = ""
	s.hasState = true
}

// LastQ returns a copy of the current commanded configuration.
func (s *Solver) LastQ() []float64 {
	out := make([]float64, len(s.lastQ))
	copy(out, s.lastQ)
	return out
}

// candidates enumerates the geometric solutions for a target.
func (s *Solver) candidates(p Pose) ([]AnalyticSolution, bool) {
	switch {
	case s.arm.N() == 2:
		return s.arm.Analytic2(p.X, p.Y)
	case s.arm.N() == 3 && p.HasOrient:
		return s.arm.Analytic3(p.X, p.Y, p.Phi)
	case s.arm.N() == 3:
		// Redundant for position only: inherit the current absolute tool
		// orientation, reducing to a determined wrist 2-link problem.
		_, phi := s.arm.FK(s.lastQ)
		return s.arm.analytic3WithToolAngle(p.X, p.Y, phi)
	default:
		panic("ik: unsupported arm dimension")
	}
}

// analytic3WithToolAngle enumerates 3-link position solutions that keep the
// absolute end-effector orientation equal to phi (two elbow branches).
func (a *Arm) analytic3WithToolAngle(x, y, phi float64) ([]AnalyticSolution, bool) {
	wx := x - a.L[2]*math.Cos(phi)
	wy := y - a.L[2]*math.Sin(phi)
	sub := &Arm{L: a.L[:2], Limits: a.Limits[:2]}
	subSols, ok := sub.Analytic2(wx, wy)
	if !ok {
		return nil, false
	}
	sols := make([]AnalyticSolution, 0, len(subSols))
	for _, sub := range subSols {
		q := []float64{sub.Q[0], sub.Q[1], wrapPi(phi - sub.Q[0] - sub.Q[1])}
		sol := AnalyticSolution{Q: q, Branch: sub.Branch, WithinLimits: true}
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

// angularCost is the sum of wrapped absolute angular differences.
func angularCost(a, b []float64) float64 {
	c := 0.0
	for i := range a {
		c += math.Abs(angDist(a[i], b[i]))
	}
	return c
}

// selectCandidate applies the nearest-angle + branch-hysteresis policy.
func selectCandidate(cands []AnalyticSolution, curQ []float64, trackedBranch string, hysteresis float64) AnalyticSolution {
	bestIdx := 0
	bestCost := math.Inf(1)
	costs := make([]float64, len(cands))
	for i, c := range cands {
		costs[i] = angularCost(c.Q, curQ)
		if costs[i] < bestCost {
			bestCost = costs[i]
			bestIdx = i
		}
	}
	if trackedBranch != "" {
		for i, c := range cands {
			if c.Branch == trackedBranch && costs[i] <= bestCost+hysteresis {
				return c
			}
		}
	}
	return cands[bestIdx]
}

// Solve inverts one target against the current state. On failure the internal
// state is NOT advanced (the last commanded configuration is retained), which
// keeps a trajectory continuous across isolated infeasible frames.
func (s *Solver) Solve(p Pose) Solution {
	if !s.hasState {
		panic("ik: Solve called before Reset")
	}
	cands, geometric := s.candidates(p)

	// Step 2: outside workspace outright.
	if !geometric {
		return Solution{
			Status:        StatusUnreachable,
			Message:       "target is outside the geometric workspace",
			AllCandidates: cands,
		}
	}

	// Step 3: split feasible / limit-violating candidates.
	var feasible []AnalyticSolution
	for _, c := range cands {
		if c.WithinLimits {
			feasible = append(feasible, c)
		}
	}
	if len(feasible) == 0 {
		// Pick the globally nearest geometric candidate as a diagnostic.
		diag := selectCandidate(cands, s.lastQ, "", s.opts.Hysteresis)
		vmap := map[int]bool{}
		for _, j := range diag.LimitViolations {
			vmap[j] = true
		}
		var viols []int
		for j := range vmap {
			viols = append(viols, j)
		}
		return Solution{
			Status:        StatusLimitUnreachable,
			Message:       fmt.Sprintf("target is within the workspace but every configuration violates joint limits (joints %v); relaxing limits would be required", viols),
			DiagnosticQ:   append([]float64(nil), diag.Q...),
			Violations:    viols,
			Branch:        diag.Branch,
			AllCandidates: cands,
		}
	}

	// Step 4: unique selection with continuity policy.
	chosen := selectCandidate(feasible, s.lastQ, s.lastBranch, s.opts.Hysteresis)

	// Step 5: DLS refinement.
	ref := s.refine(chosen.Q, p)
	ref.Branch = chosen.Branch
	ref.AllCandidates = cands

	// A reachable geometric seed has ~zero residual; classify singularity
	// from the Jacobian at the final configuration regardless of step count.
	pts, phi := s.arm.FK(ref.Q)
	ee := pts[len(pts)-1]
	ref.PosError = math.Hypot(ee.X-p.X, ee.Y-p.Y)
	if p.HasOrient {
		ref.OriError = math.Abs(angDist(phi, p.Phi))
	}

	// Refinement must not push a feasible solution through a limit; if it
	// ever does (aggressive MaxStep), reject the refinement.
	for i, qv := range ref.Q {
		if !s.arm.Limits[i].Contains(qv) {
			ref.Q = append([]float64(nil), chosen.Q...)
			ref.Iterations = 0
			break
		}
	}

	if ref.Status == StatusOK && ref.MinSigma < s.opts.SingularThreshold {
		ref.Status = StatusSingular
		ref.Message = fmt.Sprintf("configuration is near a singularity (smallest singular value %.4f < %.4f); DLS damping engaged, joint velocities bounded", ref.MinSigma, s.opts.SingularThreshold)
	} else if ref.Status == StatusOK {
		ref.Message = "solved"
	}

	// Advance state only on success.
	s.lastQ = append([]float64(nil), ref.Q...)
	s.lastBranch = chosen.Branch
	return ref
}

// refine runs DLS Newton iteration from seed.
func (s *Solver) refine(seed []float64, p Pose) Solution {
	q := append([]float64(nil), seed...)
	n := s.arm.N()
	orient := p.HasOrient && n == 3

	sol := Solution{Q: q, Status: StatusOK}
	for it := 0; it < s.opts.MaxIter; it++ {
		pts, phi := s.arm.FK(q)
		ee := pts[len(pts)-1]
		e := []float64{p.X - ee.X, p.Y - ee.Y}
		if orient {
			e = append(e, angDist(p.Phi, phi))
		}
		posErr := math.Hypot(e[0], e[1])
		oriErr := 0.0
		if orient {
			oriErr = math.Abs(e[2])
		}

		// Evaluate conditioning before the convergence check: an analytic
		// seed is already exact, so singularity classification must not
		// depend on taking at least one Newton step.
		J := s.arm.Jacobian(q, orient)
		sig := J.SingularValues()
		minSig := sig[len(sig)-1]
		if it == 0 || minSig < sol.MinSigma {
			sol.MinSigma = minSig
		}

		if posErr <= s.opts.PosTol && (!orient || oriErr <= s.opts.OriTol) {
			sol.Iterations = it
			sol.PosError = posErr
			sol.OriError = oriErr
			break
		}

		dq, stepNorm := dlsStep(J, e, s.dampingFor(minSig), s.opts.MaxStep)
		if stepNorm > sol.MaxStepNorm {
			sol.MaxStepNorm = stepNorm
		}
		for i := 0; i < n; i++ {
			q[i] = wrapPi(q[i] + dq[i])
		}

		if it == s.opts.MaxIter-1 {
			// Ran out of iterations while still above tolerance.
			pts2, phi2 := s.arm.FK(q)
			ee2 := pts2[len(pts2)-1]
			sol.Status = StatusNoConvergence
			sol.Message = fmt.Sprintf("DLS refinement did not converge in %d iterations (position residual %.3e)", s.opts.MaxIter, math.Hypot(p.X-ee2.X, p.Y-ee2.Y))
			sol.PosError = math.Hypot(p.X-ee2.X, p.Y-ee2.Y)
			if orient {
				sol.OriError = math.Abs(angDist(p.Phi, phi2))
			}
		}
	}
	sol.Q = q
	return sol
}

// dampingFor ramps lambda smoothly from 0 at the singular threshold to the
// configured maximum at exact singularity.
func (s *Solver) dampingFor(minSig float64) float64 {
	if minSig >= s.opts.SingularThreshold {
		return 0
	}
	r := 1 - minSig/s.opts.SingularThreshold // 0 at boundary -> 1 at singularity
	return s.opts.Damping * r
}

// dlsStep computes dq = Jᵀ(JJᵀ + λ²I)⁻¹ e. The inverted matrix is square
// (2x2 or 3x3) and, thanks to λ² I, SPD/non-singular even when J is rank
// deficient — there is never a division by a vanishing Jacobian quantity.
// The step is additionally clamped to maxStep.
func dlsStep(J *Matrix, e []float64, lambda, maxStep float64) (dq []float64, norm float64) {
	Jt := J.Transpose()
	A := J.Mul(Jt)
	for i := 0; i < A.R; i++ {
		A.Set(i, i, A.At(i, i)+lambda*lambda)
	}
	// Partial pivot elimination; with lambda > 0 the matrix is guaranteed
	// invertible. lambda == 0 with a healthy Jacobian is invertible too, and
	// 1e-12 is a defensive floor only.
	y, ok := solveSquare(A, e, 1e-12)
	if !ok {
		// Defensive: should be unreachable for a damped SPD system. Return a
		// zero step rather than NaN garbage; the caller will stall safely.
		return make([]float64, J.C), 0
	}
	dq = Jt.MulVec(y)
	norm = vecNorm(dq)
	if maxStep > 0 && norm > maxStep {
		scale := maxStep / norm
		for i := range dq {
			dq[i] *= scale
		}
		norm = maxStep
	}
	return dq, norm
}

func vecNorm(v []float64) float64 {
	s := 0.0
	for _, x := range v {
		s += x * x
	}
	return math.Sqrt(s)
}

// VelocityResult is the outcome of a resolved-rate request.
type VelocityResult struct {
	DQ       []float64
	MinSigma float64
	Singular bool
	Lambda   float64
}

// JointVelocity resolves a Cartesian velocity command v (length 2, or 3 with
// orientation rate) into joint velocities with the same DLS singularity
// handling. At an exact singularity the infeasible velocity component is
// projected away and DQ stays finite and bounded instead of diverging.
func (s *Solver) JointVelocity(q, v []float64) VelocityResult {
	orient := len(v) == 3
	J := s.arm.Jacobian(q, orient)
	sig := J.SingularValues()
	minSig := sig[len(sig)-1]
	lambda := s.dampingFor(minSig)
	dq, norm := dlsStep(J, v, lambda, 0) // velocity requests are not clamped
	_ = norm
	return VelocityResult{
		DQ:       dq,
		MinSigma: minSig,
		Singular: minSig < s.opts.SingularThreshold,
		Lambda:   lambda,
	}
}
