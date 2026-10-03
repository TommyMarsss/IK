// Command demo solves a scripted trajectory that exercises every branch of
// the IK solver (continuous tracking, full-extension singularity,
// limit-unreachable folded poses, geometrically unreachable targets) and
// writes a single self-contained report.html.
//
// Usage:
//
//	go run ./cmd/demo            # writes ./report.html
//	go run ./cmd/demo out.html   # custom output path
package main

import (
	"fmt"
	"math"
	"os"

	"github.com/TommyMarsss/IK/internal/report"
	"github.com/TommyMarsss/IK/pkg/ik"
)

func main() {
	out := "report.html"
	if len(os.Args) > 1 {
		out = os.Args[1]
	}

	// 2-link arm, l1 = l2 = 1 m. The elbow is clamped to +/-2.9 rad: deep
	// folds (bend > 2.9, i.e. targets well inside r ~= 0.24 m) are geometrically
	// reachable but violate the limit, while full extension remains feasible.
	arm := ik.NewArm2(1.0, 1.0,
		ik.Limit{Min: -math.Pi, Max: math.Pi}, // shoulder q0
		ik.Limit{Min: -2.9, Max: 2.9},         // elbow  q1 (relative)
	)
	solver := ik.NewSolver(arm, nil)
	solver.Reset([]float64{0.3, -0.4})

	outer, inner := arm.Reach()

	// Targets are described by polar (r, theta); r < 0 means "keep previous
	// radius" is not used — every entry is explicit.
	type tgt struct {
		r, theta float64
		note     string
	}
	var ts []tgt
	addPolar := func(r, theta float64) {
		ts = append(ts, tgt{r: r, theta: theta})
	}

	// Segment A: smooth arc at r = 1.4, theta 0.15 -> 2.15 — branch
	// continuity is tracked across 22 dense samples.
	const a0, a1 = 0.15, 2.15
	for i := 0; i <= 22; i++ {
		t := a0 + (a1-a0)*float64(i)/22
		addPolar(1.4, t)
	}

	// Segment B: swing back toward theta = 0.6, then push radially out to
	// FULL EXTENSION (r = l1+l2 = 2.0, a singularity) and back out.
	for _, t := range []float64{1.95, 1.75, 1.5, 1.25, 1.0, 0.8, 0.6} {
		addPolar(1.4, t)
	}
	thetaB := 0.6
	for _, r := range []float64{1.5, 1.6, 1.7, 1.8, 1.85, 1.9, 1.95, 1.97, 1.99, 1.999, 2.0,
		1.999, 1.99, 1.97, 1.95, 1.9, 1.85, 1.8, 1.7, 1.6, 1.5, 1.4} {
		addPolar(r, thetaB)
	}

	// Segment C: dive toward FULL FOLD. r = 0.5/0.25 are feasible (deep
	// elbow bend); r = 0 folds completely — geometrically reachable but the
	// required +/-pi elbow angle violates the +/-2.9 rad limit, so these
	// frames must be reported as LIMIT-unreachable (arm holds position, red
	// ghost shows the forbidden candidate).
	addPolar(1.2, 0.5)
	addPolar(1.0, 0.5)
	addPolar(0.8, 0.5)
	addPolar(0.6, 0.5)
	addPolar(0.45, 0.5)
	addPolar(0.35, 0.5)
	addPolar(0.28, 0.5)
	addPolar(0.24, 0.5)
	addPolar(0.22, 0.5)
	addPolar(0.1, 0.5)
	addPolar(0.0, 0.5)
	addPolar(0.1, 0.5)

	// Segment D: GEOMETRICALLY unreachable targets (outside outer radius);
	// the arm parks. Recovery then unfolds and swings away GRADUALLY so the
	// post-failure motion stays continuous on screen.
	addPolar(2.3, 0.7)
	addPolar(2.15, -0.2)
	// Gradual recovery from the parked deep-fold pose; ends on the closing
	// arc (r=1.4, theta=0.1) so consecutive targets stay adjacent.
	for i := 1; i <= 10; i++ {
		addPolar(0.1+float64(i)*1.3/10, 0.5-float64(i)*0.4/10)
	}
	// Dense closing arc sweeping clockwise from theta = 0.1.
	for k := 1; k <= 14; k++ {
		t := 0.1 - float64(k)*0.1
		addPolar(1.4, t)
	}

	data := &report.Data{
		Title:      "平面 2 连杆机械臂逆运动学求解报告",
		NLinks:     arm.N(),
		Lengths:    append([]float64(nil), arm.L...),
		Limits:     limitsToPairs(arm.Limits),
		ReachInner: inner,
		ReachOuter: outer,
	}

	nSingular, nLimit, nUnreach := 0, 0, 0
	for i, t := range ts {
		x, y := t.r*math.Cos(t.theta), t.r*math.Sin(t.theta)
		sol := solver.Solve(ik.Pose{X: x, Y: y})

		// Commanded configuration: lastQ on success; on failure state is not
		// advanced, so LastQ() already returns the previous valid pose.
		q := solver.LastQ()
		pts, phi := arm.FK(q)
		ee := pts[len(pts)-1]

		f := report.Frame{
			Index:     i,
			Target:    [2]float64{x, y},
			Joints:    q,
			Points:    vecsToPairs(pts),
			ActualEE:  [2]float64{ee.X, ee.Y},
			ActualPhi: phi,
			Status:    sol.Status,
			Message:   sol.Message,
			Branch:    sol.Branch,
			MinSigma:  sol.MinSigma,
			PosError:  sol.PosError,
			OriError:  sol.OriError,
		}
		if sol.Status == ik.StatusLimitUnreachable && sol.DiagnosticQ != nil {
			dpts, _ := arm.FK(sol.DiagnosticQ)
			f.DiagPoints = vecsToPairs(dpts)
			f.Violations = sol.Violations
		}
		switch sol.Status {
		case ik.StatusSingular:
			nSingular++
		case ik.StatusLimitUnreachable:
			nLimit++
		case ik.StatusUnreachable:
			nUnreach++
		}
		data.Frames = append(data.Frames, f)
	}

	if err := report.Write(out, data); err != nil {
		fmt.Fprintln(os.Stderr, "report:", err)
		os.Exit(1)
	}
	fmt.Printf("trajectory: %d frames (%d singular, %d limit-unreachable, %d geometrically unreachable)\n",
		len(data.Frames), nSingular, nLimit, nUnreach)
	fmt.Println("report written to", out)
}

func limitsToPairs(ls []ik.Limit) [][2]float64 {
	out := make([][2]float64, len(ls))
	for i, l := range ls {
		out[i] = [2]float64{l.Min, l.Max}
	}
	return out
}

func vecsToPairs(pts []ik.Vec2) [][2]float64 {
	out := make([][2]float64, len(pts))
	for i, p := range pts {
		out[i] = [2]float64{p.X, p.Y}
	}
	return out
}
