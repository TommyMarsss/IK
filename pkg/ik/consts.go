package ik

// Numerical tolerances and default solver settings.
const (
	// geometricTol guards the workspace-boundary tests against roundoff.
	geometricTol = 1e-9
	// angularTol is the slack used when testing joint-limit membership.
	angularTol = 1e-6

	// DefaultSingularThreshold: a configuration is declared singular when the
	// smallest singular value of the Jacobian drops below this absolute value
	// (link lengths are order 1, so the Jacobian entries are order 1 too).
	DefaultSingularThreshold = 1e-2
	DefaultMaxIter           = 200
	DefaultPosTol            = 1e-7
	DefaultOriTol            = 1e-7
	// DefaultDamping is the maximum DLS damping factor lambda; it kicks in
	// smoothly only inside the singular region (see solverStep).
	DefaultDamping = 5e-2
	// DefaultHysteresis: branch-switching margin in radians. The tracked
	// branch is kept while its angular cost to the current configuration is
	// within this margin of the best branch, preventing flips at the
	// fully-folded/extended poses where both branches coincide.
	DefaultHysteresis = 0.2
)
