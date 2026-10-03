package ik

import "math"

// Matrix is a small dense matrix in row-major layout. The IK package only
// deals with 2x2, 2x3 and 3x3 matrices, but a general tiny implementation is
// provided so the package stays free of third-party linear algebra code.
type Matrix struct {
	R, C int
	m    []float64
}

// NewMatrix allocates an R x C zero matrix.
func NewMatrix(r, c int) *Matrix {
	return &Matrix{R: r, C: c, m: make([]float64, r*c)}
}

// At returns element (i, j).
func (m *Matrix) At(i, j int) float64 { return m.m[i*m.C+j] }

// Set assigns element (i, j).
func (m *Matrix) Set(i, j int, v float64) { m.m[i*m.C+j] = v }

// Clone returns a deep copy.
func (m *Matrix) Clone() *Matrix {
	c := NewMatrix(m.R, m.C)
	copy(c.m, m.m)
	return c
}

// Transpose returns m^T.
func (m *Matrix) Transpose() *Matrix {
	t := NewMatrix(m.C, m.R)
	for i := 0; i < m.R; i++ {
		for j := 0; j < m.C; j++ {
			t.Set(j, i, m.At(i, j))
		}
	}
	return t
}

// Mul returns m*b.
func (m *Matrix) Mul(b *Matrix) *Matrix {
	if m.C != b.R {
		panic("ik: matrix dimension mismatch in Mul")
	}
	out := NewMatrix(m.R, b.C)
	for i := 0; i < m.R; i++ {
		for k := 0; k < m.C; k++ {
			mik := m.At(i, k)
			for j := 0; j < b.C; j++ {
				out.m[i*out.C+j] += mik * b.At(k, j)
			}
		}
	}
	return out
}

// MulVec returns m*v for a column vector v.
func (m *Matrix) MulVec(v []float64) []float64 {
	if m.C != len(v) {
		panic("ik: matrix/vector dimension mismatch in MulVec")
	}
	out := make([]float64, m.R)
	for i := 0; i < m.R; i++ {
		s := 0.0
		for j := 0; j < m.C; j++ {
			s += m.At(i, j) * v[j]
		}
		out[i] = s
	}
	return out
}

// SingularValues returns the singular values of m in descending order,
// computed via the symmetric eigendecomposition of m^T m (Jacobi rotations).
// The matrices in this package are at most 3x3, so the O(n^3) Jacobi sweep is
// more than adequate and avoids any external dependency.
func (m *Matrix) SingularValues() []float64 {
	ata := m.Transpose().Mul(m)
	vals := ata.symmetricEigenvalues()
	for i := range vals {
		if vals[i] < 0 { // tiny negatives from roundoff
			vals[i] = 0
		}
		vals[i] = math.Sqrt(vals[i])
	}
	// descending order
	for i := 0; i < len(vals); i++ {
		for j := i + 1; j < len(vals); j++ {
			if vals[j] > vals[i] {
				vals[i], vals[j] = vals[j], vals[i]
			}
		}
	}
	return vals
}

// symmetricEigenvalues returns all eigenvalues of a symmetric matrix using
// cyclic Jacobi rotations. Only the eigenvalues (not eigenvectors) are needed.
func (m *Matrix) symmetricEigenvalues() []float64 {
	n := m.R
	a := m.Clone()
	for sweep := 0; sweep < 50; sweep++ {
		off := 0.0
		for p := 0; p < n; p++ {
			for q := p + 1; q < n; q++ {
				off += math.Abs(a.At(p, q))
			}
		}
		if off < 1e-14 {
			break
		}
		for p := 0; p < n; p++ {
			for q := p + 1; q < n; q++ {
				apq := a.At(p, q)
				if math.Abs(apq) < 1e-15 {
					continue
				}
				app := a.At(p, p)
				aqq := a.At(q, q)
				tau := (aqq - app) / (2 * apq)
				t := 0.0
				if tau >= 0 {
					t = 1 / (tau + math.Sqrt(1+tau*tau))
				} else {
					t = 1 / (tau - math.Sqrt(1+tau*tau))
				}
				c := 1 / math.Sqrt(1+t*t)
				s := t * c
				// A' = J^T A J with J a Givens rotation in the (p,q) plane.
				for k := 0; k < n; k++ {
					akp := a.At(k, p)
					akq := a.At(k, q)
					a.Set(k, p, c*akp-s*akq)
					a.Set(k, q, s*akp+c*akq)
				}
				for k := 0; k < n; k++ {
					apk := a.At(p, k)
					aqk := a.At(q, k)
					a.Set(p, k, c*apk-s*aqk)
					a.Set(q, k, s*apk+c*aqk)
				}
			}
		}
	}
	vals := make([]float64, n)
	for i := 0; i < n; i++ {
		vals[i] = a.At(i, i)
	}
	return vals
}

// solveSquare solves A x = b for a SQUARE matrix via Gaussian elimination with
// partial pivoting. It returns ok == false when the matrix is numerically
// singular (pivot below tol). Used only by the plain (non-damped) Newton
// fallback and by tests; production IK uses SolveDLS.
func solveSquare(A *Matrix, b []float64, tol float64) (x []float64, ok bool) {
	n := A.R
	if A.C != n || len(b) != n {
		panic("ik: solveSquare expects a square system")
	}
	aug := make([][]float64, n)
	for i := range aug {
		aug[i] = make([]float64, n+1)
		for j := 0; j < n; j++ {
			aug[i][j] = A.At(i, j)
		}
		aug[i][n] = b[i]
	}
	for col := 0; col < n; col++ {
		piv := col
		for r := col + 1; r < n; r++ {
			if math.Abs(aug[r][col]) > math.Abs(aug[piv][col]) {
				piv = r
			}
		}
		if math.Abs(aug[piv][col]) < tol {
			return nil, false
		}
		aug[col], aug[piv] = aug[piv], aug[col]
		for r := 0; r < n; r++ {
			if r == col {
				continue
			}
			f := aug[r][col] / aug[col][col]
			for k := col; k <= n; k++ {
				aug[r][k] -= f * aug[col][k]
			}
		}
	}
	x = make([]float64, n)
	for i := 0; i < n; i++ {
		x[i] = aug[i][n] / aug[i][i]
	}
	return x, true
}
