//! Sparse Cholesky solver backed by `faer`.
//!
//! Implements the [`SparseSolver`] trait using faer's supernodal LLᵀ
//! factorization. The solver operates in f64 internally for numerical
//! robustness but accepts and returns f32 at the interface boundary.
//!
//! ## Workflow
//! 1. `factorize(matrix)` — converts CSR→CSC, computes symbolic + numeric LLᵀ
//! 2. `solve(rhs, solution)` — forward/backward substitution (cached factorization)
//! 3. Repeat `solve()` with different RHS without re-factorizing

use faer::Side;
use faer::linalg::solvers::Solve;
use faer::sparse::SparseColMat;
use faer::sparse::Triplet;
use faer::sparse::linalg::solvers::{Llt, SymbolicLlt};

use crate::sparse::{CsrMatrix, SparseSolver};

/// Sparse Cholesky (LLᵀ) solver using `faer`.
///
/// Stores the factorization for reuse across multiple solves.
/// The system matrix in Projective Dynamics is constant (depends only on
/// topology and mass), so one factorization serves the entire simulation.
pub struct FaerSolver {
    /// Cached LLᵀ factorization.
    factorization: Option<Llt<usize, f64>>,
    /// Matrix dimension (N×N).
    dimension: usize,
}

impl FaerSolver {
    /// Creates a new solver (unfactorized).
    pub fn new() -> Self {
        Self {
            factorization: None,
            dimension: 0,
        }
    }

    /// Convert our CSR matrix (f32) to faer's CSC matrix (f64).
    ///
    /// Builds from faer `Triplet`s, which faer assembles into CSC format.
    fn csr_to_csc_f64(matrix: &CsrMatrix) -> Result<SparseColMat<usize, f64>, String> {
        let mut triplets: Vec<Triplet<usize, usize, f64>> =
            Vec::with_capacity(matrix.values.len());
        for row in 0..matrix.rows {
            for idx in matrix.row_ptr[row]..matrix.row_ptr[row + 1] {
                let col = matrix.col_idx[idx];
                let val = matrix.values[idx] as f64;
                triplets.push(Triplet { row, col, val });
            }
        }

        SparseColMat::try_new_from_triplets(matrix.rows, matrix.cols, &triplets)
            .map_err(|e| format!("Failed to construct faer CSC matrix: {e:?}"))
    }
}

impl Default for FaerSolver {
    fn default() -> Self {
        Self::new()
    }
}

impl SparseSolver for FaerSolver {
    fn factorize(&mut self, matrix: &CsrMatrix) -> Result<(), String> {
        if matrix.rows != matrix.cols {
            return Err(format!(
                "Matrix must be square, got {}×{}",
                matrix.rows, matrix.cols
            ));
        }
        if matrix.rows == 0 {
            return Err("Cannot factorize empty matrix".into());
        }

        self.dimension = matrix.rows;

        // Convert CSR → faer CSC
        let csc = Self::csr_to_csc_f64(matrix)?;

        // Step 1: Symbolic analysis (ordering, fill-in prediction)
        let symbolic = SymbolicLlt::try_new(csc.symbolic().as_ref(), Side::Upper)
            .map_err(|e| format!("Symbolic analysis failed: {e:?}"))?;

        // Step 2: Numeric factorization (using the symbolic structure)
        let llt = Llt::try_new_with_symbolic(symbolic, csc.as_ref(), Side::Upper)
            .map_err(|e| format!("Cholesky factorization failed: {e:?}"))?;

        self.factorization = Some(llt);
        Ok(())
    }

    fn solve(&self, rhs: &[f32], solution: &mut [f32]) -> Result<(), String> {
        let llt = self
            .factorization
            .as_ref()
            .ok_or_else(|| "Solver not factorized. Call factorize() first.".to_string())?;

        if rhs.len() != self.dimension {
            return Err(format!(
                "RHS length ({}) != matrix dimension ({})",
                rhs.len(),
                self.dimension
            ));
        }
        if solution.len() != self.dimension {
            return Err(format!(
                "Solution length ({}) != matrix dimension ({})",
                solution.len(),
                self.dimension
            ));
        }

        // Convert RHS f32 → f64 dense column vector
        let rhs_f64: faer::Mat<f64> =
            faer::Mat::from_fn(self.dimension, 1, |i, _| rhs[i] as f64);

        // Solve using cached factorization: L L^T x = b
        let sol = llt.solve(&rhs_f64);

        // Copy result f64 → f32
        for i in 0..self.dimension {
            solution[i] = sol[(i, 0)] as f32;
        }

        Ok(())
    }

    fn is_factorized(&self) -> bool {
        self.factorization.is_some()
    }
}
