//! Sparse matrix representation and solver interface.
//!
//! Provides a CSR (Compressed Sparse Row) matrix and a trait
//! for sparse Cholesky solvers. Implementations will come in
//! later tiers (faer, CHOLMOD FFI).

use serde::{Deserialize, Serialize};

/// Compressed Sparse Row (CSR) matrix.
///
/// Stores a sparse matrix in row-major order. This is the standard
/// format for sparse linear algebra libraries (faer, SuiteSparse).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CsrMatrix {
    /// Number of rows.
    pub rows: usize,
    /// Number of columns.
    pub cols: usize,
    /// Row pointer array (length = rows + 1).
    /// `row_ptr[i]..row_ptr[i+1]` are the indices into `col_idx` and `values`
    /// for non-zeros in row `i`.
    pub row_ptr: Vec<usize>,
    /// Column indices of non-zero entries.
    pub col_idx: Vec<usize>,
    /// Non-zero values.
    pub values: Vec<f32>,
}

impl CsrMatrix {
    /// Creates an empty CSR matrix with the given dimensions.
    pub fn new(rows: usize, cols: usize) -> Self {
        Self {
            rows,
            cols,
            row_ptr: vec![0; rows + 1],
            col_idx: Vec::new(),
            values: Vec::new(),
        }
    }

    /// Returns the number of non-zero entries.
    pub fn nnz(&self) -> usize {
        self.values.len()
    }

    /// Creates a CSR matrix from triplets (row, col, value).
    ///
    /// Duplicate entries are summed.
    pub fn from_triplets(
        rows: usize,
        cols: usize,
        triplets: &[(usize, usize, f32)],
    ) -> Self {
        // Count entries per row
        let mut row_counts = vec![0usize; rows];
        for &(r, _, _) in triplets {
            row_counts[r] += 1;
        }

        // Build row_ptr
        let mut row_ptr = vec![0usize; rows + 1];
        for i in 0..rows {
            row_ptr[i + 1] = row_ptr[i] + row_counts[i];
        }

        let nnz = row_ptr[rows];
        let mut col_idx = vec![0usize; nnz];
        let mut values = vec![0.0f32; nnz];

        // Fill in — use row_counts as write cursor
        let mut cursor = row_ptr[..rows].to_vec();
        for &(r, c, v) in triplets {
            let pos = cursor[r];
            col_idx[pos] = c;
            values[pos] = v;
            cursor[r] += 1;
        }

        // Sort each row by column index
        for i in 0..rows {
            let start = row_ptr[i];
            let end = row_ptr[i + 1];
            let slice = &mut col_idx[start..end];
            let val_slice = &mut values[start..end];

            // Simple insertion sort (rows are typically small)
            for j in 1..slice.len() {
                let mut k = j;
                while k > 0 && slice[k - 1] > slice[k] {
                    slice.swap(k - 1, k);
                    val_slice.swap(k - 1, k);
                    k -= 1;
                }
            }
        }

        Self {
            rows,
            cols,
            row_ptr,
            col_idx,
            values,
        }
    }
}

/// Trait for sparse symmetric positive-definite solvers.
///
/// Implementations: `FaerSolver` (coming in Tier 1), `CholmodSolver` (optional FFI).
pub trait SparseSolver {
    /// Factorize the matrix. Call once (or after topology change).
    fn factorize(&mut self, matrix: &CsrMatrix) -> Result<(), String>;

    /// Solve Ax = b using the pre-computed factorization.
    /// Returns x in the provided output buffer.
    fn solve(&self, rhs: &[f32], solution: &mut [f32]) -> Result<(), String>;

    /// Returns true if the solver holds a valid factorization.
    fn is_factorized(&self) -> bool;
}
