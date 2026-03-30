//! Integration tests for vistio-math.

use vistio_math::mat3x2::Mat3x2;
use vistio_math::decomposition::{deformation_gradient, polar_decomposition_3x2};
use vistio_math::sparse::CsrMatrix;
use vistio_math::Vec3;

// ─── Mat3x2 Tests ─────────────────────────────────────────────

#[test]
fn identity_ftf() {
    let f = Mat3x2::IDENTITY;
    let c = f.ftf();
    assert!((c[0] - 1.0).abs() < 1e-6);
    assert!((c[1]).abs() < 1e-6);
    assert!((c[2]).abs() < 1e-6);
    assert!((c[3] - 1.0).abs() < 1e-6);
}

#[test]
fn frobenius_norm_identity() {
    let f = Mat3x2::IDENTITY;
    assert!((f.frobenius_norm_sq() - 2.0).abs() < 1e-6);
}

#[test]
fn zero_matrix() {
    let f = Mat3x2::ZERO;
    assert_eq!(f.frobenius_norm_sq(), 0.0);
}

#[test]
fn mul_mat2_identity() {
    let f = Mat3x2::from_cols(Vec3::new(1.0, 2.0, 3.0), Vec3::new(4.0, 5.0, 6.0));
    let result = f.mul_mat2([1.0, 0.0, 0.0, 1.0]);
    assert_eq!(result, f);
}

// ─── Polar Decomposition Tests ────────────────────────────────

#[test]
fn polar_identity() {
    let f = Mat3x2::IDENTITY;
    let pd = polar_decomposition_3x2(&f);
    let r = pd.rotation;
    assert!((r.col0.x - 1.0).abs() < 1e-4);
    assert!((r.col0.y).abs() < 1e-4);
    assert!((r.col1.y - 1.0).abs() < 1e-4);
    assert!((pd.stretch[0] - 1.0).abs() < 1e-4);
    assert!((pd.stretch[3] - 1.0).abs() < 1e-4);
}

#[test]
fn polar_scaled() {
    let f = Mat3x2::from_cols(Vec3::new(2.0, 0.0, 0.0), Vec3::new(0.0, 2.0, 0.0));
    let pd = polar_decomposition_3x2(&f);
    assert!((pd.stretch[0] - 2.0).abs() < 1e-4);
    assert!((pd.stretch[3] - 2.0).abs() < 1e-4);
}

#[test]
fn polar_degenerate_does_not_panic() {
    let f = Mat3x2::ZERO;
    let pd = polar_decomposition_3x2(&f);
    assert!(!pd.rotation.col0.x.is_nan());
    assert!(!pd.stretch[0].is_nan());
}

#[test]
fn deformation_gradient_identity() {
    let p0 = Vec3::ZERO;
    let p1 = Vec3::X;
    let p2 = Vec3::Y;
    let dm_inv = [1.0, 0.0, 0.0, 1.0];
    let f = deformation_gradient(p0, p1, p2, dm_inv);
    assert!((f.col0.x - 1.0).abs() < 1e-6);
    assert!((f.col1.y - 1.0).abs() < 1e-6);
}

// ─── Sparse Matrix Tests ─────────────────────────────────────

#[test]
fn empty_csr() {
    let m = CsrMatrix::new(3, 3);
    assert_eq!(m.nnz(), 0);
    assert_eq!(m.rows, 3);
    assert_eq!(m.cols, 3);
    assert_eq!(m.row_ptr.len(), 4);
}

#[test]
fn csr_from_triplets() {
    let triplets = vec![(0, 0, 1.0), (1, 1, 1.0), (2, 2, 1.0)];
    let m = CsrMatrix::from_triplets(3, 3, &triplets);
    assert_eq!(m.nnz(), 3);
    assert_eq!(m.row_ptr, vec![0, 1, 2, 3]);
    assert_eq!(m.col_idx, vec![0, 1, 2]);
    assert_eq!(m.values, vec![1.0, 1.0, 1.0]);
}

#[test]
fn csr_from_triplets_unordered() {
    let triplets = vec![(0, 2, 3.0), (0, 0, 1.0), (0, 1, 2.0)];
    let m = CsrMatrix::from_triplets(1, 3, &triplets);
    assert_eq!(m.col_idx, vec![0, 1, 2]);
    assert_eq!(m.values, vec![1.0, 2.0, 3.0]);
}

// ─── FaerSolver Tests ────────────────────────────────────────

use vistio_math::faer_solver::FaerSolver;
use vistio_math::sparse::SparseSolver;

#[test]
fn faer_identity_solve() {
    // Solve I * x = b → expect x = b
    let triplets = vec![(0, 0, 1.0), (1, 1, 1.0), (2, 2, 1.0)];
    let matrix = CsrMatrix::from_triplets(3, 3, &triplets);

    let mut solver = FaerSolver::new();
    assert!(!solver.is_factorized());

    solver.factorize(&matrix).unwrap();
    assert!(solver.is_factorized());

    let rhs = [3.0_f32, 7.0, -2.0];
    let mut sol = [0.0_f32; 3];
    solver.solve(&rhs, &mut sol).unwrap();

    for i in 0..3 {
        assert!(
            (sol[i] - rhs[i]).abs() < 1e-5,
            "sol[{i}] = {}, expected {}",
            sol[i],
            rhs[i]
        );
    }
}

#[test]
fn faer_spd_matrix_solve() {
    // Solve a known 3×3 SPD system:
    //   [4 1 0]       [1]
    //   [1 3 1] * x = [2]
    //   [0 1 2]       [3]
    //
    // Solution (via manual calculation):
    //   x ≈ [0.0, 1.0, 1.0]  (verify by substitution: 4*0+1*1+0*1=1, 1*0+3*1+1*1=4... not exact)
    // Let's use a simpler approach: verify A*x_computed ≈ b
    let triplets = vec![
        (0, 0, 4.0),
        (0, 1, 1.0),
        (1, 0, 1.0),
        (1, 1, 3.0),
        (1, 2, 1.0),
        (2, 1, 1.0),
        (2, 2, 2.0),
    ];
    let matrix = CsrMatrix::from_triplets(3, 3, &triplets);

    let mut solver = FaerSolver::new();
    solver.factorize(&matrix).unwrap();

    let rhs = [1.0_f32, 2.0, 3.0];
    let mut sol = [0.0_f32; 3];
    solver.solve(&rhs, &mut sol).unwrap();

    // Verify: A * sol ≈ rhs
    let residual = [
        4.0 * sol[0] + 1.0 * sol[1] + 0.0 * sol[2] - rhs[0],
        1.0 * sol[0] + 3.0 * sol[1] + 1.0 * sol[2] - rhs[1],
        0.0 * sol[0] + 1.0 * sol[1] + 2.0 * sol[2] - rhs[2],
    ];
    for (i, &r) in residual.iter().enumerate() {
        assert!(
            r.abs() < 1e-4,
            "Residual[{i}] = {r}, expected ~0"
        );
    }
}

#[test]
fn faer_factorize_then_multi_solve() {
    // Factorize once, solve with two different RHS
    let triplets = vec![(0, 0, 2.0), (1, 1, 3.0), (2, 2, 5.0)];
    let matrix = CsrMatrix::from_triplets(3, 3, &triplets);

    let mut solver = FaerSolver::new();
    solver.factorize(&matrix).unwrap();

    // First solve
    let rhs1 = [4.0_f32, 9.0, 25.0];
    let mut sol1 = [0.0_f32; 3];
    solver.solve(&rhs1, &mut sol1).unwrap();
    assert!((sol1[0] - 2.0).abs() < 1e-5);
    assert!((sol1[1] - 3.0).abs() < 1e-5);
    assert!((sol1[2] - 5.0).abs() < 1e-5);

    // Second solve with different RHS (same factorization)
    let rhs2 = [1.0_f32, 1.0, 1.0];
    let mut sol2 = [0.0_f32; 3];
    solver.solve(&rhs2, &mut sol2).unwrap();
    assert!((sol2[0] - 0.5).abs() < 1e-5);
    assert!((sol2[1] - 1.0 / 3.0).abs() < 1e-5);
    assert!((sol2[2] - 0.2).abs() < 1e-5);
}

#[test]
fn faer_large_laplacian() {
    // Build a 100×100 graph Laplacian (tridiagonal: 2 on diagonal, -1 on off-diagonals)
    // plus a small diagonal shift for strict positive-definiteness.
    let n = 100;
    let mut triplets = Vec::new();

    for i in 0..n {
        triplets.push((i, i, 2.1_f32)); // Diagonal (2 + 0.1 shift for SPD)
        if i > 0 {
            triplets.push((i, i - 1, -1.0));
        }
        if i < n - 1 {
            triplets.push((i, i + 1, -1.0));
        }
    }

    let matrix = CsrMatrix::from_triplets(n, n, &triplets);
    let mut solver = FaerSolver::new();
    solver.factorize(&matrix).unwrap();

    // RHS: all ones
    let rhs = vec![1.0_f32; n];
    let mut sol = vec![0.0_f32; n];
    solver.solve(&rhs, &mut sol).unwrap();

    // Verify residual: ||A*x - b|| < tolerance
    let mut max_residual: f32 = 0.0;
    for i in 0..n {
        let mut ax_i = 2.1 * sol[i];
        if i > 0 {
            ax_i -= sol[i - 1];
        }
        if i < n - 1 {
            ax_i -= sol[i + 1];
        }
        max_residual = max_residual.max((ax_i - rhs[i]).abs());
    }
    assert!(
        max_residual < 1e-3,
        "Max residual = {max_residual}, expected < 1e-3"
    );
}

#[test]
fn faer_solve_before_factorize_fails() {
    let solver = FaerSolver::new();
    let rhs = [1.0_f32; 3];
    let mut sol = [0.0_f32; 3];
    assert!(solver.solve(&rhs, &mut sol).is_err());
}

#[test]
fn faer_non_square_fails() {
    let triplets = vec![(0, 0, 1.0)];
    let matrix = CsrMatrix::from_triplets(2, 3, &triplets);
    let mut solver = FaerSolver::new();
    assert!(solver.factorize(&matrix).is_err());
}

#[test]
fn faer_empty_matrix_fails() {
    let matrix = CsrMatrix::new(0, 0);
    let mut solver = FaerSolver::new();
    assert!(solver.factorize(&matrix).is_err());
}
