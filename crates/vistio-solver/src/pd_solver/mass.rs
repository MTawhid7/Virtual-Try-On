//! Lumped mass computation for the PD solver.

use crate::element::ElementData;

/// Compute an area-weighted lumped mass matrix (stored as a vector).
/// Distributes 1/3 of each triangle's mass to its three vertices.
/// Pinned vertices receive an extremely large mass (1e8) so they are
/// mathematically anchored in the implicit system matrix.
pub(crate) fn compute_lumped_masses(n: usize, elements: &ElementData, density_gsm: f32, pinned: &[bool]) -> Vec<f32> {
    let mut mass = vec![0.0; n];
    let density_kgm2 = density_gsm / 1000.0;

    for elem in &elements.elements {
        let tri_mass = elem.rest_area * density_kgm2;
        let third_mass = tri_mass / 3.0;

        for &idx in &elem.indices {
            mass[idx] += third_mass;
        }
    }

    // Ensure no zero masses for floating vertices, and infinite mass for pinned
    for (i, m) in mass.iter_mut().enumerate() {
        if pinned[i] {
            *m = 1e8; // Infinite mass for the implicit solver
        } else if *m < 1e-8 {
            *m = 1e-8;
        }
    }

    mass
}
