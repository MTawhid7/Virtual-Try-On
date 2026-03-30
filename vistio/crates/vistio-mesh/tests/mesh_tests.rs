//! Integration tests for vistio-mesh.

use vistio_mesh::generators::{quad_grid, uv_sphere};
use vistio_mesh::normals::compute_vertex_normals;
use vistio_mesh::topology::Topology;
use vistio_mesh::TriangleMesh;
use vistio_types::MaterialId;

// ─── TriangleMesh Tests ───────────────────────────────────────

fn make_single_triangle() -> TriangleMesh {
    TriangleMesh {
        pos_x: vec![0.0, 1.0, 0.0],
        pos_y: vec![0.0, 0.0, 1.0],
        pos_z: vec![0.0, 0.0, 0.0],
        normal_x: vec![0.0, 0.0, 0.0],
        normal_y: vec![0.0, 0.0, 0.0],
        normal_z: vec![1.0, 1.0, 1.0],
        uv_u: vec![0.0, 1.0, 0.0],
        uv_v: vec![0.0, 0.0, 1.0],
        indices: vec![0, 1, 2],
        material_ids: vec![MaterialId(0)],
    }
}

#[test]
fn basic_counts() {
    let mesh = make_single_triangle();
    assert_eq!(mesh.vertex_count(), 3);
    assert_eq!(mesh.triangle_count(), 1);
}

#[test]
fn position_access() {
    let mesh = make_single_triangle();
    assert_eq!(mesh.position(1), [1.0, 0.0, 0.0]);
}

#[test]
fn triangle_access() {
    let mesh = make_single_triangle();
    assert_eq!(mesh.triangle(0), [0, 1, 2]);
}

#[test]
fn validate_ok() {
    let mesh = make_single_triangle();
    assert!(mesh.validate().is_ok());
}

#[test]
fn validate_catches_inconsistent_lengths() {
    let mut mesh = make_single_triangle();
    mesh.pos_y.push(99.0);
    assert!(mesh.validate().is_err());
}

#[test]
fn validate_catches_oob_index() {
    let mut mesh = make_single_triangle();
    mesh.indices[2] = 99;
    assert!(mesh.validate().is_err());
}

#[test]
fn validate_catches_degenerate() {
    let mut mesh = make_single_triangle();
    mesh.indices = vec![0, 0, 1];
    assert!(mesh.validate().is_err());
}

#[test]
fn from_interleaved() {
    let positions = vec![0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0];
    let indices = vec![0, 1, 2];
    let uvs = vec![0.0, 0.0, 1.0, 0.0, 0.0, 1.0];
    let mesh = TriangleMesh::from_interleaved(&positions, &indices, &uvs).unwrap();
    assert_eq!(mesh.vertex_count(), 3);
    assert_eq!(mesh.pos_x, vec![0.0, 1.0, 0.0]);
    assert_eq!(mesh.uv_u, vec![0.0, 1.0, 0.0]);
}

// ─── Generator Tests ──────────────────────────────────────────

#[test]
fn quad_grid_2x2() {
    let mesh = quad_grid(2, 2, 1.0, 1.0);
    assert_eq!(mesh.vertex_count(), 9);
    assert_eq!(mesh.triangle_count(), 8);
    assert!(mesh.validate().is_ok());
}

#[test]
fn quad_grid_1x1() {
    let mesh = quad_grid(1, 1, 1.0, 1.0);
    assert_eq!(mesh.vertex_count(), 4);
    assert_eq!(mesh.triangle_count(), 2);
    assert!(mesh.validate().is_ok());
}

#[test]
fn quad_grid_20x20() {
    let mesh = quad_grid(20, 20, 2.0, 2.0);
    assert_eq!(mesh.vertex_count(), 441);
    assert_eq!(mesh.triangle_count(), 800);
    assert!(mesh.validate().is_ok());
}

#[test]
fn quad_grid_dimensions() {
    let mesh = quad_grid(4, 4, 2.0, 2.0);
    assert!((mesh.pos_x[0] - (-1.0)).abs() < 1e-6);
    assert!((mesh.pos_y[0] - 1.0).abs() < 1e-6);
    assert!((mesh.pos_x[4] - 1.0).abs() < 1e-6);
}

#[test]
fn quad_grid_uvs() {
    let mesh = quad_grid(2, 2, 1.0, 1.0);
    let last = mesh.vertex_count() - 1;
    assert!((mesh.uv_u[last] - 1.0).abs() < 1e-6);
    assert!((mesh.uv_v[last] - 1.0).abs() < 1e-6);
}

#[test]
fn uv_sphere_basic() {
    let mesh = uv_sphere(1.0, 8, 16);
    assert!(mesh.vertex_count() > 0);
    assert!(mesh.triangle_count() > 0);
    assert!(mesh.validate().is_ok());
}

#[test]
fn uv_sphere_radius() {
    let mesh = uv_sphere(2.5, 8, 16);
    for i in 0..mesh.vertex_count() {
        let x = mesh.pos_x[i];
        let y = mesh.pos_y[i];
        let z = mesh.pos_z[i];
        let dist = (x * x + y * y + z * z).sqrt();
        assert!((dist - 2.5).abs() < 1e-4, "Vertex {} at distance {}", i, dist);
    }
}

// ─── Topology Tests ───────────────────────────────────────────

#[test]
fn topology_single_quad() {
    let mesh = quad_grid(1, 1, 1.0, 1.0);
    let topo = Topology::build(&mesh);
    assert_eq!(topo.edges.len(), 5);
    assert_eq!(topo.interior_edges.len(), 1);
    assert_eq!(topo.boundary_edge_count(), 4);
    assert!(!topo.is_closed());
}

#[test]
fn topology_2x2_grid() {
    let mesh = quad_grid(2, 2, 1.0, 1.0);
    let topo = Topology::build(&mesh);
    assert!(!topo.edges.is_empty());
    assert!(!topo.interior_edges.is_empty());
    assert!(!topo.is_closed());
}

#[test]
fn vertex_triangle_adjacency() {
    let mesh = quad_grid(2, 2, 1.0, 1.0);
    let topo = Topology::build(&mesh);
    let center = 4;
    // In checkerboard triangulation, center vertex touches 4 triangles
    assert_eq!(topo.vertex_triangles[center].len(), 4);
}

#[test]
fn one_ring() {
    let mesh = quad_grid(2, 2, 1.0, 1.0);
    let topo = Topology::build(&mesh);
    let center = 4u32;
    let ring = topo.one_ring(center, &mesh);
    // In checkerboard triangulation, center vertex has 4 neighbors
    assert_eq!(ring.len(), 4);
}

#[test]
fn interior_edges_have_wings() {
    let mesh = quad_grid(2, 2, 1.0, 1.0);
    let topo = Topology::build(&mesh);
    for ie in &topo.interior_edges {
        assert_ne!(ie.wing_a, ie.v0);
        assert_ne!(ie.wing_a, ie.v1);
        assert_ne!(ie.wing_b, ie.v0);
        assert_ne!(ie.wing_b, ie.v1);
        assert_ne!(ie.wing_a, ie.wing_b);
    }
}

#[test]
fn sphere_has_boundary_at_seam() {
    let mesh = uv_sphere(1.0, 8, 16);
    let topo = Topology::build(&mesh);
    assert!(topo.boundary_edge_count() > 0, "UV sphere should have seam boundary");
}

// ─── Normal Tests ─────────────────────────────────────────────

#[test]
fn flat_grid_normals() {
    let mut mesh = quad_grid(4, 4, 1.0, 1.0);
    compute_vertex_normals(&mut mesh);
    for i in 0..mesh.vertex_count() {
        assert!(mesh.normal_x[i].abs() < 1e-5);
        assert!(mesh.normal_y[i].abs() < 1e-5);
        assert!(mesh.normal_z[i].abs() > 0.99);
    }
}

#[test]
fn normals_are_unit_length() {
    let mut mesh = quad_grid(10, 10, 2.0, 2.0);
    compute_vertex_normals(&mut mesh);
    for i in 0..mesh.vertex_count() {
        let x = mesh.normal_x[i];
        let y = mesh.normal_y[i];
        let z = mesh.normal_z[i];
        let len = (x * x + y * y + z * z).sqrt();
        assert!((len - 1.0).abs() < 1e-5, "Normal at {} has length {}", i, len);
    }
}

#[test]
fn sphere_normals_point_outward() {
    let mut mesh = uv_sphere(1.0, 8, 16);
    compute_vertex_normals(&mut mesh);
    let mut checked = 0;
    for i in 0..mesh.vertex_count() {
        let px = mesh.pos_x[i];
        let py = mesh.pos_y[i];
        let pz = mesh.pos_z[i];
        let pos_len = (px * px + py * py + pz * pz).sqrt();
        let xz_extent = (px * px + pz * pz).sqrt();
        if pos_len < 0.1 || xz_extent < 0.15 {
            continue;
        }
        let nx = mesh.normal_x[i];
        let ny = mesh.normal_y[i];
        let nz = mesh.normal_z[i];
        let dot = (px * nx + py * ny + pz * nz) / pos_len;
        assert!(dot.abs() > 0.5, "Vertex {} normal misaligned: dot={}", i, dot);
        checked += 1;
    }
    assert!(checked > 0);
}
