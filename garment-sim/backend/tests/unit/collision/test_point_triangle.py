"""
Unit tests for point-triangle projection geometry.

Tests the @ti.func helpers in isolation by wrapping them in small @ti.kernel
test harnesses — the standard pattern for testing Taichi device functions.

Tests cover all 7 Voronoi regions, signed distance correctness, and degenerate
triangle handling.
"""

import numpy as np
import pytest
import taichi as ti

import simulation  # noqa: F401 — Taichi initialization
from simulation.collision.point_triangle import (
    closest_point_on_triangle,
    closest_point_and_bary,
    signed_distance_to_triangle,
)


# ---------------------------------------------------------------------------
# Test harness kernels — wrap @ti.func for host-callable access
# ---------------------------------------------------------------------------

_result_vec3 = ti.Vector.field(3, dtype=ti.f32, shape=1)
_result_vec4 = ti.Vector.field(4, dtype=ti.f32, shape=1)
_result_f32 = ti.field(dtype=ti.f32, shape=1)


@ti.kernel
def _test_closest_point(
    px: ti.f32, py: ti.f32, pz: ti.f32,
    v0x: ti.f32, v0y: ti.f32, v0z: ti.f32,
    v1x: ti.f32, v1y: ti.f32, v1z: ti.f32,
    v2x: ti.f32, v2y: ti.f32, v2z: ti.f32,
):
    p = ti.math.vec3(px, py, pz)
    v0 = ti.math.vec3(v0x, v0y, v0z)
    v1 = ti.math.vec3(v1x, v1y, v1z)
    v2 = ti.math.vec3(v2x, v2y, v2z)
    _result_vec3[0] = closest_point_on_triangle(p, v0, v1, v2)


@ti.kernel
def _test_closest_and_bary(
    px: ti.f32, py: ti.f32, pz: ti.f32,
    v0x: ti.f32, v0y: ti.f32, v0z: ti.f32,
    v1x: ti.f32, v1y: ti.f32, v1z: ti.f32,
    v2x: ti.f32, v2y: ti.f32, v2z: ti.f32,
):
    p = ti.math.vec3(px, py, pz)
    v0 = ti.math.vec3(v0x, v0y, v0z)
    v1 = ti.math.vec3(v1x, v1y, v1z)
    v2 = ti.math.vec3(v2x, v2y, v2z)
    _result_vec4[0] = closest_point_and_bary(p, v0, v1, v2)


@ti.kernel
def _test_signed_distance(
    px: ti.f32, py: ti.f32, pz: ti.f32,
    cx: ti.f32, cy: ti.f32, cz: ti.f32,
    n0x: ti.f32, n0y: ti.f32, n0z: ti.f32,
    n1x: ti.f32, n1y: ti.f32, n1z: ti.f32,
    n2x: ti.f32, n2y: ti.f32, n2z: ti.f32,
    bu: ti.f32, bv: ti.f32, bw: ti.f32,
):
    p = ti.math.vec3(px, py, pz)
    c = ti.math.vec3(cx, cy, cz)
    n0 = ti.math.vec3(n0x, n0y, n0z)
    n1 = ti.math.vec3(n1x, n1y, n1z)
    n2 = ti.math.vec3(n2x, n2y, n2z)
    bary = ti.math.vec3(bu, bv, bw)
    _result_f32[0] = signed_distance_to_triangle(p, c, n0, n1, n2, bary)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

# Unit triangle in XZ plane (Y=0), normals pointing +Y
_TRI_V0 = (0.0, 0.0, 0.0)
_TRI_V1 = (1.0, 0.0, 0.0)
_TRI_V2 = (0.0, 0.0, 1.0)
_NORM_UP = (0.0, 1.0, 0.0)


def _closest(p, v0=_TRI_V0, v1=_TRI_V1, v2=_TRI_V2):
    _test_closest_point(*p, *v0, *v1, *v2)
    return _result_vec3[0].to_numpy()


def _closest_bary(p, v0=_TRI_V0, v1=_TRI_V1, v2=_TRI_V2):
    """Returns (closest xyz, bary_u) — bary_u is in result[3]."""
    _test_closest_and_bary(*p, *v0, *v1, *v2)
    r = _result_vec4[0].to_numpy()
    return r[:3], r[3]


def _signed_dist(p, closest, n0=_NORM_UP, n1=_NORM_UP, n2=_NORM_UP, bary=(1/3, 1/3, 1/3)):
    _test_signed_distance(*p, *closest, *n0, *n1, *n2, *bary)
    return float(_result_f32[0])


# ---------------------------------------------------------------------------
# Tests: closest_point_on_triangle
# ---------------------------------------------------------------------------

class TestClosestPointOnTriangle:
    """Test all 7 Voronoi regions of point-triangle projection."""

    def test_interior_projection(self):
        """Point directly above triangle interior projects to plane."""
        p = (0.2, 1.0, 0.2)   # Above interior of the unit right triangle
        closest = _closest(p)
        # Should project straight down to (0.2, 0, 0.2)
        np.testing.assert_allclose(closest, [0.2, 0.0, 0.2], atol=1e-5)

    def test_vertex_v0_region(self):
        """Point nearest to v0 should snap to v0."""
        p = (-0.5, 0.0, -0.5)  # Below-left of v0=(0,0,0)
        closest = _closest(p)
        np.testing.assert_allclose(closest, _TRI_V0, atol=1e-5)

    def test_vertex_v1_region(self):
        """Point nearest to v1 should snap to v1."""
        p = (1.5, 0.0, -0.5)  # Past v1=(1,0,0) corner
        closest = _closest(p)
        np.testing.assert_allclose(closest, _TRI_V1, atol=1e-5)

    def test_vertex_v2_region(self):
        """Point nearest to v2 should snap to v2."""
        p = (-0.5, 0.0, 1.5)  # Past v2=(0,0,1) corner
        closest = _closest(p)
        np.testing.assert_allclose(closest, _TRI_V2, atol=1e-5)

    def test_edge_v0_v1_region(self):
        """Point nearest to edge v0-v1 should land on that edge."""
        # Point at (0.5, 0, -0.5) — below midpoint of edge v0-v1
        p = (0.5, 0.0, -0.5)
        closest = _closest(p)
        # Should clamp to (0.5, 0, 0) — midpoint of v0-v1
        np.testing.assert_allclose(closest, [0.5, 0.0, 0.0], atol=1e-5)
        # Y should be exactly on the triangle plane
        assert abs(closest[1]) < 1e-5

    def test_edge_v0_v2_region(self):
        """Point nearest to edge v0-v2 should land on that edge."""
        p = (-0.5, 0.0, 0.5)  # Left of edge v0-v2
        closest = _closest(p)
        np.testing.assert_allclose(closest, [0.0, 0.0, 0.5], atol=1e-5)

    def test_edge_v1_v2_region(self):
        """Point nearest to hypotenuse v1-v2 should land on that edge."""
        # Midpoint of hypotenuse is (0.5, 0, 0.5); point outside it
        p = (0.8, 0.0, 0.8)
        closest = _closest(p)
        # Should be on edge v1-v2 (parametrized as v1 + t*(v2-v1))
        # v1=(1,0,0), v2=(0,0,1), hypotenuse midpoint=(0.5,0,0.5)
        # Point (0.8,0,0.8) is closest to midpoint (0.5,0,0.5)
        np.testing.assert_allclose(closest, [0.5, 0.0, 0.5], atol=1e-5)

    def test_point_on_surface(self):
        """Point exactly on the triangle surface returns itself."""
        p = (0.3, 0.0, 0.3)  # On the triangle (u+v+w<1 in this triangle)
        closest = _closest(p)
        np.testing.assert_allclose(closest, [0.3, 0.0, 0.3], atol=1e-5)

    def test_point_below_triangle(self):
        """Point below the triangle (Y<0) should still project to interior."""
        p = (0.2, -1.0, 0.2)
        closest = _closest(p)
        np.testing.assert_allclose(closest, [0.2, 0.0, 0.2], atol=1e-5)


class TestClosestPointAndBary:
    """Test barycentric coordinate correctness from closest_point_and_bary."""

    def test_barycentric_sum_interior(self):
        """Barycentric coords for interior point should sum to 1."""
        # closest_point_and_bary returns (closest_xyz, bary_u)
        # bary_v and bary_w are embedded in the xyz encoding via the function
        # We test via the vec4 result: xyz=closest, w=bary_u
        p = (0.2, 0.5, 0.2)
        _, bary_u = _closest_bary(p)
        closest = _closest(p)
        # Reconstruct bary_v and bary_w from closest and triangle verts
        # closest = u*v0 + v*v1 + w*v2 with v0=(0,0,0), v1=(1,0,0), v2=(0,0,1)
        # closest.x = v, closest.z = w, bary_u = 1 - v - w
        bary_v = closest[0]
        bary_w = closest[2]
        bary_sum = bary_u + bary_v + bary_w
        assert bary_sum == pytest.approx(1.0, abs=1e-4), (
            f"Bary sum = {bary_sum:.6f}, expected 1.0"
        )

    def test_barycentric_sum_vertex(self):
        """At vertex v0, bary = (1, 0, 0) — sum = 1."""
        p = (-1.0, 0.0, -1.0)  # v0 region
        _, bary_u = _closest_bary(p)
        # At v0, bary_u=1 (the function stores u in result[3])
        assert bary_u == pytest.approx(1.0, abs=1e-4)

    def test_barycentric_edge_sum(self):
        """On edge v0-v1, bary_w = 0, and u+v = 1."""
        p = (0.5, 0.0, -1.0)  # Below edge v0-v1
        _, bary_u = _closest_bary(p)
        closest = _closest(p)
        bary_v = closest[0]   # For this triangle, v1.x=1, so v = closest.x
        bary_w = closest[2]
        # On edge v0-v1, bary_w should be ~0
        assert bary_w == pytest.approx(0.0, abs=1e-4)
        assert bary_u + bary_v == pytest.approx(1.0, abs=1e-4)

    def test_barycentric_random_points(self):
        """For many random points above the unit triangle, bary sum ≈ 1.0."""
        rng = np.random.default_rng(42)
        # Sample points above the triangle interior
        for _ in range(20):
            u = rng.uniform(0.0, 0.5)
            v = rng.uniform(0.0, 0.5 - u)
            w = 1.0 - u - v
            # Point above interior point
            interior = np.array([v, 0.0, w])  # closest should be this
            p = tuple(interior + np.array([0.0, rng.uniform(0.1, 1.0), 0.0]))
            _, bary_u = _closest_bary(p)
            closest = _closest(p)
            bary_v = closest[0]
            bary_w = closest[2]
            bary_sum = bary_u + bary_v + bary_w
            assert bary_sum == pytest.approx(1.0, abs=1e-3), (
                f"Point {p}: bary sum = {bary_sum:.6f}"
            )


# ---------------------------------------------------------------------------
# Tests: signed_distance_to_triangle
# ---------------------------------------------------------------------------

class TestSignedDistanceToTriangle:
    """Test signed distance correctness with smoothed normals."""

    def test_positive_outside_normal_side(self):
        """Point on the +Y side of the triangle should give positive distance."""
        p = (0.2, 0.5, 0.2)
        closest = (0.2, 0.0, 0.2)
        d = _signed_dist(p, closest)
        assert d > 0, f"Expected positive signed_dist, got {d:.4f}"
        assert d == pytest.approx(0.5, abs=1e-3)

    def test_negative_below_triangle(self):
        """Point on the -Y side should give negative distance."""
        p = (0.2, -0.5, 0.2)
        closest = (0.2, 0.0, 0.2)
        d = _signed_dist(p, closest)
        assert d < 0, f"Expected negative signed_dist, got {d:.4f}"
        assert d == pytest.approx(-0.5, abs=1e-3)

    def test_zero_on_surface(self):
        """Point exactly on the triangle surface should give zero distance."""
        p = (0.2, 0.0, 0.2)
        closest = (0.2, 0.0, 0.2)
        d = _signed_dist(p, closest)
        assert d == pytest.approx(0.0, abs=1e-5)

    def test_degenerate_zero_normal(self):
        """Zero-length interpolated normal (degenerate) should not crash."""
        p = (0.2, 0.5, 0.2)
        closest = (0.2, 0.0, 0.2)
        # Pass zero normals — should fall back to +Y
        zero_n = (0.0, 0.0, 0.0)
        d = _signed_dist(p, closest, n0=zero_n, n1=zero_n, n2=zero_n)
        # Should still return a reasonable value (fallback to +Y)
        assert not np.isnan(d), "Degenerate normal should not produce NaN"
        assert d > 0, "Fallback to +Y should give positive dist for p above triangle"

    def test_distance_magnitude_matches_euclidean(self):
        """Magnitude of signed distance should equal Euclidean distance to closest."""
        p = (0.2, 0.7, 0.2)
        closest = np.array([0.2, 0.0, 0.2])
        euclidean = float(np.linalg.norm(np.array(p) - closest))
        d = _signed_dist(p, tuple(closest))
        assert abs(d) == pytest.approx(euclidean, abs=1e-3)


# ---------------------------------------------------------------------------
# Tests: degenerate triangle (collinear vertices)
# ---------------------------------------------------------------------------

class TestDegenerateTriangle:
    """Degenerate (zero-area) triangles should not crash or produce NaN."""

    def test_degenerate_collinear_no_crash(self):
        """Collinear vertices (zero-area triangle) must not crash."""
        v0 = (0.0, 0.0, 0.0)
        v1 = (0.5, 0.0, 0.0)
        v2 = (1.0, 0.0, 0.0)  # All on the same line
        p = (0.5, 1.0, 0.0)
        closest = _closest(p, v0, v1, v2)
        assert not np.any(np.isnan(closest)), "Degenerate triangle produced NaN"

    def test_degenerate_coincident_vertices_no_crash(self):
        """All vertices at same point — must not crash."""
        v = (0.5, 0.0, 0.5)
        p = (0.5, 1.0, 0.5)
        closest = _closest(p, v, v, v)
        assert not np.any(np.isnan(closest)), "Coincident vertices produced NaN"
