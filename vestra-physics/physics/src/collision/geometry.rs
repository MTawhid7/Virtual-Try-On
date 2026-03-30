// physics/src/collision/geometry.rs
use glam::Vec3;

/// A simple geometric Triangle struct used for collision testing.
#[derive(Clone, Copy, Debug)]
pub struct Triangle {
    pub v0: Vec3,
    pub v1: Vec3,
    pub v2: Vec3,
    #[allow(dead_code)]
    pub index: usize,
}

impl Triangle {
    pub fn new(v0: Vec3, v1: Vec3, v2: Vec3, index: usize) -> Self {
        Self { v0, v1, v2, index }
    }

    /// Computes the Axis-Aligned Bounding Box (AABB) of the triangle.
    pub fn aabb(&self) -> (Vec3, Vec3) {
        let min = self.v0.min(self.v1).min(self.v2);
        let max = self.v0.max(self.v1).max(self.v2);
        (min, max)
    }

    /// Computes the squared distance from a point `p` to the triangle's AABB.
    /// Returns 0.0 if the point is inside the AABB.
    pub fn aabb_dist_sq(&self, p: Vec3) -> f32 {
        let (min, max) = self.aabb();

        // Compute the closest point on the AABB to p
        let closest = p.max(min).min(max);

        // Distance squared between p and closest point on box
        closest.distance_squared(p)
    }

    /// Finds the closest point on this triangle to a given point `p`.
    /// Returns the point and the barycentric coordinates (u, v, w).
    /// Uses Voronoi regions to determine feature (vertex, edge, face) proximity.
    pub fn closest_point(&self, p: Vec3) -> (Vec3, [f32; 3]) {
        let ab = self.v1 - self.v0;
        let ac = self.v2 - self.v0;
        let ap = p - self.v0;

        let d1 = ab.dot(ap);
        let d2 = ac.dot(ap);

        if d1 <= 0.0 && d2 <= 0.0 {
            return (self.v0, [1.0, 0.0, 0.0]);
        }

        let bp = p - self.v1;
        let d3 = ab.dot(bp);
        let d4 = ac.dot(bp);

        if d3 >= 0.0 && d4 <= d3 {
            return (self.v1, [0.0, 1.0, 0.0]);
        }

        let vc = d1 * d4 - d3 * d2;
        if vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 {
            let v = d1 / (d1 - d3);
            return (self.v0 + ab * v, [1.0 - v, v, 0.0]);
        }

        let cp = p - self.v2;
        let d5 = ab.dot(cp);
        let d6 = ac.dot(cp);

        if d6 >= 0.0 && d5 <= d6 {
            return (self.v2, [0.0, 0.0, 1.0]);
        }

        let vb = d5 * d2 - d1 * d6;
        if vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 {
            let w = d2 / (d2 - d6);
            return (self.v0 + ac * w, [1.0 - w, 0.0, w]);
        }

        let va = d3 * d6 - d5 * d4;
        if va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0 {
            let w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
            return (self.v1 + (self.v2 - self.v1) * w, [0.0, 1.0 - w, w]);
        }

        let denom = 1.0 / (va + vb + vc);
        let v = vb * denom;
        let w = vc * denom;
        let u = 1.0 - v - w;

        (self.v0 + ab * v + ac * w, [u, v, w])
    }

    /// Möller–Trumbore intersection algorithm.
    /// Checks if the segment p1->p2 intersects the triangle.
    /// Returns Some((intersection_point, normal, t)) if intersection occurs within [0, 1].
    pub fn intersect_segment(&self, p1: Vec3, p2: Vec3) -> Option<(Vec3, Vec3, f32)> {
        let epsilon = 1e-7;
        let edge1 = self.v1 - self.v0;
        let edge2 = self.v2 - self.v0;
        let ray_vector = p2 - p1;
        let h = ray_vector.cross(edge2);
        let a = edge1.dot(h);

        if a > -epsilon && a < epsilon {
            return None; // Ray is parallel to triangle
        }

        let f = 1.0 / a;
        let s = p1 - self.v0;
        let u = f * s.dot(h);

        if u < 0.0 || u > 1.0 {
            return None;
        }

        let q = s.cross(edge1);
        let v = f * ray_vector.dot(q);

        if v < 0.0 || u + v > 1.0 {
            return None;
        }

        let t = f * edge2.dot(q);

        if t > epsilon && t < 1.0 {
            let intersection_point = p1 + ray_vector * t;
            let normal = edge1.cross(edge2).normalize();
            // Ensure normal points against the ray
            let final_normal = if normal.dot(ray_vector) < 0.0 {
                normal
            } else {
                -normal
            };
            return Some((intersection_point, final_normal, t));
        }

        None
    }
}
