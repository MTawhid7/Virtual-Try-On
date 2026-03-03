use std::f32;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DistanceType {
    PointTriangleFace,
    PointTriangleEdge(u8),
    PointTriangleVertex(u8),
    EdgeEdgeInterior,
    Degenerate,
}

#[derive(Debug, Clone, Copy)]
pub struct Vec3 {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl Vec3 {
    pub const ZERO: Vec3 = Vec3 { x: 0.0, y: 0.0, z: 0.0 };
    pub fn new(x: f32, y: f32, z: f32) -> Self { Self { x, y, z } }
    pub fn dot(self, other: Self) -> f32 { self.x * other.x + self.y * other.y + self.z * other.z }
    pub fn cross(self, other: Self) -> Self {
        Self {
            x: self.y * other.z - self.z * other.y,
            y: self.z * other.x - self.x * other.z,
            z: self.x * other.y - self.y * other.x,
        }
    }
    pub fn length_squared(self) -> f32 { self.dot(self) }
}
impl std::ops::Sub for Vec3 {
    type Output = Self;
    fn sub(self, other: Self) -> Self {
        Self { x: self.x - other.x, y: self.y - other.y, z: self.z - other.z }
    }
}
impl std::ops::Add for Vec3 {
    type Output = Self;
    fn add(self, other: Self) -> Self {
        Self { x: self.x + other.x, y: self.y + other.y, z: self.z + other.z }
    }
}
impl std::ops::Mul<f32> for Vec3 {
    type Output = Self;
    fn mul(self, scalar: f32) -> Self {
        Self { x: self.x * scalar, y: self.y * scalar, z: self.z * scalar }
    }
}

pub fn point_segment_distance_squared(p: Vec3, a: Vec3, b: Vec3) -> f32 {
    let ab = b - a;
    let ap = p - a;
    let ab_sq = ab.length_squared();
    if ab_sq < 1e-20 {
        return ap.length_squared();
    }
    let t = (ap.dot(ab) / ab_sq).clamp(0.0, 1.0);
    let closest = a + ab * t;
    (p - closest).length_squared()
}

pub fn point_triangle_distance_squared(
    p: Vec3,
    a: Vec3,
    b: Vec3,
    c: Vec3,
) -> (f32, DistanceType) {
    let ab = b - a;
    let ac = c - a;
    let ap = p - a;
    let bp = p - b;
    let cp = p - c;

    let normal = ab.cross(ac);
    let area2_sq = normal.length_squared();
    println!("area2_sq: {}", area2_sq);

    if area2_sq < 1e-20 {
        let d_ab = point_segment_distance_squared(p, a, b);
        let d_bc = point_segment_distance_squared(p, b, c);
        let d_ca = point_segment_distance_squared(p, c, a);
        return if d_ab <= d_bc && d_ab <= d_ca {
            (d_ab, DistanceType::PointTriangleEdge(0))
        } else if d_bc <= d_ca {
            (d_bc, DistanceType::PointTriangleEdge(1))
        } else {
            (d_ca, DistanceType::PointTriangleEdge(2))
        };
    }

    let d1 = ab.dot(ap);
    let d2 = ac.dot(ap);
    if d1 <= 0.0 && d2 <= 0.0 {
        return (ap.length_squared(), DistanceType::PointTriangleVertex(0));
    }

    let d3 = ab.dot(bp);
    let d4 = ac.dot(bp);
    if d3 >= 0.0 && d4 <= d3 {
        return (bp.length_squared(), DistanceType::PointTriangleVertex(1));
    }

    let d5 = ab.dot(cp);
    let d6 = ac.dot(cp);
    if d6 >= 0.0 && d5 <= d6 {
        return (cp.length_squared(), DistanceType::PointTriangleVertex(2));
    }

    let vc = d1 * d4 - d3 * d2;
    if vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 {
        let v = d1 / (d1 - d3);
        let closest = a + ab * v;
        return ((p - closest).length_squared(), DistanceType::PointTriangleEdge(0));
    }

    let va = d3 * d6 - d5 * d4;
    if va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0 {
        let w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        let closest = b + (c - b) * w;
        return ((p - closest).length_squared(), DistanceType::PointTriangleEdge(1));
    }

    let vb = d5 * d2 - d1 * d6;
    if vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 {
        let w = d2 / (d2 - d6);
        let closest = a + ac * w;
        return ((p - closest).length_squared(), DistanceType::PointTriangleEdge(2));
    }

    let denom = 1.0 / (va + vb + vc);
    let v = vb * denom;
    let w = vc * denom;
    let closest = a + ab * v + ac * w;
    ((p - closest).length_squared(), DistanceType::PointTriangleFace)
}

fn get_pos(i: usize) -> Vec3 {
    let cols = 20;
    let rows = 20;
    let width = 1.5;
    let height = 1.5;
    let verts_x = cols + 1;
    let half_w = width / 2.0;
    let half_h = height / 2.0;

    let xi = i % verts_x;
    let yi = i / verts_x;

    let u = xi as f32 / cols as f32;
    let v = yi as f32 / rows as f32;

    let pos_x = -half_w + u * width;
    let pos_y = 0.9973; // after pred
    let pos_z = half_h - v * height;

    Vec3::new(pos_x, pos_y, pos_z)
}

fn main() {
    let p = get_pos(271);
    let a = get_pos(81);
    let b = get_pos(103);
    let c = get_pos(82);

    println!("P: {:?}", p);
    println!("A: {:?}", a);
    println!("B: {:?}", b);
    println!("C: {:?}", c);

    let (d_sq, dtype) = point_triangle_distance_squared(p, a, b, c);
    println!("Distance Sq: {}, Type: {:?}", d_sq, dtype);

    let aabb_min_x = a.x.min(b.x).min(c.x) - 0.0015;
    let aabb_max_x = a.x.max(b.x).max(c.x) + 0.0015;
    let p_min_x = p.x - 0.0015;
    let p_max_x = p.x + 0.0015;

    let aabb_min_z = a.z.min(b.z).min(c.z) - 0.0015;
    let aabb_max_z = a.z.max(b.z).max(c.z) + 0.0015;
    let p_min_z = p.z - 0.0015;
    let p_max_z = p.z + 0.0015;

    println!("AABB overlsaps: {} {}",
        p_min_x <= aabb_max_x && p_max_x >= aabb_min_x,
        p_min_z <= aabb_max_z && p_max_z >= aabb_min_z
    );
}
