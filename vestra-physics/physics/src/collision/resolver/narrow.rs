// physics/src/collision/resolver/narrow.rs

use super::{CollisionResolver, Contact};
use crate::collision::collider::MeshCollider;
use crate::engine::config::PhysicsConfig;
use crate::engine::state::PhysicsState;
use glam::{Vec3, Vec4};
#[cfg(feature = "parallel")]
use rayon::prelude::*;

/// Executes the Narrow Phase of collision detection.
/// Performs precise geometric tests (Point-Triangle) on candidates found in the Broad Phase.
/// Supports both Continuous (CCD) and Discrete collision detection.
pub fn perform_narrow_phase(
    resolver: &mut CollisionResolver,
    state: &mut PhysicsState,
    collider: &MeshCollider,
    config: &PhysicsConfig,
    dt: f32,
) {
    resolver.contacts.clear();

    let max_v_per_step = config.contact_thickness * 0.9;
    let max_v = max_v_per_step / dt;
    let discrete_radius = 0.05;

    // Parallelize logic:
    // We cannot write to `state.prev_positions` and `resolver.contacts` concurrently easily.
    // 1. We can collect Contact structs and "Position Corrections" (tuples of index, new_prev_pos).
    // 2. Apply position corrections serially (or parallel if disjoint, which they are).
    // 3. Extend contacts.

    #[cfg(feature = "parallel")]
    {
        // define a struct to hold the result of the parallel computation
        struct NarrowResult {
            contact: Option<Contact>,
            correction: Option<(usize, Vec4)>,
        }

        let results: Vec<NarrowResult> = (0..state.count)
            .into_par_iter()
            .map(|i| {
                let count = resolver.candidate_counts[i];
                if count == 0 {
                    return NarrowResult {
                        contact: None,
                        correction: None,
                    };
                }

                let offset = resolver.candidate_offsets[i];

                // FIX: Truncate to Vec3
                let pos_v4 = state.positions[i];
                let prev_v4 = state.prev_positions[i];
                let pos = pos_v4.truncate();
                let prev = prev_v4.truncate();

                let mut best_contact: Option<(Vec3, Vec3, f32)> = None;
                let mut min_metric = f32::MAX;
                let mut is_continuous = false;

                for j in 0..count {
                    let tri_idx = resolver.candidate_indices[offset + j];
                    let tri = &collider.triangles[tri_idx];

                    // 1. Continuous Check
                    if let Some((hit_point, hit_normal, t)) = tri.intersect_segment(prev, pos) {
                        if t < min_metric {
                            let normal = if hit_normal.dot(pos - prev) < 0.0 {
                                hit_normal
                            } else {
                                -hit_normal
                            };
                            best_contact = Some((hit_point, normal, t));
                            min_metric = t;
                            is_continuous = true;
                        }
                    }

                    // 2. Discrete Check
                    if !is_continuous {
                        // OPTIMIZATION: AABB Pruning
                        if tri.aabb_dist_sq(pos) < discrete_radius * discrete_radius {
                            let (closest, _bary) = tri.closest_point(pos);
                            let dist_sq = closest.distance_squared(pos);

                            if dist_sq < discrete_radius * discrete_radius {
                                if dist_sq < min_metric {
                                    let idx0 = collider.indices[tri_idx * 3] as usize;
                                    let idx1 = collider.indices[tri_idx * 3 + 1] as usize;
                                    let idx2 = collider.indices[tri_idx * 3 + 2] as usize;
                                    let n0 = collider.normals[idx0];
                                    let n1 = collider.normals[idx1];
                                    let n2 = collider.normals[idx2];

                                    let (_, bary) = tri.closest_point(pos);
                                    let smooth_normal =
                                        (n0 * bary[0] + n1 * bary[1] + n2 * bary[2]).normalize();

                                    best_contact = Some((closest, smooth_normal, dist_sq));
                                    min_metric = dist_sq;
                                }
                            }
                        }
                    }
                }

                let mut correction = None;
                let mut contact = None;

                if let Some((surface_point, normal, _metric)) = best_contact {
                    // Velocity Clamping (Airbag)
                    let velocity = (pos - prev) / dt;
                    let v_normal = velocity.dot(normal);

                    if v_normal < -max_v {
                        let v_tangent = velocity - normal * v_normal;
                        let v_clamped = normal * -max_v;
                        let new_velocity = v_tangent + v_clamped;

                        // FIX: Convert Vec3 result back to Vec4
                        let corr_vec = Vec4::from((new_velocity * dt, 0.0));
                        // calculating the new prev_position but returning the correction is safer?
                        // actually we can just return the new value
                        let new_prev = pos_v4 - corr_vec;
                        correction = Some((i, new_prev));
                    }

                    contact = Some(Contact {
                        particle_index: i,
                        normal,
                        surface_point,
                    });
                }

                NarrowResult {
                    contact,
                    correction,
                }
            })
            .collect();

        // Apply results
        // This part is serial but very fast (just assignments)
        for res in results {
            if let Some((i, new_prev)) = res.correction {
                state.prev_positions[i] = new_prev;
            }
            if let Some(c) = res.contact {
                resolver.contacts.push(c);
            }
        }
    }

    #[cfg(not(feature = "parallel"))]
    {
        for i in 0..state.count {
            let count = resolver.candidate_counts[i];
            if count == 0 {
                continue;
            }

            let offset = resolver.candidate_offsets[i];

            // FIX: Truncate to Vec3
            let pos_v4 = state.positions[i];
            let prev_v4 = state.prev_positions[i];
            let pos = pos_v4.truncate();
            let prev = prev_v4.truncate();

            let mut best_contact: Option<(Vec3, Vec3, f32)> = None;
            let mut min_metric = f32::MAX;
            let mut is_continuous = false;

            for j in 0..count {
                let tri_idx = resolver.candidate_indices[offset + j];
                let tri = &collider.triangles[tri_idx];

                // 1. Continuous Check
                if let Some((hit_point, hit_normal, t)) = tri.intersect_segment(prev, pos) {
                    if t < min_metric {
                        let normal = if hit_normal.dot(pos - prev) < 0.0 {
                            hit_normal
                        } else {
                            -hit_normal
                        };
                        best_contact = Some((hit_point, normal, t));
                        min_metric = t;
                        is_continuous = true;
                    }
                }

                // 2. Discrete Check
                if !is_continuous {
                    // OPTIMIZATION: AABB Pruning
                    // Skip expensive closest_point if particle is far from triangle AABB
                    if tri.aabb_dist_sq(pos) < discrete_radius * discrete_radius {
                        let (closest, _bary) = tri.closest_point(pos);
                        let dist_sq = closest.distance_squared(pos);

                        if dist_sq < discrete_radius * discrete_radius {
                            if dist_sq < min_metric {
                                let idx0 = collider.indices[tri_idx * 3] as usize;
                                let idx1 = collider.indices[tri_idx * 3 + 1] as usize;
                                let idx2 = collider.indices[tri_idx * 3 + 2] as usize;
                                let n0 = collider.normals[idx0];
                                let n1 = collider.normals[idx1];
                                let n2 = collider.normals[idx2];

                                let (_, bary) = tri.closest_point(pos);
                                let smooth_normal =
                                    (n0 * bary[0] + n1 * bary[1] + n2 * bary[2]).normalize();

                                best_contact = Some((closest, smooth_normal, dist_sq));
                                min_metric = dist_sq;
                            }
                        }
                    }
                }
            }

            if let Some((surface_point, normal, _metric)) = best_contact {
                // Velocity Clamping (Airbag)
                let velocity = (pos - prev) / dt;
                let v_normal = velocity.dot(normal);

                if v_normal < -max_v {
                    let v_tangent = velocity - normal * v_normal;
                    let v_clamped = normal * -max_v;
                    let new_velocity = v_tangent + v_clamped;

                    // FIX: Convert Vec3 result back to Vec4
                    let correction = Vec4::from((new_velocity * dt, 0.0));
                    state.prev_positions[i] = pos_v4 - correction;
                }

                resolver.contacts.push(Contact {
                    particle_index: i,
                    normal,
                    surface_point,
                });
            }
        }
    }
}
