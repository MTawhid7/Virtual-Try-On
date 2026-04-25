"""Collision module — body-cloth and cloth self-collision via spatial hash + point-triangle projection."""

from simulation.collision.sphere_collider import SphereCollider
from simulation.collision.body_collider import BodyCollider
from simulation.collision.self_collider import ClothSelfCollider

__all__ = ["SphereCollider", "BodyCollider", "ClothSelfCollider"]
