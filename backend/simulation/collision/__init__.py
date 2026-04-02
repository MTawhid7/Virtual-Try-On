"""Collision module — body-cloth collision via spatial hash + point-triangle projection."""

from simulation.collision.sphere_collider import SphereCollider
from simulation.collision.body_collider import BodyCollider

__all__ = ["SphereCollider", "BodyCollider"]
