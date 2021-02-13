import numpy as np
import pybullet as p

from pybullet_planning.utils import CIRCULAR_LIMITS
from pybullet_planning.interfaces.env_manager.pose_transformation import Euler, Pose, unit_pose, multiply, set_pose, get_pose
from pybullet_planning.interfaces.geometry.bounding_box import get_center_extent, get_aabb, aabb_contains_aabb, aabb2d_from_aabb, \
    aabb_contains_point

from pybullet_planning.interfaces.robots.body import get_point

#####################################
# Placements

def stable_z_on_aabb(client_id, body, aabb):
    center, extent = get_center_extent(client_id, body)
    _, upper = aabb
    return (upper + extent/2 + (get_point(client_id, body) - center))[2]

def stable_z(client_id, body, surface, surface_link=None):
    return stable_z_on_aabb(client_id, body, get_aabb(client_id, surface, link=surface_link))

def is_placed_on_aabb(client_id, body, bottom_aabb, above_epsilon=1e-2, below_epsilon=0.0):
    assert (0 <= above_epsilon) and (0 <= below_epsilon)
    top_aabb = get_aabb(client_id, body) # TODO: approximate_as_prism
    top_z_min = top_aabb[0][2]
    bottom_z_max = bottom_aabb[1][2]
    return ((bottom_z_max - below_epsilon) <= top_z_min <= (bottom_z_max + above_epsilon)) and \
           (aabb_contains_aabb(aabb2d_from_aabb(top_aabb), aabb2d_from_aabb(bottom_aabb)))

def is_placement(client_id, body, surface, **kwargs):
    return is_placed_on_aabb(client_id, body, get_aabb(client_id, surface), **kwargs)

def is_center_on_aabb(client_id, body, bottom_aabb, above_epsilon=1e-2, below_epsilon=0.0):
    # TODO: compute AABB in origin
    # TODO: use center of mass?
    assert (0 <= above_epsilon) and (0 <= below_epsilon)
    center, extent = get_center_extent(client_id, body) # TODO: approximate_as_prism
    base_center = center - np.array([0, 0, extent[2]])/2
    top_z_min = base_center[2]
    bottom_z_max = bottom_aabb[1][2]
    return ((bottom_z_max - abs(below_epsilon)) <= top_z_min <= (bottom_z_max + abs(above_epsilon))) and \
           (aabb_contains_point(base_center[:2], aabb2d_from_aabb(bottom_aabb)))

def is_center_stable(client_id, body, surface, **kwargs):
    return is_center_on_aabb(client_id, body, get_aabb(client_id, surface), **kwargs)

def sample_placement_on_aabb(client_id, top_body, bottom_aabb, top_pose=unit_pose(),
                             percent=1.0, max_attempts=50, epsilon=1e-3):
    # TODO: transform into the coordinate system of the bottom
    # TODO: maybe I should instead just require that already in correct frame
    for _ in range(max_attempts):
        theta = np.random.uniform(*CIRCULAR_LIMITS)
        rotation = Euler(yaw=theta)
        set_pose(client_id, top_body, multiply(Pose(euler=rotation), top_pose))
        center, extent = get_center_extent(client_id, top_body)
        lower = (np.array(bottom_aabb[0]) + percent*extent/2)[:2]
        upper = (np.array(bottom_aabb[1]) - percent*extent/2)[:2]
        if np.less(upper, lower).any():
            continue
        x, y = np.random.uniform(lower, upper)
        z = (bottom_aabb[1] + extent/2.)[2] + epsilon
        point = np.array([x, y, z]) + (get_point(client_id, top_body) - center)
        pose = multiply(Pose(point, rotation), top_pose)
        set_pose(client_id, top_body, pose)
        return pose
    return None

def sample_placement(client_id, top_body, bottom_body, bottom_link=None, **kwargs):
    bottom_aabb = get_aabb(client_id, bottom_body, link=bottom_link)
    return sample_placement_on_aabb(client_id, top_body, bottom_aabb, **kwargs)
