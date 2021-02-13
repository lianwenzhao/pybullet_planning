
from collections import defaultdict, deque, namedtuple

import numpy as np
import pybullet as p

from pybullet_planning.utils import CLIENT, BASE_LINK, STATIC_MASS

#####################################
# https://docs.google.com/document/d/10sXEhzFRSnvFcl3XxNGhnD4N2SedqwdAvK3dsihxVUA/edit#
DynamicsInfo = namedtuple('DynamicsInfo', ['mass', 'lateral_friction',
                                           'local_inertia_diagonal', 'local_inertial_pos',  'local_inertial_orn',
                                           'restitution', 'rolling_friction', 'spinning_friction',
                                           'contact_damping', 'contact_stiffness', 'body_type', 'collision_margin'])

def get_dynamics_info(client_id, body, link=BASE_LINK):
    return DynamicsInfo(*p.getDynamicsInfo(body, link, physicsClientId=client_id))

get_link_info = get_dynamics_info

def get_mass(client_id, body, link=BASE_LINK):
    # TOOD: get full mass
    return get_dynamics_info(client_id, body, link).mass

def set_dynamics(client_id, body, link=BASE_LINK, **kwargs):
    # TODO: iterate over all links
    p.changeDynamics(body, link, physicsClientId=client_id, **kwargs)

def set_mass(client_id, body, mass, link=BASE_LINK):
    set_dynamics(client_id, body, link=link, mass=mass)

def set_static(client_id, body):
    """set all the body's links to be static (infinite mass, doesn't move under gravity)

    Parameters
    ----------
    body : int
        [description]
    """
    from pybullet_planning.interfaces.robots.link import get_all_links
    for link in get_all_links(client_id, body):
        set_mass(client_id, body, mass=STATIC_MASS, link=link)

def set_all_static(client_id):
    from pybullet_planning.interfaces.env_manager.simulation import disable_gravity
    from pybullet_planning.interfaces.robots.body import get_bodies
    # TODO: mass saver
    disable_gravity(client_id)
    for body in get_bodies(client_id):
        set_static(client_id, body)

def get_joint_inertial_pose(client_id, body, joint):
    dynamics_info = get_dynamics_info(client_id, body, joint)
    return dynamics_info.local_inertial_pos, dynamics_info.local_inertial_orn

def get_local_link_pose(client_id, body, joint):
    from pybullet_planning.interfaces.env_manager.pose_transformation import Pose, multiply, invert
    from pybullet_planning.interfaces.robots.joint import get_joint_parent_frame
    from pybullet_planning.interfaces.robots.link import parent_link_from_joint

    parent_joint = parent_link_from_joint(client_id, body, joint)
    #world_child = get_link_pose(body, joint)
    #world_parent = get_link_pose(body, parent_joint)
    ##return multiply(invert(world_parent), world_child)
    #return multiply(world_child, invert(world_parent))

    # https://github.com/bulletphysics/bullet3/blob/9c9ac6cba8118544808889664326fd6f06d9eeba/examples/pybullet/gym/pybullet_utils/urdfEditor.py#L169
    parent_com = get_joint_parent_frame(client_id, body, joint)
    tmp_pose = invert(multiply(get_joint_inertial_pose(client_id, body, joint), parent_com))
    parent_inertia = get_joint_inertial_pose(client_id, body, parent_joint)
    #return multiply(parent_inertia, tmp_pose) # TODO: why is this wrong...
    _, orn = multiply(parent_inertia, tmp_pose)
    pos, _ = multiply(parent_inertia, Pose(parent_com[0]))
    return (pos, orn)
