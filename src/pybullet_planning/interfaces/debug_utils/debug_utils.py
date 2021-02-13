import math
import numpy as np
import pybullet as p
from itertools import product, combinations

from pybullet_planning.utils import CLIENT, BASE_LINK, GREEN, RED, BLUE, BLACK, WHITE, NULL_ID, YELLOW

from pybullet_planning.interfaces.env_manager.pose_transformation import unit_pose, tform_point, unit_from_theta, get_distance
from pybullet_planning.interfaces.geometry.bounding_box import get_aabb
from pybullet_planning.interfaces.geometry.camera import apply_alpha, set_camera_pose

def get_lifetime(lifetime):
    if lifetime is None:
        return 0
    return lifetime

def add_debug_parameter():
    # TODO: make a slider that controls the step in the trajectory
    # TODO: could store a list of savers
    #targetVelocitySlider = p.addUserDebugParameter("wheelVelocity", -10, 10, 0)
    #maxForce = p.readUserDebugParameter(maxForceSlider)
    raise NotImplementedError()

def add_text(client_id, text, position=(0, 0, 0), color=BLACK, lifetime=None, parent=NULL_ID, parent_link=BASE_LINK):
    return p.addUserDebugText(str(text), textPosition=position, textColorRGB=color[:3], # textSize=1,
                              lifeTime=get_lifetime(lifetime), parentObjectUniqueId=parent, parentLinkIndex=parent_link,
                              physicsClientId=client_id)

def add_line(client_id, start, end, color=BLACK, width=1, lifetime=None, parent=NULL_ID, parent_link=BASE_LINK):
    """[summary]

    Parameters
    ----------
    start : [type]
        [description]
    end : [type]
        [description]
    color : tuple, optional
        [description], by default (0, 0, 0)
    width : int, optional
        [description], by default 1
    lifetime : [type], optional
        [description], by default None
    parent : int, optional
        [description], by default NULL_ID
    parent_link : [type], optional
        [description], by default BASE_LINK

    Returns
    -------
    [type]
        [description]
    """
    return p.addUserDebugLine(start, end, lineColorRGB=color[:3], lineWidth=width,
                              lifeTime=get_lifetime(lifetime), parentObjectUniqueId=parent, parentLinkIndex=parent_link,
                              physicsClientId=client_id)

def remove_debug(client_id, debug):
    p.removeUserDebugItem(debug, physicsClientId=client_id)

remove_handle = remove_debug

def remove_handles(client_id, handles):
    for handle in handles:
        remove_debug(client_id, handle)

def remove_all_debug(client_id):
    p.removeAllUserDebugItems(physicsClientId=client_id)

def add_body_name(client_id, body, name=None, **kwargs):
    from pybullet_planning.interfaces.env_manager.pose_transformation import set_pose
    from pybullet_planning.interfaces.env_manager.savers import PoseSaver
    from pybullet_planning.interfaces.robots.body import get_name

    if name is None:
        name = get_name(client_id, body)
    with PoseSaver(client_id, body):
        set_pose(client_id, body, unit_pose())
        lower, upper = get_aabb(client_id, body)
    #position = (0, 0, upper[2])
    position = upper
    return add_text(client_id, name, position=position, parent=body, **kwargs)  # removeUserDebugItem

def add_segments(client_id, points, closed=False, **kwargs):
    lines = []
    for v1, v2 in zip(points, points[1:]):
        lines.append(add_line(client_id, v1, v2, **kwargs))
    if closed:
        lines.append(add_line(client_id, points[-1], points[0], **kwargs))
    return lines

def draw_link_name(client_id, body, link=BASE_LINK):
    from pybullet_planning.interfaces.robots.link import get_link_name
    return add_text(client_id, get_link_name(client_id, body, link), position=(0, 0.2, 0),
                    parent=body, parent_link=link)

def draw_pose(client_id, pose, length=0.1, **kwargs):
    origin_world = tform_point(pose, np.zeros(3))
    handles = []
    for k in range(3):
        axis = np.zeros(3)
        axis[k] = 1
        axis_world = tform_point(pose, length*axis)
        handles.append(add_line(client_id, origin_world, axis_world, color=axis, **kwargs))
    return handles

def draw_base_limits(client_id, limits, z=1e-2, **kwargs):
    lower, upper = limits
    vertices = [(lower[0], lower[1], z), (lower[0], upper[1], z),
                (upper[0], upper[1], z), (upper[0], lower[1], z)]
    return add_segments(client_id, vertices, closed=True, **kwargs)

def draw_circle(client_id, center, radius, n=24, **kwargs):
    vertices = []
    for i in range(n):
        theta = i*2*math.pi/n
        unit = np.append(unit_from_theta(theta), [0])
        vertices.append(center+radius*unit)
    return add_segments(client_id, vertices, closed=True, **kwargs)

def draw_aabb(client_id, aabb, **kwargs):
    d = len(aabb[0])
    vertices = list(product(range(len(aabb)), repeat=d))
    lines = []
    for i1, i2 in combinations(vertices, 2):
        if sum(i1[k] != i2[k] for k in range(d)) == 1:
            p1 = [aabb[i1[k]][k] for k in range(d)]
            p2 = [aabb[i2[k]][k] for k in range(d)]
            lines.append(add_line(client_id, p1, p2, **kwargs))
    return lines

def draw_point(client_id, point, size=0.01, **kwargs):
    lines = []
    for i in range(len(point)):
        axis = np.zeros(len(point))
        axis[i] = 1.0
        p1 = np.array(point) - size/2 * axis
        p2 = np.array(point) + size/2 * axis
        lines.append(add_line(client_id, p1, p2, **kwargs))
    return lines
    #extent = size * np.ones(len(point)) / 2
    #aabb = np.array(point) - extent, np.array(point) + extent
    #return draw_aabb(aabb, **kwargs)

def get_face_edges(face):
    #return list(combinations(face, 2))
    return list(zip(face, face[1:] + face[:1]))

def draw_mesh(client_id, mesh, **kwargs):
    verts, faces = mesh
    lines = []
    for face in faces:
        for i1, i2 in get_face_edges(face):
            lines.append(add_line(client_id, verts[i1], verts[i2], **kwargs))
    return lines

def draw_ray(client_id, ray, ray_result=None, visible_color=GREEN, occluded_color=RED, **kwargs):
    if ray_result is None:
        return [add_line(client_id, ray.start, ray.end, color=visible_color, **kwargs)]
    if ray_result.objectUniqueId == NULL_ID:
        hit_position = ray.end
    else:
        hit_position = ray_result.hit_position
    return [
        add_line(client_id, ray.start, hit_position, color=visible_color, **kwargs),
        add_line(client_id, hit_position, ray.end, color=occluded_color, **kwargs),
    ]


def get_body_from_pb_id(client_id, i):
    return p.getBodyUniqueId(i, physicsClientId=client_id)

def draw_collision_diagnosis(client_id, pb_closest_pt_output, viz_last_duration=-1, line_color=YELLOW, \
    focus_camera=True, camera_ray=np.array([0.1, 0, 0.05])):
    """[summary]

    Parameters
    ----------
    pb_closest_pt_output : [type]
        [description]
    viz_last_duration : int, optional
        [description], by default -1
    """
    from pybullet_planning.interfaces.env_manager.simulation import has_gui
    from pybullet_planning.interfaces.env_manager.user_io import HideOutput
    from pybullet_planning.interfaces.env_manager.user_io import wait_for_user, wait_for_duration
    from pybullet_planning.interfaces.robots.link import get_link_name, get_links
    from pybullet_planning.interfaces.robots.body import set_color, remove_body, clone_body, get_name

    if not has_gui(client_id) and pb_closest_pt_output:
        return
    # if paint_all_others:
    #     set_all_bodies_color()
        # for b in obstacles:
        #     set_color(b, (0,0,1,0.3))
    for u_cr in pb_closest_pt_output:
        handles = []
        b1 = get_body_from_pb_id(client_id, u_cr[1])
        b2 = get_body_from_pb_id(client_id, u_cr[2])
        l1 = u_cr[3]
        l2 = u_cr[4]
        b1_name = get_name(client_id, b1)
        b2_name = get_name(client_id, b2)
        l1_name = get_link_name(client_id, b1, l1)
        l2_name = get_link_name(client_id, b2, l2)

        print('*'*10)
        print('pairwise link collision: (Body #{0}, Link #{1}) - (Body #{2} Link #{3})'.format(
            b1_name, l1_name, b2_name, l2_name))
        clone1_fail = False
        clone2_fail = False
        try:
            with HideOutput():
                cloned_body1 = clone_body(client_id, b1, links=[l1] if get_links(client_id, b1) else None, collision=True, visual=False)
        except:
            print('cloning (body #{}, link #{}) fails.'.format(b1_name, l1_name))
            clone1_fail = True
            cloned_body1 = b1
        try:
            with HideOutput():
                cloned_body2 = clone_body(client_id, b2, links=[l2] if get_links(client_id, b2) else None, collision=True, visual=False)
        except:
            print('cloning (body #{}, link #{}) fails.'.format(b2_name, l2_name))
            clone2_fail = True
            cloned_body2 = b2

        set_color(client_id, cloned_body1, apply_alpha(RED, 0.2))
        set_color(client_id, cloned_body2, apply_alpha(GREEN, 0.2))

        handles.append(add_body_name(client_id, b1))
        handles.append(add_body_name(client_id, b2))
        handles.append(draw_link_name(client_id, b1, l1))
        handles.append(draw_link_name(client_id, b2, l2))

        handles.append(add_line(client_id, u_cr[5], u_cr[6], color=line_color, width=5))
        print('Penetration depth: {:.2E}'.format(get_distance(u_cr[5], u_cr[6])))
        if focus_camera:
            camera_base_pt = u_cr[5]
            camera_pt = np.array(camera_base_pt) + camera_ray
            set_camera_pose(client_id, tuple(camera_pt), camera_base_pt)

        if viz_last_duration < 0:
            wait_for_user(client_id)
        else:
            wait_for_duration(viz_last_duration)

        # restore lines and colors
        for h in handles: remove_debug(client_id, h)
        if not clone1_fail :
            remove_body(client_id, cloned_body1)
        else:
            set_color(client_id, b1, apply_alpha(WHITE, 0.5))
        if not clone2_fail :
            remove_body(client_id, cloned_body2)
        else:
            set_color(client_id, b2, apply_alpha(WHITE, 0.5))
