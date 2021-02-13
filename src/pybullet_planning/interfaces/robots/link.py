from itertools import product, combinations
from collections import defaultdict, deque, namedtuple
import pybullet as p

from pybullet_planning.utils import BASE_LINK, CLIENT
from pybullet_planning.interfaces.robots.joint import get_num_joints, get_joints, get_joint_info, is_movable, prune_fixed_joints

#####################################
# Links

get_num_links = get_num_joints
get_links = get_joints # Does not include BASE_LINK

def child_link_from_joint(joint):
    return joint # link

def parent_joint_from_link(link):
    return link # joint

def get_all_links(client_id, body):
    return [BASE_LINK] + list(get_links(client_id, body))

def get_link_name(client_id, body, link):
    from pybullet_planning.interfaces.robots.body import get_base_name
    if link == BASE_LINK:
        return get_base_name(client_id, body)
    return get_joint_info(client_id, body, link).linkName.decode('UTF-8')

def get_link_parent(client_id, body, link):
    if link == BASE_LINK:
        return None
    return get_joint_info(client_id, body, link).parentIndex

parent_link_from_joint = get_link_parent

def link_from_name(client_id, body, name):
    from pybullet_planning.interfaces.robots.body import get_base_name
    if name == get_base_name(client_id, body):
        return BASE_LINK
    for link in get_joints(client_id, body):
        if get_link_name(client_id, body, link) == name:
            return link
    raise ValueError(body, name)


def has_link(client_id, body, name):
    try:
        link_from_name(client_id, body, name)
    except ValueError:
        return False
    return True

LinkState = namedtuple('LinkState', ['linkWorldPosition', 'linkWorldOrientation',
                                     'localInertialFramePosition', 'localInertialFrameOrientation',
                                     'worldLinkFramePosition', 'worldLinkFrameOrientation'])

def get_link_state(client_id, body, link, kinematics=True, velocity=True):
    # TODO: the defaults are set to False?
    # https://github.com/bulletphysics/bullet3/blob/master/examples/pybullet/pybullet.c
    return LinkState(*p.getLinkState(body, link, #computeLinkVelocity=velocity, computeForwardKinematics=kinematics,
                                     physicsClientId=client_id))

def get_com_pose(client_id, body, link): # COM = center of mass
    link_state = get_link_state(client_id, body, link)
    return link_state.linkWorldPosition, link_state.linkWorldOrientation

def get_link_inertial_pose(client_id, body, link):
    link_state = get_link_state(client_id, body, link)
    return link_state.localInertialFramePosition, link_state.localInertialFrameOrientation

def get_link_pose(client_id, body, link):
    from pybullet_planning.interfaces.env_manager.pose_transformation import get_pose
    if link == BASE_LINK:
        return get_pose(client_id, body)
    # if set to 1 (or True), the Cartesian world position/orientation will be recomputed using forward kinematics.
    link_state = get_link_state(client_id, body, link) #, kinematics=True, velocity=False)
    return link_state.worldLinkFramePosition, link_state.worldLinkFrameOrientation

def get_relative_pose(client_id, body, link1, link2=BASE_LINK):
    """compute relative transformation between two links of a body

    Parameters
    ----------
    body : [type]
        [description]
    link1 : [type]
        [description]
    link2 : [type], optional
        [description], by default BASE_LINK

    Returns
    -------
    [type]
        [description]
    """
    from pybullet_planning.interfaces.env_manager.pose_transformation import multiply, invert
    world_from_link1 = get_link_pose(client_id, body, link1)
    world_from_link2 = get_link_pose(client_id, body, link2)
    link2_from_link1 = multiply(invert(world_from_link2), world_from_link1)
    return link2_from_link1

#####################################

def get_all_link_parents(client_id, body):
    return {link: get_link_parent(client_id, body, link) for link in get_links(client_id, body)}

def get_all_link_children(client_id, body):
    children = {}
    for child, parent in get_all_link_parents(client_id, body).items():
        if parent not in children:
            children[parent] = []
        children[parent].append(child)
    return children

def get_link_children(client_id, body, link):
    children = get_all_link_children(client_id, body)
    return children.get(link, [])

def get_link_ancestors(client_id, body, link):
    # Returns in order of depth
    # Does not include link
    parent = get_link_parent(client_id, body, link)
    if parent is None:
        return []
    return get_link_ancestors(client_id, body, parent) + [parent]

def get_joint_ancestors(client_id, body, joint):
    link = child_link_from_joint(joint)
    return get_link_ancestors(client_id, body, link) + [link]

def get_movable_joint_ancestors(client_id, body, link):
    return prune_fixed_joints(client_id, body, get_joint_ancestors(client_id, body, link))

def get_joint_descendants(client_id, body, link):
    return list(map(parent_joint_from_link, get_link_descendants(client_id, body, link)))

def get_movable_joint_descendants(client_id, body, link):
    return prune_fixed_joints(client_id, body, get_joint_descendants(client_id, body, link))

def get_link_descendants(client_id, body, link, test=lambda l: True):
    descendants = []
    for child in get_link_children(client_id, body, link):
        if test(child):
            descendants.append(child)
            descendants.extend(get_link_descendants(client_id, body, child, test=test))
    return descendants

def get_link_subtree(client_id, body, link, **kwargs):
    return [link] + get_link_descendants(client_id, body, link, **kwargs)

def are_links_adjacent(client_id, body, link1, link2):
    return (get_link_parent(client_id, body, link1) == link2) or \
           (get_link_parent(client_id, body, link2) == link1)

def get_adjacent_links(client_id, body):
    adjacent = set()
    for link in get_links(client_id, body):
        parent = get_link_parent(client_id, body, link)
        adjacent.add((link, parent))
        #adjacent.add((parent, link))
    return adjacent

def get_adjacent_fixed_links(client_id, body):
    return list(filter(lambda item: not is_movable(client_id, body, item[0]),
                       get_adjacent_links(client_id, body)))

def get_fixed_links(client_id, body):
    edges = defaultdict(list)
    for link, parent in get_adjacent_fixed_links(client_id, body):
        edges[link].append(parent)
        edges[parent].append(link)
    visited = set()
    fixed = set()
    for initial_link in get_links(client_id, body):
        if initial_link in visited:
            continue
        cluster = [initial_link]
        queue = deque([initial_link])
        visited.add(initial_link)
        while queue:
            for next_link in edges[queue.popleft()]:
                if next_link not in visited:
                    cluster.append(next_link)
                    queue.append(next_link)
                    visited.add(next_link)
        fixed.update(product(cluster, cluster))
    return fixed


def get_moving_links(client_id, body, joints):
    moving_links = set()
    for joint in joints:
        link = child_link_from_joint(joint)
        if link not in moving_links:
            moving_links.update(get_link_subtree(client_id, body, link))
    return list(moving_links)


def get_moving_pairs(client_id, body, moving_joints):
    """
    Check all fixed and moving pairs
    Do not check all fixed and fixed pairs
    Check all moving pairs with a common
    """
    moving_links = get_moving_links(client_id, body, moving_joints)
    for link1, link2 in combinations(moving_links, 2):
        ancestors1 = set(get_joint_ancestors(client_id, body, link1)) & set(moving_joints)
        ancestors2 = set(get_joint_ancestors(client_id, body, link2)) & set(moving_joints)
        if ancestors1 != ancestors2:
            yield link1, link2


def get_self_link_pairs(client_id, body, joints, disabled_collisions=set(), only_moving=True):
    moving_links = get_moving_links(client_id, body, joints)
    fixed_links = list(set(get_links(client_id, body)) - set(moving_links))
    check_link_pairs = list(product(moving_links, fixed_links))
    if only_moving:
        check_link_pairs.extend(get_moving_pairs(client_id, body, joints))
    else:
        check_link_pairs.extend(combinations(moving_links, 2))
    check_link_pairs = list(filter(lambda pair: not are_links_adjacent(client_id, body, *pair), check_link_pairs))
    check_link_pairs = list(filter(lambda pair: (pair not in disabled_collisions) and
                                                (pair[::-1] not in disabled_collisions), check_link_pairs))
    return check_link_pairs


def get_link_attached_body_pairs(client_id, body, attachments=[]):
    link_body_pairs = []
    body_links = get_links(client_id, body)

    for attach in attachments:
        if attach.child in [pair[1] for pair in link_body_pairs]:
            continue
        if attach.parent == body:
            body_check_links = list(filter(lambda b_l : attach.parent_link != b_l and \
                not are_links_adjacent(client_id, body, b_l, attach.parent_link), body_links))
            link_body_pairs.append((body_check_links, attach.child))

    return link_body_pairs


def get_disabled_collisions(client_id, robot, disabled_link_pair_names):
    return {tuple(link_from_name(client_id, robot, link)
                  for link in pair if has_link(client_id, robot, link))
                  for pair in disabled_link_pair_names}


def get_body_body_disabled_collisions(client_id, body1, body2, disabled_link_pair_names):
    disabled_link_pairs = set()
    for link_name1, link_name2 in disabled_link_pair_names:
        if has_link(client_id, body1, link_name1) and has_link(client_id, body2, link_name2):
            pair = ((body1, link_from_name(client_id, body1, link_name1)), (body2, link_from_name(client_id, body2, link_name2)))
        elif has_link(client_id, body2, link_name1) and has_link(client_id, body1, link_name2):
            pair = ((body1, link_from_name(client_id, body1, link_name2)), (body2, link_from_name(client_id, body2, link_name1)))
        else:
            continue
        disabled_link_pairs.add(pair)
    return disabled_link_pairs
