import os
from collections import namedtuple
import numpy as np
import pybullet as p

from pybullet_planning.utils import CLIENT, CLIENTS, GRAVITY, INFO_FROM_BODY, STATIC_MASS
from pybullet_planning.utils import is_darwin, is_windows, get_client

from pybullet_planning.interfaces.env_manager.savers import Saver
from pybullet_planning.interfaces.env_manager.user_io import HideOutput, update_viewer, user_input
from pybullet_planning.interfaces.env_manager.pose_transformation import set_pose
from pybullet_planning.interfaces.env_manager.shape_creation import ModelInfo, create_obj, get_urdf_flags

#####################################

# class World(object):
#     def __init__(self, client):
#         self.client = client
#         self.bodies = {}
#     def activate(self):
#         set_client(self.client)
#     def load(self, path, name=None, fixed_base=False, scale=1.):
#         body = p.loadURDF(path, useFixedBase=fixed_base, physicsClientId=self.client)
#         self.bodies[body] = URDFInfo(name, path, fixed_base, scale)
#         return body
#     def remove(self, body):
#         del self.bodies[body]
#         return p.removeBody(body, physicsClientId=self.client)
#     def reset(self):
#         p.resetSimulation(physicsClientId=self.client)
#         self.bodies = {}
#     # TODO: with statement
#     def copy(self):
#         raise NotImplementedError()
#     def __repr__(self):
#         return '{}({})'.format(self.__class__.__name__, len(self.bodies))


def disable_viewer(client_id):
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, False, physicsClientId=client_id)
    p.configureDebugVisualizer(p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, False, physicsClientId=client_id)
    p.configureDebugVisualizer(p.COV_ENABLE_DEPTH_BUFFER_PREVIEW, False, physicsClientId=client_id)
    p.configureDebugVisualizer(p.COV_ENABLE_RGB_BUFFER_PREVIEW, False, physicsClientId=client_id)
    #p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, False, physicsClientId=client_id)
    #p.configureDebugVisualizer(p.COV_ENABLE_SINGLE_STEP_RENDERING, True, physicsClientId=client_id)
    #p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, False, physicsClientId=client_id)
    #p.configureDebugVisualizer(p.COV_ENABLE_WIREFRAME, True, physicsClientId=client_id)
    #p.COV_ENABLE_MOUSE_PICKING, p.COV_ENABLE_KEYBOARD_SHORTCUTS

def set_renderer(client_id, enable):
    client = client_id
    if not has_gui(client):
        return
    CLIENTS[client] = enable
    p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, int(enable), physicsClientId=client)

class LockRenderer(Saver):
    # disabling rendering temporary makes adding objects faster
    def __init__(self, client_id, lock=True):
        self._client_id = client_id
        self.state = CLIENTS[self._client_id]
        # skip if the visualizer isn't active
        if has_gui(self._client_id) and lock:
            set_renderer(self._client_id, enable=False)

    def restore(self):
        if not has_gui(self._client_id):
            return
        assert self.state is not None
        if self.state != CLIENTS[self._client_id]:
           set_renderer(self._client_id, enable=self.state)


def connect(use_gui=True, shadows=True, color=None, width=None, height=None):
    # Shared Memory: execute the physics simulation and rendering in a separate process
    # https://github.com/bulletphysics/bullet3/blob/master/examples/pybullet/examples/vrminitaur.py#L7
    # make sure to compile pybullet with PYBULLET_USE_NUMPY enabled
    if use_gui and not is_darwin() and not is_windows() and ('DISPLAY' not in os.environ):
        use_gui = False
        print('No display detected!')
    method = p.GUI if use_gui else p.DIRECT
    with HideOutput():
        #  --window_backend=2 --render_device=0'
        # options="--width=1024 --height=768"
        # options="--mp4=\"test.mp4\" --mp4fps=240"
        options = ''
        if color is not None:
            options += '--background_color_red={} --background_color_green={} --background_color_blue={}'.format(*color)
        if width is not None:
            options += '--width={}'.format(width)
        if height is not None:
            options += '--height={}'.format(height)
        sim_id = p.connect(method, options=options) # key=None,
        #sim_id = p.connect(p.GUI, options="--opengl2") if use_gui else p.connect(p.DIRECT)
    assert 0 <= sim_id
    #sim_id2 = p.connect(p.SHARED_MEMORY)
    #print(sim_id, sim_id2)
    CLIENTS[sim_id] = True if use_gui else None
    if use_gui:
        # p.COV_ENABLE_PLANAR_REFLECTION
        # p.COV_ENABLE_SINGLE_STEP_RENDERING
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, False, physicsClientId=sim_id)
        p.configureDebugVisualizer(p.COV_ENABLE_TINY_RENDERER, False, physicsClientId=sim_id)
        p.configureDebugVisualizer(p.COV_ENABLE_RGB_BUFFER_PREVIEW, False, physicsClientId=sim_id)
        p.configureDebugVisualizer(p.COV_ENABLE_DEPTH_BUFFER_PREVIEW, False, physicsClientId=sim_id)
        p.configureDebugVisualizer(p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, False, physicsClientId=sim_id)
        p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, shadows, physicsClientId=sim_id)

    # you can also use GUI mode, for faster OpenGL rendering (instead of TinyRender CPU)
    #visualizer_options = {
    #    p.COV_ENABLE_WIREFRAME: 1,
    #    p.COV_ENABLE_SHADOWS: 0,
    #    p.COV_ENABLE_RENDERING: 0,
    #    p.COV_ENABLE_TINY_RENDERER: 1,
    #    p.COV_ENABLE_RGB_BUFFER_PREVIEW: 0,
    #    p.COV_ENABLE_DEPTH_BUFFER_PREVIEW: 0,
    #    p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW: 0,
    #    p.COV_ENABLE_VR_RENDER_CONTROLLERS: 0,
    #    p.COV_ENABLE_VR_PICKING: 0,
    #    p.COV_ENABLE_VR_TELEPORTING: 0,
    #}
    #for pair in visualizer_options.items():
    #    p.configureDebugVisualizer(*pair)
    return sim_id


def disconnect(client_id):
    # TODO: change CLIENT?
    if CLIENT in CLIENTS:
        del CLIENTS[CLIENT]
    with HideOutput():
        return p.disconnect(physicsClientId=client_id)

def is_connected(client_id):
    return p.getConnectionInfo(physicsClientId=client_id)['isConnected']

def get_connection(client_id):
    return p.getConnectionInfo(physicsClientId=get_client(client_id))['connectionMethod']

def has_gui(client_id):
    return get_connection(get_client(client_id)) == p.GUI

def get_data_path():
    import pybullet_data
    return pybullet_data.getDataPath()

def add_data_path(data_path=None):
    if data_path is None:
        data_path = get_data_path()
    p.setAdditionalSearchPath(data_path)
    return data_path

def enable_gravity(client_id):
    p.setGravity(0, 0, -GRAVITY, physicsClientId=client_id)

def disable_gravity(client_id):
    p.setGravity(0, 0, 0, physicsClientId=client_id)

def set_real_time(client_id, real_time):
    p.setRealTimeSimulation(int(real_time), physicsClientId=client_id)

def enable_real_time(client_id):
    set_real_time(client_id, True)

def disable_real_time(client_id):
    set_real_time(client_id, False)

def update_state(client_id):
    # TODO: this doesn't seem to automatically update still
    disable_gravity(client_id)
    #step_simulation()
    #for body in get_bodies():
    #    for link in get_links(body):
    #        # if set to 1 (or True), the Cartesian world position/orientation
    #        # will be recomputed using forward kinematics.
    #        get_link_state(body, link)
    #for body in get_bodies():
    #    get_pose(body)
    #    for joint in get_joints(body):
    #        get_joint_position(body, joint)
    #p.getKeyboardEvents()
    #p.getMouseEvents()

def reset_simulation(client_id):
    """resetSimulation will remove all objects from the world and reset the world to initial conditions.
    """
    p.resetSimulation(physicsClientId=client_id)

#####################################

# Simulation

def load_pybullet(client_id, filename, fixed_base=False, scale=1., **kwargs):
    # fixed_base=False implies infinite base mass
    with LockRenderer(client_id):
        if filename.endswith('.urdf'):
            flags = get_urdf_flags(**kwargs)
            body = p.loadURDF(filename, useFixedBase=fixed_base, flags=flags,
                              globalScaling=scale, physicsClientId=client_id)
        elif filename.endswith('.sdf'):
            body = p.loadSDF(filename, physicsClientId=client_id)
        elif filename.endswith('.xml'):
            body = p.loadMJCF(filename, physicsClientId=client_id)
        elif filename.endswith('.bullet'):
            body = p.loadBullet(filename, physicsClientId=client_id)
        elif filename.endswith('.obj'):
            # TODO: fixed_base => mass = 0?
            body = create_obj(client_id, filename, scale=scale, **kwargs)
        else:
            raise ValueError(filename)
    INFO_FROM_BODY[CLIENT, body] = ModelInfo(None, filename, fixed_base, scale)
    return body

def set_caching(client_id, cache):
    p.setPhysicsEngineParameter(enableFileCaching=int(cache), physicsClientId=client_id)

def load_model_info(client_id, info):
    # TODO: disable file caching to reuse old filenames
    # p.setPhysicsEngineParameter(enableFileCaching=0, physicsClientId=client_id)
    if info.path.endswith('.urdf'):
        return load_pybullet(client_id, info.path, fixed_base=info.fixed_base, scale=info.scale)
    if info.path.endswith('.obj'):
        mass = STATIC_MASS if info.fixed_base else 1.
        return create_obj(client_id, info.path, mass=mass, scale=info.scale)
    raise NotImplementedError(info.path)

URDF_FLAGS = [p.URDF_USE_INERTIA_FROM_FILE,
              p.URDF_USE_SELF_COLLISION,
              p.URDF_USE_SELF_COLLISION_EXCLUDE_PARENT,
              p.URDF_USE_SELF_COLLISION_EXCLUDE_ALL_PARENTS]

def get_model_path(rel_path): # TODO: add to search path
    directory = os.path.dirname(os.path.abspath(__file__))
    return os.path.join(directory, '..', rel_path)

def save_state(client_id):
    return p.saveState(physicsClientId=client_id)

def restore_state(client_id, state_id):
    p.restoreState(stateId=state_id, physicsClientId=client_id)

def save_bullet(client_id, filename):
    p.saveBullet(filename, physicsClientId=client_id)

def restore_bullet(client_id, filename):
    p.restoreState(fileName=filename, physicsClientId=client_id)
