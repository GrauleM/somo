import pybullet as p
import pybullet_data

import numpy as np

import os
import sys

path = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..", "..", "..")
)  # this is a bit hacky... just in case the user doesnt have somo installed...
sys.path.insert(0, path)

from somo.sm_manipulator_definition import SMManipulatorDefinition
from somo.sm_actuator_definition import SMActuatorDefinition
from somo.sm_link_definition import SMLinkDefinition
from somo.sm_joint_definition import SMJointDefinition
from somo.sm_continuum_manipulator import SMContinuumManipulator

from somo.utils import load_constrained_urdf

import sorotraj

# select whether you want to record a video or not
VIDEO_LOGGING = False

######## SIMULATION SETUP ########

### prepare everything for the physics client / rendering
## Pretty rendering
opt_str = "--background_color_red=1.0 --background_color_green=1.0 --background_color_blue=1.0"  # this opens the gui with a white background and no ground grid
cam_width, cam_height = 1920, 1640
if cam_width is not None and cam_height is not None:
    opt_str += " --width=%d --height=%d" % (cam_width, cam_height)

physicsClient = p.connect(
    p.GUI, options=opt_str
)  # starts the physics client with the options specified above. replace p.GUI with p.DIRECT to avoid gui

p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)

# Set the camera position. This goes right after you instantiate the GUI:
cam_distance, cam_yaw, cam_pitch, cam_xyz_target = 6.5, 30.0, -30.0, [0.0, 0.0, 2.4]
p.resetDebugVisualizerCamera(
    cameraDistance=cam_distance,
    cameraYaw=cam_yaw,
    cameraPitch=cam_pitch,
    cameraTargetPosition=cam_xyz_target,
)

## Set physics parameters and simulation properties
p.setGravity(0, 0, -10)
p.setPhysicsEngineParameter(enableConeFriction=1)
p.setRealTimeSimulation(
    0
)  # this is necessary to enable torque control. only if this is set to 0 and the simulation is done with explicit steps will the torque control work correctly

## Specify time steps
time_step = 0.001
p.setTimeStep(time_step)
n_steps = 60000

### load all the objects into the environment
# load the ground plane
planeId = p.loadURDF(
    "additional_urdfs/plane/plane.urdf", flags=p.URDF_USE_MATERIAL_COLORS_FROM_MTL
)
p.changeDynamics(planeId, -1, lateralFriction=1)  # set ground plane friction

# load the bball hoop
hoopStartPos = [-3.6, 0, 1]
hoopStartOr = p.getQuaternionFromEuler([0, 0, 0])
hoopId = p.loadURDF(
    "additional_urdfs/bball_court/hoop.urdf",
    hoopStartPos,
    hoopStartOr,
    flags=p.URDF_USE_MATERIAL_COLORS_FROM_MTL,
    useFixedBase=1,
    globalScaling=2.0,
)

# load the bball
ballStartPos = [3.5, 0.75, 0.5]
ballStartOr = p.getQuaternionFromEuler([0, 0, 0])
ballId = p.loadURDF(
    "additional_urdfs/bball_court/ball.urdf",
    ballStartPos,
    ballStartOr,
    flags=p.URDF_USE_MATERIAL_COLORS_FROM_MTL,
    globalScaling=0.5,
)
p.changeDynamics(ballId, -1, lateralFriction=1)  # set ball friction

### Create and load the manipulator / arm
# load the manipulator definition
arm_manipulator_def = SMManipulatorDefinition.from_file("definitions/bb_arm.yaml")

# create the arm manipulator...
arm = SMContinuumManipulator(arm_manipulator_def)
# ... and load it
startPos = [0, 0, 0]
startOr = p.getQuaternionFromEuler([0, 0, 0])
arm.load_to_pybullet(
    baseStartPos=startPos,
    baseStartOrn=startOr,
    baseConstraint="static",  # other options are free and constrained, but those are not recommended rn
    physicsClient=physicsClient,
)

# below is an example of how lateral friction and restitution can be changed for the whole manipulator.
contact_properties = {
    "lateralFriction": 1,
    # 'restitution': 0.0, # uncomment to change restitution
}
arm.set_contact_property(contact_properties)


######## PRESCRIBE A TRAJECTORY ########
# here, the trajectory is hard-coded (booh!) and prepared using the sorotraj format
traj = sorotraj.TrajBuilder(graph=False)
traj.load_traj_def("trajectory")
trajectory = traj.get_trajectory()
interp = sorotraj.Interpolator(trajectory)
actuation_fn = interp.get_interp_function(
    num_reps=1, speed_factor=1.2, invert_direction=False, as_list=False
)


######## EXECUTE SIMULATION ########
# if desired, start video logging - this goes before the run loop
if VIDEO_LOGGING:
    vid_filename = "vid.mp4"
    logIDvideo = p.startStateLogging(p.STATE_LOGGING_VIDEO_MP4, vid_filename)

# this for loop is the actual simulation
for i in range(n_steps):

    torques = actuation_fn(
        i * time_step
    )  # retrieve control torques from the trajectory.

    # applying the control torques
    arm.apply_actuation_torques(
        actuator_nrs=[0, 0, 1, 1, 2, 2, 3, 3, 4, 4],
        axis_nrs=[0, 1, 0, 1, 0, 1, 0, 1, 0, 1],
        actuation_torques=torques.tolist(),
    )

    p.stepSimulation()


######## CLEANUP AFTER SIMULATION ########
# this goes after the run loop
if VIDEO_LOGGING:
    p.stopStateLogging(logIDvideo)
# ... aaand disconnect pybullet
p.disconnect()
