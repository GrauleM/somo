import pybullet as p
import pybullet_data
import numpy as np
import os
import sys
import time


path = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
sys.path.insert(0, path)

from somo.sm_manipulator_definition import SMManipulatorDefinition
from somo.create_cmassembly_urdf import create_cmassembly_urdf
from somo.sm_link_definition import SMLinkDefinition

# load manipulator definition
manipulator_definition = SMManipulatorDefinition.from_file("finger_def_hand.yaml")
# offset_0= [0, 0, 0, np.pi/4, 0, 0]
# offset_1= [1, 0, 0, np.pi/4, 0, 0]
# offset_2= [1, 1, 0, np.pi/4, 0, np.pi]
# offset_3= [0, 1, 0, np.pi/4, 0, np.pi]

manipulator_definition_pairs = []
offsets = (
    [1, -0.75, 0, 0, np.pi / 2, 0],
    [1, -0.25, 0, 0, np.pi / 2, 0],
    [1, 0.25, 0, 0, np.pi / 2, 0],
    [1, 0.75, 0, 0, np.pi / 2, 0],
    [0, 1, 0, -np.pi / 4, np.pi / 2, 0],
)
for offset in offsets:
    manipulator_definition_pairs.append((manipulator_definition, offset))
# offset_0= [0, 0, 0, np.pi/3, 0, 0]
# offset_1= [0, 0, 0, 0, 0, 0]
# offset_2= [0, 0, 0, np.pi/3, 0, np.pi/3]
# offset_3= [0, 0, 0, -np.pi/3, -np.pi/3, np.pi/3]
# offset_3= [0, 0, 0, -np.pi/3, -np.pi/3, np.pi/3]

# offset_0= [1, 0, 1, 0*np.pi/4, 0, 0]
# offset_1= [0, 0, 1, 0*np.pi/4, 0, 0]
# offset_2= [1, 1, 1, 0*np.pi/4, 0, 0*np.pi]
# offset_3= [0, 1, 1, 0*np.pi/4, 0, 0*np.pi]
# create the urdf

base_link1 = SMLinkDefinition(
    shape_type="box",
    dimensions=[2, 2, 0.5],
    mass=1.0,
    material_color=[0.7, 0.2, 0.2, 1],
    inertial_values=[1, 0, 0, 1, 0, 1],
    material_name="base_color",
    origin_offset=[0.0, 0.0, 0.25, 0, 0, 0],
)


test_urdf = create_cmassembly_urdf(
    base_links=[base_link1],
    manipulator_definition_pairs=manipulator_definition_pairs,
    assembly_name="starfish_test",
)


# prepare everything for the physics client
physicsClient = p.connect(
    p.GUI
)  # p.GUI for graphical, or p.DIRECT for non-graphical version
n_steps = 1500000

p.setGravity(0, 0, -10)
p.setRealTimeSimulation(
    0
)  # only if this is set to 0 and the simulation is done with explicit steps will the torque control work correctly
p.setPhysicsEngineParameter(
    enableFileCaching=0
)  # xx todo: check if this makes things faster
time_step = 0.01
p.setTimeStep(time_step)

# load the ground plane into pybullet simulator
p.setAdditionalSearchPath(
    pybullet_data.getDataPath()
)  # defines the path used by p.loadURDF
planeId = p.loadURDF("plane.urdf")

# load the test urdf into pybullet
bodyUniqueId = p.loadURDF(test_urdf, physicsClientId=physicsClient, useFixedBase=1)

# run a few simulation steps
for i in range(n_steps):
    # todo: change structure such that spring torque is always applied and joints are always turned off automatically.

    # todo: add assertion in apply_actuatorTorque that the actuator_nr is not larger than the number of actuators, and same for axis_nr
    # xx todo: passive_torque should always be applied in every timestep. how can this be done quickly?
    time.sleep(1000)
    p.stepSimulation()

print(i)
p.disconnect(physicsClient)
