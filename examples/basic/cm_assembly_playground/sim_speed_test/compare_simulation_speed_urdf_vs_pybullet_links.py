import pybullet as p
import pybullet_data

import time  # for waiting
import numpy as np
import matplotlib.pyplot as plt

import os
import sys

# TODO: define world scaling factor and apply to g (gravity), measured forces, ...

path = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../..", ".."))
sys.path.insert(0, path)
from somo.sm_manipulator_definition import SMManipulatorDefinition
from somo.sm_continuum_manipulator import SMContinuumManipulator
from somo.utils import load_constrained_urdf

# xx major todo: fix proper packaging and importing, this is awful
path = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../..", "somo"))
sys.path.insert(0, path)

world_scaling = 1.0

physicsClient = p.connect(
    p.GUI
)  # p.GUI for graphical, or p.DIRECT for non-graphical version

p.setGravity(0, 0, -world_scaling * 9.81)
p.setPhysicsEngineParameter(enableConeFriction=1)
p.setRealTimeSimulation(0)

# add white ground plane for pretty screen grabs and videos
p.setAdditionalSearchPath(
    pybullet_data.getDataPath()
)  # defines the path used by p.loadURDF
planeId = p.loadURDF("plane.urdf")

boxStartPos = [
    0,
    0,
    4,
]  # have to be careful to set this position such that the box and atuator just touch (to replicate experimental condition)
boxStartOr = p.getQuaternionFromEuler([0, 0, 0])
boxId = p.loadURDF(
    "twoBoxes.urdf",
    basePosition=boxStartPos,
    baseOrientation=boxStartOr,
    physicsClientId=physicsClient,
)
p.changeDynamics(boxId, -1, lateralFriction=1.2)

time_step = 0.0001
p.setTimeStep(time_step)
n_steps = 10000

start_time = time.time()
for i in range(n_steps):
    p.stepSimulation()
end_time = time.time()
print(f"execution time: {end_time-start_time}")

p.disconnect()
