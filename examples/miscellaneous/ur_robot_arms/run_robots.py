'''
Run the robot arm playground example.

Tags
----------
- Robot
- Integration

Maintainer
----------
Clark Teeple, last updated March 26, 2021
'''

# Be sure to run this from the ur_robot_arms folder
#   cd examples/miscellaneous/ur_robot_arms/

import pybullet as p
import pybullet_data

import time  # for waiting
import numpy as np
import matplotlib.pyplot as plt

import os
import sys

# prepare everything for the physics client
physicsClient = p.connect(
    p.GUI
)  # p.GUI for graphical, or p.DIRECT for non-graphical version
p.setGravity(0, 0, -10)
p.setPhysicsEngineParameter(enableConeFriction=1)
p.setRealTimeSimulation(
    0
)  # only if this is set to 0 and the simulation is done with explicit steps will the torque control work correctly
p.setPhysicsEngineParameter(
    enableFileCaching=0
)  # x todo: check if this makes things faster

# load the ground plane into pybullet
p.setAdditionalSearchPath(
    pybullet_data.getDataPath()
)  # defines the path used by p.loadURDF
planeId = p.loadURDF("plane.urdf")

# Set up simulation paramters
time_step = 0.005
p.setTimeStep(time_step)
n_steps = 500000

# Position Function - Define position control function for arms.
pos_fn = lambda t: [
    1 * (np.sin(1 * t)),
    -1 * (np.sin(1 * t)) - np.pi / 2,  # Offset the shoulder to lift straight up.
    1 * (np.sin(1 * t)),
    -1 * (np.sin(1 * t)),
    1 * (np.sin(1 * t)),
    -1 * (np.sin(1 * t)),
]

# Force Function - if used, the joints have motors which can withstand some torque. Joints behave like spring/dampers
#                - if not used, joints are perfectly rigid.
force_fn = lambda t: [200] * 6  # Plenty of torque for all the robots
# force_fn = lambda t: [80]*6 # Not enough torque for the UR10e to stay upright
# force_fn = None            # Perfect position control with infinite motor torque


# Load the UR3e
robStartPos = [0, -1, 0]
robStartOr = p.getQuaternionFromEuler([0, 0, 0])
ur3eId = p.loadURDF(
    "additional_urdfs/universal_robots/urdf/ur3e_robot.urdf", robStartPos, robStartOr
)

# Load the UR5e
robStartPos = [0, 0, 0]
robStartOr = p.getQuaternionFromEuler([0, 0, 0])
ur5eId = p.loadURDF(
    "additional_urdfs/universal_robots/urdf/ur5e_robot.urdf", robStartPos, robStartOr
)

# Load the UR10e
robStartPos = [0, 1, 0]
robStartOr = p.getQuaternionFromEuler([0, 0, 0])
ur10eId = p.loadURDF(
    "additional_urdfs/universal_robots/urdf/ur10e_robot.urdf", robStartPos, robStartOr
)

# Make a list of robot objectIDs
robots = [ur3eId, ur5eId, ur10eId]

# Select the joints of the robot by name
jointNameToId = {}
for i in range(p.getNumJoints(robots[0])):
    jointInfo = p.getJointInfo(robots[0], i)
    jointNameToId[jointInfo[1].decode("UTF-8")] = jointInfo[0]

joint_ids = [
    jointNameToId["shoulder_pan_joint"],
    jointNameToId["shoulder_lift_joint"],
    jointNameToId["elbow_joint"],
    jointNameToId["wrist_1_joint"],
    jointNameToId["wrist_2_joint"],
    jointNameToId["wrist_3_joint"],
]

# Initialize the robots
for robot in robots:
    numJoints = p.getNumJoints(robot)
    pos_init = pos_fn(0)
    for j in range(numJoints):
        # print(p.getJointInfo(robot, j))
        p.changeVisualShape(robot, j, rgbaColor=[1, 1, 1, 1])
        p.setJointMotorControl2(robot, j, p.POSITION_CONTROL, 0.0)

    # p.setJointMotorControlArray(robot,joint_ids,p.POSITION_CONTROL,pos_init)
    for robot_joint, init_p in zip(joint_ids, pos_init):
        p.resetJointState(robot, robot_joint, init_p)

# Begin Simulation
sim_time = 0
start_time = time.time()

for i in range(n_steps):

    # Apply position control to all robots simultaneously
    for robot in [ur3eId, ur5eId, ur10eId]:
        if force_fn is not None:
            p.setJointMotorControlArray(
                robot,
                joint_ids,
                p.POSITION_CONTROL,
                pos_fn(sim_time),
                forces=force_fn(sim_time),
            )
        else:
            p.setJointMotorControlArray(
                robot, joint_ids, p.POSITION_CONTROL, pos_fn(sim_time)
            )

    p.stepSimulation()
    sim_time += time_step

    time.sleep(time_step)

    # Calculate how fast the simulation is running
    if np.mod(i, 10000) == 0 and i != 0:
        delta = (time_step * i) / (time.time() - start_time) * 100
        print("%0.2f %% Realtime" % (delta))

p.disconnect()
