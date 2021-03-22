import pybullet as p
import pybullet_data

import time  # for waiting
import numpy as np
import matplotlib.pyplot as plt

import importlib

import argparse


import os
import sys

# xx major todo: fix proper packaging and importing, this is awful
path = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../.."))
sys.path.insert(0, path)
from somo.sm_continuum_manipulator import SMContinuumManipulator
from somo.utils import load_constrained_urdf


# xx major todo: fix proper packaging and importing, this is awful
path = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "old_examples/exp_details")
)
sys.path.insert(0, path)

from finger_blocked_force_demo_exp import manipulator_definition

# xx todo: change experiment to json for better loading (importing modules is not cool)

# debugging insight: there are many more problems when I use the non-planar version
# start with quasiPlanar debugging. note: when I pull it far enough, it flips to the wrong side


def run():
    physicsClient = p.connect(
        p.GUI
    )  # p.GUI for graphical, or p.DIRECT for non-graphical version
    p.setAdditionalSearchPath(
        pybullet_data.getDataPath()
    )  # defines the path used by p.loadURDF
    p.setGravity(0, 0, -10)
    p.setPhysicsEngineParameter(enableConeFriction=1)
    p.setRealTimeSimulation(
        0
    )  # only if this is set to 0 and with explicit steps will the torque control work correctly

    # load the ground plane
    planeId = p.loadURDF("plane.urdf")

    # add a box for blocked force
    boxStartPos = [0, -1, 1.4]
    boxStartOr = p.getQuaternionFromEuler([0, 0, 0])
    boxId, boxConstraintId = load_constrained_urdf(
        "urdfs/smallSquareBox.urdf",
        boxStartPos,
        boxStartOr,
        physicsClient=physicsClient,
    )
    p.changeDynamics(boxId, -1, lateralFriction=2)
    # load finger
    finger = SMContinuumManipulator(manipulator_definition)

    finger_startPos = [0, 1, 2.05]
    finger_StartOr = p.getQuaternionFromEuler([-np.pi / 2, 0, np.pi])

    finger.load_to_pybullet(
        baseStartPos=finger_startPos,
        baseStartOrn=finger_StartOr,
        baseConstraint="static",
        physicsClient=physicsClient,
    )
    p.changeDynamics(finger.bodyUniqueId, -1, lateralFriction=2)
    p.changeDynamics(finger.bodyUniqueId, -1, restitution=1)

    time_step = 0.001
    p.setTimeStep(time_step)
    n_steps = 20000
    sim_time = 0.0

    lam = 1.0
    omega = 1.0
    torque_fns = [
        lambda t: 20 * np.sin(omega * t),
        lambda t: 20 * np.sin(omega * t - 1 * np.pi),
    ]  # - 0pi goes right, - pi goes left

    real_time = time.time()

    normal_forces = np.zeros((n_steps,))
    normal_forces_lastLink = np.zeros((n_steps,))
    time_plot = np.zeros((n_steps,))

    last_link = p.getNumJoints(finger.bodyUniqueId) - 1  # last link of the finger

    # wait for screen recording for demo
    wait_for_rec = False
    if wait_for_rec:
        time.sleep(5)
        print("6 more seconds")
        time.sleep(6)

    for i in range(n_steps):

        print(torque_fns[0](sim_time))
        finger.apply_actuationTorque(
            actuator_nr=0, axis_nr=0, actuation_torques=torque_fns[0](sim_time)
        )
        finger.apply_actuationTorque(actuator_nr=0, axis_nr=1, actuation_torques=0)

        p.stepSimulation()
        sim_time += time_step

        # time it took to simulate
        delta = time.time() - real_time
        real_time = time.time()
        # print(delta)

        contactPoints = p.getContactPoints(
            boxId, finger.bodyUniqueId, physicsClientId=physicsClient
        )

        contactPointsTip = p.getContactPoints(
            bodyA=boxId,
            bodyB=finger.bodyUniqueId,
            linkIndexB=last_link,
            physicsClientId=physicsClient,
        )

        # print(" ---------- normal force ---------")
        time_plot[i] = sim_time
        total_normal_force = 0
        for contactPoint in contactPoints:
            normal_force = contactPoint[9]
            total_normal_force += normal_force

        total_normal_force_lastLink = 0
        print(len(contactPoints), len(contactPointsTip))
        for contactPointTip in contactPointsTip:
            normal_forceTip = contactPointTip[9]
            print(normal_forceTip)
            total_normal_force_lastLink += normal_forceTip

        normal_forces[i] = total_normal_force
        normal_forces_lastLink[i] = total_normal_force_lastLink

    p.disconnect()

    plt.plot(time_plot, normal_forces)
    plt.xlabel("time")
    plt.ylabel("total normal force")
    plt.show()

    plt.plot(time_plot, normal_forces_lastLink)
    plt.xlabel("time")
    plt.ylabel("total normal force actuator tip")
    plt.show()


# xx todo: correctly read in and apply spring constants from the actuator definitions

if __name__ == "__main__":
    # xx todo: load experiments
    run()
