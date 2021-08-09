import numpy as np

import pybullet as p
import pybullet_data

import os
import sys
import pytest
from pathlib import Path


path = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, path)
from somo.sm_manipulator_definition import SMManipulatorDefinition
from somo.sm_continuum_manipulator import SMContinuumManipulator


def manipulator_actuation_tester(gui: bool = False, total_sim_steps=1000):
    """
    tests whether a manipulator can be instantiated in pybullet and whether a sinusoidal torque actuation can be applied
    """

    palmBaseHeight = 0.05
    startPositions = [0, 0, palmBaseHeight]
    startOrientations = p.getQuaternionFromEuler([0, 0, 0])

    # load the manipulator definition
    manipulater_def_path = os.path.join(
        Path(__file__).parent, "manipulator_test_def.yaml"
    )

    manipulator_def = SMManipulatorDefinition.from_file(manipulater_def_path)

    manipulator = SMContinuumManipulator(manipulator_def)

    if gui:
        physicsClient = p.connect(p.GUI)
    else:
        physicsClient = p.connect(p.DIRECT)

    # setting up the physics client
    p.setGravity(0, 0, -10)
    p.setPhysicsEngineParameter(enableConeFriction=1)
    p.setRealTimeSimulation(
        0
    )  # only if this is set to 0 and the simulation is done with explicit steps will the torque control work correctly
    p.setPhysicsEngineParameter(
        enableFileCaching=0
    )  # x todo: check if this makes things faster
    bullet_time_step = 0.0001
    p.setTimeStep(bullet_time_step)

    # load the ground plane into pybullet
    p.setAdditionalSearchPath(
        pybullet_data.getDataPath()
    )  # defines the path used by p.loadURDF
    # _planeId = p.loadURDF("plane.urdf")

    manipulator.load_to_pybullet(
        baseStartPos=startPositions,
        baseStartOrn=startOrientations,
        baseConstraint="static",
        physicsClient=physicsClient,
    )  # todo: something looks funny for constrained and free baseConstraint

    p.changeDynamics(manipulator.bodyUniqueId, -1, lateralFriction=2)
    p.changeDynamics(manipulator.bodyUniqueId, -1, restitution=1)

    for i in range(int(total_sim_steps / 2)):

        action = 50 * np.sin(0.00005 * i)

        manipulator.apply_actuation_torques(
            actuator_nrs=[0],
            axis_nrs=[0],
            actuation_torques=[action],
        )

        p.stepSimulation(physicsClient)

    for i in range(int(total_sim_steps / 2)):
        action = 500 * np.sin(0.00005 * i)

        manipulator.apply_actuation_torques(
            actuator_nrs=[0],
            axis_nrs=[1],
            actuation_torques=[action],
        )

        p.stepSimulation(physicsClient)

    p.disconnect(physicsClient)

    # delete the test urdf
    os.remove(manipulator.manipulator_definition.urdf_filename)


def test_manipulator_actuation():
    manipulator_actuation_tester()


@pytest.mark.gui  # annotate it as a "gui" test
def test_manipulator_actuation_gui():
    manipulator_actuation_tester(gui=True, total_sim_steps=5000000)


# test_manipulator_actuation_gui()
