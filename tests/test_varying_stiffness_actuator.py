"""
tests actuators with planar_flag=2, which are planar but have multiple joints with varying stiffnesses
"""

import numpy as np
import copy
import yaml

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


def manipulator_actuation_tester_varStiffness(gui: bool = False, total_sim_steps=1000):
    """
    tests whether a manipulator can be instantiated in pybullet and whether a sinusoidal torque actuation can be applied
    """

    palmBaseHeight = 0.05
    startPositions = [0, 0, palmBaseHeight]
    startOrientations = p.getQuaternionFromEuler([0, 0, 0])

    # test for a manipulator with one and two actuators
    for definition_path in [
        "manipulator_test_def_varying_stiffness_2act.yaml",
        "manipulator_test_def_varying_stiffness.yaml",
    ]:
        # load the manipulator definition
        manipulater_def_path = os.path.join(Path(__file__).parent, definition_path)

        with open(manipulater_def_path, "r") as manipulater_def_file:
            manipulater_def_dict_template = yaml.safe_load(manipulater_def_file)

        manipulater_def_dict = copy.deepcopy(manipulater_def_dict_template)

        manipulator_def = SMManipulatorDefinition(**manipulater_def_dict)

        # instantiate manipulator
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

        num_axes = 1
        for axis_nr in range(num_axes):
            for i in range(int(total_sim_steps / num_axes)):

                action = 50 * np.sin(0.00005 * i)

                manipulator.apply_actuation_torques(
                    actuator_nrs=[0],
                    axis_nrs=[axis_nr],
                    actuation_torques=[action],
                )

                p.stepSimulation(physicsClient)

        p.disconnect(physicsClient)

        # delete the test urdf
        os.remove(manipulator.manipulator_definition.urdf_filename)


def test_manipulator_actuation_varStiffness():
    manipulator_actuation_tester_varStiffness()


@pytest.mark.gui  # annotate it as a "gui" test
def test_manipulator_actuation_varStiffness_gui():
    manipulator_actuation_tester_varStiffness(gui=True, total_sim_steps=int(100000))


if __name__ == "__main__":
    test_manipulator_actuation_varStiffness_gui()
