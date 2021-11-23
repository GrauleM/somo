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


def manipulator_actuation_tester(
    gui: bool = False, total_sim_steps=1000, num_axes=None
):
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

    with open(manipulater_def_path, "r") as manipulater_def_file:
        manipulater_def_dict_template = yaml.safe_load(manipulater_def_file)

    manipulater_def_dict = copy.deepcopy(manipulater_def_dict_template)
    # if num_axes input is None, use the template directly.
    if not num_axes:
        pass
    else:  # if num_axes is provided, overwrite the number of axes that are actually instantiated
        # import pdb; pdb.set_trace()
        manipulater_def_dict["actuator_definitions"][0][
            "joint_definitions"
        ] = manipulater_def_dict_template["actuator_definitions"][0][
            "joint_definitions"
        ][
            :num_axes
        ]
        if (
            num_axes == 1
        ):  # if this is 1, we have a planar actuator in the test and need to set the planar_flag accordingly
            manipulater_def_dict["actuator_definitions"][0]["planar_flag"] = 1

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


def test_manipulator_actuation():
    for i in range(1, 4):
        manipulator_actuation_tester(num_axes=i)


@pytest.mark.gui  # annotate it as a "gui" test
def test_manipulator_actuation_gui():
    for i in range(1, 4):
        manipulator_actuation_tester(
            gui=True, total_sim_steps=int(i * 100000), num_axes=i
        )


# test_manipulator_actuation_gui()
