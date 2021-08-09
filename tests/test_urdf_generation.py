import pybullet as p
import pybullet_data
import numpy as np
import os
import sys
import pytest

path = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, path)
from somo.sm_manipulator_definition import SMManipulatorDefinition

from somo.sm_manipulator_definition import SMManipulatorDefinition
from somo.sm_actuator_definition import SMActuatorDefinition
from somo.sm_link_definition import SMLinkDefinition
from somo.sm_joint_definition import SMJointDefinition
from somo.sm_continuum_manipulator import SMContinuumManipulator
from somo.create_manipulator_urdf import create_manipulator_urdf

from pathlib import Path

TEST_UDRF_PATH = os.path.join(Path(__file__).parent, "test.urdf")

standard_link_defitinion_dict = {
    "shape_type": "box",
    "dimensions": [0.1, 0.1, 0.15],
    "mass": 0.350,
    "inertial_values": [1, 0, 0, 1, 0, 1],
    "material_color": [0.3, 1, 0.1, 1.0],
    "material_name": "green",
}


def manipulator_simulation(manipulator_definition, gui=False):
    """
    loads and simulates a manipulator_definition in pybullet
    """

    # create the urdf
    test_urdf = create_manipulator_urdf(manipulator_definition)

    # prepare everything for the physics client
    if gui:
        physicsClient = p.connect(
            p.GUI
        )  # p.GUI for graphical, or p.DIRECT for non-graphical version
        n_steps = 50000

    else:
        physicsClient = p.connect(p.DIRECT)
        n_steps = 10

    p.setGravity(0, 0, -10)
    p.setRealTimeSimulation(
        0
    )  # only if this is set to 0 and the simulation is done with explicit steps will the torque control work correctly
    p.setPhysicsEngineParameter(
        enableFileCaching=0
    )  # xx todo: check if this makes things faster
    time_step = 0.0001
    p.setTimeStep(time_step)

    # load the ground plane into pybullet simulator
    p.setAdditionalSearchPath(
        pybullet_data.getDataPath()
    )  # defines the path used by p.loadURDF

    # load the test urdf into pybullet
    bodyUniqueId = p.loadURDF(test_urdf, physicsClientId=physicsClient, useFixedBase=1)

    # run a few simulation steps
    for i in range(n_steps):
        # todo: change structure such that spring torque is always applied and joints are always turned off automatically.

        # todo: add assertion in apply_actuatorTorque that the actuator_nr is not larger than the number of actuators, and same for axis_nr
        # xx todo: passive_torque should always be applied in every timestep. how can this be done quickly?

        p.stepSimulation()
    p.disconnect(physicsClient)

    # delete the test urdf
    os.remove(test_urdf)


def link_definition_from_shape(shape: str = None):
    """
    creates a link definition dict with the specified shape
    """
    if shape is None or shape == "box":
        shape = "box"

    if shape in ["box", "stadium"]:
        dimension = [0.15, 0.15, 0.05]
    elif shape in ["cylinder", "capsule"]:
        dimension = [0.05, 0.15]

    link_definition_dict = {
        "shape_type": shape,
        "dimensions": dimension,
        "mass": 0.350,
        "inertial_values": [1, 0, 0, 1, 0, 1],
        "material_color": [0.3, 0.1, 1.0, 1.0],
        "material_name": "blue",
    }

    return link_definition_dict


def manipulators_from_links(link_definition_dict_1, link_definition_dict_2):
    """
    helper fn to quickly create a few manipulators with varying joint, base, and tip definitions from two link definitions
    """
    joint_definition_dict_1 = {
        "joint_type": "revolute",
        "axis": [1, 0, 0],
        "limits": [
            -np.pi,
            np.pi,
            100,
            3,
        ],  # TODO: distinguish between hard and soft joint limits
        "spring_stiffness": 1000,
        "joint_neutral_position": 0,
        "neutral_axis_offset": [0, 0.05, 0, 0.0, 0.0, 0.0],
        # todo: adjust joint definition; this should also be normal to the joint axis. add check to rule out non-zero angle offsets (not implemented)
        "joint_control_limit_force": 2.0,
    }

    joint_definition_dict_2 = {
        "joint_type": "revolute",
        "axis": [0, 1, 0],
        "limits": [-np.pi, np.pi, 100, 3],
        "spring_stiffness": 1000,
        "joint_neutral_position": 0,
        "joint_control_limit_force": 2.0,
    }
    actuator_definition_dict1 = {
        "actuator_length": 0.5,
        "n_segments": 10,
        "link_definition": link_definition_dict_1,
        "joint_definitions": [joint_definition_dict_1],
        "planar_flag": 1,
    }
    actuator_definition_dict2 = {
        "actuator_length": 1.5,
        "n_segments": 10,
        "link_definition": link_definition_dict_2,
        "joint_definitions": [joint_definition_dict_1, joint_definition_dict_2],
        "planar_flag": 0,
    }
    tip_definition_dict_sphere = {
        "shape_type": "sphere",
        "dimensions": [0.15],
        "mass": 0.350,
        "inertial_values": [1, 0, 0, 1, 0, 1],
        "material_color": [0.8, 0.8, 0.8, 1.0],
        "material_name": "white",
        "origin_offset": [0, -0.1, -0.3, 0, 0, 0],
    }
    base_definition = {
        "shape_type": "box",
        "dimensions": [0.2, 0.2, 0.3],
        "mass": 1.350,
        "inertial_values": [1, 0, 0, 1, 0, 1],
        "material_color": [0.1, 0.8, 0.1, 1.0],
        "material_name": "green",
    }

    manipulator_definitions = [
        SMManipulatorDefinition(
            n_act=2,
            base_definition=None,
            actuator_definitions=[actuator_definition_dict1, actuator_definition_dict2],
            tip_definition=None,
            manipulator_name="test1",
            urdf_filename=TEST_UDRF_PATH,
        ),
        SMManipulatorDefinition(
            n_act=2,
            base_definition=base_definition,
            actuator_definitions=[actuator_definition_dict1, actuator_definition_dict2],
            tip_definition=tip_definition_dict_sphere,
            manipulator_name="test2",
            urdf_filename=TEST_UDRF_PATH,
        ),
    ]

    return manipulator_definitions


def urdf_generation_tester(shape: str, gui: bool):
    manipulator_definitions = manipulators_from_links(
        link_definition_from_shape(shape=shape), standard_link_defitinion_dict
    )

    for manipulator_definition in manipulator_definitions:
        manipulator_simulation(manipulator_definition, gui)

    return True


def test_urdf_generation():
    gui = False
    for shape in ["box", "cylinder", "stadium", "capsule"]:
        assert urdf_generation_tester(
            shape=shape, gui=gui
        ), f"Couldn't instantiate manipulator for link shape '{shape}'"


@pytest.mark.gui  # annotate it as a "gui" test
def test_urdf_generation_gui():
    gui = True
    for shape in ["box", "cylinder", "stadium", "capsule"]:
        assert urdf_generation_tester(
            shape=shape, gui=gui
        ), f"Couldn't instantiate manipulator for link shape '{shape}'"
