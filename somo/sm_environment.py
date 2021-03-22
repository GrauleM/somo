"""

example serialization:

{

  "experiment_name"   : "IHM_example",
  "manipulator_details" : [
                          ["manipulator_definition.json", [0, 0, 2], [0, 0, 0], "static"],
                          ["manipulator_definition.json", [0, 0, 0], [0, 0, 0], "static"]
                        ],
  "pybullet_urdfs"    : [
                          ["plane.urdf", [], []],
                        ],
  "additional_urdfs"  : [
                          ["complex_palm.urdf", [0.3, 0.3, 2], [0, 0, 0], "constrained"],
                          ["smallSquareBox.urdf", [0.3, 0,3, 3], [0, 0, 0], "free"]
                        ]

}



"""

"""
- folder: experiment_name
    - folder: urdfs
    - folder: definitions
        - file: experiment definition
        - file: manipulator_definition.json (all of them)
    - file: run_exp.py


"""

physicsClient = p.connect(
    p.GUI
)  # p.GUI for graphical, or p.DIRECT for non-graphical version

manipulators = []
for definition_tuple in manipulator_definition_tuple:
    (definition_path, startPos, startOr) = definition_tuple
    definition = SMManipulatorDefinition.from_file(definition_path)
    manipulator = SMContinuumManipulator(definition)
    manipulators.append(manipulator)
    manipulator.load_to_pybullet(
        baseStartPos=startPos,
        baseStartOrn=startOr,
        baseConstraint="static",
        physicsClient=physicsClient,
    )  # todo: something looks funny for constrained and free baseConstraint

import json
from typing import List, Dict, Union
from somo.sm_link_definition import SMLinkDefinition
from somo.sm_actuator_definition import SMActuatorDefinition

from somo.sm_continuum_manipulator import SMContinuumManipulator
from somo.sm_manipulator_definition import SMManipulatorDefinition
from somo.utils import dict_from_file


# todo split: experiment is loaded from an experiment_definition
# xx todo: consider having the experiment definition be one large file with all the dicts in it directly). would this be easier to read?
class SMEnvironmentDefinition:
    def __init__(
        self,
        experiment_name: str,
        manipulator_details: [list],
        pybullet_urdfs: [list],
        additional_urdfs: [list],
    ):
        # todo: all the asserts

        self.experiment_name = experiment_name
        self.manipulator_details = manipulator_details
        self.pybullet_urdfs = pybullet_urdfs
        self.additional_urdfs = additional_urdfs

    @staticmethod
    def from_json(json_file_path: str) -> "SMEnvironmentDefinition":
        # todo: should there be an assert that the file exists?
        with open(json_file_path) as file:
            json_dict = json.load(file)
            SMEnvironmentDefinition.assert_required_fields(
                json_dict
            )  # todo: write asser_required_fields
            return SMEnvironmentDefinition(**json_dict)

    @staticmethod
    def from_file(file_path: str) -> "SMEnvironmentDefinition":
        in_dict = dict_from_file(file_path)
        SMEnvironmentDefinition.assert_required_fields(in_dict)
        return SMEnvironmentDefinition(**in_dict)

    # todo: put somewhere else?
    def load_exp_to_pybullet(self):
        manipulators = []
        for manipulator_detail in self.manipulator_details:
            (definition_path, startPos, startOr) = manipulator_detail
            definition = SMManipulatorDefinition.from_file(definition_path)
            manipulator = SMContinuumManipulator(definition)
            manipulator.load_to_pybullet(
                baseStartPos=startPos,
                baseStartOrn=startOr,
                baseConstraint="static",
                physicsClient=physicsClient,
            )  # todo: something looks funny for constrained and free baseConstraint
            manipulators.append(manipulator)

        return manipulators
