import json
from typing import List, Dict, Union
from somo.sm_link_definition import SMLinkDefinition
from somo.sm_joint_definition import SMJointDefinition
from somo.utils import dict_from_file


class SMActuatorDefinition:
    def __init__(
        self,
        actuator_length: Union[float, int],
        n_segments: int,
        link_definition: Union[SMLinkDefinition, Dict, str],
        # todo: consider adding the capability of making this a list for multiple links
        joint_definitions: [Union[SMJointDefinition, Dict, str]],
        planar_flag: Union[bool, int],
        # indicates whether the actuator should be treated as quasiplanar (True) or not (False); meaning whether the actuator should have 1 or 2 deformable axes
    ):

        # assert that all arguments are of the right type. may not be necessary to do this, but won't hurt...
        assert isinstance(
            actuator_length, (float, int)
        ), f"actuator_length has to be float or int"
        assert isinstance(
            n_segments, (float, int)
        ), f"n_segments has to be float or int"
        assert isinstance(
            link_definition, (SMLinkDefinition, Dict, str)
        ), f"type(link_definition) has to be a SMLinkDefinition or a dict"
        assert isinstance(
            joint_definitions, List
        ), f"type(joint_definitions) has to be List of SMJointDefinition"
        for element in joint_definitions:
            assert isinstance(
                element, (SMJointDefinition, Dict, str)
            ), f"elements in joint_definitions have to be of type SMJointDefinition or dict"
        assert isinstance(
            planar_flag, (bool, int)
        ), f"planar_flag has to be a bool or int"  # xx todo: make sure bool is serializable

        # additional assertions:
        if planar_flag == 1:
            assert (
                len(joint_definitions) == 1
            ), f"currently, quasi-planar actuators with planar_flag=1 only support a single entry in joint_definitions - this joint_definition defines identical joints along the actuator"
        elif planar_flag == 2:
            assert (
                len(joint_definitions) > 1
            ), f"for planar actuators with planar_flag==2, you must speficy multiple joint_definitions with identical axis"
            first_axis = joint_definitions[0]["axis"]
            for joint_nr in range(1, len(joint_definitions)):
                axis = joint_definitions[joint_nr]["axis"]
                assert (
                    first_axis == axis
                ), f"for planar actuators with planar_flag==2, all joints must have the same axis."
            assert (
                len(joint_definitions) == n_segments
            ), f"for planar actuators with planar_flag==2, you must specify exactly as many joint definitions as n_segments in your actuator"

        elif not planar_flag:
            assert (
                len(joint_definitions) == 2 or len(joint_definitions) == 3
            ), f"exactly two or three joint_definitions are required for non-planar actuators (one for each bending axis and optionally one more for torsion)"

        assert n_segments > 0.0, f"n_segments has to be larger than 0"

        # if the link or joint definitions are Dicts, instantiate the SMLinkDefinition/SMJointDefinition from them. todo: loading form json file path?
        if isinstance(link_definition, Dict):
            SMLinkDefinition.assert_required_fields(link_definition)
            link_definition = SMLinkDefinition(**link_definition)
        elif isinstance(link_definition, str):
            link_definition = SMLinkDefinition.from_file(link_definition)

        new_joint_definitions = []
        for joint_definition in joint_definitions:
            if isinstance(joint_definition, Dict):
                SMJointDefinition.assert_required_fields(joint_definition)
                joint_definition = SMJointDefinition(**joint_definition)
            elif isinstance(joint_definition, str):
                joint_definition = SMJointDefinition.from_file(joint_definition)

            new_joint_definitions.append(joint_definition)

        assert (
            n_segments * link_definition.height == actuator_length
        ), f"the prescribed links don't add up to an actuator of the given total length"

        self.actuator_length = actuator_length
        self.n_segments = n_segments  # xx todo: change n_segments to n_joints? half segments i added change the number of segments...
        self.link_definition = link_definition
        self.joint_definitions = new_joint_definitions
        self.planar_flag = planar_flag

        # assert that all required attributes are present. may not be necessary to do this, but won't hurt...
        required_attributes = [
            "actuator_length",
            "n_segments",
            "link_definition",
            "joint_definitions",
            "planar_flag",
        ]
        for a in required_attributes:
            assert hasattr(self, a), f"attribute {a} is missing"

    @staticmethod
    def assert_required_fields(dict_definition: dict):
        required_fields = [
            "actuator_length",
            "n_segments",
            "link_definition",
            "joint_definitions",
            "planar_flag",
        ]

        for field_name in required_fields:
            assert (
                field_name in dict_definition
            ), f"Field '{field_name}' is missing in actuator definition."

    @staticmethod
    def from_json(json_file_path: str) -> "SMActuatorDefinition":
        with open(json_file_path) as file:
            json_dict = json.load(file)

            return SMActuatorDefinition(**json_dict)

    def to_json(self):
        raise NotImplementedError

    @staticmethod
    def from_file(file_path: str) -> "SMActuatorDefinition":
        in_dict = dict_from_file(file_path)
        SMActuatorDefinition.assert_required_fields(in_dict)
        return SMActuatorDefinition(**in_dict)
