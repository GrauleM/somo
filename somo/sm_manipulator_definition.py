import json
from typing import List, Dict, Union
from somo.sm_link_definition import SMLinkDefinition
from somo.sm_actuator_definition import SMActuatorDefinition
from somo.utils import dict_from_file


class SMManipulatorDefinition:
    def __init__(
        self,
        n_act: Union[float, int],
        base_definition: Union[SMLinkDefinition, type(None)],
        actuator_definitions: [Union[SMActuatorDefinition, Dict, str]],
        # todo: double check whether it is necessary/helpful to handle also dicts and str here
        manipulator_name: str,
        tip_definition: Union[SMLinkDefinition, type(None)] = None,
        urdf_filename: Union[str, type(None)] = None,
        tip_definitions: Union[List, type(None)] = None,
    ):

        # assert that all arguments are of the right type. may not be necessary to do this, but won't hurt...
        assert isinstance(n_act, (float, int)), f"n_act has to be float or int"
        assert isinstance(
            base_definition, (SMLinkDefinition, Dict, str, type(None))
        ), f"type(base_definition) has to be either a LinkDefinition (or dict/str describing one), or NoneType"
        assert isinstance(
            actuator_definitions, List
        ), f"type(actuator_definitions) has to be List or list of ActuatorDefinition"
        for element in actuator_definitions:
            assert isinstance(
                element, (SMActuatorDefinition, Dict, str)
            ), f"elements in actuator_definitions have to be of type SMActuatorDefinition, dict, or a string pointing to a json file describing an SMActuatorDefinition"

        assert isinstance(
            tip_definition, (SMLinkDefinition, Dict, str, type(None))
        ), f"type(tip_definition) has to be either a LinkDefinition (or dict/str describing one), or NoneType"

        assert isinstance(manipulator_name, str), f"Manipulator_name has to be a string"
        assert isinstance(
            urdf_filename, (str, type(None))
        ), f"urdf_filename has to be a string or None"

        # additional assertions:
        assert n_act > 0.0, f"n_act has to be larger than 0"
        assert (
            len(actuator_definitions) == n_act
        ), f"number of provided actuator definitions has to match n_act"

        # if the actuator definitions are Dicts or str, instantiate the SMActuatorDefinition from them.
        new_actuator_definitions = []
        for actuator_definition in actuator_definitions:
            if isinstance(actuator_definition, Dict):
                SMActuatorDefinition.assert_required_fields(actuator_definition)
                actuator_definition = SMActuatorDefinition(**actuator_definition)
            elif isinstance(actuator_definition, str):
                actuator_definition = SMActuatorDefinition.from_file(
                    actuator_definition
                )
            new_actuator_definitions.append(actuator_definition)

        # if the tip and base definitions are Dicts or str, instantiate the SMLinkDefinitions from them.
        if isinstance(base_definition, Dict):
            SMLinkDefinition.assert_required_fields(base_definition)
            base_definition = SMLinkDefinition(**base_definition)
        elif isinstance(base_definition, str):
            if (
                base_definition == ""
            ):  # an empty string is interpreted as None / no base
                base_definition = None
            else:
                base_definition = SMLinkDefinition.from_file(base_definition)

        def check_and_convert_tip_defonition(tip_definition):
            if isinstance(tip_definition, Dict):
                SMLinkDefinition.assert_required_fields(tip_definition)
                tip_definition = SMLinkDefinition(**tip_definition)
            elif isinstance(tip_definition, str):
                if (
                    tip_definition == ""
                ):  # an empty string is interpreted as None / no dedicated tip
                    tip_definition = None
                else:
                    tip_definition = SMLinkDefinition.from_file(tip_definition)

            if tip_definition:
                assert tip_definition.shape_type in [
                    "box",
                    "cylinder",
                    "sphere",
                ], f"tip shape_type has to be box, cylinder, or sphere - others are not implemented yet in urdf generation."
            return tip_definition

        tip_definition = check_and_convert_tip_defonition(tip_definition)

        if tip_definition:
            assert isinstance(
                tip_definitions, type(None)
            ), f"if tip_definition is provided, tip_definitions has to be None."
            tip_definitions = [tip_definition]
        elif tip_definitions:
            assert isinstance(
                tip_definition, type(None)
            ), f"if tip_definitions is provided, tip_definition has to be None."
            assert isinstance(
                tip_definitions, list
            ), f"tip_definitions has to be a list of Dicts or SMLinkDefinition or file path"

            tip_definitions = [
                check_and_convert_tip_defonition(tip_definition)
                for tip_definition in tip_definitions
            ]

        # allow for multiple tips all
        # todo: do same as above (i.e., loading for different representations) for the base_definition and tip_definitino

        # todo: consider adding a conversion from empty string to None for urdf_filename and manipulator_name (meaning that urdf_filename="" is converted to urdf_filename = None)
        # todo: assert that urdf filename ends with .urdf

        self.n_act = n_act
        self.actuator_definitions = new_actuator_definitions
        self.base_definition = base_definition
        self.tip_definitions = tip_definitions  # xx todo: check if this should always be turned into SMLinkDefinition thingy. i think it should be
        self.manipulator_name = manipulator_name
        self.urdf_filename = urdf_filename

        # assert that all required attributes are present. may not be necessary to do this, but won't hurt...
        required_attributes = [
            "n_act",
            "base_definition",
            "actuator_definitions",
            "manipulator_name",
            "urdf_filename",
            "tip_definitions",
        ]
        for a in required_attributes:
            assert hasattr(self, a), f"attribute {a} is missing"

    @staticmethod
    def assert_required_fields(dict_definition: dict):
        required_fields = [
            "n_act",
            "base_definition",
            "actuator_definitions",
            "manipulator_name",
        ]

        for field_name in required_fields:
            assert (
                field_name in dict_definition
            ), f"Field '{field_name}' is missing in manipulator definition."

        assert ("tip_definitions" in dict_definition) or (
            "tip_definition" in dict_definition
        ), f"at least one of tip_definitions and tip_definition has to be in the manipulator definition."

    @staticmethod
    def from_json(json_file_path: str) -> "SMManipulatorDefinition":
        # todo: should there be an assert that the file exists?
        with open(json_file_path) as file:
            json_dict = json.load(file)
            SMManipulatorDefinition.assert_required_fields(json_dict)
            return SMManipulatorDefinition(**json_dict)

    def to_json(self):
        raise NotImplementedError

    @staticmethod
    def from_file(file_path: str) -> "SMManipulatorDefinition":
        in_dict = dict_from_file(file_path)
        SMManipulatorDefinition.assert_required_fields(in_dict)
        return SMManipulatorDefinition(**in_dict)
