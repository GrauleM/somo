import json
from typing import List, Dict, Union
from somo.utils import spaced_str, make_inertia_dict, dict_from_file


class SMJointDefinition:
    """
    SMJointDefinition is correct upon instantiation.

    Example json representation:
    link_example.json # todo fix
    {
       xx
    }

    """

    def __init__(
        self,
        joint_type: str,
        axis: Union[type(None), List] = None,
        limits: Union[List, type(None)] = None,
        spring_stiffness: Union[float, int, type(None)] = None,
        joint_neutral_position: Union[float, int, type(None)] = None,
        neutral_axis_offset: [Union[float, int, type(None)]] = None,
        joint_control_limit_force: [Union[float, int, type(None)]] = None,
    ):
        """
        Args:
            joint_type:          xx
            axis:                xx.
            limits:              xx.
            spring_stiffness:    xx.
            joint_neutral_position:    xx.
        """

        # assert that all arguments are of the right type. may not be necessary to do this, but won't hurt...
        assert joint_type in [
            "revolute",
            "continuous",
            "prismatic",
            "fixed",
            "floating",
            "planar",
        ], f"{joint_type} is not a valid joint type in URDFs."
        assert joint_type in [
            "revolute",
            "continuous",
            "fixed",
        ], f"so far, joints of type {joint_type} are not supported"

        assert isinstance(axis, (list, type(None))), f"axis has to be a list or None"
        if isinstance(axis, List):
            assert len(axis) == 3, f"len(axis) has to be 3"
            for element in axis:
                assert isinstance(
                    element, (float, int)
                ), f"elements in axis have to be float or int"
        # assign the default axis
        if isinstance(
            axis, type(None)
        ):  # xx todo: correctly assign default values everywhere
            axis = [1, 0, 0]

        if joint_type in ["revolute", "continuous"]:
            assert axis, f"specify an axis for joints of type {joint_type}"

        assert isinstance(
            limits, (List, type(None))
        ), f"type(limits) has to be either a List or NoneType"
        if isinstance(limits, List):
            assert len(limits) == 4, f"len(limits) has to be 4"
            for element in limits:
                assert isinstance(
                    element, (float, int)
                ), f"elements in limits have to be float or int"

        if joint_type == "revolute":
            assert isinstance(limits, List), f"revolute joints require specified limits"

        assert isinstance(
            spring_stiffness, (float, int, type(None))
        ), f"spring_stiffness has to be float or int"

        if neutral_axis_offset:
            assert isinstance(
                neutral_axis_offset, List
            ), f"type(neutral_axis_offset) is not a List"
            assert len(neutral_axis_offset) == 6, f"length(origin_offset) has to be 6"
            for element in neutral_axis_offset:
                assert isinstance(
                    element, (float, int)
                ), f"type(element in origin_offset) is not a float or int"

            for element in neutral_axis_offset[3:]:
                # only positional offset in the neutral axis is implemented
                assert (
                    int(element) == 0
                ), f"neutral axis offset currently only implemented for positional offsets"

            if int(sum(i[0] * i[1] for i in zip(neutral_axis_offset[:3], axis))) != 0:
                print(
                    f"Warning: neutral_axis_offset xyz ({neutral_axis_offset[:3]}) is not perpendicular to axis ({axis}). is this intentional?"
                )

        if joint_control_limit_force:
            assert isinstance(
                joint_control_limit_force, (float, int)
            ), f"type(joint_control_limit_force) is not a float or int"

        # additional assertions
        # todo: think of more assertions. e.g., enforce meaningful joint limits?
        # todo add assertion: axis should either have length 3

        # create the limits_dict if limits is not None
        if limits:  #
            limits_dict = {
                "lower": str(limits[0]),
                "upper": str(limits[1]),
                "effort": str(limits[2]),
                "velocity": str(limits[3]),
            }
            self.limits_dict = limits_dict

        if neutral_axis_offset is None:
            neutral_axis_offset = [0, 0, 0, 0, 0, 0]

        if joint_control_limit_force is None:
            joint_control_limit_force = 0.0

        if joint_control_limit_force > 10.0:
            print(
                "consider a smaller force limit on the joint controller"
            )  # xx todo: check what is a good value here; consider adding recommendation to go larger than 0 on limit force

        # finally assign everything else
        self.joint_type = joint_type
        self.axis = axis
        self.limits = limits
        self.spring_stiffness = spring_stiffness  # todo: extend everything such that spring stiffness can be obtained from a generator (as a function of joint position in the actuator)
        self.joint_neutral_position = joint_neutral_position
        self.neutral_axis_offset = neutral_axis_offset
        self.joint_control_limit_force = joint_control_limit_force

        # assert that all required attributes are present. may not be necessary to do this, but won't hurt... # todo: consider removing
        required_attributes = [
            "joint_type",
            "axis",
            "limits",
            "spring_stiffness",
            "neutral_axis_offset",
        ]

        for a in required_attributes:
            assert hasattr(
                self, a
            ), f"attribute {a} is missing"  # xx todo: also add assertion that they aren't none? not sure this is necessary
            # xx todo: add setting relevant values from none to 0 (e.g. neutral_position)

    @staticmethod
    def assert_required_fields(dict_definition: dict):
        required_fields = [
            "joint_type",
            "axis",
            "limits",
            "spring_stiffness",
        ]

        for field_name in required_fields:
            assert (
                field_name in dict_definition
            ), f"Field '{field_name}' is missing in joint definition."

    @staticmethod
    def from_json(json_file_path: str) -> "SMJointDefinition":
        with open(json_file_path) as file:
            json_dict = json.load(file)
            SMJointDefinition.assert_required_fields(json_dict)
            return SMJointDefinition(**json_dict)

    def to_json(self):
        raise NotImplementedError
        # xx TODO...

    @staticmethod
    def from_file(file_path: str) -> "SMJointDefinition":
        in_dict = dict_from_file(file_path)
        SMJointDefinition.assert_required_fields(in_dict)
        return SMJointDefinition(**in_dict)
