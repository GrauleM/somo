import json
from typing import List, Dict, Union
from somo.utils import spaced_str, make_inertia_dict, dict_from_file


class SMLinkDefinition:
    """
    SMLinkDescription is correct upon instantiation.

    Example json representation:
    link_example.json # todo update example presentation
    {
        shape_type:         xx finish,
        dimensions:         xx finish,
        mass:               xx,
        inertial_values:    xx,
        material_color:     xx,
        material_name:      ,
    }

    """

    def __init__(
        self,
        shape_type: str,
        dimensions: [Union[float, int]],
        mass: Union[float, int],
        inertial_values: [Union[float, int]],
        material_color: [Union[float, int]],
        material_name: str,
        origin_offset: [Union[float, int]] = None,
    ):
        """
        Args:
            shape_type:         xx
            dimensions:         xx.
            mass:               xx.
            inertial_values:    xx.
            material_color:     xx
                                  xxxxxxx more .
            material_name:      xx.
        """

        # assert that all arguments are of the right type. may not be necessary to do this, but won't hurt...
        assert isinstance(shape_type, str), f"type(shape_type) is not a string"
        assert isinstance(dimensions, List), f"type(dimensions) is not a List"
        for element in dimensions:
            assert isinstance(
                element, (float, int)
            ), f"type(element in dimensions) is not a float or int"
        assert isinstance(mass, (float, int)), f"type(mass) is not a float or int"
        assert isinstance(inertial_values, List), f"type(inertial_values) is not a List"
        assert len(inertial_values) == 6, f"length(inertial_values) has to be 6"
        for element in inertial_values:
            assert isinstance(
                element, (float, int)
            ), f"type(element in inertial_values) is not a float or int"
        assert isinstance(material_color, List), f"type(material_color) is not a List"
        for element in material_color:
            assert isinstance(
                element, (float, int)
            ), f"type(element in material_color) is not a float or int"
        assert isinstance(material_name, str), f"type(material_name) is not a string"

        if origin_offset:
            assert isinstance(origin_offset, List), f"type(origin_offset) is not a List"
            assert len(origin_offset) == 6, f"length(origin_offset) has to be 6"
            for element in origin_offset:
                assert isinstance(
                    element, (float, int)
                ), f"type(element in origin_offset) is not a float or int"
        else:
            origin_offset = [0, 0, 0, 0, 0, 0]

        # assert that the specified shape and dimensions match and create shape_dimensions_dict & height attribute
        if shape_type == "box" or shape_type == "stadium":
            assert (
                len(dimensions) == 3
            ), f"dimension must have length 3 for link_type {shape_type}"
            shape_dimensions_dict = {"size": spaced_str(dimensions)}
            height = dimensions[2]
        elif shape_type in ["cylinder", "capsule"]:
            assert (
                len(dimensions) == 2
            ), f"dimension must have length 2 for link_type {shape_type}"
            shape_dimensions_dict = {
                "length": str(dimensions[0]),
                "radius": str(dimensions[1]),
            }
            height = dimensions[0]
        elif shape_type == "sphere":
            assert (
                len(dimensions) == 1
            ), f"dimension must have length 1 for link_type {shape_type}"
            shape_dimensions_dict = {"radius": str(dimensions[0])}
            height = 2 * dimensions[0]
        else:
            assert False, f"link shape {shape_type} is not defined"

        if origin_offset is None:
            origin_offset = [0, 0, 0, 0, 0, 0]

        self.shape_type = shape_type
        self.dimensions = dimensions
        self.shape_dimensions_dict = shape_dimensions_dict
        self.height = height

        self.mass = mass
        self.inertial_values = inertial_values
        self.inertial_value_dict = make_inertia_dict(inertial_values)

        self.material_name = material_name
        self.material_color = material_color
        self.origin_offset = origin_offset

        # assert that all required attributes are present. may not be necessary to do this, but won't hurt...
        required_attributes = [
            "shape_type",
            "dimensions",
            "shape_dimensions_dict",
            "mass",
            "inertial_values",
            "inertial_value_dict",
            "material_name",
            "material_color",
            "origin_offset",
        ]

        for a in required_attributes:
            assert hasattr(self, a), f"attribute {a} is missing"

    def reduce_height(self, height_fraction):

        if self.shape_type == "box" or self.shape_type == "stadium":
            self.dimensions = [
                self.dimensions[0],
                self.dimensions[1],
                self.dimensions[2] * height_fraction,
            ]
            self.shape_dimensions_dict = {"size": spaced_str(self.dimensions)}
            self.height = self.dimensions[2]
        elif self.shape_type in ["cylinder", "capsule"]:
            self.dimensions = [self.dimensions[0] * height_fraction, self.dimensions[1]]
            self.shape_dimensions_dict = {
                "length": str(self.dimensions[0]),
                "radius": str(self.dimensions[1]),
            }
            self.height = self.dimensions[0]
        elif self.shape_type == "sphere":
            self.dimensions = [self.dimensions[0] * height_fraction]
            self.shape_dimensions_dict = {"radius": str(self.dimensions[0])}
            self.height = 2 * self.dimensions[2]
        else:
            assert False, f"link reduction is not defined for shape {self.shape_type}"

    @staticmethod
    def assert_required_fields(dict_definition: dict):
        required_fields = [
            "shape_type",
            "dimensions",
            "mass",
            "inertial_values",
            "material_color",
            "material_name",
        ]

        for field_name in required_fields:
            assert (
                field_name in dict_definition
            ), f"Field '{field_name}' is missing in link definition."

    @staticmethod
    def from_json(json_file_path: str) -> "SMLinkDefinition":
        with open(json_file_path) as file:
            json_dict = json.load(file)
            SMLinkDefinition.assert_required_fields(json_dict)
            return SMLinkDefinition(**json_dict)

    def to_json(self):
        raise NotImplementedError

    @staticmethod
    def from_file(file_path: str) -> "SMLinkDefinition":
        in_dict = dict_from_file(file_path)
        SMLinkDefinition.assert_required_fields(in_dict)
        return SMLinkDefinition(**in_dict)
