import json
from typing import List, Dict, Union
from somo.utils import (
    spaced_str,
    make_inertia_dict,
    create_scaled_shape_dimension_dict,
    dict_from_file,
)


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
        visual_geometry_scaling_factor=1.0,
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
                visual_geometry_scaling_factor scales the visual geometry relative to the contact geometry

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

        _, contact_shape_dimensions_dict, height = create_scaled_shape_dimension_dict(
            shape_type, dimensions
        )
        _, visual_shape_dimensions_dict, _ = create_scaled_shape_dimension_dict(
            shape_type, dimensions, height_scaling_factor=visual_geometry_scaling_factor
        )

        if origin_offset is None:
            origin_offset = [0, 0, 0, 0, 0, 0]

        self.shape_type = shape_type
        self.dimensions = dimensions
        self.contact_shape_dimensions_dict = contact_shape_dimensions_dict
        self.visual_shape_dimensions_dict = visual_shape_dimensions_dict
        self.height = height
        self.visual_geometry_scaling_factor = visual_geometry_scaling_factor
        assert (
            visual_geometry_scaling_factor == 1 or visual_geometry_scaling_factor == 1.0
        ), "fpr now, visual_geometry_scaling_factor has to be 1.0"
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
            "contact_shape_dimensions_dict",
            "visual_shape_dimensions_dict",
            "mass",
            "inertial_values",
            "inertial_value_dict",
            "material_name",
            "material_color",
            "origin_offset",
        ]

        for a in required_attributes:
            assert hasattr(self, a), f"attribute {a} is missing"

    def reduce_height(self, height_scaling_factor):

        (
            self.dimensions,
            self.contact_shape_dimensions_dict,
            self.height,
        ) = create_scaled_shape_dimension_dict(
            self.shape_type,
            self.dimensions,
            height_scaling_factor=height_scaling_factor,
        )
        self.visual_shape_dimensions_dict = (
            self.contact_shape_dimensions_dict
        )  # todo: change this - this should technically be adjusted for the overall relative sccaling between visual and contact geometries

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
