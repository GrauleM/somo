import xml.etree.ElementTree as ET  # good hints on using the XML module: https://stackabuse.com/reading-and-writing-xml-files-in-python/

from somo.sm_link_definition import SMLinkDefinition
from somo.sm_joint_definition import SMJointDefinition


############# utility to suppress printing - temporarily suppresses console output from python and lower level C code when the urdf is built
# todo: write utility to suppress output
# check links below
# https://stackoverflow.com/questions/5081657/how-do-i-prevent-a-c-shared-library-to-print-on-stdout-in-python/17954769#17954769
# https://stackoverflow.com/questions/26475443/python-iterable-and-context-manager

######### utilities for the automated generation of urdf files #########


def clean_xml_indentation(root, level=0):
    """the urdf generator creates a urdf / xml file that is one long string.
    this function adds proper indents to it such that the urdf is human-readable can be loaded easily.

    this function needs to be applied to the root of the xml tree representing the urdf.
    The urdf can then be written to file as tree.write(filename, encoding="utf-8", xml_declaration=True)
    """

    elem = root
    i = "\n" + level * "  "
    if len(elem):
        if not elem.text or not elem.text.strip():
            elem.text = i + "  "
        if not elem.tail or not elem.tail.strip():
            elem.tail = i
        for elem in elem:
            clean_xml_indentation(elem, level + 1)
        if not elem.tail or not elem.tail.strip():
            elem.tail = i
    else:
        if level and (not elem.tail or not elem.tail.strip()):
            elem.tail = i


def spaced_str(input_list):
    """takes a list containing floats and creates a string of its entries such that each float value is separated
    by a space. such a string is often used in urdfs - helper function for urdf creation
    """
    # todo: nicer doc and comment
    assert isinstance(input_list, list), "input_list has to be a list"

    string = ""
    for elem in input_list:
        string = string + str(elem) + " "

    # remove last space
    string = string[:-1]

    return string


def make_inertia_dict(inertia_list):
    # xx todo doc

    assert isinstance(inertia_list, list), "inertia_list has to be a list"
    assert len(inertia_list) == 6, "length of inertia_list has to be 6"

    inertia_dict = {
        "ixx": str(inertia_list[0]),
        "ixy": str(inertia_list[1]),
        "ixz": str(inertia_list[2]),
        "iyy": str(inertia_list[3]),
        "iyz": str(inertia_list[4]),
        "izz": str(inertia_list[5]),
    }

    return inertia_dict


def add_joint(
    robot_root,
    joint_name,
    parent_name,
    child_name,
    joint_definition: SMJointDefinition,
    origin,  # add doc. origin is [origin_x, origin_y, origin_x, origin_r, origin_p, origin_y]; rpy: roll pitch yaw
):
    """adds a joint to the xml tree of robot_root."""

    # todo: add asserts?

    joint = ET.SubElement(
        robot_root, "joint", {"name": joint_name, "type": joint_definition.joint_type}
    )
    joint_parent = ET.SubElement(joint, "parent", {"link": parent_name})
    joint_child = ET.SubElement(joint, "child", {"link": child_name})

    origin_dict = {"xyz": spaced_str(origin[:3]), "rpy": spaced_str(origin[3:])}
    joint_origin = ET.SubElement(joint, "origin", origin_dict)

    if joint_definition.axis:
        axis_dict = {"xyz": spaced_str(joint_definition.axis)}
        joint_axis = ET.SubElement(joint, "axis", axis_dict)

    if joint_definition.limits:
        joint_limits = ET.SubElement(joint, "limit", joint_definition.limits_dict)

    return joint


def add_link(robot_root, link_name, link_definition: SMLinkDefinition, origin):
    """adds a link to the xml tree of robot_root
    link_type: either "box", "cylinder", or "sphere"
    """

    # todo: assertions, in particular that it is a valid SMLinkDefinition? add ability to have a dict or json file that defines an SMLinkDefinition.

    mass_dict = {"value": str(link_definition.mass)}
    origin_dict = {"xyz": spaced_str(origin[:3]), "rpy": spaced_str(origin[3:])}

    link = ET.SubElement(robot_root, "link", {"name": link_name})

    visual = ET.SubElement(link, "visual")
    vis_orig = ET.SubElement(visual, "origin", origin_dict)
    vis_geom = ET.SubElement(visual, "geometry")

    vis_box = ET.SubElement(
        vis_geom, link_definition.shape_type, link_definition.shape_dimensions_dict
    )
    vis_mat = ET.SubElement(visual, "material", {"name": link_definition.material_name})
    vis_mat_col = ET.SubElement(
        vis_mat, "color", {"rgba": spaced_str(link_definition.material_color)}
    )

    collision = ET.SubElement(link, "collision")
    col_orig = ET.SubElement(collision, "origin", origin_dict)
    col_geom = ET.SubElement(collision, "geometry")
    col_box = ET.SubElement(
        col_geom, link_definition.shape_type, link_definition.shape_dimensions_dict
    )

    inertial = ET.SubElement(link, "inertial")
    inner_mass = ET.SubElement(inertial, "mass", mass_dict)
    inner_in = ET.SubElement(inertial, "inertia", link_definition.inertial_value_dict)

    return link


def add_joint_link_pair(
    robot_root,
    parent_link_name,
    child_link_name,
    joint_name,
    link: SMLinkDefinition,
    joint: SMJointDefinition,
    previous_link: SMLinkDefinition,
    previous_joint: SMJointDefinition,
):
    """
    helps making the urdf generation easier to read.
    link being added is always the child of the last link on the robot todo: clarify this statement
    """

    segment_origin = [0, 0, link.height / 2.0, 0, 0, 0]
    joint_origin = [0, 0, previous_link.height, 0, 0, 0]

    # adjust segment and joint origins for shifted neutral axis
    segment_origin = [x - y for x, y, in zip(segment_origin, joint.neutral_axis_offset)]
    joint_origin = [
        x + y - z
        for x, y, z in zip(
            joint_origin, joint.neutral_axis_offset, previous_joint.neutral_axis_offset
        )
    ]

    add_link(robot_root, child_link_name, link, segment_origin)

    add_joint(
        robot_root, joint_name, parent_link_name, child_link_name, joint, joint_origin
    )


def add_empty_link(robot_root, link_name):
    """todo: copy some of the blurb from answer to
    https://answers.ros.org/question/289031/what-is-the-correct-way-to-introduce-a-dummy-link-in-urdf/
    """
    link = ET.SubElement(robot_root, "link", {"name": link_name})
    return link
