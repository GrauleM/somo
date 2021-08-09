import xml.etree.ElementTree as ET  # good hints on using the XML module: https://stackabuse.com/reading-and-writing-xml-files-in-python/

from somo.sm_link_definition import SMLinkDefinition
from somo.sm_joint_definition import SMJointDefinition
from somo.sm_actuator_definition import SMActuatorDefinition
from somo.sm_manipulator_definition import SMManipulatorDefinition

from somo.utils import spaced_str, clean_xml_indentation

import numpy as np

import copy
import pdb

# TODO: URGENT: error in stadium geometry when using cylinders with uneven dimensions


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
    previous_link: SMLinkDefinition,  # xx todo: add none default with proper typing
    previous_joint: SMJointDefinition,  # xx todo: add none default with proper typing
):
    """
    helps making the urdf generation easier to read.
    the link being added is always the child of the last link on the robot todo: clarify this statement
    """

    if previous_link is None:
        segment_origin = [0, 0, 0, 0, 0, 0]
        joint_origin = [
            0,
            0,
            0,
            0,
            0,
            0,
        ]  # xx todo: see if this can be cleaned up - maybe we can avoid the previous link height
    else:
        segment_origin = [0, 0, link.height / 2.0, 0, 0, 0]
        joint_origin = [
            0,
            0,
            previous_link.height,
            0,
            0,
            0,
        ]  # xx todo: see if this can be cleaned up - maybe we can avoid the previous link height

    # adjust segment and joint origins for shifted neutral axis
    segment_origin = [
        x - y + u
        for x, y, u in zip(
            segment_origin, joint.neutral_axis_offset, link.origin_offset
        )
    ]
    if previous_joint is None:
        joint_origin = [x + y for x, y in zip(joint_origin, joint.neutral_axis_offset)]
    else:
        joint_origin = [
            x + y - z
            for x, y, z in zip(
                joint_origin,
                joint.neutral_axis_offset,
                previous_joint.neutral_axis_offset,
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


# todo: creation of base or links is not handled correctly for spherical links. add warning. fix eventually.
def create_cmassembly_urdf(
    base_links: [SMLinkDefinition],  # todo: fix typing hints
    manipulator_definition_pairs: [(SMManipulatorDefinition, [float])],
    assembly_name: str = "cm_assembly",
):
    """creates a manipulator. xx todo: add more documentation"""

    # todo: empty base link gives warning ": No inertial data for link, using mass=1, localinertiadiagonal = 1,1,1, identity local inertial frame" - fix by adding very light base link?

    # create the robot
    robot = ET.Element("robot", {"name": assembly_name})

    # start with an empty base link, as this is considered good urdf practice
    base_link_name = "base_link"

    add_empty_link(robot_root=robot, link_name=base_link_name)

    # xx todo: try to remove the need for this fake base link by restructuring 'add_join_link_pair)
    # pick some other link of the robot and make the base link 1000x lighter and less inertia
    # base_link = copy.copy(
    #     manipulator_definition_pairs[0][0].actuator_definitions[0].link_definition
    # )
    # base_link.height = 0.0
    # base_link.dimensions = [x / 1000 for x in base_link.dimensions]
    # base_link.mass = 0.0
    # base_link.inertial_values = [x / 1000 for x in base_link.inertial_values]

    # add all base elements
    parent_name = base_link_name
    if base_links is not None:
        for i, link_def in enumerate(base_links):
            child_name = "base_link" + str(i)
            joint_to_add = SMJointDefinition(joint_type="fixed")
            link_to_add = copy.copy(link_def)
            joint_name = parent_name + "_to_" + child_name + "_fixed"

            add_joint_link_pair(
                robot_root=robot,
                parent_link_name=parent_name,
                child_link_name=child_name,
                joint_name=joint_name,
                link=link_to_add,
                joint=joint_to_add,
                previous_link=None,  # xx todo: have it automatically track 'previous_link' with self....
                previous_joint=None,
            )

    for manip_nr, manipulator_offset_pair in enumerate(manipulator_definition_pairs):

        # link each manipulator to the base
        child_name = base_link_name
        total_segment_counter = 0  # counts the total number of actuator segments (excl. base) that have been added to the current manipulator (across all actuators)

        previous_link = None
        previous_joint = None

        manipulator_definition = manipulator_offset_pair[0]
        manipulator_offset = manipulator_offset_pair[1]
        roll, pitch, yaw = (
            manipulator_offset[3],
            manipulator_offset[4],
            manipulator_offset[5],
        )
        roll_rot = np.array(
            [
                [1, 0, 0],
                [0, np.cos(roll), -np.sin(roll)],
                [0, np.sin(roll), np.cos(roll)],
            ]
        )
        pitch_rot = np.array(
            [
                [np.cos(pitch), 0, np.sin(pitch)],
                [0, 1, 0],
                [-np.sin(pitch), 0, np.cos(pitch)],
            ]
        )
        yaw_rot = np.array(
            [[np.cos(yaw), -np.sin(yaw), 0], [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]]
        )

        rot = np.matmul(np.matmul(roll_rot, pitch_rot), yaw_rot)
        z_vec = np.array([[0], [0], [1]])
        manip_principle_axis = np.matmul(rot, z_vec)

        for actuator_definition, actuator_nr in zip(
            manipulator_definition.actuator_definitions,
            range(manipulator_definition.n_act),
        ):

            for segment_nr in range(
                actuator_definition.n_segments + 1
            ):  # xx todo: enable varying cross-sections/properties - link_description could be a generator here

                if (
                    total_segment_counter == 0
                ):  # the first joint connects to the robot base - let's make it a fixed joint/segment
                    # joint_to_add = SMJointDefinition(joint_type="fixed")
                    joint_to_add = copy.copy(actuator_definition.joint_definitions[0])
                    ax_str = ""
                    previous_joint = joint_to_add
                    previous_link = copy.copy(actuator_definition.link_definition)
                    # previous_link = base_link # this somehow doesnt work either...
                    # this causes an error
                    # previous_joint = None
                    # previous_link = None
                else:

                    if actuator_definition.planar_flag or segment_nr % 2:
                        ax_str = "_ax0"
                        joint_to_add = copy.copy(
                            actuator_definition.joint_definitions[0]
                        )

                    else:
                        ax_str = "_ax1"
                        joint_to_add = copy.copy(
                            actuator_definition.joint_definitions[1]
                        )

                # finalize all the naming
                segment_name = (
                    "Man"
                    + str(manip_nr)
                    + "Act"
                    + str(actuator_nr)
                    + "Seg"
                    + str(segment_nr)
                )
                parent_name = child_name
                child_name = segment_name
                joint_name = parent_name + "_to_" + child_name + ax_str

                # if the shape type is 'box','cylinder',or 'sphere': the link can be added directly from the link_definition
                if actuator_definition.link_definition.shape_type in [
                    "box",
                    "cylinder",
                    "sphere",
                ]:
                    link_to_add = copy.copy(actuator_definition.link_definition)

                # if the shape type is ''stadium' or 'capsule': a link needs to be added that has the right base shape
                elif actuator_definition.link_definition.shape_type in ["stadium"]:
                    link_to_add = copy.copy(actuator_definition.link_definition)
                    link_to_add.shape_type = "box"

                elif actuator_definition.link_definition.shape_type in ["capsule"]:
                    link_to_add = copy.copy(actuator_definition.link_definition)
                    link_to_add.shape_type = "cylinder"

                if segment_nr == 0:
                    link_to_add.reduce_height(height_fraction=0.5)
                    joint_to_add.joint_type = "fixed"
                    # add a correction in the position offset to account for the effect of the euler angles
                    pos_correction = 20 * np.squeeze(
                        np.matmul(rot, z_vec) * link_to_add.height
                    )  # correct for the offset from the euler angles
                    pos_correction = [0, 0, 0]
                    correction = [
                        pos_correction[0],
                        pos_correction[1],
                        pos_correction[2],
                        0,
                        0,
                        0,
                    ]
                    # import pdb
                    # pdb.set_trace()
                    joint_to_add.neutral_axis_offset = [
                        x - y - z
                        for x, y, z in zip(
                            joint_to_add.neutral_axis_offset,
                            manipulator_offset,
                            correction,
                        )
                    ]
                elif segment_nr == actuator_definition.n_segments:
                    link_to_add.reduce_height(height_fraction=0.5)

                add_joint_link_pair(
                    robot_root=robot,
                    parent_link_name=parent_name,
                    child_link_name=child_name,
                    joint_name=joint_name,
                    link=link_to_add,
                    joint=joint_to_add,
                    previous_link=previous_link,  # xx todo: have it automatically track 'previous_link' with self....
                    previous_joint=previous_joint,
                )

                # add the additional links required to achieve the stadium shape # todo: add and test capsule shape?
                if (
                    not segment_nr == 0
                    and actuator_definition.link_definition.shape_type
                    in ["stadium", "capsule"]
                ):

                    if actuator_definition.link_definition.shape_type in ["stadium"]:

                        helper_shape = "cylinder"
                        # helper_shape_height = actuator_definition.link_definition.dimensions[1]

                        if joint_to_add.axis == [1, 0, 0]:
                            additional_link_origin_rotation = [0, 0, 0, 0, np.pi / 2, 0]
                            dim = [
                                actuator_definition.link_definition.dimensions[0],
                                actuator_definition.link_definition.dimensions[1] / 2.0,
                            ]
                        elif joint_to_add.axis == [0, 1, 0]:
                            additional_link_origin_rotation = [0, 0, 0, np.pi / 2, 0, 0]
                            dim = [
                                actuator_definition.link_definition.dimensions[1],
                                actuator_definition.link_definition.dimensions[0] / 2.0,
                            ]
                        else:
                            assert f"shape type 'stadium' only works with joint axis with direction [1,0,0] or [0,1,0] that do not have an offset from neutral axis"  # xx todo: add this as an assertion when defining the actator

                        helper_shape_height = dim[1] * 2

                    elif actuator_definition.link_definition.shape_type in ["capsule"]:
                        dim = [actuator_definition.link_definition.dimensions[1]]
                        helper_shape = "sphere"
                        additional_link_origin_rotation = [0, 0, 0, 0, 0, 0]
                        helper_shape_height = (
                            2 * actuator_definition.link_definition.dimensions[1]
                        )

                    helper_offset = [
                        x - y - z
                        for x, y, z in zip(
                            actuator_definition.link_definition.origin_offset,
                            additional_link_origin_rotation,
                            [0, 0, helper_shape_height / 2.0, 0, 0, 0],
                        )
                    ]

                    additional_link_to_add = SMLinkDefinition(
                        shape_type=helper_shape,
                        dimensions=dim,
                        mass=actuator_definition.link_definition.mass / 1000.0,
                        # somewhat arbitrarily picked something much smaller for mass and inertia
                        inertial_values=[
                            x / 1000.0
                            for x in actuator_definition.link_definition.inertial_values
                        ],
                        material_color=actuator_definition.link_definition.material_color,
                        material_name=actuator_definition.link_definition.material_name,
                        origin_offset=helper_offset,
                    )

                    additional_joint_to_add = SMJointDefinition(
                        joint_type="fixed",
                    )

                    add_joint_link_pair(
                        robot_root=robot,
                        parent_link_name=parent_name,
                        child_link_name=child_name + "helper_shape",
                        joint_name=joint_name + "helper_shape",
                        link=additional_link_to_add,
                        joint=additional_joint_to_add,
                        previous_link=previous_link,  # xx todo: have it automatically track 'previous_link' with self. ...
                        previous_joint=previous_joint,
                    )

                total_segment_counter += 1

                previous_joint = joint_to_add
                previous_link = link_to_add

        if manipulator_definition.tip_definition:
            tip_link_name = "Man" + str(manip_nr) + "_tip"
            parent_name = child_name
            child_name = tip_link_name
            joint_name = parent_name + "_to_" + child_name

            # make a fixed joint
            tip_joint_definition = SMJointDefinition(joint_type="fixed")

            add_joint_link_pair(
                robot_root=robot,
                parent_link_name=parent_name,
                child_link_name=child_name,
                joint_name=joint_name,
                link=manipulator_definition.tip_definition,
                joint=tip_joint_definition,
                previous_link=previous_link,  # xx todo: have it automatically track 'previous_link' with self....
                previous_joint=previous_joint,
            )

    # xx TODO: the saving below this comment here needs to be fixed for the assembly
    # save everything to file. formats the xml so it's easier to read, then write the file
    urdf_filename = assembly_name + ".urdf"
    tree = ET.ElementTree(robot)
    clean_xml_indentation(robot)
    tree.write(urdf_filename, encoding="utf-8", xml_declaration=True)

    return urdf_filename  # xx todo: also return start number of join indexes? do this somewhere
