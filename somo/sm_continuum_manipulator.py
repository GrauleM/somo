from typing import List, Dict, Union

from somo.sm_manipulator_definition import SMManipulatorDefinition
from somo.create_manipulator_urdf import create_manipulator_urdf

import pybullet as p
import numpy as np

# from somo.utils import stdout_redirected
import io

import pdb


# TODO: thoughts on where to compute non-linear passive torque: consider adding other actuators that are non-linear?
#  try keep one very simple version/class. avoid making things overly complicated if it's not needed.

# todo: consider removing this? not sure it will be used; depends on whether non-linear joint-angle vs restoring force relationship will be needed
def apply_actuation_torques(
    bodyIndex, jointIndices, actuationTorques, passiveTorqueFn, **kwargs
):
    try:  # xx todo: do this better with asserts
        actuationTorques = float(
            actuationTorques
        )  # in case actuationTorque is a numpy float and not a regular float
    except:
        pass

    if type(actuationTorques) is int or type(actuationTorques) is float:
        act_torque = actuationTorques
        extended_actuationTorques = [act_torque for x in range(len(jointIndices))]
        actuationTorques = extended_actuationTorques
    elif len(actuationTorques) == 1:
        extended_actuationTorques = [
            actuationTorques[0] for x in range(len(jointIndices))
        ]
        actuationTorques = extended_actuationTorques

    for jointId, torque in zip(jointIndices, actuationTorques):
        jointState = p.getJointState(bodyIndex, jointId)

        passive_torque = passiveTorqueFn(jointState, **kwargs)

        p.setJointMotorControl2(
            bodyIndex=bodyIndex,
            jointIndex=jointId,
            controlMode=p.TORQUE_CONTROL,
            force=passive_torque + torque,
        )


def disable_joint_controllers(bodyIndex, jointIndices, forces):
    for jointId, force in zip(jointIndices, forces):
        p.setJointMotorControl2(
            bodyIndex=bodyIndex,
            jointIndex=jointId,
            controlMode=p.VELOCITY_CONTROL,
            force=force,
        )


class SMContinuumManipulator:
    """
    assumes linear relationship between joint angle difference from neutral position and restoring spring force
    """

    def __init__(
        self,
        manipulator_definition: Union[SMManipulatorDefinition, Dict, str],
    ):

        # assert that all arguments are of the right type.
        assert isinstance(
            manipulator_definition, (SMManipulatorDefinition, Dict, str)
        ), f"the manipulator_definition has to be an SMManipulatorDefinition"

        if isinstance(manipulator_definition, Dict):
            SMManipulatorDefinition.assert_required_fields(manipulator_definition)
            manipulator_definition = SMManipulatorDefinition(**manipulator_definition)
        elif isinstance(manipulator_definition, str):
            manipulator_definition.from_file(manipulator_definition)

        self.manipulator_definition = manipulator_definition

        # create the urdf
        self.manipulator_definition.urdf_filename = create_manipulator_urdf(
            self.manipulator_definition
        )

        # turn of velocity_control

        self.actuators = list(
            range(self.manipulator_definition.n_act)
        )  # actuator_names #actuator_names are act0,act1,... with numbering starting from base. xx fix this comment, i think actuator names are 0,1,...
        self.actuator_joints = (
            {}
        )  # a dict that returns all the jointIds belonging to each of the actuators
        self.actuatorAxis_joints = (
            {}
        )  # a dict that returns all the joints belonging to a specific axis of an actuator
        self.bodyUniqueId = None
        self.baseConstraintUniqueId = None

        self.flexible_joint_indices = (
            []
        )  # list that contains the indices of all flexible joints
        self.joint_controller_limit_forces = (
            []
        )  # list that contains the joint controller limit force for each flexible joints; same order of joints as self.flexible_joint_indices

        # todo consider deleting the two lines below if not needed
        # self.main_link_Ids = [] # contains the ids of the main segment links (not the helper links used in the capsule and stadium shapes)
        # self.arc_length_by_link_Ids = [] # arc_length_by_link_Ids[i] contains the arc length associated with link main_link_Ids[i] (measured to the center of mass of the link)
        self.linkId_to_arcLength = (
            []
        )  # linkId_to_arcLength[i] contains the summed (cumsum) arc_length from base to the center of link i

        # this is for linear passive torque only! by default, everything is initialized as linear springs.
        # self.passive_torque_fn = compute_linSpring_joint_torque  # xx todo: expand and refactor do make it easy to use other fns. maybe this should be a new class?
        # todo: give the option to pass in the torque function?

        self.instantiated = False

    def load_to_pybullet(
        self, baseStartPos, baseStartOrn, baseConstraint, physicsClient, flags=None
    ):
        """adds the manipulator to the simulation defined in physicsCient. if fixedBase is 1 or True, a constraint
        is added that holds the base in place"""
        # todo: documentation

        assert baseConstraint in [
            "constrained",
            "free",
            "static",
        ], f"{baseConstraint} has to be either 'constrained','free', or 'static'"  # xx todo: make enum?

        if baseConstraint == "static":
            static_base = 1
            constrained_base = 0

        elif baseConstraint == "constrained":
            print(
                "you provided a constrained base, but not a static base. if you do not anticipate moving the manipulator by modifying it's base constraint within the simulation loop, "
                "it is highly recommended to use a static base instead for improved numerical stability."
            )
            static_base = 0
            constrained_base = 1
        else:
            static_base = 0
            constrained_base = 0

        if flags:
            all_flags = flags | p.URDF_MAINTAIN_LINK_ORDER | p.URDF_MERGE_FIXED_LINKS
        else:
            all_flags = p.URDF_MAINTAIN_LINK_ORDER | p.URDF_MERGE_FIXED_LINKS

        # clear all the lists and dicts that get populated (or appended to) when the manipulator is loaded into bullet
        # to avoid that they grow each time a given manipulator is loaded into a physics server
        self.actuator_joints = {}
        self.actuatorAxis_joints = {}
        self.flexible_joint_indices = []
        self.joint_controller_limit_forces = []
        self.linkId_to_arcLength = []

        # pdb.set_trace()
        # todo: write utility to suppress this output for merging link...
        # we dont want to print the info about merging the links... just clutters the console
        self.bodyUniqueId = p.loadURDF(
            self.manipulator_definition.urdf_filename,
            baseStartPos,
            baseStartOrn,
            physicsClientId=physicsClient,
            useFixedBase=static_base,
            flags=all_flags,
        )
        # pdb.set_trace()

        num_joints = p.getNumJoints(
            bodyUniqueId=self.bodyUniqueId, physicsClientId=physicsClient
        )
        # todo: comment to explain logic
        for joint in range(num_joints):

            info = p.getJointInfo(
                bodyUniqueId=self.bodyUniqueId,
                jointIndex=joint,
                physicsClientId=physicsClient,
            )

            joint_name = info[1].decode("UTF-8")
            joint_type = info[2]

            # build up the dict that maps all the joints belonging to a specific axis for each actuator
            if (
                joint_type is not p.JOINT_FIXED
            ):  # make sure the joint type is not fixed, bcs we don't apply torques to fixed joints

                # todo: add some more clarifying comments
                self.flexible_joint_indices.append(joint)

                joint_name_split = joint_name.split("_")

                ax = joint_name_split[-1].replace("ax", "")
                act = (joint_name_split[0]).split("Seg")[0].replace("act", "")

                if act in self.actuator_joints:
                    self.actuator_joints[act].append(joint)

                else:
                    self.actuator_joints.update({act: []})
                    self.actuator_joints[act].append(joint)

                if act in self.actuatorAxis_joints:
                    if ax in self.actuatorAxis_joints[act]:
                        self.actuatorAxis_joints[act][ax].append(joint)
                    else:
                        self.actuatorAxis_joints[act].update({ax: []})
                        self.actuatorAxis_joints[act][ax].append(joint)

                else:
                    self.actuatorAxis_joints.update({act: {}})
                    self.actuatorAxis_joints[act].update({ax: []})
                    self.actuatorAxis_joints[act][ax].append(joint)

                limit_force = (
                    self.manipulator_definition.actuator_definitions[int(act)]
                    .joint_definitions[int(ax)]
                    .joint_control_limit_force
                )

                self.joint_controller_limit_forces.append(limit_force)

        cummulative_length = 0.0

        linkId = 0
        for actuator_nr in range(len(self.manipulator_definition.actuator_definitions)):
            for segment_nr in range(
                self.manipulator_definition.actuator_definitions[actuator_nr].n_segments
            ):

                cummulative_length += (
                    self.manipulator_definition.actuator_definitions[
                        actuator_nr
                    ].link_definition.height
                    / 2
                )
                self.linkId_to_arcLength.append(cummulative_length)
                cummulative_length += (
                    self.manipulator_definition.actuator_definitions[
                        actuator_nr
                    ].link_definition.height
                    / 2
                )

                linkId += 1

                # if the shape is stadium or capsule, we also need to add an entry for the helper shapes
                # todo: check if this is even still necessary after merging the fixed links; clean up
                if 0:
                    if self.manipulator_definition.actuator_definitions[
                        actuator_nr
                    ].link_definition.shape_type in ["stadium", "capsule"]:
                        self.linkId_to_arcLength.append(cummulative_length)
                        linkId += 1

        disable_joint_controllers(
            self.bodyUniqueId,
            self.flexible_joint_indices,
            self.joint_controller_limit_forces,
        )

        if constrained_base:

            constId = p.createConstraint(
                parentBodyUniqueId=self.bodyUniqueId,
                parentLinkIndex=-1,  # -1 for base
                childBodyUniqueId=-1,
                childLinkIndex=-1,
                jointType=p.JOINT_FIXED,
                jointAxis=[0, 0, 0],
                parentFramePosition=[0, 0, 0],
                childFramePosition=baseStartPos,
                parentFrameOrientation=p.getQuaternionFromEuler([0, 0, 0]),
                childFrameOrientation=baseStartOrn,
            )

        else:
            constId = None

        self.baseConstraintUniqueId = constId  # todo: unify naming convention

        # todo: set collision filter for the capsule and stadium geometry to make sure that there is no collision with the parent link and the helper geometry (ball/circle) when self-collision is on
        self.instantiated = True

        return

    def print_actuators(self):
        print("not implemented")

    # def apply_torques_to_joints(self, joint_indices, torques):
    #     # todo: consider adding this outside of class?
    #     assert isinstance(joint_indices, list), f"joint_indices has to be a list of floats"
    #     assert isinstance(torques, list), f"torques has to be a list of floats"
    #     assert len(joint_indices) == len(torques), f"joint_indices and torques have to have same length"
    #
    #     for jointId, torque in zip(joint_indices, torques):
    #         jointState = p.getJointState(self.bodyUniqueId, jointId)
    #         jointPos = jointState[0]
    #
    #         passive_torque = -spring_const * (jointPos - offset)
    #
    #         p.setJointMotorControl2(bodyIndex=self.bodyUniqueId,
    #                                 jointIndex=jointId,
    #                                 controlMode=p.TORQUE_CONTROL,
    #                                 force=passive_torque + torque)

    def apply_passive_spring_torques(self, actuator_nrs, axis_nrs):

        assert isinstance(
            actuator_nrs, list
        ), f"actuator_nrs has to be a list of floats"
        assert isinstance(axis_nrs, list), f"actuator_nrs has to be a list of floats"
        assert len(actuator_nrs) == len(
            axis_nrs
        ), f"actuator_nrs and axis_nrs have to have same length"

        self.apply_actuation_torques(
            actuator_nrs=actuator_nrs,
            axis_nrs=axis_nrs,
            actuation_torques=[0 for x in actuator_nrs],
            apply_zero_torques=False,
        )

    # todo: disentangle apply_actuation_torque and apply_passive_torque and make sure infinite recursion cannot occur.

    # todo: type annotation
    def apply_actuation_torques(
        self, actuator_nrs, axis_nrs, actuation_torques, apply_zero_torques=True
    ):
        """applies the actuation torque to all the joints that belong to axis in actuator.
        the same torque is applied to each of the joints.
        """

        assert isinstance(actuator_nrs, list), f"actuator_nrs has to be a list of ints"
        assert isinstance(axis_nrs, list), f"axis_nrs has to be a list of ints"
        assert isinstance(
            actuation_torques, list
        ), f"actuation_torques has to be a list of floats"
        assert len(actuator_nrs) == len(
            axis_nrs
        ), f"actuator_nrs and axis_nrs have to have same length"
        assert len(actuator_nrs) == len(
            actuation_torques
        ), f"actuator_nrs and actuation_torques have to have same length"

        addressed_actuator_axis_pairs = []

        for actuator_nr, axis_nr, act_torque in zip(
            actuator_nrs, axis_nrs, actuation_torques
        ):

            if self.manipulator_definition.actuator_definitions[
                actuator_nr
            ].planar_flag:
                assert (
                    axis_nr <= 0.0
                ), f"planar actuators only have one addressable axis (with axis_nr = 0), but you provided axis_nr = {axis_nr}"

            offset = (
                self.manipulator_definition.actuator_definitions[actuator_nr]
                .joint_definitions[axis_nr]
                .joint_neutral_position
            )
            spring_const = (
                self.manipulator_definition.actuator_definitions[actuator_nr]
                .joint_definitions[axis_nr]
                .spring_stiffness
            )

            joint_indices = self.actuatorAxis_joints[str(actuator_nr)][str(axis_nr)]

            # pybullet expects floats, not np.floats
            if isinstance(act_torque, (np.float, np.float32, np.float64)):
                act_torque = float(act_torque)

            extended_actuation_torque = [act_torque for x in range(len(joint_indices))]

            for jointId, torque in zip(joint_indices, extended_actuation_torque):
                jointState = p.getJointState(self.bodyUniqueId, jointId)
                jointPos = jointState[0]

                passive_torque = -spring_const * (jointPos - offset)

                p.setJointMotorControl2(
                    bodyIndex=self.bodyUniqueId,
                    jointIndex=jointId,
                    controlMode=p.TORQUE_CONTROL,
                    force=passive_torque + torque,
                )

            addressed_actuator_axis_pairs.append((actuator_nr, axis_nr))

        actuators_to_address = []
        axes_to_address = []

        # apply zero torque to all joints that did not have a torque applied yet
        if apply_zero_torques:
            for act in self.actuatorAxis_joints:
                for axis in self.actuatorAxis_joints[act]:
                    addressed = False

                    for addressed_tuple in addressed_actuator_axis_pairs:
                        if int(addressed_tuple[0]) == int(act) and int(
                            addressed_tuple[1]
                        ) == int(axis):
                            addressed = True
                            break

                    if not addressed:
                        actuators_to_address.append(int(act))
                        axes_to_address.append(int(axis))

            self.apply_passive_spring_torques(
                actuator_nrs=actuators_to_address, axis_nrs=axes_to_address
            )

        # some entanglement / race condition with calling recursively. TODO: untangle this

        # todo: see if apply_actuation_torque can be done more efficiently; or prettier;

    def apply_joint_torques(self, actuator_nrs, axis_nrs, actuation_torques):
        """
        actuation_torques is a list of torques for each entry in actuator_nrs, axis_nrs. each joint gets the corresponding torque
        """
        raise NotImplementedError

    def update_joint_control_limitForce(
        self, jointIds, limit_forces
    ):  # xx todo: consider renaming

        assert (
            False
        ), f"update_joint_control_limitForce has not been tested sufficiently"
        assert isinstance(
            limit_forces, float
        ), f"the joint_controller_limit_force has to be a float"

        if isinstance(
            actuation_torques, np.ndarray
        ):  # pybullet expects floats, not np.floats
            actuation_torques = list(actuation_torques)

        # todo: assert same length of inputs?
        # todo: clean up debugging stuff

        if isinstance(jointIds, list):
            pass
        else:
            jointIds = [jointIds]

        if isinstance(limit_forces, list):
            pass
        else:
            limit_forces = [limit_forces]

        for jointId, limit_force in zip(jointIds, limit_forces):
            index = self.flexible_joint_indices.index(jointId)
            self.joint_controller_limit_forces[index] = limit_force

        disable_joint_controllers(
            bodyIndex=self.bodyUniqueId, jointIndices=jointIds, forces=limit_forces
        )

    def set_contact_property(self, property_dict):
        # todo: assert that dict only has valid keys
        # todo: test this with the snake example
        for i in range(p.getNumJoints(self.bodyUniqueId)):
            p.changeDynamics(
                self.bodyUniqueId,
                i,
                **property_dict,
            )

    def get_backbone_position(self, s):

        # todo: does not work properly yet if s = manipulator_length (related to need to making this more precise)
        i = 0
        while (
            self.linkId_to_arcLength[i] < s
        ):  # get the segment id for which the arc length is s
            i += 1

        # todo: make more precise; don't just return the center of the link into which the arc length end falls
        linkState = p.getLinkState(bodyUniqueId=self.bodyUniqueId, linkIndex=i)
        link_pos = linkState[0]

        return link_pos

    # lists all link positions; todo: make sure this does not return the positions of helper shapes
    def get_backbone_positions(self):
        positions = []
        for ind in range(len(self.linkId_to_arcLength)):
            linkState = p.getLinkState(bodyUniqueId=self.bodyUniqueId, linkIndex=ind)
            link_pos = linkState[0]
            positions.append(link_pos)
        return positions

    # lists all the angles of the flexible joins. todo: be careful when using this in non-planar manipulators
    def get_backbone_angles(self):
        angles = []
        for ind in self.flexible_joint_indices:
            jointState = p.getJointState(self.bodyUniqueId, ind)
            jointPos = jointState[0]
            angles.append(jointPos)
        return angles

    def get_backbone_curvatures(self):
        print("WARNING: offset by a constant factor")  # todo: fix this
        curvatures = []
        for ind in self.flexible_joint_indices:
            jointState = p.getJointState(self.bodyUniqueId, ind)
            segment_length = 1  # TODO: add correct segment length
            angle = jointState[0]
            curvature = np.cos(angle / 2) / (segment_length / 2)
            curvatures.append(curvature)
        return curvatures

    # lists all link positions; todo: make sure this does not return the positions of helper shapes
    def get_backbone_link_positions_and_velocities(self):
        positions = []
        velocities = []
        for ind in range(len(self.linkId_to_arcLength)):
            linkState = p.getLinkState(
                bodyUniqueId=self.bodyUniqueId, linkIndex=ind, computeLinkVelocity=1
            )
            link_pos = linkState[0]
            link_vel = linkState[6]
            positions.append(link_pos)
            velocities.append(link_vel)
        return positions, velocities
