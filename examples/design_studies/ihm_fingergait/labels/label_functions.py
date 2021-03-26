# Be sure to run this file from the "region_of_acquisition" folder
#     cd examples/region_of_acquisition
#
import numpy as np
import pandas as pd


def default(objectpose, **kwargs):
    """Default label function"""

    return height_check(objectpose, **kwargs)


# Helper
def save_raw_data(objectpose, actuation):
    pass


def height_check(objectpose, **kwargs):
    """Label the grasp successful based on a height threshold."""

    if "min_height" in kwargs:
        min_height = kwargs.get("min_height")
    else:
        min_height = 0.9
    z_pos = objectpose["posZ"].to_list()
    if z_pos[-1] > min_height:
        return 1
    else:
        return 0


def bounding_box(objectpose, **kwargs):
    """Label the trial successful if the object is within a boudning box at the end"""
    bound_x = [-0.05, 0.05]
    bound_y = [-0.05, 0.05]
    bound_z = [0.04, 0.20]

    if "bound_x" in kwargs:
        bound_x = kwargs.get("bound_x")
    if "bound_y" in kwargs:
        bound_x = kwargs.get("bound_y")
    if "bound_z" in kwargs:
        bound_x = kwargs.get("bound_z")

    if "global_scale" in kwargs:
        global_scale = kwargs.get("global_scale")
        for idx, _ in enumerate(bound_x):
            bound_x[idx] = bound_x[idx] * global_scale
            bound_y[idx] = bound_y[idx] * global_scale
            bound_z[idx] = bound_z[idx] * global_scale

    last_pose = objectpose.iloc[-1]

    x_inside = bound_x[0] <= last_pose["posX"] <= bound_x[1]
    y_inside = bound_y[0] <= last_pose["posY"] <= bound_y[1]
    z_inside = bound_z[0] <= last_pose["posZ"] <= bound_z[1]
    if x_inside and y_inside and z_inside:
        return 1

    else:
        return 0


def final_pose(objectpose, **kwargs):
    """Return the final pose"""
    last_pose = objectpose.iloc[-1][
        ["posX", "posY", "posZ", "eulerX", "eulerY", "eulerZ"]
    ]
    return last_pose.values.tolist()


def std_dev_abs(objectpose, **kwargs):
    """Return the standard deviation over the entire run"""
    pose_only = objectpose[["posX", "posY", "posZ", "eulerX", "eulerY", "eulerZ"]].abs()
    return pose_only.std().values.tolist()


def max_cycle(actuation):
    max_val = float(actuation["cycles_" + "fingers"].max())
    return max_val


def max_cycle_complete(actuation, **kwargs):
    if "value" in kwargs:
        value = kwargs.get("value")
    else:
        value = 9

    max_val = float(actuation["cycles_" + "fingers"].max())

    if max_val == value:
        return 1
    else:
        return 0


def grasp_after_prefix(actuation, contact, **kwargs):
    if "bodyUniqueIdA" in kwargs:
        fingerIDs = kwargs.get("bodyUniqueIdA")
    else:
        fingerIDs = [1, 2, 3, 4]

    if "bodyUniqueIdB" in kwargs:
        objectIDs = kwargs.get("bodyUniqueIdB")
    else:
        objectIDs = [6]

    if "contact_threshold" in kwargs:
        contact_threshold = kwargs.get("contact_threshold")
    else:
        contact_threshold = 1

    prefix_times = actuation[actuation["cycles_" + "fingers"] == -2]
    end_prefix = prefix_times.iloc[-1]["timeStamp"]

    contacts_prefix = contact[contact.timeStamp == end_prefix]

    finger_condition = [False] * len(contacts_prefix)
    for fingerID in fingerIDs:
        conditionA = contacts_prefix.bodyUniqueIdA == fingerID
        conditionB = contacts_prefix.bodyUniqueIdB == fingerID
        finger_condition = finger_condition | conditionA | conditionB

    object_condition = [False] * len(contacts_prefix)
    for objectID in objectIDs:
        conditionA = contacts_prefix.bodyUniqueIdA == objectID
        conditionB = contacts_prefix.bodyUniqueIdB == objectID
        object_condition = object_condition | conditionA | conditionB

    contacts_prefix = contacts_prefix[finger_condition & object_condition]

    num_fingers_in_contact = len(contacts_prefix)

    if num_fingers_in_contact >= contact_threshold:
        return True
    else:
        return False


def cycle_magnitude(objectpose, actuation, **kwargs):
    """Calculate the average magnitude of motion during cycles"""
    df = objectpose[["timeStamp", "posX", "posY", "posZ", "eulerX", "eulerY", "eulerZ"]]
    df = df.assign(cycles=actuation["cycles_" + "fingers"].values)

    # Combine data by cycle
    df = df[df["cycles"] >= 0]
    df = df.sort_values(["cycles", "timeStamp"])

    for cycle_curr in df["cycles"].unique():
        times_curr = df[df.cycles == cycle_curr].values
        df[df.cycles == cycle_curr] = times_curr - times_curr[0, :]

    df["timeStamp"] = np.round(df["timeStamp"].values, 2)
    del df["cycles"]

    group = df.groupby("timeStamp")
    means = np.abs(group.mean().values)
    std = group.std().values
    # times = group.groups.keys()

    if len(means) > 0:
        max_vals = np.max(means, axis=0)
        max_idx = np.argmax(means, axis=0)
        max_std = std[max_idx, range(len(max_idx))]
        out = {"mean": max_vals.tolist(), "std": max_std.tolist()}

    else:
        out = {"mean": None, "std": None}

    return out


def mean_link(objectpose, contact=None):
    """Label the grasp type using which part of the fingers are in contact with the object"""

    posZ = objectpose["posZ"].to_list()

    if contact is not None:
        # Get the contacts between fingers and the object at the final timestep
        last_time = contact["stepCount"].iloc[-1]
        last_contacts = contact[contact.stepCount == last_time]
        last_contacts = last_contacts[
            ((last_contacts.bodyUniqueIdA == 1) | (last_contacts.bodyUniqueIdA == 2))
            & (last_contacts.bodyUniqueIdB == 3.0)
        ]

        # Get the average contact location
        links = last_contacts["linkIndexA"].to_list()
        if len(links) > 0:
            mean_link = np.mean(links) + 1
        else:
            mean_link = 0
    else:
        mean_link = 0

    # If the object was picked up
    if posZ[-1] > 0.1:
        return int(
            mean_link
        )  # The success label is the average contact location wit the object.
    # Otherwise, fail
    else:
        return 0


# Define max magnitude functions
def max_magnitude_z(objectpose):
    z_pos_max = objectpose["posZ"].max()
    return float(z_pos_max)


def max_magnitude_x(objectpose):
    pos_max = objectpose["posX"].max()
    return float(pos_max)


def max_magnitude_y(objectpose):
    pos_max = objectpose["posY"].max()
    return float(pos_max)


def max_magnitude_orix(objectpose):
    pos_max = objectpose["eulerX"].max()
    return float(pos_max)


def max_magnitude_oriy(objectpose):
    pos_max = objectpose["eulerY"].max()
    return float(pos_max)


def max_magnitude_oriz(objectpose):
    pos_max = objectpose["eulerZ"].max()
    return float(pos_max)


# Define average function
def average_all(objectpose, actuation_df):
    x_pos = objectpose["posX"].mean()
    y_pos = objectpose["posY"].mean()
    z_pos = objectpose["posZ"].mean()
    return [x_pos, y_pos, z_pos]


def average_all(objectpose, actuation_df):
    x_pos = objectpose["posX"].mean()
    y_pos = objectpose["posY"].mean()
    z_pos = objectpose["posZ"].mean()
    return [x_pos, y_pos, z_pos]


def mean_link_4finger(objectpose, contact=None):
    """Label the grasp type using which part of the fingers are in contact with the object"""

    posZ = objectpose["posZ"].to_list()

    if contact is not None:
        # Get the contacts between fingers and the object at the final timestep
        last_time = contact["stepCount"].iloc[-1]
        last_contacts = contact[contact.stepCount == last_time]
        last_contacts = last_contacts[
            (
                (last_contacts.bodyUniqueIdA == 1)
                | (last_contacts.bodyUniqueIdA == 2)
                | (last_contacts.bodyUniqueIdA == 3)
                | (last_contacts.bodyUniqueIdA == 4)
            )
            & (last_contacts.bodyUniqueIdB == 5)
        ]

        # Get the average contact location
        links = last_contacts["linkIndexA"].to_list()
        if len(links) > 0:
            mean_link = np.mean(links)
        else:
            mean_link = 0
    else:
        mean_link = 0

    # If the object was picked up
    if posZ[-1] > 0.1:
        return int(
            mean_link
        )  # The success label is the average contact location wit the object.
    # Otherwise, fail
    else:
        return 0


def grasp_types(objectpose, contact=None):
    """Label the grasp type as power, fingertip, or pinch"""

    posZ = objectpose["posZ"].to_list()
    eulerX = objectpose["eulerX"].to_list()
    eulerY = objectpose["eulerY"].to_list()
    eulerZ = objectpose["eulerZ"].to_list()

    rot_end = max(np.abs([eulerX[-1], eulerY[-1], eulerZ[-1]]))

    if contact is not None:
        # Get the contacts between fingers and the object at the final timestep
        last_time = contact["stepCount"].iloc[-1]
        last_contacts = contact[contact.stepCount == last_time]
        last_contacts = last_contacts[
            ((last_contacts.bodyUniqueIdA == 1) | (last_contacts.bodyUniqueIdA == 2))
            & (last_contacts.bodyUniqueIdB == 3.0)
        ]

        # Get the average contact location
        links = last_contacts["linkIndexA"].to_list()
        if len(links) > 0:
            mean_link = np.mean(links)
        else:
            mean_link = 0
    else:
        mean_link = 0

    # TODO: Make a better success label scheme like this:
    # Check which finger link the object is touching, and use it to label the grasp
    #   0 = Fail
    #   1 = Power grasp (object touching toward the base of the finger)
    #   2 = Pinch grasp (object touching toward the tip of the finger)
    #   3 = Fingertip grasp (object touches only the fingertip) (NOT USED YET)

    # Using a simple cutoff after a link number to distinguish between power and pinch grasps for now
    # (also fingertip grasps are just counted as pinch grasps for now)
    link_cutoff = 6

    # If the object was picked up
    if posZ[-1] > 0.1:

        # If the contact points are toward the fingertips - pinch
        if mean_link >= link_cutoff:
            return 2
        # If the contact points are toward the base - power
        elif mean_link < link_cutoff:
            return 1

    # Otherwise, fail
    else:
        return 0
