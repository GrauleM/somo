import pybullet as p

from contextlib import contextmanager
import ctypes
import io
import os, sys
import tempfile

import json
import yaml


############# utility to suppress printing - temporarily suppresses console output from python and lower level C code
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


######### utilities to simplify setting up everything in pybullet #########


def load_constrained_urdf(urdf_filename, startPos, startOrn, physicsClient):
    bodyUniqueId = p.loadURDF(
        urdf_filename, startPos, startOrn, physicsClientId=physicsClient
    )

    constraintUniqueId = p.createConstraint(
        parentBodyUniqueId=bodyUniqueId,
        parentLinkIndex=-1,  # -1 for base
        childBodyUniqueId=-1,
        childLinkIndex=-1,
        jointType=p.JOINT_FIXED,
        jointAxis=[0, 0, 0],
        parentFramePosition=[0, 0, 0],
        childFramePosition=startPos,
        parentFrameOrientation=p.getQuaternionFromEuler([0, 0, 0]),
        childFrameOrientation=startOrn,
    )
    return bodyUniqueId, constraintUniqueId


def dict_from_file(file_path):
    with open(file_path) as file:
        if file_path.endswith(".json"):
            in_dict = json.load(file)
        elif file_path.endswith(".yaml"):
            in_dict = yaml.load(file)
        else:
            print("Please use a json or yaml file")
            raise
    return in_dict
