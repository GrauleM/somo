# Be sure to run this file from the "region_of_acquisition" folder
#     cd examples/region_of_acquisition
#
import yaml
import time
import os
import sys
import copy
import pickle
import pybullet as p
import numpy as np
import pandas as pd
import pathos.multiprocessing as mp
from functools import partial
from itertools import repeat
import itertools
import matplotlib.pyplot as plt
from matplotlib.collections import PatchCollection
from matplotlib.patches import Rectangle
from matplotlib import rcParams
import inspect
from natsort import natsorted

from somo.sweep import iter_utils

# from utils import label_functions # Need to fix label functions pipeline to come from example folder


def starmap_with_kwargs(pool, fn, args_iter, kwargs_iter):
    args_for_starmap = zip(repeat(fn), args_iter, kwargs_iter)
    return pool.starmap(apply_args_and_kwargs, args_for_starmap)


def apply_args_and_kwargs(fn, args, kwargs):
    return fn(*args, **kwargs)


def process_run(config_file, label_function=None, kwargs={}):
    """Process one dataset from a config file using the prescribed label function"""
    global_scale = kwargs.get("global_scale")
    label_functions_pkg = kwargs.get("label_functions_pkg")
    data_filenames = kwargs.get("data_filenames")
    plot_raw = kwargs.get("plot_raw", False)
    cycle_key = kwargs.get("cycle_key", None)

    # Get data folder
    config = iter_utils.load_yaml(config_file)

    # Get the global scale
    global_scale = float(config.get("setup", {}).get("global_scale", 1.0))
    if global_scale is not None:
        global_scale = global_scale

    global_scale_inv = 1.0 / global_scale

    # Set up the label functions
    if label_function is None:
        process = config.get("setup", {})
        label_funs_to_get = process.get("label_functions", "default")
    else:
        label_funs_to_get = label_function

    default_fun = getattr(label_functions_pkg, "default")
    default_args = inspect.getfullargspec(default_fun)[0]

    if isinstance(label_funs_to_get, str):
        label_funs_to_get = [label_funs_to_get]

    if isinstance(label_funs_to_get, list):
        methods_to_call = dict()
        for curr_name in label_funs_to_get:
            curr_fun = getattr(label_functions_pkg, curr_name, None)
            if curr_fun is not None:
                args = inspect.getfullargspec(curr_fun)[0]
                methods_to_call[curr_name] = {"function": curr_fun, "args": args}

        if methods_to_call:
            label_function = methods_to_call
        else:
            label_function = {
                "default": {"function": default_fun, "args": default_args}
            }

    else:
        label_function = {"default": {"function": default_fun, "args": default_args}}

    # Get the names of the args specified in the label function
    label_fun_inputs = []
    for label_fun_key in label_function:
        label_fun_inputs.extend(label_function[label_fun_key]["args"])

    label_fun_inputs = list(set(label_fun_inputs))

    # Get file locations
    folder = iter_utils.get_group_folder(config)
    print(folder)
    success_filename = os.path.join(folder, "summary.yaml")

    # Unzip the sweep
    sweep = config["sweep"]
    sweep_vars = []
    sweep_labels = []
    sweep_values = []
    sweep_diffs = []
    sweep_lookup = []
    for param in sweep:
        sweep_vars.append(iter_utils.parse_variable_name(param["variable"]))
        sweep_values.append([])

        if param.get("max", None) is not None:
            if param["num_steps"] > 1:
                sweep_diffs.append(
                    (param["max"] - param["min"]) / (param["num_steps"] - 1)
                )
            else:
                sweep_diffs.append(0.0)

        folder_param = param.get("folder", None)
        if folder_param is not None:
            folder_setup = os.path.join(folder_param, "sweep_values.yaml")
            if os.path.isfile(folder_setup):
                curr_lookup = iter_utils.load_yaml(folder_setup)
                f = {}
                f["names"] = [
                    os.path.join(folder_param, row["name"])
                    for row in curr_lookup["files"]
                ]
                f["values"] = [row["values"] for row in curr_lookup["files"]]

                curr_lookup["files"] = f
                sweep_lookup.append(curr_lookup)
                sweep_labels.append(curr_lookup["variables"])
            else:
                sweep_lookup.append(None)
        else:
            sweep_lookup.append(None)
            sweep_labels.append(
                iter_utils.parse_variable_name(param.get("label", None))
            )

    print(sweep_vars)

    # Get the list of all folders
    run_folders = iter_utils.get_folders(folder)

    # Read in each data file and parse it
    label_vals = {}
    for key in label_function:
        label_vals[key] = []

    num_finger_segs = []

    for curr_folder in run_folders:
        print(curr_folder)
        param_filename = os.path.join(curr_folder, "params.yaml")
        params = iter_utils.load_yaml(param_filename)

        for idx, var in enumerate(sweep_vars):
            val = iter_utils.get_from_dict(params, var)

            if sweep_lookup[idx] is not None:
                try:
                    num_idx = sweep_lookup[idx]["files"]["names"].index(val)
                    val_use = sweep_lookup[idx]["files"]["values"][num_idx]
                except ValueError:
                    val_use = val
            else:
                val_use = val

            sweep_values[idx].append(val_use)

        # Get object position data if needed
        if "objectpose" in label_fun_inputs:
            fields = [
                "timeStamp",
                "objectId",
                "posX",
                "posY",
                "posZ",
                "oriX",
                "oriY",
                "oriZ",
                "oriW",
            ]
            pose_file = os.path.join(curr_folder, data_filenames["objectpose"])
            reader = iter_utils.read_parse_data(pose_file)
            df = reader.make_dataframe(fields)

            for pos in ["posX", "posY", "posZ"]:
                df[pos] = global_scale_inv * df[pos]

            # Get euler angles from the quaternions
            euler = []
            for quaternion in zip(df["oriX"], df["oriY"], df["oriZ"], df["oriW"]):
                # print(quaternion)
                euler.append(p.getEulerFromQuaternion(quaternion))

            euler = np.array(euler)
            euler = np.unwrap(euler, axis=0)
            euler = np.rad2deg(euler)

            df["eulerX"] = euler[:, 0]
            df["eulerY"] = euler[:, 1]
            df["eulerZ"] = -euler[:, 2]

            df_rel = df - df.iloc[0].values.squeeze()

            if plot_raw and cycle_key is not None:
                act_file = os.path.join(curr_folder, data_filenames["actuation"])
                iter_utils.graph_data(
                    df_rel,
                    filename=pose_file,
                    cyc_filename=act_file,
                    cyclic_key=cycle_key,
                )
                # iter_utils.graph_cyclic(df, act_file, cycle_key)

        else:
            df = None

        # Get contact data if needed
        if "contact" in label_fun_inputs:
            filename_contact = os.path.join(curr_folder, data_filenames["contact"])
            if os.path.exists(filename_contact):
                fields = [
                    "timeStamp",
                    "stepCount",
                    "bodyUniqueIdA",
                    "bodyUniqueIdB",
                    "linkIndexA",
                    "linkIndexB",
                ]
                reader = iter_utils.read_parse_data(filename_contact)
                df_contact = reader.make_dataframe(fields)
            else:
                df_contact = None
        else:
            df_contact = None

        # Get actuation data if needed
        if "actuation" in label_fun_inputs:
            filename_actuation = os.path.join(curr_folder, data_filenames["actuation"])
            if os.path.exists(filename_actuation):
                df_actuation = pd.read_pickle(filename_actuation)
                for col in df_actuation.columns.values:
                    if "actuation" in col:
                        df_actuation[col] = pow(global_scale_inv, 2) * df_actuation[col]
            else:
                df_actuation = None
        else:
            df_actuation = None

        # Get the number of finger segments
        calc_file = os.path.join(curr_folder, data_filenames["calculated"])
        calc_params = iter_utils.load_yaml(calc_file)
        num_finger_segs.append(calc_params.get("num_finger_segs", []))

        # package the correct data to give to the label function
        label_fun_send_list = {
            "objectpose": df,
            "contact": df_contact,
            "actuation": df_actuation,
        }

        # Get the labels from the label functions
        for label_fun_key in label_function:
            label_fun_send = dict()
            for key in label_function[label_fun_key]["args"]:
                label_fun_send[key] = label_fun_send_list[key]
            curr_val = label_function[label_fun_key]["function"](**label_fun_send)
            label_vals[label_fun_key].append(curr_val)

        if "save_raw_data" in label_function.keys():
            out = {}
            if df is not None:
                out["objectpose"] = df.to_dict(orient="list")
            if df_contact is not None:
                out["contact"] = df_contact.to_dict(orient="list")
            if df_actuation is not None:
                out["actuation"] = df_actuation.to_dict(orient="list")

            out_file = os.path.join(curr_folder, "raw_data.pkl")
            with open(out_file, "wb") as f:
                pickle.dump(out, f)

    results = dict()
    results["labels"] = label_vals
    results["vars"] = sweep_vars
    results["varlabels"] = sweep_labels
    results["sweep"] = sweep_values
    results["diffs"] = sweep_diffs
    results["num_finger_segs"] = num_finger_segs

    iter_utils.save_yaml(results, success_filename)

    data = flatten_data(results)
    filename, ext = os.path.splitext(success_filename)
    iter_utils.save_yaml(data, filename + "_flattened" + ext)

    return results


def flatten_dict(dd, separator="_", prefix=""):
    return (
        {
            prefix + separator + k if prefix else k: v
            for kk, vv in dd.items()
            for k, v in flatten_dict(vv, separator, kk).items()
        }
        if isinstance(dd, dict)
        else {prefix: dd}
    )


def flatten_data(results):
    # Flatten data in labels
    labels_in_graph = list(results["labels"].keys())
    for label_name in labels_in_graph:
        new_label_keys = []
        data_in = results["labels"][label_name]

        if isinstance(data_in[0], dict):
            for idx, row in enumerate(data_in):
                new_row = flatten_dict(row)
                for key in new_row:
                    new_key = label_name + "_" + key
                    if not results["labels"].get(new_key, False):
                        results["labels"][new_key] = []
                        new_label_keys.append(new_key)

                    results["labels"][new_key].append(new_row[key])

            del results["labels"][label_name]

            labels_in_graph.extend(new_label_keys)
            labels_in_graph.remove(label_name)

    df = pd.DataFrame(results["labels"])

    # Add in variable values.
    varlabels = results["varlabels"]
    print(varlabels)
    for idx, vals in enumerate(results["sweep"]):
        # If there are other dimensions added, add them to the dataframe
        df.loc[:, varlabels[idx]] = vals

    data = df.to_dict(orient="list")
    return data


class DataLabeler:
    def __init__(self, label_functions):
        self.label_functions_pkg = label_functions
        all_filenames = iter_utils.load_yaml("save_paths.yaml")
        self.data_filenames = all_filenames["data"]
        self.global_scale = None

    def set_global_scale(self, scale):
        self.global_scale = float(scale)

    def process_all(self, config_file, label_function=None, **kwargs):
        """Process all datasets within a config file"""

        kwargs["global_scale"] = copy.deepcopy(self.global_scale)
        kwargs["label_functions_pkg"] = self.label_functions_pkg
        kwargs["data_filenames"] = copy.deepcopy(self.data_filenames)

        config = iter_utils.load_yaml(config_file)
        setup = config.get("setup", {})
        slices_2d = setup.get("slices_2d", False)

        base_folder = iter_utils.get_group_folder(config)
        print(base_folder)

        if slices_2d:
            dir_list = os.listdir(base_folder)
            dir_list = natsorted(dir_list)
            folders = [
                os.path.join(base_folder, subdir)
                for subdir in dir_list
                if os.path.isdir(os.path.join(base_folder, subdir))
            ]

            # Create a variable summary for folders
            num_vars_to_use = len(config["sweep"]) - 2
            out = {}
            out["vars"] = []
            out["sweep"] = []

            for idx in range(num_vars_to_use):
                if config["sweep"][idx].get("folder", False):
                    folder_param = config["sweep"][idx].get("folder")
                    folder_setup = os.path.join(folder_param, "sweep_values.yaml")
                    folder_config = iter_utils.load_yaml(folder_setup)
                    out["vars"].extend(folder_config["variables"])
                    for idx, _ in enumerate(folder_config["variables"]):
                        values = []
                        for file_col in folder_config["files"]:
                            values.append(file_col["values"][idx])
                        out["sweep"].append(values)
                elif config["sweep"][idx].get("values", False):
                    values = config["sweep"][idx].get("values")
                    out["vars"].append(config["sweep"][idx].get("label"))
                    out["sweep"].append(values)
                elif config["sweep"][idx].get("max", False):
                    gs_inv = 1.0 / (self.global_scale)
                    maxi = config["sweep"][idx].get("max") * gs_inv
                    mini = config["sweep"][idx].get("min") * gs_inv
                    steps = config["sweep"][idx].get("num_steps")
                    values = np.linspace(mini, maxi, steps).tolist()
                    out["vars"].append(config["sweep"][idx].get("label"))
                    out["sweep"].append(values)

            param_list = itertools.product(*out["sweep"])

            all_permutations = []
            for var in out["vars"]:
                all_permutations.append([])

            for item in param_list:
                for var_idx in range(len(out["vars"])):
                    all_permutations[var_idx].append(item[var_idx])
            out["sweep"] = all_permutations
            iter_utils.save_yaml(out, os.path.join(base_folder, "summary.yaml"))

        else:
            folders = [""]

        # Summarize all the data
        if True:
            parallel = kwargs.get("parallel", True)
            if parallel:
                if "num_processes" in kwargs:
                    num_processes = kwargs.get("num_processes")
                else:
                    num_processes = os.cpu_count()

                all_config_files = []
                for folder in folders:
                    all_config_files.append(
                        os.path.join(base_folder, folder, "config.yaml")
                    )

                pool = mp.Pool(num_processes)
                # args_iter = zip(all_config_files,repeat(label_function))
                # kwargs_iter = repeat(kwargs)

                process_run_simple = partial(
                    process_run, label_function=label_function, kwargs=kwargs
                )
                pool.map(process_run_simple, all_config_files)

            else:

                for folder in folders:
                    new_config_file = os.path.join(base_folder, folder, "config.yaml")
                    print(new_config_file)
                    process_run(new_config_file, label_function, kwargs)


if __name__ == "__main__":
    config_file = "sweeps/grid_design_grasps.yaml"
    labeler = DataLabeler()
    labeler.process_all(config_file)
