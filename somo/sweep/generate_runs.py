# Be sure to run this file from the "region_of_acquisition" folder
#     cd examples/region_of_acquisition
#
import os
import yaml
import numpy as np
import itertools
import copy
from functools import reduce  # forward compatibility for Python 3
import operator

from somo.sweep import iter_utils


class RunGenerator:
    def __init__(self):
        save_paths = iter_utils.load_yaml("save_paths.yaml")
        self.base_folder = save_paths.get("save_path", "data")

    def from_file(self, config_file, todo_filename="runs_todo.yaml"):
        """Generate a set of runs from a config file"""

        # Read in the configuration and get relavant parameters
        self.config = iter_utils.load_yaml(config_file)
        self.setup = self.config.get("setup", {})
        self.slices_2d = self.setup.get("slices_2d", False)
        self.todo_filename = todo_filename

        # Generate the runs
        if self.slices_2d:
            print("Generating a series of 2D slices")
            self.make_2d_slices(self.config)
        else:
            print("Generating a simple set of runs")
            self.make_simple(self.config)

    def generate_params(self, config):
        """Generate all permutations of a given set of sweep parameters"""

        # Generate filename
        folder = iter_utils.get_group_folder(config)
        print(folder)

        # Read in the sweep parameters
        sweep = config["sweep"]

        # Generate the speed parameter matrix
        name_list = []
        permute_list = []
        for var in sweep:
            name_list.append(iter_utils.parse_variable_name(var["variable"]))
            vals = var.get("values", None)
            folder_iter = var.get("folder", None)

            # Process direct values
            if isinstance(vals, list):
                permute_list.append(vals)

            # Process folder iterator
            elif isinstance(folder_iter, str):
                blacklist = var.get("file_blacklist", None)
                filter_extensions = var.get("filetypes_to_use", None)

                val_list = iter_utils.scrape_folder(
                    folder_iter, filter_extensions, blacklist
                )

                if len(val_list) != 0:
                    permute_list.append(val_list)

            # Process numeric value vector
            elif vals is None:
                permute_list.append(
                    np.linspace(var["min"], var["max"], var["num_steps"]).tolist()
                )

            else:
                raise

        param_list = itertools.product(*permute_list)

        return param_list, name_list, permute_list

    def make_simple(self, config, save_todo=True):
        """Make a simple set of runs using all permutations of sweep parameters"""

        # Save a copy of the sweep config in the root folder
        # config['save']['folder'] = self.base_folder
        out_folder = config["save"]["group_name"]
        if not os.path.exists(os.path.join(self.base_folder, out_folder)):
            os.makedirs(os.path.join(self.base_folder, out_folder))

        out_filename = os.path.join(self.base_folder, out_folder, "config.yaml")
        iter_utils.save_yaml(config, out_filename)

        # Generate parameters
        param_list, name_list, _ = self.generate_params(config)

        # Create all of the individual config files, and store them in folders
        run_names = []
        for set_num, param_set in enumerate(param_list):
            config_new = copy.deepcopy(config)
            config_new.pop("sweep", None)
            for idx, param in enumerate(param_set):
                var_name = name_list[idx]
                iter_utils.set_in_dict(config_new, var_name, param)

            config_new["save"]["run_name"] = "param_set_%04d" % (set_num)
            _, out_folder = iter_utils.generate_save_location(config_new)
            run_filename = os.path.join(out_folder, "params.yaml")
            iter_utils.save_yaml(config_new, run_filename)

            run_names.append(run_filename)

        # Save the set of runs todo
        if save_todo:
            iter_utils.save_yaml(run_names, self.todo_filename)

        return run_names

    def make_2d_slices(self, config):
        """Make a simple set of runs using all permutations of sweep parameters"""

        # Get the sweep settings
        sweep = config["sweep"]

        num_vars = len(sweep)

        # Create inner and outer parameter sets to condese things down into 2D slices
        if num_vars > 2:
            config_inner = copy.deepcopy(config)
            config_inner["sweep"] = config_inner["sweep"][num_vars - 2 :]

            config_outer = copy.deepcopy(config)
            config_outer["sweep"] = config_outer["sweep"][0 : num_vars - 2]

            param_list, name_list, param_values = self.generate_params(config_outer)

            param_list_real = []
            all_runs = []
            for param_idx, param_set in enumerate(param_list):
                # Create a new config file that only has two sweep parameters

                param_list_real.append(param_set)
                config_new = copy.deepcopy(config_inner)

                var_str = ""
                for idx, param in enumerate(param_set):
                    var_name = name_list[idx]
                    iter_utils.set_in_dict(config_new, var_name, param)

                    for piece in var_name:
                        var_str += "__" + piece
                    var_str = var_str[0:]
                    var_str = var_str.strip(" ")
                    if isinstance(param, str):
                        curr_idx = param_values[idx].index(param)
                        var_str = var_str + "_%04d" % (curr_idx)
                    else:
                        var_str = var_str + "_%0.4f" % (param)

                group_ext = "/" + var_str
                config_new["save"]["group_name"] += group_ext

                # Make a simple sweep with the new 2D config
                run_names = self.make_simple(config_new, save_todo=False)

                all_runs.extend(run_names)

            label = []
            for vari in config_outer["sweep"]:
                label.append(vari["label"])
            summary = {}
            summary["vars"] = label
            summary["sweep"] = param_list_real

            summary_file = os.path.join(
                iter_utils.get_group_folder(config), "summary.yaml"
            )

            iter_utils.save_yaml(summary, summary_file)

            # Save the set of runs todo
            iter_utils.save_yaml(all_runs, self.todo_filename)


if __name__ == "__main__":
    config_file = "sweeps/grid_design_grasps.yaml"
    run_gen = RunGenerator()
    run_gen.from_file(config_file)
