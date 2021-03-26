# Be sure to run this file from the "region_of_acquisition" folder
#     cd examples/region_of_acquisition
#
import yaml
import time
import copy
import shutil
import os
import pybullet as p
import numpy as np
import pandas as pd
import matplotlib as mpl
import matplotlib.pyplot as plt
import matplotlib.tri as tri
from matplotlib import rcParams
import seaborn as sns
import sys
from natsort import natsorted


from somo.sweep import iter_utils
from somo.sweep import ContourPlotter


class IHMPerformance:
    def __init__(self, config_file):
        self.config = iter_utils.load_yaml(config_file)

        self.status_colors_def = iter_utils.load_yaml("labels/color_sets.yaml")
        self.fingertip_color_def = self.status_colors_def.get(
            "fingertip", [1.0, 0.7, 0.0]
        )

        self.setup = self.config.get("setup", {})
        self.slices_2d = self.setup.get("slices_2d", False)
        self.mirror_over_x = False
        self.status_colors = {"default": self.status_colors_def["default"]}
        self.fingertip_color = self.fingertip_color_def
        self.emphasize_fingertip = False
        self.axes_equal = True
        self.xlabel = None
        self.ylabel = None
        self.labels_to_graph = []
        self.label_columns = []
        self.label_scales = None
        self.input_vars = None

    def set_status_colors(self, label_set=None, color_set=None, color_labels=None):
        if not (isinstance(label_set, list) and isinstance(color_set, list)):
            raise TypeError("Label and color sets must be lists")

        if len(label_set) != len(color_set):
            raise ValueError("The length of your label and color sets but be equal")

        status_colors = {}
        color_label_set = {}
        for idx, c_set in enumerate(color_set):
            status_colors[label_set[idx]] = c_set

            if isinstance(color_labels, str):
                color_label_set[label_set[idx]] = color_labels
            elif isinstance(color_labels, list):
                color_label_set[label_set[idx]] = color_labels[idx]
            else:
                color_label_set[label_set[idx]] = ""

        if status_colors:
            self.status_colors = status_colors
            self.color_labels = color_label_set

    def set_status_colors_dict(self, label_list):
        if not isinstance(label_list, list):
            raise TypeError("Input must be list of dictionaries")

        status_colors = {}
        color_label_set = {}
        for label_dict in label_list:
            curr_colors = self.status_colors_def.get(label_dict["colormap"], None)
            if curr_colors is not None:
                status_colors[label_dict["name"]] = curr_colors
            else:
                status_colors[label_dict["name"]] = label_dict["colormap"]

            color_label_set[label_dict["name"]] = label_dict["axis_label"]

        if status_colors:
            self.status_colors = status_colors
            self.color_labels = color_label_set

    def set_status_colors_label(self, label_name="default", color_set=None):
        if isinstance(color_set, list):
            self.status_colors[label_name] = color_set
        else:
            raise TypeError("Expected a list of colors or a list of lists")

    def set_colors(self, status_colors=None):
        if status_colors is not None:
            self.status_colors = status_colors

    def set_axes_equal(self, in_set):
        self.axes_equal = in_set

    def set_label_columns(
        self, cols, tie_to_input, input_vals, scales=None, labels=None
    ):
        if labels is None:
            labels = cols

        if scales == None:
            scales = [1.0] * len(cols)
        self.label_columns = {}
        self.label_scales = {}
        self.label_labels = {}
        for idx, val in enumerate(input_vals):
            self.label_columns[val] = cols[idx]
            self.label_scales[val] = scales[idx]
            self.label_labels[val] = labels[idx]

    def _plot_label(
        self,
        results,
        label_name,
        success_filename,
        inputs=None,
        iterate=None,
        filter_col=None,
        show=False,
        replace=False,
        aux_savepath=None,
    ):

        # Find the axis values and get the success labels
        axis_lims = []
        dims = []

        if inputs is None:
            inputs = self.input_vars

        if not isinstance(label_name, list):
            label_name = [label_name]

        # Set up the figure
        rcParams["font.size"] = 14
        rcParams["font.family"] = "Arial"
        sns.set_style("ticks")

        fig = plt.figure()
        ax = plt.gca()

        for label in label_name:
            iter_vals = self.df[iterate].unique()

            self.df11 = self.df[self.df[filter_col] == 1]

            for curr_iter_val in iter_vals:
                self.df12 = self.df11[self.df11[iterate] == curr_iter_val]
                col = self.label_columns[curr_iter_val]

                X = np.asarray(self.df12[inputs[0]].values)
                Y = np.asarray(self.df12[inputs[1]].values)

                Z_all = np.stack(self.df12[label].values)
                Z = float(self.label_scales[curr_iter_val]) * Z_all[:, col]

                plt.tricontourf(X, Y, Z, 12, cmap="Blues")
                plt.plot(X, Y, "ok", markersize=5)

                cbar = plt.colorbar()
                cbar.set_label(self.label_labels[curr_iter_val])

                plt.xlabel(inputs[0])
                plt.ylabel(inputs[1])

                # Save the plot in the default location
                outfile = success_filename.replace(".yaml", "")
                plt.savefig(
                    outfile + "_" + label + "_" + str(curr_iter_val) + ".png", dpi=450
                )
                plt.savefig(
                    outfile + "_" + label + "_" + str(curr_iter_val) + ".svg", dpi=450
                )

                # Save the plot in the second location if one is given
                if aux_savepath is not None:
                    aux_path = aux_savepath.replace(".yaml", "")
                    plt.savefig(
                        aux_path + "_" + label + "_" + str(curr_iter_val) + ".png",
                        dpi=450,
                    )

                # Show plots if that option is selected
                if show:
                    plt.show()

                # Close the plot
                plt.close()

    def plot_one(
        self,
        success_filename,
        inputs=None,
        labels=None,
        iterate=None,
        filter_col=None,
        show=False,
        replace=False,
        aux_savepath=None,
    ):
        """Make a plot of the grasp type/success rate of 2D sweep data."""

        try:
            results = iter_utils.load_yaml(success_filename)
            filename, ext = os.path.splitext(success_filename)
            data = iter_utils.load_yaml(filename + "_flattened" + ext)
            self.df = pd.DataFrame(data)

        except:
            raise

        if len(results["vars"]) == 2:
            if labels is not None:
                label_data = labels
            else:
                label_data = results["labels"]

            for key in label_data:
                self._plot_label(
                    results,
                    key,
                    success_filename,
                    inputs=inputs,
                    iterate=iterate,
                    filter_col=filter_col,
                    show=show,
                    replace=replace,
                    aux_savepath=aux_savepath,
                )
        else:
            raise ValueError(
                "You are trying to plot a 2D grid with %dD data. You can only plot grids of 2D data"
                % (len(results["vars"]))
            )

    def plot_multi(
        self,
        success_filename,
        label_groups=[],
        labels=None,
        show=False,
        replace=False,
        aux_savepath=None,
    ):
        """Make a plot of the grasp type/success rate of 2D sweep data."""

        try:
            results = iter_utils.load_yaml(success_filename)
            self.expand_data(results)
        except:
            raise

            for keys in label_groups:
                self._plot_label(
                    results,
                    keys,
                    success_filename,
                    show=show,
                    replace=replace,
                    aux_savepath=aux_savepath,
                )

    def make_plots(
        self,
        labels=None,
        inputs=None,
        iterate=None,
        filter_col=None,
        show=False,
        recalculate=False,
        num_bins=12,
    ):
        self.num_bins = num_bins
        base_folder = iter_utils.get_group_folder(self.config)
        print(base_folder)

        if self.slices_2d:
            dir_list = os.listdir(base_folder)
            dir_list = natsorted(dir_list)
            folders = [
                os.path.join(base_folder, subdir)
                for subdir in dir_list
                if os.path.isdir(os.path.join(base_folder, subdir))
            ]
        else:
            folders = [""]

        for folder in folders:
            print(folder)
            success_filename = os.path.join(base_folder, folder, "summary.yaml")

            if self.slices_2d:
                aux_savepath = os.path.join(base_folder, folder + ".yaml")
            else:
                aux_savepath = None

            self.plot_one(
                success_filename,
                inputs=inputs,
                labels=labels,
                iterate=iterate,
                filter_col=filter_col,
                show=show,
                replace=recalculate,
                aux_savepath=aux_savepath,
            )
