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
from matplotlib.collections import PatchCollection
from matplotlib.patches import Rectangle
from matplotlib.colors import ListedColormap
import matplotlib.tri as tri
from matplotlib import rcParams
import seaborn as sns
import sys


from somo.sweep import iter_utils

# from utils import label_functions # Need to fix label functions pipeline to come from example folder
from somo.sweep import DataLabeler


class ContourPlotter:
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

    def _plot_label(
        self,
        results,
        label_name,
        success_filename,
        show=False,
        replace=False,
        aux_savepath=None,
    ):

        # Find the axis values and get the success labels
        diffs = results["diffs"]
        axis_lims = []

        dims = []

        for idx, vals in enumerate(results["sweep"]):
            dims.append(len(np.unique(vals).tolist()))
            fullscale = max(vals) - min(vals)
            axis_lims.append(
                (
                    min(vals) - abs(diffs[idx]) / 2 - fullscale * 0.05,
                    max(vals) + abs(diffs[idx]) / 2 + fullscale * 0.05,
                )
            )

        data = np.array(results["labels"][label_name])

        # Set up the figure
        rcParams["font.size"] = 14
        rcParams["font.family"] = "Arial"
        sns.set_style("ticks")

        fig = plt.figure()
        ax = plt.gca()
        # ax = plt.subplot(1,2,1)

        # Find all the unique values of success labels
        status_options = np.unique(data).tolist()
        status = [[] for i in status_options]

        X = []
        Y = []
        Z = []
        XY = []

        # If mirroring is turned on, mirror about the y-axis
        if self.mirror_over_x:
            dims[0] = dims[0] * 2
            axis_lims[0] = (-max(np.abs(axis_lims[0])), max(np.abs(axis_lims[0])))
            len_data = len(data)
            for idx, succ in enumerate(np.flip(data)):
                x = -results["sweep"][0][len_data - 1 - idx]
                y = results["sweep"][1][len_data - 1 - idx]

                X.append(x)
                Y.append(y)
                XY.append((x, y))
                Z.append(succ)

        # Set up all the points to be drawn based on the coordinates in parameter space
        for idx, succ in enumerate(data):
            x = results["sweep"][0][idx]
            y = results["sweep"][1][idx]

            X.append(x)
            Y.append(y)
            XY.append((x, y))
            Z.append(succ)

        # Get the correct colormap
        status_colors_temp = copy.deepcopy(self.status_colors.get(label_name, ""))

        status_colors_use = self.status_colors_def.get(
            status_colors_temp, status_colors_temp
        )

        if isinstance(status_colors_use, list):
            cmap = ListedColormap(status_colors_use)
        else:
            cmap = status_colors_use

        # Sort the grid so its ascending
        all_data = np.vstack((X, Y, Z)).T
        df = pd.DataFrame(all_data, columns=["X", "Y", "Z"])

        df = df.sort_values(["X", "Y"])
        df = df.drop_duplicates()

        dims[0] = int(len(df["X"].values) / dims[1])

        # X_plot, Y_plot = np.meshgrid(df['X'],df['Y'])
        X_plot = np.reshape(df["X"].values, tuple(dims)).T
        Y_plot = np.reshape(df["Y"].values, tuple(dims)).T
        Z_plot = np.reshape(df["Z"].values, np.array(dims)).T

        # print(X_plot)
        # print(Y_plot)
        # print(Z_plot)

        # Plot the contours
        plt.contour(X_plot, Y_plot, Z_plot, self.num_bins, colors="black")
        contours = plt.contourf(X_plot, Y_plot, Z_plot, self.num_bins, cmap=cmap)

        # plt.contour(X_plot, Y_plot, Z_plot, 12, colors='black')
        # contours = plt.contourf(X_plot, Y_plot, Z_plot, 12, cmap="RdBu_r")

        # plt.tricontour(X, Y, Z, 12, colors='black')
        # contours = plt.tricontourf(X, Y, Z, 12, cmap="RdBu_r")
        plt.clabel(contours, inline=True, fontsize=8)

        plt.plot(X, Y, "ko", ms=3)

        # plt.imshow(Z_plot, extent=[min(X), max(X), min(Y), max(Y)], origin='lower',
        #        cmap=cmap, alpha=0.5, aspect='auto')

        # Make the graph look nice
        if self.axes_equal:
            ax.set_aspect("equal", "box")
        else:
            pass
            # plt.tight_layout()
        # ax.set_xlim(axis_lims[0])
        # ax.set_ylim(axis_lims[1])

        # Label axes
        if self.xlabel:
            plt.xlabel(self.xlabel)
        elif len(results["varlabels"]) == 2:
            plt.xlabel(results["varlabels"][0])
        else:
            plt.xlabel(results["vars"][0][-1])

        if self.ylabel:
            plt.ylabel(self.ylabel)
        elif len(results["varlabels"]) == 2:
            plt.xlabel(results["varlabels"][1])
        else:
            plt.ylabel(results["vars"][1][-1])

        plt.title(label_name)

        # Add a colorbar
        cbar = plt.colorbar()
        cbar.set_label(self.color_labels[label_name])

        # Save the plot in the default location
        outfile = success_filename.replace(".yaml", "")
        plt.savefig(outfile + "_" + label_name + "_contour.png", dpi=450)
        plt.savefig(outfile + "_" + label_name + "_contour.svg", dpi=450)

        # Save the plot in the second location if one is given
        if aux_savepath is not None:
            aux_path = aux_savepath.replace(".yaml", "")
            plt.savefig(aux_path + "_" + label_name + "_contour.png", dpi=450)

        # Show plots if that option is selected
        if show:
            plt.show()

        # Close the plot
        plt.close()

    def plot_one(
        self,
        success_filename,
        labels=None,
        show=False,
        replace=False,
        aux_savepath=None,
    ):
        """Make a plot of the grasp type/success rate of 2D sweep data."""

        # If the results summary has already been calculated and saved, read it in
        if os.path.exists(success_filename) and replace == False:
            results = iter_utils.load_yaml(success_filename)

        # Otherwise, process the data and generate the results summary
        else:
            labeler = DataLabeler()
            results = labeler.process_run(
                success_filename.replace("summary.yaml", "config.yaml")
            )

        if len(results["vars"]) == 2:
            if labels is not None:
                label_data = labels
            else:
                label_data = results["labels"]

            for key in label_data:
                self._plot_label(
                    results, key, success_filename, show, replace, aux_savepath
                )
        else:
            raise ValueError(
                "You are trying to plot a 2D grid with %dD data. You can only plot grids of 2D data"
                % (len(results["vars"]))
            )

    def make_plots(self, labels=None, show=False, recalculate=False, num_bins=12):
        self.num_bins = num_bins
        base_folder = iter_utils.get_group_folder(self.config)
        print(base_folder)

        if self.slices_2d:
            folders = next(os.walk(base_folder))[1]
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
                labels=labels,
                show=show,
                replace=recalculate,
                aux_savepath=aux_savepath,
            )
