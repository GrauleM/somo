# Be sure to run this file from the "ihm_fingergait" folder
#     cd examples/design_studies/ihm_fingergait/

import os
import sys

# Be sure to install somo as a package with pip
from somo.sweep import RunGenerator
from somo.sweep import BatchSimulation
from somo.sweep import DataLabeler
from somo.sweep import GridPlotter
from somo.sweep import ContourPlotter

import run_single as sim_function
from object2urdf import ObjectUrdfBuilder

path = os.path.abspath(os.path.join(os.path.dirname(__file__), "labels"))
sys.path.insert(0, path)
import label_functions

path = os.path.abspath(os.path.join(os.path.dirname(__file__), "process_data"))
sys.path.insert(0, path)
from graph_performance import IHMPerformance

config_file = "sweeps/fingergait_tuned_video.yaml"
build = True
do_simulations = True
process_results = False
plot_raw = False


label_fun_list_all = ["save_raw_data"]

# GENERATE RUNS
if build:
    object_set = "objects/palms/flat"
    builder = ObjectUrdfBuilder(object_set)
    builder.build_library(
        force_overwrite=True,
        decompose_concave=True,
        force_decompose=False,
        center="top",
    )

    object_set = "objects/real_objects"
    builder = ObjectUrdfBuilder(object_set)
    builder.build_library(
        force_overwrite=True,
        decompose_concave=True,
        force_decompose=False,
        center="mass",
    )

    run_gen = RunGenerator()
    run_gen.from_file(config_file, todo_filename="_runs_todo.yaml")


# DO SIMULAITONS
if do_simulations:
    # Run the simulation batch
    batchsim = BatchSimulation()
    batchsim.load_run_list(todo_filename="_runs_todo.yaml", recalculate=True)
    batchsim.run(run_function=sim_function.run_single, parallel=False, num_processes=4)


# CALCULATE PERFORMANCE METRICS
if process_results:
    # Proccess the simulation results
    labeler = DataLabeler(label_functions)
    labeler.set_global_scale(20)
    if plot_raw:
        labeler.process_all(
            config_file,
            label_function=label_fun_list_all,
            plot_raw=True,
            cycle_key="fingers",
            parallel=False,
        )
    else:
        labeler.process_all(
            config_file, label_function=label_fun_list_all, parallel=False
        )
