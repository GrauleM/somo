# Be sure to run this file from the "palm_sweeps" folder
#     cd examples/palm_sweeps

import os
import sys
from datetime import datetime, timedelta
path=os.path.abspath(os.path.join(os.path.dirname( __file__ ), '..','..'))
sys.path.insert(0, path)

from pybullet_cm.sweep import iter_utils


config_file = 'sweeps/grid_dsr.yaml'
todo_file = '_runs_todo.yaml'

num_files_per_folder_end = 5
num_files_per_folder_start = 1

time_per_run = 5*1/0.15 #seconds
avg_size     = 5.3 #MB
parallel_cores = 4

# Get data from config files
config=iter_utils.load_yaml(config_file)
todo=iter_utils.load_yaml(todo_file)
total_runs = len(todo)

# Calculate the time
total_time_min = (time_per_run/60.0)*total_runs / parallel_cores
total_time_hr  = total_time_min/60.0
total_time_day = total_time_hr/24.0

# Calculate total data size
total_size_GB = float(avg_size)*total_runs/1000.0

# Calculate the percent complete
folder_to_count = iter_utils.get_group_folder(config)
cpt = sum([len(files) for r, d, files in os.walk(folder_to_count)])

total_files_expected_end = total_runs*num_files_per_folder_end
total_files_expected_start = total_runs*num_files_per_folder_start
progress = (cpt-total_files_expected_start)/(total_files_expected_end-total_files_expected_start)

eta_min = total_time_min*(1.0-progress)
eta_hr  = eta_min/60.0
eta_day = eta_hr/24.0

eta_wall = datetime.now()+timedelta(minutes=eta_min)

# Print info
print('')
print('Current time: '+datetime.now().strftime("%I:%M:%S %p"))
print('=================================')
print("Number of runs to complete: %d"%(total_runs))
print("Estimated total data saved @ %0.1f MB per run: %0.2f GB"%(avg_size, total_size_GB))
print("Estimated total time @ %0.1f sec per run with %d cores: %0.1f min, %0.2f hrs, %0.3f days"%(time_per_run, parallel_cores, total_time_min, total_time_hr, total_time_day))
print('---------------------------------')
print("Percent Complete: %0.3f %%"%(progress*100))
print("Estimated time left: %0.1f min, %0.2f hrs, %0.3f days"%( eta_min, eta_hr, eta_day))
print("Estimated time of completion: "+eta_wall.strftime("%I:%M:%S %p"))
print('')