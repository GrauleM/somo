# Be sure to run this file from the "region_of_acquisition" folder
#     cd examples/region_of_acquisition
#
import yaml
import time
import multiprocessing as mp
import os
import sys

from somo.sweep import iter_utils


class BatchSimulation:
    def __init__(self):
        self.num_cpus = os.cpu_count()

    def mute(self):
        sys.stdout = open(os.devnull, "w")

    # Define the parallelization function
    def _run_parallel(self, run_function, num_processes=None):
        """Run the batch with parallel processing"""

        # Use one less than the number of processors you have
        # (so you can still use your computer while this runs)
        if num_processes is None:
            num_processes = self.num_cpus - 1

        elif num_processes > self.num_cpus:
            print(
                "WARNING: Using %d processes on %d CPUs"
                % (num_processes, self.num_cpus)
            )

        pool = mp.Pool(num_processes)
        pool.map(run_function, self.run_params, 1)

        return True

    def _run_sequential(self, run_function):
        """Explicitly run the batch sequentially"""

        for param_set in self.run_params:
            run_function(param_set)

        return True

    def load_run_list(self, todo_filename="runs_todo.yaml", recalculate=False):
        runs_todo = iter_utils.load_yaml(todo_filename)
        self.run_params = [
            {"filename": run, "index": idx, "replace": recalculate}
            for run, idx in zip(runs_todo, range(len(runs_todo)))
        ]

    def run(self, run_function, parallel=True, num_processes=None):
        # Run experiments
        try:
            iter_utils.add_tmp("_tmp")
            start = time.time()
            if parallel:
                self._run_parallel(run_function, num_processes)
            else:
                self._run_sequential(run_function)

            end = time.time()
            iter_utils.delete_tmp("_tmp")

            print("____________________________")
            print(
                "TOTAL TIME: %0.1f sec (%0.2f min)"
                % ((end - start), (end - start) / 60)
            )
            print("____________________________")

        except KeyboardInterrupt:
            print("\n" + "BATCH TERMINATED EARLY")
            iter_utils.delete_tmp("_tmp")
