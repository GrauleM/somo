Somo Framework Example:

# Region of Acquisition
This example shows how to set up a parametric simulation and run multidimensional sweeps over parameters

## Basic Structure
We utilize a basic simulation function + outer-loop structure to make simulations modular. The Simulation function defines how a single simulation should run, and the outer-loop runs through sets of parameters 

## Setup

1. Set your savepath - this is where all data will be saved.
    - Inside of **"save_path.yaml"**, set the _"save_path"_ feild.
2. Make a function that performs one simulation given a set of parameters.
    - Inside of **"run_single.py"**, set up a function called _"run_single()"_
    - _"run_single()"_ must take only one input, but that can be a dictionary of parameters.
    - In this example, the parameters are stored in a .yaml file, which is loaded by _"run_single()"_. This function is where you set up dependencies on parameters that you might want to change later.
2. Make a configuration file for a parameter sweep.
    - A sweep cofig file defines 1) a default set of parameters, and 2) a list of parameters to sweep over (including thier values)
    - You can find examples in the **"sweeps"** folder.
    - Be sure to define the _"save folder"_ and _"group name"_ unique to each config file. This is where all of the parameter files and data will be saved.
    - The _setup_ field defines properties of the runs related to labeling and graphing the data
    - Entries in the _"sweep"_ list can take three different forms:
         - Define the min value, max value, and number of steps using the _"min"_, _"max"_, and _"num_steps"_ keys. This option uses numpy.linspace to generate parameter values
         - Define values directly in a list using the _"values"_ key
         - Define a folder of files to iterate over, with the _"folder"_ key and associated _"filetypes_to_use"_ and _"file_blacklist"_ keys.
    - Variable names are defined in a special way
        - "/" works just like defining directories, allowing you to navigate nested parameters
        - If a variable is part of a list (like the x-position of the hand is part of the _"manipulator/hand/position"_ variable), you can define indices in the sweep (for example _"manipulator/hand/position[0]"_)

_Note: You can only define scalar sweep variables for now._

Example of a sweep list:
```yaml
sweep:
    -   variable: 'manipulator/hand/position[0]' # Turn the hand x-position into a sweep variable
        max:  0.6                                # Define the maximum value
        min: -0.6                                # Define the minimum value
        num_steps: 13                            # Define the number of steps

    -   variable: 'manipulator/fingers/file'                         # Turn the finger definition file into a sweep variable
        values:   ['definitions/manipulator_definition_000.yaml',    # Define values to use directly
                   'definitions/manipulator_definition_001.yaml',
                   'definitions/manipulator_definition_003.yaml']

    -   variable: 'object/file'             # Turn the object file into a sweep variable
        folder: "../_objects/ycb"           # Iterate over files in this folder
        filetypes_to_use: [".urdf"]         # Only use files with these extensions (if empty, then allow all file extensions)
        file_blacklist: ["_prototype.urdf"] # Ignore these files (if empty, then allow all filenames)
```

Example of a standard setup feild (from the _"grid_design_grasps.yaml"_ file):
```yaml
setup:
    label_functions: ["default", "mean_link"] # A list of label functions
    slices_2d: True
    graph:
        xlabel: X Position (0.1 mm)
        ylabel: Y Position (0.1 mm)
        mirror_over_x: True
        color_set: ["default", "mean_link"]  # A list of color sets to acompany each label function
        color_label: ['Grasp Success','Finger segment in-contact'] # A list of labels to label the colorbar
```

## Build Sweeps
If you have set up your sweep config file correctly, it should be easy to generate all the nessecary parameter files. Use the **"RunGenerator"** class to do this. 

### Generate arbitrary multidimensional sweeps
Set `slices_2d: False` inside of the _"setup"_ field.
- This will make one large group of parameter sets inside of the _"group_name"_ folder you defined in the sweep config file.
- The resulting set of folders will each have one **"params.yaml"** file defining the parameters for one simulation. 

If you choose to do things with `slices_2d = False`, then you can perform parameter sweeps over an arbitrary number of dimensions (corresponding to an arbitrary number of elements in your _"sweep"_ list).

_NOTE: If you make a parameter sweep with more than 2 dimensions, plotting the results will become trickier since the current generation of plotting scripts only handles 2D plots._

### Generate 3D sweeps as a group of 2D sweeps
Set `slices_2d: True` inside of the _"setup"_ field.
    - This will make several groups of parameter sets inside of the _"group_name"_  folder
    - The first variable in your _"sweep"_ list gets used as the folder name for each new parameter set
    - Within each folder, the second and third variables in your _"sweep"_ list are re-cast as the only variables in a 2D sweep.

_NOTE: This only works with 3D sweeps for now._


## Run Sweeps
After you've built sweeps, just use the **"BatchSimulation"** class to run the simulations. By default the sweeps run with parallel queuing which performs several simulations in parallel with python's built-in queuing system and true parallel processing (see python's [multiprocess library](https://docs.python.org/3/library/multiprocessing.html)). If you want to run simulations in strict sequence, pass `parallel=False` to the **"BatchSimulation.run()"** function.

### How this works: 
Regaurdless of how you generated the sweeps, a list of all sweeps to do are saved in **"runs_todo.yaml"**. Since the parameters for single sweeps are all saved, we can just loop through all these files and complete simulations with these parameters one-by-one (or several in parallel)


## Graph Data from Sweeps
In the case of this example, the goal is to determine the region of hand positions in the x-y plane that result in successful grasps (this is known as the region of acquisition). As such, we have set up plotting functions to generate a 2D grid of success rates based on the parameter values.

### Determine the "label function"
Inside **"utils/label_functions.py"**, you can define a label function or use an existing one. These functions have access to all data you saved in the simulations. For this example, successful grasps result in the object being lifted above 0.1m (in simulation units).

### Plot 2D grids of success rate
If `slices_2d` is set to `true` in the _"setup"_ feild of your sweep config file, one 2D plot for each subfolder will be generated.


## TODO:
### High-Priority:
- [x] Set up a "folder" sweep datatype that loops over filenames in a folder (i.e. object urdfs)
- [x] Enable multiple label functions to be used to calculate multiple performance metrics for each run.
- [ ] Enable variables to be linked together in a sweep (for example, use a different actuation profile for each finger definition in the example above)

### Medium Priority
- [ ] Make a separate, minimal example with 1D, 2D, and 3D sweep examples
- [ ] Modify load_run_list function should also accept a list of parameters as well as a filename to laod parameters from a file.
- [ ] Enable vector sweeps rather than just single scalar variables.


### Low Priority
- [ ] Write plotting functions that let you choose 2D slices in parameter space (or show 3D success rate volumes)
- [ ] Make the "break into 2D sweeps" function work with arbitrary dimensions
- [x] Add a graphing class for individual runs