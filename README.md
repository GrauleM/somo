# SoMo
> This module contains a light wrapper around pybullet that facilitates the simulation of continuum manipulators.

> The following is a pre-release. We are actively working on cleaning up this code and providing a thorough documentation. 


SoMo (**So**ft**Mo**tion) is a framework to facilitate the simulation of continuum manipulator (CM) motion in 
[pybullet](https://github.com/bulletphysics/bullet3/tree/master/examples/pybullet). 
In SoMo, continuum manipulators are approximated as a series of rigid links connected by spring-loaded joints. 
SoMo makes it easy to create URDFs of such approximated manipulators and load them into pybullet's rigid body simulator. 
With SoMo, environments with various continuum manipulators, such as hands with soft fingers (xxx links), or snakes, 
can be created and controlled with only a few lines of code.


![Palm example](https://github.com/GrauleM/somo/blob/master/docs/img/importance_of_palms.png)

**Todos:** see [todos.md](todos.md)

## Installation
### Requirements
- [Python 3.6](https://www.python.org/downloads/release/python-360/)+
- Tested on:
	- Ubuntu 16.04 and Ubuntu 18.04 with Python 3.6.9
	- Ubuntu 20.04 with Python 3.6.9, 3.7.9 and 3.8.2
	- Windows 10 with Python 3.7 through [Anaconda](https://www.anaconda.com/products/individual#Downloads)
- Recommended: pip (`sudo apt-get install python3-pip`) 
- Recommended (for Ubuntu): [venv](https://docs.python.org/3/library/venv.html) (`sudo apt-get install python3-venv`)

### Setup
0. Make sure your system meets the requirements
1. Clone this repository
2. Set up a dedicated virtual environment using `venv`
3. Activate virtual environment 
4. Install requirements from this repo: `$ pip install -r requirements.txt`
5. Install this module:
    - either by cloning this repo to your machine and using `$  pip install -e .` from the repo root, or with
    - `$ pip install git+https://github.com/graulem/somo`
6. To upgrade to the newest version: `$ pip install git+https://github.com/graulem/somo --upgrade`


### Explore the examples
- run any of the files in the examples folder. xx is a great place to start xx should not be part of the installation

### Contributing
- only through a new branch and reviewed PR (no pushes to master!)
- always use [Black](https://pypi.org/project/black/) for code formatting
- always bump the version of your branch by increasing the version number listed in somo/_version.py

### Testing
SoMo uses pytest for testing. In most cases, it will be better to ignore the tests that rely on the GUI -test coverage will be identical. You can run all tests with `$ pytest` from the repository's root and ignore the tests involving the GUI with `$ pytest -m "not gui"`


## Using this framework
This framework relies on two key components: a collection of methods that facilitates the automated generation of urdf 
files (xx); and a class that relies heavily on pybullet to define a CMClass object (xx rephrase). A CMClass object can 
be instantiated from  a json file (xx not implemented yet) that specifies the properties of the continuum manipulator 
in human-readable form, enabling the user to easily keep track of and vary the properties of different CMs through 
a series of experiments.

<!--
the continuum manipulator class, which relies heavily on pybullet, defines a CM object and provides an intuitive interface to 
-->



## Citation
xx 

## License

MIT open source?? tbd. xx

Copyright (c) 2020 Moritz A. Graule. xx add license file xx


## CMClass - Manipulator definition
Each manipulator consists of a series of `n_act` actuators and an un-actuated base (which can be `None`, i.e. non-existent). 
A manipulator definition is a Python `dict` that contains definitions for the base and each of the actuators.


```python
manipulator_definition = {
    "n_act" : int, # number of actuators in the manipulator
    "base_description" : None, # can be None (no base) or a valid link_description
    "actuators" : [actuator_definition] # a list of actuator definitions
}
```

```python
actuator_definition = {
    "actuator_lenght" : float, # length of the actuator
    "n_seg" : int, # number of segments in the actuator
    "link_description" : dict, # describes each of the links. in the future a generator to enable changing properties along the manip
    "joint_description" : dict, # describes the joints
}
```
xx todo: add isvalid(); len([actuator_definition]) has to be same as `n_act`


```python
link_description = {
    "geometry" : str, # str can be "box", "cylinder", or "sphere"
    "dimensions" : [float], # dimension of an individual actuator element.
    "mass" : float, # mass of the link 
    "inertial_values" : dict, # inertial values for an individual segment 
    "material_dict" : dict, # describes the link material
    "step_between_links" : float # should be the segment length (i.e., cylinder height) for cylindrical segments 
}
```
The `material_dict` describes the link material color and color name:

```python
material_dict = {
    "name" : str, # material name
    "color" : [] # list of four floats between 0. and 1.; rgba
}
```

```python
joint_description = {
    "planar_flag" : int, # indicates whether the actuator should be treated as quasiplanar (1) or not (0); meaning 1 vs. 2 deformable axes  
    "joint_limits" : [dict], # limits for the joints. has one entry if quasi-planar, 2 entries otherwise (one for each axis)
    "k_spring" : [float], #  the stiffness of each link. has one entry if quasi-planar, 2 entries otherwise (one for each axis)
    "bending_axes" : [axes], # axes are e.g. [[1,0,0],[0,1,0]]
}
```

## Submodules for a Streamlined Workflow
### somo.sweep
Perform multidimensional parameter sweeps with true parallel processing and easy data handling. More info about the **sweep** module is located in the _"parameter_sweep"_ example
	
### somo.logger
Parse logged data from pybullet's builtin loggers, and trim it to only include the columns you want. You can also convert to pandas dataframes.



## Useful Companion Packages

Here are a few useful packages that were written along-side this framework. You do not need them to use the SoMo framework, especially if you already have a system you like, but the dev team uses these for our own work.

-  [sorotraj](https://pypi.org/project/sorotraj/) - Trajectory generation for soft robots using waypoints
	- `pip install sorotraj`
-  [object2urdf](https://pypi.org/project/object2urdf/) - Manage a library of objects, and auto-generate URDFs from templates.
	- `pip install object2urdf`

## Other Useful Packages

These 3rd-party packages are generally useful when working with pybullet.

- [trimesh](https://trimsh.org/) - a pure Python library for loading and using triangular meshes







## Old notes - ignore for now. Actuator definition

xx todo: add check whether act_params is valid,e.g. the dimensions length matches the segment_geometry (check already implemented in add_segment function)

The entry with key `"joint_limits"` has the following form if `quasi_planar_flag=1`: `[limit_dict_1]`
The entry with key `"joint_limits"` has the following form if `quasi_planar_flag=0`: `[limit_dict_1,limit_dict_2]`

`[limit_dict_i]` describes the joint limits for the joints that bend along axis `i`; it is a `dict` that has the following form: 

```python
limit_dict_i = {
    "lower": str(lower_lim),
    "upper": str(upper_lim),
    "effort": str(eff),
    "velocity": str(vel)
}
```
Here, `lower_lim` and `upper_lim` are `floats` that prescribe the lower and upper limit on the joint position; `eff` and `vel` prescribe limits on the joint's effort and velocity. xx double check; but I beliebe the latter two are overruled when the actuator controllers are turned off and the torques are applied using pybullet's xx function xx

The entry with key `"inertial_values"` is a `dict` that has the following form: 

```python
inertial_values = {
    "ixx" : str(ixx),
    "ixy" : str(ixy),
    "ixz" : str(ixz),
    "iyy" : str(iyy),
    "iyz" : str(iyz),
    "izz" : str(izz)
}
```

Here, `inm` are floats. This only allows for symmetrical inertial matrices (meaning `inm = imn`) - which should be sufficient anyways

xx todo: extend to array of radii, rho, seg_inertial_values for tapered actuators
xx todo: think about more complicated stiffness profiles



## Old notes - Base definition
Manipulators may have an un-actuated base in addition to the actuators. The base properties are defined in `base_params` as follows:

xx todo: extend this to accept various shapes for the base; (do same for actuator cross section)

```python
base_params = {
    "inertial_values" : dict, # inertial values for an individual segment; same format as for actuator segments
    "geometry" : str, # str can be "box", "cylinder", or "sphere"
    "dimensions" : [float], # dimension of an individual actuator element.
    "mass" : float, # mass of the base 
    "inertial_values" : dict, # inertial values for an individual segment 
}
```

xx todo: make base_params and act_params as similar as possible

`"l w h"` can be obtained using the utility function `spaced_str(geom_list)`, where `geom_list` is a list of floats: `[l,w,h]`.

