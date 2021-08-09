# Todos

## URGENT
- [ ] major problem: inconsistency in applying torques (see note below)
- [ ] make sure all forced assert statements use 'assert False, f"error message"' 
- look at saefty controller properties of joints
- fix where urdfs are saved when they are being generated (see complaint from Tom)
- revisit 'link definition origin' vs 'origin_offset' for both assembly and old CM urdf generation
- add option to set joint types as passive; generally revise joint definitions to make everything more userfriendly
- i think i found a cleaner way to stack elements - see recently pushed 'simple_angled_manipulator_debug.urdf' that is different from how the manipulators are currently being grown. use this to simplify code.
- [ ] limit_force = 0. leads to drastic speed-up of simulation (~2x) - find out why and document
- [ ] something looks funny for constrained and free baseConstraint
- [ ] improve loading of definitions from yaml and/or json and mix and match (combine long format for manipulator with short links to individual actuators); make it easier to define a joint or link once and then reuse it

### note: inconsistency in applying torques
the following commands (if used in bball example) lead to different results in the simulation. the results are also different from the old 'apply_actuationTorque' function.
```python
arm.apply_actuation_torques(
    actuator_nrs=[act,act], axis_nrs=[0,1], actuation_torques=torques[2 * act:2 * act+2].tolist()
)
```
```python
arm.apply_actuation_torques(
    actuator_nrs=[act], axis_nrs=[0], actuation_torques=[float(torques[2 * act])]
)
arm.apply_actuation_torques(
    actuator_nrs=[act], axis_nrs=[1], actuation_torques=[float(torques[2 * act + 1])]
)
```
This command achieves (almost) the previous behavior, but is again different from both the examples shown above...
```python
arm.apply_actuation_torques(
    actuator_nrs=[0,0,1,1,2,2,3,3,4,4], axis_nrs=[0,1,0,1,0,1,0,1,0,1], actuation_torques=torques.tolist()
)
```
between old and new somo backend, damping also seems to be different a little, even with the 3rd option that is otherwise very similar to the old example.


## General
- [ ] clean up readme
- [ ] clean up examples
- [ ] mention how joint_control_limit_force relates to damping
- [ ] consider adding a direct damping term into the joint force computation
- [ ] accelerating the simulations in RL: for restart: consider not re-initializing fully (not loadURDF) but just resetting the environment to start state instead.
- [ ] changing json/yaml definitions while env training is running affects experiments. this should clearly not be the case. general todo: look more carefully at saving/creating/using urdf
- [ ] fix where things are loaded from in somo - should only be defined once such that we don't need to write definitions/manipulator_definition
- [ ] maybe: add note in readme recommending json validation with this tool https://jsonlint.com/ ; consider implementing json validation
- [ ] check how `p.setPhysicsEngineParameter(enableFileCaching=0)` affects simulation speed
- 

## Expanding capabilities
- [ ] URGENT: add non-symmetric stiffness/actuation
- [ ] maybe: change apply_actuation_torques fn to take lambda fn as input.
- [ ] change apply_actuationTorque fn to take lambda fn as input
- [ ] pyQT sliders for control? or debug variables.
- [ ] introduce manipulator groups, such that hands can be moved around as a unit (requested by many, including Dave C.)
- [ ] calibration routine similar to webots approach
- [ ] sensor placement, simulated sensors

## Gazebo support

## Webots support

## Better module packaging
- [ ] get minimal requirements so they can be listed in setup.py
- [ ] set up read the docs

## Setting up 'read the docs'
- [ ] plenty of documentation. document each function.

## Testing
- [ ] code coverage badge
- [ ] expand list of desired tests

### Desired tests
- [ ] test all schemas: good schemas work, bad schemas have to throw errors
- [ ] test manipulator instantiation for various parameters
- [ ] test all link types (capsule, stadium, cylindrical, box, spherical)
- [ ] various axes offsets
- [ ] run a few full simulations

## General code base improvements
- [ ] introduce marshmallow schemas for the definitions
- [ ] change how zero torques are applied. if they should be applied, simply expand the list of actuators and torques.
- [ ] add decorators for inputs and outputs to all functions

## For the ICRA paper

