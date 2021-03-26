Somo Framework Example:

# In-Hand Manipulation: Finger Gait
We show that using a calibrated soft finger definition, SoMo achieves similar task-level results to real hardware. Using the soft hand with four soft fingers from [Abondance et. al 2020](https://ieeexplore.ieee.org/document/9134855), we apply a finger gait for contiunuous object rotation in hardware and compare it to a simulated hand performing the same gait. The results of simulations look visually similar to real hardware, and the resulting object rotation about is within 9% of the real hardware over the whole trajetory. The off-axis motion is also of the same order of magnitude. These results validate SoMo as a physically-acurate soft robot simulator.

## Background

### Finger Calibration
The finger definition in this example is calibrated using the procedure discussed in ["docs/calibration_of_real_world_actuator.pdf"](https://github.com/GrauleM/somo/blob/master/docs/calibration_of_real_world_actuator.pdf).
- Based on the measured stiffness at the fingertip, we created a SoMo actuator definition with 10 segments, and spring-loaded 2DOFs.
- Using blocked force measurments, we obtain the conversion from actuator pressure (in the real system) to actuation torques in the grasping direction (forthe simulated system). That calibration constant is: 100 KPa = 30 "simulation units".
- Based on an estimate of the motion of real hardware in response to differential input pressure, the side-to-side calibration constant for the simulated system is found to be: 100 KPa differential = 90 "imulation torque units".

### Task: Finger Gait for Continuous Rotation
In this example, the soft fingers perform a finger gait developed in [Abondance et. al 2020](https://ieeexplore.ieee.org/document/9134855). The gait results in rotation about the z-axis of the object, and very little off-axis translation or rotation. This task was chosen due to its complex contact events (multiple fingers applying contact to the object at any given time) as a demonstration of the versatillity of the SoMo framework.


## Validation of actuation torque conversions
We validated the actuation conversions from (real) differential pressure to (simulated) orthogonal actuation toqrues. To do this, we used a REAL-LIFE finger gait trajectory (defined in _"actuation/gaits/rotate_gait_sorotraj.yaml"_) and developed a linear conversion for obtaining the simulated torques. The conversion is implemented as a line-by-line trajectory conversion inside of **"run_single.py"** in the function: _"real2somo()"_.

For each finger:
- Main-axis torque = WEIGHT_0 * average of the two differential pressure channels
- Side-to-side axis toqrue = WEIGHT_1 * difference of two channels

To find WEIGHT_0, we use the finger calibration discussed above. The real gait trajectory is defined in PSI (sorry, we are in the USA), so 100 kPa = 14.5 psi = 30 "simulation torque units (STU)". Thus, WEIGHT_0 = 30/14.5  = 2.06 STU/psi

To find the WEIGHT_1, we estimate using the finger calibration discussed above. Thus, WEIGHT_1 = 90/14.5  = -6.2 STU/psi. The weight is negative due to the fact that the differential pressure pushes the finger opposite its direction.

Finally, we validated WEIGHT_1 by performing a sweep over WEIGHT_1 from -4.0 to -8.0 in increments of 0.5. This study can be replicated by running **"run_fingergait_weight_explore.py"**.


## Validation of task-level performance
With calibrated actuators and a trajectory, we demonstrate that the task-evel performance is also similar both qualitatively and quantitatively. On the real system, we performed the finger gait for 10 cycles on a 60mm cube while measuring the cube's 3D pose using [April Tags](https://april.eecs.umich.edu/software/apriltag.html). On the simulated system, we perfomed the same (converted) gait for 10 cycles on a 60mm cube. We then compared the results in a comparison plot as well as the error in the object's angle between simulated and real systems.

To replicate the SoMo part of this study, perform the simulation and export the raw data using **"run_fingergait_tuned.py"**. Once finished, plot the resulting object motion against the motion observed on real harware using the Jupyter Notebook  _"plot_fingergait.ipynb"_. This notebook uses the real measured data from the _"validation"_ folder, and generates comparison plots, storing them in the _"validation"_ folder.


## Generating a video of the resulting finger gait
To build a nice video of the finger gait that we can directly compare with a video of the real hardware, we set up a single experiments, but tested several camera angles to find the one that best displays what's going on. This is implemented as a 1D sweep over the camera pitch. 

To replicate this study, run the file: **"run_fingergait_tuned_video.py"** to regenerate the videos.

_Note: Saving videos using pybullet's builtin video logger attempts to save frames less frequenetly if simulations are running slow, but the resulting videos end up having a significant amount of variation. To sync this with real video, each gait cycle was individually chopped in Adobe Premiere and time-stretched to match the real video. Also, pybullet's builtin video logger only works reliably on Ubuntu._

## Generating pictures of the hand for a cover photo
We mirrored a picture taken of the real hardware with a rubiks cube sitting on the palm. We imported a rubiks cube from the [YCB Object Set](http://www.ycbbenchmarks.org), and tested several camera angles to find the one that matches the real photo closely-enough. We did not spend any time calibrating the simulated camera characteristics, so the resulting photos look a bit different, but they are close enough.

To replicate this study, run the file: **"run_fingergait_tuned_rubiks.py"** to regenerate the videos.
