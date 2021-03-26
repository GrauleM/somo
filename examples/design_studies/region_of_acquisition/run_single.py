# Be sure to run this file from the "region_of_acquisition" folder
#     cd examples/design_studies/region_of_acquisition

import pybullet as p
import pybullet_data

import time  # for waiting
import numpy as np
import matplotlib.pyplot as plt

import os
import sys
import sorotraj
import pandas as pd
from datetime import datetime

from somo.sm_manipulator_definition import SMManipulatorDefinition
from somo.sm_actuator_definition import SMActuatorDefinition
from somo.sm_link_definition import SMLinkDefinition
from somo.sm_joint_definition import SMJointDefinition
from somo.sm_continuum_manipulator import SMContinuumManipulator

from somo.utils import load_constrained_urdf, dict_from_file

from somo.sweep import iter_utils

path=os.path.abspath(os.path.join(os.path.dirname( __file__ ), 'labels'))
sys.path.insert(0, path)
import label_functions


# Generate the motion functions using sorotraj
def get_motion_functions(motion_config, default_inv = True, graph=False):
    traj_file = motion_config['file']
    traj_build = sorotraj.TrajBuilder(graph=graph)
    traj_build.load_traj_def(traj_file)
    traj = traj_build.get_trajectory()
    interp = sorotraj.Interpolator(traj)
    actuation_fn = interp.get_interp_function(
                    num_reps = motion_config.get('num_reps',1.0),
                    speed_factor = motion_config.get('speed_factor',1.0),
                    invert_direction= motion_config.get('invert_direction',default_inv),
                    as_list=True)

    cycle_fn = interp.get_cycle_function(
                    num_reps = motion_config.get('num_reps',1.0),
                    speed_factor = motion_config.get('speed_factor',1.0),
                    invert_direction= motion_config.get('invert_direction',default_inv),
                    as_list=True)

    final_time = interp.get_final_time()

    motion_fn = { 'actuation_fn' : actuation_fn,
                  'cycle_fn'  : cycle_fn,
                  'final_time': final_time
                }

    return motion_fn


# Run a single simulation
def run_single(in_args):
    filename = in_args['filename']
    index = in_args['index']
    replace_existing = in_args.get('replace',True)
    print(filename)
    config = iter_utils.load_yaml(filename)

    # Set the data logging locations
    all_filenames = iter_utils.load_yaml('save_paths.yaml')
    data_filenames = all_filenames['data']
    
    log_filename, _ = iter_utils.generate_save_location(config, data_filenames['objectpose'])
    log_filename2, _ = iter_utils.generate_save_location(config, data_filenames['contact'])
    calc_filename, _ = iter_utils.generate_save_location(config, data_filenames['calculated'])
    actuation_filename, _ = iter_utils.generate_save_location(config, data_filenames['actuation'])
    vid_filename, _  = iter_utils.generate_save_location(config, data_filenames['video'])

    vid_filename = vid_filename.replace(" ", "\\ ")

    if os.path.exists(log_filename) and not replace_existing:
        return


    # Prepare everything for the physics client
    if config['simulation']['gui']:
        cam_settings = config['simulation'].get('camera',None)
        # Open the gui with a white background and no ground grid
        opt_string = '--background_color_red=1.0 --background_color_green=1.0 --background_color_blue=1.0'
        cam_width = cam_settings.get('width',None)
        cam_height = cam_settings.get('height',None)
        if cam_width is not None and cam_height is not None:
            opt_string+=' --width={} --height={}'.format(cam_settings['width'],cam_settings['height'])

        physicsClient = p.connect(p.GUI, options=opt_string)

        p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)
        if cam_settings is not None:
            p.resetDebugVisualizerCamera( cameraDistance=cam_settings['distance'],
                                          cameraYaw=cam_settings['yaw'],
                                          cameraPitch=cam_settings['pitch'],
                                          cameraTargetPosition=cam_settings['target'])
    else:
        physicsClient = p.connect(p.DIRECT)
    p.setGravity(0, 0, config['simulation']['gravity'])
    p.setPhysicsEngineParameter(enableConeFriction=1)
    p.setRealTimeSimulation(0)  # only if this is set to 0 and the simulation is done with explicit steps will the torque control work correctly
    p.setPhysicsEngineParameter(enableFileCaching=0) # x todo: check if this makes things faster

    # Load the ground plane into pybullet (a custom white ground plane with no grid)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())  # defines the path used by p.loadURDF
    planeId = p.loadURDF("objects/plane/plane.urdf", flags = p.URDF_USE_MATERIAL_COLORS_FROM_MTL)


    # Build the trajectories
    finger_motion_config = config['manipulator']['motion']['fingers']
    finger_motion_fn = get_motion_functions(finger_motion_config, default_inv=True)
    finger_torque_fn  = finger_motion_fn['actuation_fn']
    finger_cycle_fn   = finger_motion_fn['cycle_fn']
    finger_final_time = finger_motion_fn['final_time']
    
    hand_motion_config = config['manipulator']['motion']['hand']
    hand_motion_fn  = get_motion_functions(hand_motion_config, default_inv=False)
    hand_height_fn  = hand_motion_fn['actuation_fn']
    hand_cycle_fn   = hand_motion_fn['cycle_fn']
    hand_final_time = hand_motion_fn['final_time']


    # Set the simulation times
    final_time = float(np.max([finger_final_time,hand_final_time]))

    time_step = config['simulation']['time_step']
    p.setTimeStep(time_step)
    n_steps = int(final_time/time_step)
    check_time = 0.5 # seconds
    check_iter = int(check_time/time_step)

    global_scale = float(config['setup'].get('global_scale',1.0))

    # Manually prescribe the finger start positions and orientations
    finger_poses = config['manipulator']['fingers']['poses']
    finger_offset = config['manipulator']['fingers']['offset']
    hand_offset = config['manipulator']['hand']
    
    fingerStartPositions = []
    fingerStartOrientations = []
    for finger_pose in finger_poses:
        finger_position = np.array(finger_pose['position']) + np.array(hand_offset['position']) + np.array(finger_offset['position'])
        fingerStartPositions.append(finger_position)
        ori = p.getQuaternionFromEuler(np.deg2rad(finger_pose['orientation']))
        fingerStartOrientations.append(ori)


    # Load the manipulator definition
    manipulator_def_file = config['manipulator']['fingers']['file']
    manip_def = dict_from_file(manipulator_def_file)

    # update finger def params to change the DSR
    dsr=config['manipulator']['fingers']['distal_seg_ratio']

    link_len = float(manip_def['actuator_definitions'][0]['link_definition']['dimensions'][2])
    tot_len = float(manip_def['actuator_definitions'][0]['actuator_length'] + manip_def['actuator_definitions'][1]['actuator_length'])

    proximal_len = (1-dsr)*tot_len
    distal_len = dsr*tot_len

    num_proximal_links = round(proximal_len/link_len)
    num_distal_links = round(distal_len/link_len) 

    print(num_proximal_links)
    print(num_distal_links)

    if num_proximal_links <= 1:
        raise(ValueError("FAIL: Only one or fewer proximal links would be created"))

    elif num_distal_links <= 1:
        print( "Only one or fewer distal links would be created, so removing distal link")
        proximal_len       += link_len*num_distal_links
        num_proximal_links += num_distal_links

        manip_def['actuator_definitions'][0]['actuator_length'] = proximal_len
        manip_def['actuator_definitions'][0]['n_segments'] = num_proximal_links
        del manip_def['actuator_definitions'][1]
        manip_def['n_act'] = 1
    
    else:
        manip_def['actuator_definitions'][0]['actuator_length'] = proximal_len
        manip_def['actuator_definitions'][1]['actuator_length'] = distal_len

        manip_def['actuator_definitions'][0]['n_segments'] = num_proximal_links
        manip_def['actuator_definitions'][1]['n_segments'] = num_distal_links
    

    # Check for existing instances of the urdf, and append number to it
    old_urdf=os.path.join('_tmp',iter_utils.auto_inc_file(manip_def['urdf_filename'], index=index) )
    
    # Pass the dictionary of manipulator defimition parameters as arguments
    # to the SoMo manipulator definition class

    fingers = []
    for i in range(len(finger_poses)):
        manip_def['urdf_filename'] = iter_utils.auto_inc_file(old_urdf)
        finger_manipulator_def = SMManipulatorDefinition(**manip_def)
        fingers.append(SMContinuumManipulator(finger_manipulator_def))
    


    # Create the manipulators
    #fingers = [SMContinuumManipulator(finger_manipulator_def) for i in range(len(finger_poses))]

    fingerIDs = []
    num_segs = []

    hand_motion_start = np.array([hand_height_fn[i](0) for i in range(len(hand_height_fn))])

    for finger, startPos, startOr in zip(fingers, fingerStartPositions, fingerStartOrientations):
        startPos_actuated = np.array(startPos)+hand_motion_start
        finger.load_to_pybullet(
                baseStartPos=startPos_actuated.tolist(),
                baseStartOrn=startOr,
                baseConstraint='constrained',
                physicsClient=physicsClient)

        total_actuators = p.getNumJoints(finger.bodyUniqueId)
        for idx in range(total_actuators):
            p.changeDynamics(finger.bodyUniqueId, idx, lateralFriction=config['manipulator']['fingers']['lateral_friction'])
            p.changeDynamics(finger.bodyUniqueId, idx, restitution=config['manipulator']['fingers']['restitution']) 

            p.changeDynamics(finger.bodyUniqueId, idx, frictionAnchor=True)
            p.changeDynamics(finger.bodyUniqueId, idx, collisionMargin=0.0001)

            if config['manipulator']['fingers'].get('spinning_friction',None) is not None:
                p.changeDynamics(finger.bodyUniqueId, idx, spinningFriction=config['manipulator']['fingers']['spinning_friction'])
        

        fingerIDs.append(finger.bodyUniqueId)
        num_segs.append(p.getNumJoints(finger.bodyUniqueId))

    #Save calculated parameters
    calculated_params = {'num_finger_segs':num_segs}
    iter_utils.save_yaml(calculated_params,calc_filename)

    # Load the object
    object_file = config['object']['file']
    objStartPos = config['object']['position']
    objStartOr = p.getQuaternionFromEuler(np.deg2rad(config['object']['orientation']))
    objId = p.loadURDF(object_file, objStartPos, objStartOr)
    p.changeDynamics(objId, -1, lateralFriction=config['object']['lateral_friction'])
    p.changeDynamics(objId, -1, restitution=1.0)
    p.changeDynamics(finger.bodyUniqueId, -1, collisionMargin=0.0001)

    obj_zoffset=config['object'].get('above_ground',None)
    if obj_zoffset is not None:
        # Get the bounding box of the object and palm, then spawn the object resting on the palm.
        bbox = list(p.getAABB(objId))
        obj_min = bbox[0][2]
        palm_max=0.0
        tot_obj_zoffset = palm_max-obj_min + obj_zoffset
        box_pos = list(objStartPos)
        box_pos[2] = box_pos[2]+ tot_obj_zoffset
        p.resetBasePositionAndOrientation(objId, tuple(box_pos), objStartOr )

        
    # Set up data logging
    objects_to_log = [objId]
    logID = p.startStateLogging(p.STATE_LOGGING_GENERIC_ROBOT, log_filename, objectUniqueIds = objects_to_log)
    logIDContact = p.startStateLogging(p.STATE_LOGGING_CONTACT_POINTS, log_filename2, objectUniqueIds = fingerIDs)

    if config['simulation'].get('video',None) and config['simulation']['gui']:
        logIDvideo = p.startStateLogging(p.STATE_LOGGING_VIDEO_MP4, vid_filename)

    # Run the simulation
    sim_time = 0
    real_time = time.time()

    for i in range(n_steps):
        if np.mod(i+1,check_iter)==0:
            curr_time = time.time()
            ms_elapsed = (curr_time-real_time)
            print("Simulation running at %0.1f %% of realtime"%(check_time/ms_elapsed*100))
            real_time = curr_time


        hand_offset=np.array([hand_height_fn[i](sim_time) for i in range(len(hand_height_fn))])

        for finger, finger_idx, fingerStartPosition in zip(fingers, range(len(fingers)), fingerStartPositions):

            # Apply actuation torques to the fingers
            total_actuators = len(finger.actuators)
            finger.apply_actuation_torques(actuator_nrs=[0], axis_nrs=[0], actuation_torques=[float(finger_torque_fn[2*finger_idx](sim_time))])
            finger.apply_actuation_torques(actuator_nrs=[0], axis_nrs=[1], actuation_torques=[float(finger_torque_fn[2*finger_idx+1](sim_time))])

            # Update the finger position
            fingerPosition_new = fingerStartPosition+hand_offset
            p.changeConstraint(finger.baseConstraintUniqueId, fingerPosition_new)


        p.stepSimulation()
        sim_time += time_step

    p.stopStateLogging(logID)
    p.stopStateLogging(logIDContact)

    if config['simulation'].get('video',None) and config['simulation']['gui']:
        p.stopStateLogging(logIDvideo)
    p.disconnect()

    iter_utils.log_actuation(log_filename, actuation_filename, 
                  actuation_fn={'fingers': finger_torque_fn, 'hand':hand_height_fn},
                  cycle_fn=    {'fingers': finger_cycle_fn,  'hand':hand_cycle_fn} )


if __name__ == '__main__':
    filename = 'config_test.yaml'
    try:
        iter_utils.add_tmp('_tmp')
        start = time.time()
        run_single({'filename':filename, 'index': 0})
        end = time.time()
        iter_utils.delete_tmp('_tmp')

        print('____________________________')
        print("TOTAL TIME: %0.1f sec (%0.2f min)"%((end-start), (end-start)/60))
        print('____________________________')

    except:
        print('\n'+"RUN TERMINATED EARLY")
        iter_utils.delete_tmp('_tmp')
        raise