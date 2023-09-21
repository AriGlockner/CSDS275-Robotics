#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Jun 21 21:59:50 2023

@author: zxc703
"""

import coppeliasim_zmqremoteapi_client as zmq
import matplotlib.pyplot as plt
import numpy as np
import ecse275utils as util
from scipy.spatial.transform import Rotation as R

#%%

def get_range_data(sim,scriptfuncname,transform_pose=None):
    '''
    Gets the range data of the LiDAR from Coppelia Sim

    Parameters
    ----------
    sim : object handle
        the simulation object handle
    scriptfuncname : string
        the name of the function in the childscript of the Hokuyo sensor that gets the lidar points
    transform_pose : 7-element numpy float vector, optional
        A 7-element numpy vector (x,y,z,qx,qy,qz,qw) defining base frame of the sensor w.r.t to the world. The default is None.

    Returns
    -------
    output_robotframe : Array of float64 of dimension (Nx3)
        Array of (x,y,z) positions for N LIDAR obstacle points.

    '''
    
    output = sim.callScriptFunction(scriptfuncname,sim.scripttype_childscript)
    try:
        output_robotframe = np.array(output).reshape((-1,3))
    except:
        output_robotframe = np.zeros_like(output_robotframe)
    
    
    if transform_pose is not None:
        
        robot_rotmat = R.from_quat(transform_pose[3:])
        #robot_angle = robot_rotmat.as_euler('XYZ')*180/np.pi
        output_robotframe = robot_rotmat.apply(output_robotframe) + transform_pose[:3]
        
    return output_robotframe


def set_wheel_velocity(dU,sim,scriptfuncname):
    '''
    
    Sets the wheel target velocities in the speed controller
    
    Parameters
    ----------
    dU : 2-element numpy float vector
        the left and right wheel control inputs.
    sim : object handle
        the simulation object handle
    scriptfuncname : string
        the name of the function in the childscript of the Pure_Robot that set the actuator target efforts in the controller.

    Returns
    -------
    None.

    '''
    sim.callScriptFunction(scriptfuncname,sim.scripttype_childscript,[dU[0],dU[1]])
    
    
def compute_reppotgrad(point,robot_pos,eps=3, min_thresh = 1, max_thresh=15):
    '''
    ****************** FILL THIS IN ******************
    Repulsive gradient computation
    
    Parameters
    ----------
    point : 3-element numpy float vector
        position of the obstacle point in the world frame
    robot_pos : 3-element numpy float vector
        position of the robot in the world frame
    eps : float
        repulsive parameterr.
    min_thresh: float
        minimum threshold distance in (m) for the repulsive for to max out.
    max_thresh: float
        maximum threshold distance in (m) for the repulsive to act
    Returns
    -------
    dU: 2-element float64 numpy vector
        the gradient of the repulsive potential

    '''
    
        
    return dU

def compute_attpotgrad(point,robot_pos,eps1=5, eps2=5, max_thresh=5):
    '''
    ****************** FILL THIS IN ******************
    Attractive gradient computation
    
    Parameters
    ----------
    point : 3-element numpy float vector
        position of the goal point in the world frame
    robot_pos : 3-element numpy float vector
        position of the robot in the world frame
    eps1 : float, optional
        attractive parameter for the quadratic potential. The default is 5.
    eps2 : float, optional
        attractive parameter for the conic potential. The default is 5.
    max_thresh : float, optional
        threshold distance in (m) for the boundardy between the conic and quadratic potential. The default is 5.

    Returns
    -------
    dU : 2-element float64 numpy vector
        the gradient of the attractive potential

    '''
    
    return dU

#%%

run_reactive_control = True
stop_condition = 0.05 #meters

if __name__ == '__main__':
    client = zmq.RemoteAPIClient()
    sim = client.getObject('sim')
    
    # initialize the worldmap from the vision sensor for visualization purposes
    worldmap = util.gridmap(sim,5.0)
    worldmap.normalize_map()
    
    robot = sim.getObjectHandle("/Pure_Robot/Dummy")
    goal = sim.getObjectHandle("/goal_point")
    
    # get the robot pose data
    robot_pose = sim.getObjectPose(robot,sim.handle_world)
    robot_pos_world = robot_pose[:3]
    
    # transform the obstacle data into the world coordinate frame and plot
    worldmap.get_obs_in_world_coords(plot=True)
    # plot the robot position
    plt.plot(robot_pos_world[0],robot_pos_world[1],'.')
    
    # acquire obstacle data from laser range finder in the object frame
    rangedata_worldframe = get_range_data(sim,'getMeasuredData@/fastHokuyo',transform_pose = robot_pose)
    # plot the point data from the laser range finder
    plt.plot(rangedata_worldframe[:,0],rangedata_worldframe[:,1],'.')
    
    while True and run_reactive_control: # this is the while loop that runs the control
        
        # FILL THIS PART IN
        
        # HINT: GET THE RELEVANT POSITION AND LIDAR DATA 
        
        
        # HINT MAKE A IF ELSE CONDITION
        # if we are less than the stop condition slow down to a stop, else navigate to the goal
        
        # set the wheel velocities
        set_wheel_velocity(dU,sim,'set_F@/Pure_Robot')
        
