#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Jun 21 21:59:50 2023

@author: ajg228
"""
import math

import coppeliasim_zmqremoteapi_client as zmq
import matplotlib.pyplot as plt
import numpy as np
import ecse275utils as util
from scipy.spatial.transform import Rotation as R


# %%

def get_range_data(sim, scriptfuncname, transform_pose=None):
    """
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

    """

    output = sim.callScriptFunction(scriptfuncname, sim.scripttype_childscript)
    try:
        output_robotframe = np.array(output).reshape((-1, 3))
    except:
        output_robotframe = np.zeros_like(output_robotframe)

    if transform_pose is not None:
        robot_rotmat = R.from_quat(transform_pose[3:])
        # robot_angle = robot_rotmat.as_euler('XYZ')*180/np.pi
        output_robotframe = robot_rotmat.apply(output_robotframe) + transform_pose[:3]

    return output_robotframe


def set_wheel_velocity(dU, sim, scriptfuncname):
    """

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

    """
    sim.callScriptFunction(scriptfuncname, sim.scripttype_childscript, [dU[0], dU[1]])


def compute_reppotgrad(point, robot_pos, eps=3, min_thresh=1, max_thresh=15):
    """
    Repulsive gradient computation

    Parameters
    ----------
    point : 3-element numpy float vector
        position of the obstacle point in the world frame
    robot_pos : 3-element numpy float vector
        position of the robot in the world frame
    eps : float
        repulsive parameter.
    min_thresh: float
        minimum threshold distance in (m) for the repulsive for to max out.
    max_thresh: float
        maximum threshold distance in (m) for the repulsive to act
    Returns
    -------
    dU: 2-element float64 numpy vector
        the gradient of the repulsive potential

    """

    # Compute the distance between the robot and the obstacle point
    diff = [point[0] - robot_pos[0], point[1] - robot_pos[1]]
    dist = math.sqrt(diff[0] ** 2 + diff[1] ** 2)

    # distance is too far or too close -> no repulsive force
    if dist > max_thresh or dist < min_thresh:
        return np.zeros(2)
    # distance is between the min and max threshold -> repulsive force
    else:
        return eps * (1 / dist - 1 / max_thresh) ** 2 * (1 / dist ** 3) * np.array(diff)


def compute_attpotgrad(point, robot_pos, eps1=5, eps2=5, max_thresh=5):
    """
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
        threshold distance in (m) for the boundary between the conic and quadratic potential. The default is 5.

    Returns
    -------
    dU : 2-element float64 numpy vector
        the gradient of the attractive potential

    """

    # Compute the distance between the robot and the goal point
    diff = [point[0] - robot_pos[0], point[1] - robot_pos[1]]
    dist = math.sqrt(diff[0] ** 2 + diff[1] ** 2)

    # distance is too far -> no attractive force
    if dist > max_thresh:
        return np.zeros(2)
    # distance is between the min and max threshold -> attractive force -> conic potential
    elif dist > 1:
        return eps2 ** diff // dist
    # distance is between the min and max threshold -> attractive force -> quadratic potential
    else:
        return eps1 ** diff


# %%
run_reactive_control = True
stop_condition = 0.05  # meters

if __name__ == '__main__':
    client = zmq.RemoteAPIClient()
    sim = client.getObject('sim')

    # initialize the worldmap from the vision sensor for visualization purposes
    worldmap = util.gridmap(sim, 5.0)
    worldmap.normalize_map()

    robot = sim.getObjectHandle("/Pure_Robot/Dummy")
    goal = sim.getObjectHandle("/goal_point")

    # get the robot pose data
    robot_pose = sim.getObjectPose(robot, sim.handle_world)
    robot_pos_world = robot_pose[:3]

    # transform the obstacle data into the world coordinate frame and plot
    worldmap.get_obs_in_world_coords(plot=True)
    # plot the robot position
    plt.plot(robot_pos_world[0], robot_pos_world[1], '.')

    # acquire obstacle data from laser range finder in the object frame
    rangedata_worldframe = get_range_data(sim, 'getMeasuredData@/fastHokuyo', transform_pose=robot_pose)
    # plot the point data from the laser range finder
    plt.plot(rangedata_worldframe[:, 0], rangedata_worldframe[:, 1], '.')

    while run_reactive_control:  # this is the while loop that runs the control
        # Get the relevant position and LiDAR data
        robot_pose = sim.getObjectPose(robot, sim.handle_world)
        robot_pos_world = robot_pose[:3]
        rangedata_worldframe = get_range_data(sim, 'getMeasuredData@/fastHokuyo', transform_pose=robot_pose)

        # Compute the attractive potential gradient
        # compute_attpotgrad(point, robot_pos, eps1=5, eps2=5, max_thresh=5)
        dU_att = compute_attpotgrad(sim.getObjectPosition(goal, sim.handle_world), robot_pos_world, eps1=3, eps2=2,
                                    max_thresh=5000)

        # Compute the repulsive potential gradient
        dU_rep = np.zeros(2)
        for obstacle in rangedata_worldframe:
            # compute_reppotgrad(point, robot_pos, eps=3, min_thresh=1, max_thresh=15):
            dU_rep += compute_reppotgrad(obstacle, robot_pos_world, eps=2, min_thresh=1, max_thresh=2)

        # Compute the total potential gradient
        dU = dU_att + dU_rep

        # Check if the robot is close enough to the goal
        if np.linalg.norm(dU_att) < stop_condition:
            run_reactive_control = False

        # Set the wheel velocity
        set_wheel_velocity(dU, sim, 'set_F@/Pure_Robot')

    # stop the robot
    set_wheel_velocity(np.zeros(2), sim, 'set_F@/Pure_Robot')
