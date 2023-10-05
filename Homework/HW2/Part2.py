#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Aug 14 19:28:07 2023

@author: ajg228
"""

import coppeliasim_zmqremoteapi_client as zmq
import matplotlib.pyplot as plt
import numpy as np
import ecse275utils as util


def rep_hyper_vec(distances, eta=1, max_dist=2, min_dist=1):
    """
    Question 1
    hyperbolic repulsive field function

    Parameters
    ----------
    distances : (H x W x Num Obstacle Points) float array
        Array holding the distances to each obstacle for every cell in the grid world.
    eta : float, optional
        parameter for the repulsive field. The default is 1.
    max_dist : int, optional
        maximum distance of the repulsive field. The default is 2.
    min_dist : int, optional
        minimum distance threshold for the repulsive field to become a constant value. The default is 1.

    Returns
    -------
    u : TYPE
        DESCRIPTION.

    """
    # initialize the output array
    u = np.zeros_like(distances)

    # distance >= the max distance -> 0
    u[distances >= max_dist] = 0

    # distance between the min and max distance -> hyperbolic function
    u[(min_dist <= distances) & (distances < max_dist)] = (
            eta * (1 / distances[(min_dist <= distances) & (distances < max_dist)] - 1 / max_dist))

    # distances < the min distance -> constant value
    u[distances < min_dist] = eta * (1 / min_dist - 1 / max_dist)

    # return the output array
    return u


def att_quadcone_vec(dist, eta1=1, eta2=10, thresh=2):
    """
    Question 2
    quadratic-conic attractive field function

    Parameters
    ----------
    dist : (H x W) float array
        Array holding the distances to the goal for every cell in the grid world.
    eta1 : float, optional
        parameter for the quadratic attractive field. The default is 1.
    eta2 : float, optional
        parameter for the conic attractive field.. The default is 10.
    thresh : int, optional
        the threshold distance at which the field goes from quadratic to conic. The default is 2.

    Returns
    -------
    TYPE
        DESCRIPTION.

    """
    # initialize the output array
    u = np.zeros_like(dist)

    # distance >= the threshold distance -> conic function
    u[dist >= thresh] = eta2 * dist[dist >= thresh] - eta2 * thresh + eta1 * thresh ** 2

    # distance < the threshold distance -> quadratic function
    u[dist < thresh] = eta1 * dist[dist < thresh] ** 2

    # return the output array
    return u


# %%
if __name__ == '__main__':
    # RUN THE MAIN FUNCTION TO COMPUTE THE POTENTIAL FIELD, GRADIENTS AND GRADIENT DESCENT FOR QUESTIONS 3 ONWARDS.

    # DON'T MODIFY THIS PART OF THE CODE #

    client = zmq.RemoteAPIClient()
    sim = client.getObject('sim')

    # grab the world map from CoppeliaSim
    worldmap = util.gridmap(sim, 5.0)
    # worldmap.inflate_obstacles(num_iter=10)
    worldmap.normalize_map()

    # grab the robot and goal positions
    goal = sim.getObjectHandle("/goal_point")
    goal_world = sim.getObjectPosition(goal, sim.handle_world)
    goal_grid = worldmap.get_grid_coords(goal_world)
    worldmap.plot_point(goal_grid)
    robot = sim.getObjectHandle("/Pure_Robot/Dummy")
    start_world = sim.getObjectPosition(robot, sim.handle_world)
    start_grid = worldmap.get_grid_coords(start_world)
    worldmap.plot_point(start_grid, color='red')

    # compute a grid of distance from each occupied cell
    grid_height, grid_width = worldmap.gridmap.shape
    eval_points = np.indices((grid_height, grid_width), dtype=float).transpose(1, 2, 0)  # create the evaluation grid
    obs_idx = np.argwhere(worldmap.norm_map == 1)  # get index of the map where the obstacles are
    distances_rep = np.linalg.norm(np.expand_dims(eval_points, axis=2) - np.expand_dims(obs_idx, axis=0),
                                   axis=3)  # compute distance of each occupied point w.r.t to the evaluation grid

    distances_att = np.linalg.norm(np.expand_dims(eval_points, axis=2) - np.expand_dims(goal_grid, axis=0),
                                   axis=-1)  # apply the
    distances_att = np.reshape(distances_att, (grid_height, grid_width))

    # %%

    # QUESTION 7
    # YOU CAN ALTER THE PARAMETERS HERE! ************
    potentials = np.apply_along_axis(rep_hyper_vec, axis=-1, arr=distances_rep, eta=1, max_dist=15, min_dist=1)

    repulsive_field = np.sum(potentials, axis=-1)  # don't need to change this.

    attractive_field = att_quadcone_vec(distances_att, eta1=0.1, eta2=0.2, thresh=5)  # you can change this

    # END OF MODIFICATION ZONE ***********************

    world_u = attractive_field + repulsive_field

    # do some visualizations
    util.visualize_potential_field_2D(world_u)
    util.visualize_potential_field_3D(world_u)

    # do discrete gradient descent and compute the paths
    world_ugrad = util.compute_discrete_gradient(world_u)
    util.visualize_gradient(worldmap.norm_map, world_ugrad.transpose(1, 2, 0), np.flip(goal_grid))

    robot_pos = sim.getObjectPosition(robot, sim.handle_world)
    robot_pos_grid = worldmap.get_grid_coords(robot_pos)

    # QUESTION 8
    # this line computes gradient descent. you can run this individually
    path = util.discrete_grad_descent(robot_pos_grid, goal_grid, world_ugrad, max_iter=300)

    util.visualize_potential_field_2D(world_u)
    worldmap.plot(normalized=True)
    util.plot_gradient_descent(plt.gca(), path)
    plt.show()
# %%
