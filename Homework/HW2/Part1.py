#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Jul  1 17:16:06 2023

@author: ajg228
"""
import coppeliasim_zmqremoteapi_client as zmq
import matplotlib.pyplot as plt
import numpy as np
import ecse275utils as util


# %%
class ASTAR:
    """
    The key object here is the state object which is a dictionary to track our state

    it comprises the following
    'explored_set' is a list for you to populate with individual tuples of (row,column) index for nodes that have been
    explored
    'open_set' is a list for you to populate with nodes that have yet to be explored. We initialized it for you with
    one element, the start point.
    'world_map' is the grid map that contains information about the grid world, obstacles are 1, and the goal is 2,
    open space is 0
    'world_map_g' is the grid that should store the cost to go values corresponding to that row and column in the
    world_map
    'world_map_h' is the grid that should store the cost to reach/come values corresponding to that row and column in
    the world_map
    'parents_map' is a 4 dimensional array, the first two dimensions are the same dimensions as the world map,
    the next two are for storing the row column values corresponding to the parent cell for that given position
    in the grid map
    """

    def __init__(self, world_map, start, end):
        """
        Parameters
        ----------
        world_map :gridmap object
            This object is generated by the ECSE275 Util Library and contains the world information
        start : 2-element tuple
            start point x and y pixel coordinates
        end : 2-element tuple
            end point in x and y pixel coordinates

        Returns
        -------
        None.

        """

        self.state = {'explored_set': [],
                      'open_set': [list(np.flip(start))],
                      'world_map': world_map,
                      'world_map_g': np.zeros_like(world_map),
                      'world_map_h': np.zeros_like(world_map),
                      'parents_map': np.zeros((world_map.shape[0], world_map.shape[1], 2))}

        self.world_dims = world_map.shape
        self.end_point = np.flip(end)
        self.start_point = np.flip(start)
        self.traced_path_rc = None
        self.trace_path_xyzq = None

        self.state['world_map'][self.end_point[0], self.end_point[1]] = 2
        self.state['world_map_h'][self.start_point[0], self.start_point[1]] = self.get_cost_to_goal(self.start_point[0],
                                                                                                    self.start_point[1])

    def get_parent_point(self, gridrow, gridcol):
        """
        Parameters
        ----------
        gridrow : int
            the row position of the cell you are querying
        gridcol: int
            the column position of the cell you are querying
        Returns
        -------
        tuple of the parent cell location as a row and column in the grid world

        """

        parent_r = int(self.state['parents_map'][gridrow, gridcol][0])
        parent_c = int(self.state['parents_map'][gridrow, gridcol][1])

        return parent_r, parent_c

    def set_parent_point(self, gridrow_parent, gridcol_parent):
        """
        Parameters
        ----------
        gridrow_parent : int
            the row location of the parent cell
        gridcol_parent : int
            the column location of the parent cell

        Returns
        -------
        2-element numpy array
            it packages the information into the numpy array for storage.

        """

        return np.array([gridrow_parent, gridcol_parent])

    def get_cost_to_reach(self, gridrow_parent, gridcol_parent):
        """

        FILL THIS IN

        Parameters
        ----------
        gridrow_parent : int
            row location of the parent cell
        gridcol_parent : int
            column location of the parent cell

        Returns
        -------
        float
            the cost to reach the current cell

        """

        # return self.state['world_map_h'][gridrow_parent, gridcol_parent] + 1
        return abs(gridrow_parent - self.start_point[0]) + abs(gridcol_parent - self.start_point[1])

    def get_cost_to_goal(self, gridrow_parent, gridcol_parent):
        """
        Compute this using the Manhattan distance

        Parameters
        ----------
        gridrow_parent : int
            row location of the parent cell
        gridcol_parent : int
            column location of the parent cell

        Returns
        -------
        float
            the estimated cost to reach the goal

        """

        return abs(gridrow_parent - self.end_point[0]) + abs(gridcol_parent - self.end_point[1])

    def search_surrounding_nodes(self, gridrow, gridcol):
        """
        Function to add the surrounding nodes around a cell into the open set.

        1. For each node surrounding the current node do the following:
            a. check if it is in the workspace bounds
            b. check if it is already explored or in the open set
            c. if it is not then add it to the open set
            d. calculate the cost to go and the cost to come and store them

        2. Remove the current node from the open set and add it to the explored set

        Parameters
        ----------
        gridrow : int
            the row position of the cell you are querying
        gridcol: int
            the column position of the cell you are querying

        Returns
        -------
        None.

        """
        # Define possible moves (up, down, left, right)
        moves = [(0, 1), (0, -1), (1, 0), (-1, 0)]

        # Check neighboring nodes
        for move in moves:
            new_row, new_col = gridrow + move[0], gridcol + move[1]

            # Check if the node is in the workspace bounds
            if 0 <= new_row < self.world_dims[0] and 0 <= new_col < self.world_dims[1]:
                # Check if the node is already explored or in the open set
                if self.state['world_map'][new_row, new_col] != 1 and [new_row, new_col] not in self.state[
                    'explored_set'] and [new_row, new_col] not in self.state['open_set']:
                    # Add the node to the open set
                    self.state['open_set'].append([new_row, new_col])

                    # Calculate the cost to go and the cost to come and store them
                    self.state['world_map_g'][new_row, new_col] = (self.get_cost_to_reach(new_row, new_col)
                                                                   + self.get_cost_to_goal(new_row, new_col))
                    # Add the parent node to the parents map
                    self.state['parents_map'][new_row, new_col] = self.set_parent_point(gridrow, gridcol)

        # Remove the current node from the open set and add it to the explored set
        self.state['open_set'].remove([gridrow, gridcol])
        self.state['explored_set'].append([gridrow, gridcol])

    def process_node(self, gridrow, gridcol):
        """
        ****************** FILL THIS IN ******************

        In this function we want to

        1. check if we reach the goal
        2. else check if we hit an obstacle, if so remove it from the open set and immediately add it to the explored set
        3. else execute the search of surrounding nodes function


        Parameters
        ----------
        gridrow : int
            the row position of the cell you are querying
        gridcol: int
            thee column position of the cell you are querying

        Returns
        -------
        int
            the state of the cell, either 2 for goal, 1 for obstacle, 0 for empty space
        """

        # Check if we reach the goal
        if gridrow == self.end_point[0] and gridcol == self.end_point[1]:
            return 2
        # Check if we hit an obstacle
        if self.state['world_map'][gridrow, gridcol] == 1:
            self.state['explored_set'].append([gridrow, gridcol])
            self.state['open_set'].remove([gridrow, gridcol])
            return 1
        # Execute the search of surrounding nodes function
        self.search_surrounding_nodes(gridrow, gridcol)
        return 0

    def visualize_state(self):
        """

        Returns
        -------
        None.

        """

        for pt in self.state['explored_set']:
            plt.plot(pt[1], pt[0], '.', markersize=4, color="yellow")
        for pt in self.state['open_set']:
            plt.plot(pt[1], pt[0], '.', markersize=2, color="green")

        if self.traced_path_rc is not None:
            trace_vector = np.array(self.traced_path_rc)
            plt.plot(trace_vector[:, 1], trace_vector[:, 0], linewidth=2)

    def run(self, max_iter=1000):
        """
        ****************** FILL THIS IN ******************

        This function will run ASTAR by
        1. Checking the open set and ranking the nodes by the cost.
        2. Process the lowest cost node until we find the goal or hit the maximum number of iterations
        3. Once this is done the path through the grid map must be traced backward by looking up the parent nodes until the start point
        4. Return the paths


        Parameters
        ----------
        max_iter : int, optional
            number of iterations to run A STAR The default is 1000.

        Returns
        -------
        list
            sequence of 2D points (row,column) for each cell of the path found by ASTAR
        """
        print('Start Point', self.start_point)
        print('End Point', self.end_point)

        while max_iter > 0:
            max_iter -= 1

            # Check if the open set is empty
            if len(self.state['open_set']) == 0:
                return []

            # Process the lowest cost node
            node = self.state['open_set'][0]
            for open_node in self.state['open_set']:
                if self.state['world_map_g'][open_node[0], open_node[1]] < self.state['world_map_g'][node[0], node[1]]:
                    node = open_node

            # Check if we reach the goal
            if self.process_node(node[0], node[1]) == 2:
                self.traced_path_rc = self.trace_path(node[0], node[1])
                return self.traced_path_rc

        print("Max iteration reached")
        return []

    def trace_path(self, x, y):
        """
        This function computes the path to the goal node once the goal has been reached
        :param x: the x coordinate of the current node
        :param y: the y coordinate of the current node
        :return: the path from the start node to the goal
        """
        if x == self.start_point[0] and y == self.start_point[1]:
            return [[x, y]]
        else:
            return [[x, y]] + self.trace_path(self.get_parent_point(x, y)[0], self.get_parent_point(x, y)[1])



# %%

if __name__ == '__main__':
    '''
     This main function initializes the world from the vision sensor in coppelia sim.
     Once this is done it creates an ASTAR object and then runs ASTAR for the specified number of iterations
     It uses the path list to define a real path in coppelia sim from which the robot will follow.
     '''

    client = zmq.RemoteAPIClient()
    sim = client.getObject('sim')

    worldmap = util.gridmap(sim, 5.0)
    worldmap.inflate_obstacles(num_iter=10)  # YOU CAN MODIFY THE INFLATION ITERATIONS
    worldmap.normalize_map()
    worldmap.plot(normalized=True)

    goal = sim.getObjectHandle("/goal_point")
    goal_world = sim.getObjectPosition(goal, sim.handle_world)
    goal_grid = worldmap.get_grid_coords(goal_world)
    worldmap.plot_point(goal_grid)
    robot = sim.getObjectHandle("/Pure_Robot/Dummy")
    start_world = sim.getObjectPosition(robot, sim.handle_world)
    start_grid = worldmap.get_grid_coords(start_world)
    worldmap.plot_point(start_grid, color='red')

    astar = ASTAR(worldmap.norm_map, start_grid, goal_grid)
    trace_grid = astar.run(max_iter=50000)  # YOU CAN MODIFY THE ASTAR ITERATIONS
    astar.visualize_state()

    trace_grid = np.fliplr(np.array(trace_grid))
    m, n = trace_grid.shape
    trace_grid = np.hstack((trace_grid, np.zeros((m, 1))))
    trace_world = worldmap.get_world_coords(np.array(trace_grid).reshape((-1, 3)))
    coppelia_path = util.generate_path_from_trace(sim, trace_world, 100)

    trackpoint = sim.getObjectHandle("/track_point")
    util.execute_path(coppelia_path, sim, trackpoint, robot, thresh=0.1)
    plt.show()
