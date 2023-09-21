#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Jul  1 20:31:06 2023

@author: zxc703
"""

import coppeliasim_zmqremoteapi_client as zmq
import matplotlib.pyplot as plt
import numpy as np

class gridmap:
    
    def __init__(self,sim,world_size,name="/world_camera",robot_name="/Pure_Robot"):
        '''
        

        Parameters
        ----------
        sim : TYPE
            DESCRIPTION.
        world_size : TYPE
            DESCRIPTION.
        name : TYPE, optional
            DESCRIPTION. The default is "/world_camera".
        robot_name : TYPE, optional
            DESCRIPTION. The default is "/Pure_Robot".

        Returns
        -------
        None.

        '''
        
        self.handle = sim.getObjectHandle(name)
        robot_handle = sim.getObjectHandle(robot_name)
        robot_pos = sim.getObjectPosition(robot_handle,-1)
        sim.setObjectPosition(robot_handle,-1,(100,100,100))
        self.image,self.resolution = sim.getVisionSensorImg(self.handle,1)
        sim.setObjectPosition(robot_handle,-1,robot_pos)
        self.gridmap = np.array(bytearray(self.image),dtype='uint8').reshape(self.resolution[0],self.resolution[1])    
        
        self.world_size = world_size
        self.scaling = world_size/self.resolution[0]
        self.offset = np.array([self.resolution[0]/2,self.resolution[1]/2,0])
        self.norm_map = None
    def inflate_obstacles(self,num_iter=1,obs_thresh = 100,infl_val = 99):
        
        rows = self.gridmap.shape[0]
        cols = self.gridmap.shape[1]
        inflated_grid = np.copy(self.gridmap)
        
        # Define the possible movements (up, down, left, right, and diagonals)
        movements = [
            (0, -1),  # Up
            (0, 1),   # Down
            (-1, 0),  # Left
            (1, 0),   # Right
            (-1, -1), # Diagonal: Up-Left
            (-1, 1),  # Diagonal: Up-Right
            (1, -1),  # Diagonal: Down-Left
            (1, 1)    # Diagonal: Down-Right
        ]
        
        # Iterate through the grid
        for i in range(num_iter):
            inflated_temp = np.copy(inflated_grid)
            for row in range(rows):
                for col in range(cols):
                    if inflated_grid[row][col] < obs_thresh:  # Found an obstacle
                        for move in movements:  # Inflate the obstacle
                            new_row = row + move[0]
                            new_col = col + move[1]
                            if 0 <= new_row < rows and 0 <= new_col < cols and inflated_grid[new_row][new_col] > 200:
                                inflated_temp[new_row][new_col] = infl_val
            inflated_grid = np.copy(inflated_temp)
            self.gridmap = np.copy(inflated_grid)
            
        return self.gridmap
    
    def get_grid_coords(self,point_xyz):
        try:
            pos = np.round(np.array(point_xyz)/self.scaling+self.offset).astype(int)[:,0:2]
        except:
            pos = np.round(np.array(point_xyz)/self.scaling+self.offset).astype(int)[0:2]
        
        return pos
    
    def get_world_coords(self,point_xyz):
        
        pos = (np.array(point_xyz)-self.offset)*self.scaling
        
        return pos
    
    def get_obs_in_world_coords(self,plot=False):
        
        if self.norm_map is not None:
            obs_points = np.argwhere(self.norm_map==1)
        else:
            print("norm map doesn't exist!")
        self.obs_world = self.get_world_coords(np.hstack((obs_points,np.zeros((obs_points.shape[0],1)))))
        
        
        if plot:
            plt.plot(self.obs_world[:,1],self.obs_world[:,0],'.')    
            plt.gca().set_aspect('equal')
    
    def plot(self,normalized=False):
        
        if normalized:
            plt.imshow(self.norm_map,cmap='binary',origin="lower")
        else:
            plt.imshow(self.gridmap,cmap='gray', vmin=0, vmax=255,origin="lower")
            
    def plot_point(self,point_xy,color='blue'):
        
        plt.plot(point_xy[0],point_xy[1],'.',markersize=5,color=color)
    
    def normalize_map(self):
        
        gridmap_temp = np.copy(self.gridmap)
        gridmap_temp[gridmap_temp>99] = 255
        self.norm_map =(gridmap_temp/255).astype(int)
        self.norm_map[self.norm_map==0] = 3 # temporarily assign obstacles as 3
        self.norm_map[self.norm_map==1] = 0 # set free space to 0s
        self.norm_map[self.norm_map==3] = 1 # convert obstacle back into 1s
        
        
def generate_path_from_trace(sim,trace_path, num_smoothing_points=100):
    # path must be Nx3 and already scaled back to the world coords
    
    n,m = trace_path.shape
    
    trace_path = np.hstack((trace_path,np.zeros((n,3)),np.ones((n,1))))
    
    path = np.array(trace_path).astype(float)
    #path_handle = sim.createPath(list(path.reshape(-1)),28,num_smoothing_points,1.0,0,[1.0,0.0,0.0])
    path_handle = sim.createPath(list(path.reshape(-1)),16,num_smoothing_points,1.0,0,[1.0,0.0,0.0])
    #pathShape = sim.getObjectHandle("/Path/shape")
    #sim.setShapeColor(pathShape,"",sim.colorcomponent_ambient_diffuse,[1.0,0.0,0.0])
    
    # obtain the smoothed path data from coppeliasim
    pathData=sim.unpackDoubleTable(sim.readCustomDataBlock(path_handle,'PATH'))
    pathData_array = np.array(pathData).reshape((-1,7))
    
    return pathData_array

def execute_path(pathData_array,sim,trackpoint_handle,robot_handle,thresh=0.1):
    
    path_index = 1
    while path_index<=pathData_array.shape[0]:
        # set the track point pos
        target_point = pathData_array[-path_index,:]
        if any(np.isnan(target_point)):
            target_point[3:] = [0.0,0.0,0.0,1.0]
        sim.setObjectPose(trackpoint_handle,sim.handle_world,list(pathData_array[-path_index,:]))
        # get the current robot position
        robot_pos = sim.getObjectPosition(robot_handle,sim.handle_world)
        trackpt_pos = sim.getObjectPosition(trackpoint_handle,sim.handle_world)
        # compute the distance between the trackpt position and the robot
        rob_trackpt_dist = np.linalg.norm(np.array(robot_pos)-np.array(trackpt_pos))
        print(rob_trackpt_dist)
        if rob_trackpt_dist < thresh:
            path_index = path_index + 1
            print("next_point")
            
            
def visualize_potential_field_2D(world_u):
    '''
    

    Parameters
    ----------
    world_u : TYPE
        DESCRIPTION.

    Returns
    -------
    None.

    '''
    # Create the heatmap plot using imshow()
    plt.imshow(world_u, cmap='hot', origin='lower')
    # Add colorbar
    plt.colorbar()
    # Add labels and title
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('2D Heatmap Plot')
    # Show the plot
    plt.show()
    
def visualize_potential_field_3D(world_u):
    '''
    

    Parameters
    ----------
    world_u : TYPE
        DESCRIPTION.

    Returns
    -------
    None.

    '''
    
    grid_height,grid_width = world_u.shape
    
    # Generate data for the surface plot
    x = np.linspace(0, grid_width-1, grid_width)
    y = np.linspace(0, grid_height-1, grid_height)
    X, Y = np.meshgrid(x, y)
    Z = world_u

    # Create a 3D figure
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Plot the surface
    surf = ax.plot_surface(X, Y, Z, cmap='viridis')

    # Add labels and title
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('3D Surface Plot')

    # Add a colorbar
    fig.colorbar(surf)

    # Show the plot
    plt.show()
    
def discrete_grad_descent(start,end,grad,step=1,max_iter=2000):
    '''
    

    Parameters
    ----------
    start : TYPE
        DESCRIPTION.
    end : TYPE
        DESCRIPTION.
    grad : TYPE
        DESCRIPTION.
    step : TYPE, optional
        DESCRIPTION. The default is 1.
    max_iter : TYPE, optional
        DESCRIPTION. The default is 500.

    Returns
    -------
    TYPE
        DESCRIPTION.

    '''
    current_point = np.flip(start)
    end = np.flip(end)
    point_list = []
    n = 0
    point_list.append(current_point)
    
    height, width = grad[0,:,:].shape
    
    print("start_point (y,x): " + str(current_point))
    while ((current_point[0] != end[0]) or (current_point[1]!=end[1])) and (n<=max_iter):
        print("iteration: " + str(n))
        #get gradient at the point:
        point_grad = -grad[:,int(current_point[0]),int(current_point[1])]
        print(point_grad)
        #point_grad = np.round(point_grad)
        mask = np.abs(point_grad)>0
        point_grad[mask] = point_grad[mask]/np.abs(point_grad[mask])
        print(point_grad)
        
        next_point = current_point+point_grad*step
        print(next_point)
        if (next_point[0] > height-1 or next_point[0]<0) or (next_point[1] > width-1 or next_point[1]<0) :
            pass
        else:
            current_point = next_point
            
        print("current_point (y,x): " + str(current_point))
        point_list.append(current_point)
        n=n+1

    return np.array(point_list)

def compute_discrete_gradient(world_u):
    '''
    

    Parameters
    ----------
    world_u : TYPE
        DESCRIPTION.

    Returns
    -------
    world_ugrad : TYPE
        DESCRIPTION.

    '''
    
    world_ugrad = np.gradient(world_u)
    
    world_ugrad = np.array(world_ugrad)
    
    return world_ugrad

def visualize_gradient(world,world_ugrad,goal,step=5):
    '''
    

    Parameters
    ----------
    world : TYPE
        DESCRIPTION.
    world_ugrad : TYPE
        DESCRIPTION.
    start : TYPE
        DESCRIPTION.
    goal : TYPE
        DESCRIPTION.

    Returns
    -------
    None.

    '''
    
    plt.figure(figsize=(6, 6))
    plt.imshow(world, cmap='binary', origin='lower',aspect="equal")
    # plt.xticks(range(grid_width))
    # plt.yticks(range(grid_height))
    # plt.grid(color='black', lw=1)
    plt.plot(goal[0],goal[1],'.',markersize=10,color="red")
    #plt.plot(start[0],start[1],'.',markersize=10,color="blue")
    grid_height,grid_width = world.shape
    x = np.linspace(0, grid_width-1, grid_width)
    y = np.linspace(0, grid_height-1, grid_height)
    X, Y = np.meshgrid(x, y)
    plt.quiver(X[::step,::step],Y[::step,::step], -world_ugrad[::step,::step,1], -world_ugrad[::step,::step,0])
    
    
def plot_gradient_descent(fig,path):
    '''    

    Parameters
    ----------
    fig : TYPE
        DESCRIPTION.
    path : TYPE
        DESCRIPTION.

    Returns
    -------
    None.

    '''
    
    fig.plot(path[:,1],path[:,0],'-')     
    fig.plot(path[:,1],path[:,0],'.',markersize=5)    
    plt.gca().set_aspect('equal')