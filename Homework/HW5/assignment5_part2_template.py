#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Oct 19 08:14:13 2023

@author: zxc703
"""

import coppeliasim_zmqremoteapi_client as zmq
import matplotlib.pyplot as plt
import numpy as np
import copy
import ecse275_vision_utils as util
from Homework.HW5 import my_functions

''' Initialization code. DON'T MODIFY'''
# Declare the following parameters
f = 0.020 # focal length in meters
pixels_per_inch = 560.0165995731867 
z = 0.805 # vertical distance to the centerpoint of the blocks on the table

vision_mode = "RGB" # or "RGB"

client = zmq.RemoteAPIClient()
sim = client.getObject('sim')

# Define the position of our drop "zone"
drop_target = sim.getObject('/drop_target')
droppt = sim.getObjectPose(drop_target,-1)

# Get the camera handle
camera_handle = sim.getObject("/Vision_sensor")
# Obtain the transformation of the camera in the world frame
T_cam_world = np.array(sim.getObjectMatrix(camera_handle,-1)).reshape(3,4)

# Obtain the image from the image sensor
if vision_mode == "gray":
    image,resolution = sim.getVisionSensorImg(camera_handle,1)
    image = np.array(bytearray(image),dtype='uint8').reshape(resolution[0],resolution[1]) 
elif vision_mode == "RGB":
    image,resolution = sim.getVisionSensorImg(camera_handle,0)
    image = np.array(bytearray(image),dtype='uint8').reshape(resolution[0],resolution[1],3)
else:
    print("invalid!")

# We need to flip the image to align the horizontal axis with the camera frame
image = np.flip(image,axis=1)

# Visualize our sensor image
if vision_mode == "gray":
    plt.imshow(image,cmap="binary")
elif vision_mode =="RGB":
    plt.imshow(image)

''' END OF INITIALIZATION CODE. MODIFY BELOW'''

''' 
The following quantities could be helpful

droppt - is the pose vector of the drop position you can use this with move_to(droppt) to command the robot to the drop point
T_cam_world - is a 3x4 transformation matrix (where the fourth row is omitted) you can use this with hand_eye_transform to convert points in the camera frame of reference to the world frame of reference
resolution - is a 2-element tuple with the image sensor resolution for the horizontal and vertical pixels 
f - is the focal length in meters
pixels_per_inch - is exactly what it says it is
z - the known z-distance along the camera axis from the vision sensor to the cubes
'''

''' PART 2 QUESTION 2
Write code so that you can populate the pos_world_list with the coordinates for the red cube, green cube and the blue cube in that order
Hint: you can create lists and iterate through them performing very similar operations to those in Part 1.
'''

pos_world_list = []
for i in range(3):
    # 1. threshold the image to get the pixel locations where the cube is located
    pixel_locations = my_functions.threshold_RGB(image, i)

    # 2. calculate the pixel centroid
    pixcel_centroid = my_functions.get_pixel_centroid(pixel_locations)

    # 3. calculate the position of the centroid in the camera coordinate frame
    pos_cam = my_functions.compute_pos_from_pix(pixcel_centroid, resolution, f, pixels_per_inch, z)

    # 4. compute pos_world by converting the centroid position from the camera coordinate frame to the world coordinate frame using the util.hand_eye_transform function
    pos_world_list.append(util.hand_eye_transform(pos_cam, T_cam_world))


#pos_world_list = pos_world_list[0]


#%% Movement Commands

''' END OF MODIFIABLE SECTION. DO NOT MODIFY THE CODE BELOW'''

for i in range(3):
    util.move_to(sim, list(pos_world_list[i]))
    util.toggle_gripper(sim)
    droppt[2] = droppt[2]+0.015
    util.move_to(sim, droppt)
    util.toggle_gripper(sim)


