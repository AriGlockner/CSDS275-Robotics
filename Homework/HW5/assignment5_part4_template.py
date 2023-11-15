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
import cv2
import torch.nn as nn
import torch.nn.functional as F
import torch
import ecse275_vision_utils as util
import my_functions_solution as func

'''PASTE YOUR NEURAL NETWORKS HERE'''
class Net(nn.Module):
    def __init__(self):
        super(Net, self).__init__()
        self.fc1 = nn.Linear(784, 128)
        self.fc2 = nn.Linear(128, 64)
        self.fc3 = nn.Linear(64, 10)

    def forward(self, x):
        x = x.view(-1, 28 * 28)  # Flatten the input
        x = self.fc1(x) # Linear Layer ReLU
        x = self.fc2(x) # Linear Layer ReLU
        x = self.fc3(x) # Linear Layer
        return x

class CNN(nn.Module):
    def __init__(self):
        super(CNN, self).__init__()
        self.conv1 = nn.Conv2d(1, 128, 5)
        self.pool1 = nn.MaxPool2d(2, 2)
        self.conv2 = nn.Conv2d(128, 128, 3)
        self.pool2 = nn.MaxPool2d(2, 2)
        self.conv3 = nn.Conv2d(128, 64, 2)
        self.pool3 = nn.MaxPool2d(2, 2)
        self.fc1 = nn.Linear(256, 128)
        self.fc2 = nn.Linear(128, 10)

    def forward(self, x):
        x = self.pool1(self.conv1(x)) # Conv ReLU Pool
        x = self.pool2(self.conv2(x)) # Conv ReLU Pool
        x = self.pool3(self.conv3(x)) # Conv ReLU Pool
        x = x.view(-1, 256)
        x = self.fc1(x) # ReLU Linear Layer
        x = self.fc2(x) # Linear Layer
        return x

'''END OF NEURAL NETWORKS'''

plt.close("all")
    
f = 0.020 # focal length in meters
pixels_per_inch = 560.0165995731867 
z = 0.805 # vertical distance to the centerpoint of the blocks on the table

vision_mode = "gray" # or "RGB"


client = zmq.RemoteAPIClient()
sim = client.getObject('sim')

drop_target = sim.getObject('/drop_target')
droppt = sim.getObjectPose(drop_target,-1)

camera_handle = sim.getObject("/Vision_sensor")

if vision_mode == "gray":
    image,resolution = sim.getVisionSensorImg(camera_handle,1)
    image = np.array(bytearray(image),dtype='uint8').reshape(resolution[0],resolution[1]) 
elif vision_mode == "RGB":
    image,resolution = sim.getVisionSensorImg(camera_handle,0)
    image = np.array(bytearray(image),dtype='uint8').reshape(resolution[0],resolution[1],3)
else:
    print("invalid!")

image = np.flip(image,axis=1)

if vision_mode == "gray":
    plt.imshow(image,cmap="binary")
elif vision_mode =="RGB":
    plt.imshow(image)
    
centroids,list_of_cube_images = util.process_image(image)

for i in range(len(list_of_cube_images)):
    plt.figure()
    plt.imshow(list_of_cube_images[i],cmap="binary")

model_type = "CNNaug" # specify which model you would like here. Choices are FCN, CNN, CNNaug

if model_type=="FCN":
    net = Net()
    net.load_state_dict(torch.load("model_weights.pth"))
elif model_type=="CNN":
    net = CNN()
    net.load_state_dict(torch.load("model_weights_CNN.pth"))
elif model_type=="CNNaug":
    net = CNN()
    net.load_state_dict(torch.load("model_weights_CNN_aug.pth"))

predictions = []
for i in range(len(list_of_cube_images)):
    predictions.append(torch.argmax(net(torch.Tensor(list_of_cube_images[i]).unsqueeze(0))).numpy())
print("identified numbers are:")
print(np.array(predictions))

order_sequence = np.argsort(predictions) #sort by descending order

    
#%%
T_cam_world = np.array(sim.getObjectMatrix(camera_handle,-1)).reshape(3,4)
pos_cam_list = []
pos_world_list = []
for i in order_sequence:
    pos_cam_list.append(func.compute_pos_from_pix(centroids[i],resolution[0],f,pixels_per_inch,z))
for i in range(len(pos_cam_list)):
    pos_world_list.append(util.hand_eye_transform(pos_cam_list[i],T_cam_world))

#%% Movement Commands

for i in range(len(pos_world_list)):
    print("picking..." + str(predictions[order_sequence[i]]))
    util.move_to(sim,list(pos_world_list[i]),offset=0.02)
    util.toggle_gripper(sim)
    droppt[2] = droppt[2]+0.015
    util.move_to(sim,droppt,offset=0.02)
    util.toggle_gripper(sim)
    reset_point = [droppt[0],droppt[1],droppt[2]+0.1]
    util.move_to(sim,reset_point,approach_height=0)
