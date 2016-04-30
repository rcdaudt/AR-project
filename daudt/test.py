# -*- coding: utf-8 -*-
"""
Created on Sat Apr 30 20:19:23 2016

@author: daudt
"""
from scipy.ndimage import imread
from rrt import rrt
import matplotlib.pyplot as plt


#=============================================================================
# function inputs
map1 = imread('map.png')
q_start = [70,80]
q_goal = [615,707]
k = 10000
delta_q = 20
p = 0.3

path = rrt(map1,q_start,q_goal,k,delta_q,p)
print(path)
print(path.shape[0])
fig = plt.figure(1)
plt.imshow(map1,cmap='Greys')
fig = plt.figure(2)
plt.plot(-path[:,1],path[:,0])