# -*- coding: utf-8 -*-
"""
Created on Mon Mar  7 22:47:49 2016

@author: daudt
"""
import os
import numpy as np

f=open('coords.txt')
s = f.readline().strip()
print(os.path.dirname(os.path.abspath(__file__)))

a = np.array([[1,2,3],[7,6.2,5],[1,4,7]])
I = np.array([[1,0,0],[0,1,0],[0,0,1]])