# -*- coding: utf-8 -*-
"""
Created on Sat Apr 30 17:16:18 2016

@author: daudt
"""

import numpy as np
from numpy.random import rand
from numpy.linalg import norm

#=============================================================================
def is_visible(in_map,q,q_new,delta=1):
    N = np.ceil(norm(q-q_new)/delta).astype(int)
    x = np.array(np.floor(np.linspace(q[0],q_new[0],num=N))).astype(int)
    y = np.array(np.floor(np.linspace(q[1],q_new[1],num=N))).astype(int)
    print 'IS VISIBLE:', N
    vals = in_map[x,y]
    
    if any(vals):
        return False
    else:
        return True
        



#=============================================================================
def smooth(in_map,path,delta):
    if not path.any():
        return False
        
    path_smooth = path[0,:]
    N = len(path)
    reached_end = False
    last_added = 0
    
    while reached_end == False:
        added = False
        for i in np.arange(1,N-last_added-1):
            ind = N - i
            if is_visible(in_map,path[last_added,:],path[ind,:]):
                path_smooth = np.vstack((path_smooth,path[ind,:]))
                last_added = ind
                added = True
                break
        
        if added == False:
            last_added += 1
            path_smooth = np.vstack((path_smooth,path[last_added,:]))
            
        reached_end = last_added == (N-1)        
    
    return path_smooth







#=============================================================================
def rrt(in_map,q_start,q_goal,k,delta_q,p):

    print('RRT: Start')

    # Get map size
    [s1,s2] = in_map.shape

    # Checks input parameters
    assert(in_map.ndim == 2)
    assert(p > 0)
    assert(p < 1)
    assert(max(q_start[0],q_goal[0]) <= s1)
    assert(min(q_start[0],q_goal[0]) >= 0)
    assert(max(q_start[1],q_goal[1]) <= s2)
    assert(min(q_start[1],q_goal[1]) >= 0)
    assert(delta_q < min(s1,s2))
    
    #-------------------------------------------------
    # Initialize output arrays
    vertices = np.zeros((k+1,2))
    vertices[0,:] = q_start
    edges = np.zeros((k,2))
    path = np.zeros((k))
    
    
    # Function variables
    added_goal = False
    last_added_vertex = 0
    
    
    print('RRT: Calculating trajectory')
    
    # RRT loop
    for i in range(k):
        # Generate q_rand
        if rand() <= p:
            q_rand = q_goal
        else:
            q_rand = rand(2)*[s1-1,s2-1]
            
        # Find index of closest vertex to q_rand
        min_d = s1+s2
        index = -1
        for vertex in range(last_added_vertex+1):
            dist_v = norm(q_rand - vertices[vertex,:])
            if dist_v < min_d:
                min_d = dist_v
                index = vertex
                
        # Generate q_new
        dxy = q_rand - vertices[index,:]
        q_new = vertices[index,:] + delta_q*dxy/norm(dxy)
        # Ensure q_new is inside map boundaries
        q_new[0] = min(max(q_new[0],0),s1-1);
        q_new[1] = min(max(q_new[1],0),s2-1);
        
        # Check visibility and add to vertices list if visible
        added = False
        if is_visible(in_map,vertices[index,:],q_new):
            last_added_vertex += 1
            vertices[last_added_vertex,:] = q_new
            edges[last_added_vertex-1,:] = [last_added_vertex,index]
            added = True
            
        # Check if we reached the end
        if added:
            if norm(q_new - q_goal) <= delta_q:
                if is_visible(in_map,q_new,q_goal):
                    last_added_vertex = last_added_vertex + 1
                    vertices[last_added_vertex,:] = q_goal
                    edges[last_added_vertex-1,:] = [last_added_vertex,last_added_vertex-1]
                    added_goal = True
                    break
    # End of RRT loop -----------------------------------------------------
    print 'FINISHED'
    
    # Clean vertices and edges arrays
    vertices = vertices[:last_added_vertex+1,:]
    edges = edges[:last_added_vertex,:]
    
    # Build path
    if added_goal == False:
        return False
    
    last_added_to_path = last_added_vertex
    path = np.array(vertices[last_added_to_path,:])
    while last_added_to_path != 0:
        last_added_to_path = edges[last_added_to_path-1,1]
        path = np.vstack((path,vertices[last_added_to_path,:]))
    path = np.flipud(path)
    
    print('RRT: Smoothing trajectory')

    # Smooth path
    path_smooth = smooth(in_map,path,5)    

    # Return path
    return path_smooth