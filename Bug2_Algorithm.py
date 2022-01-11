# -*- coding: utf-8 -*-
"""
Created on Wed Sep 29 21:59:03 2021

@author: hp
"""

import os
import glob
import shutil 
import sys
from numpy import* 
import math
import operator
from scipy.spatial import ConvexHull, convex_hull_plot_2d
import matplotlib.pyplot as plt
from matplotlib import path



def main():
    #Start_Config = array(sys.argv[1])
    #Goal_Config = array(sys.argv[2])
    #Obstacle_C_Space = array(sys.argv[3])
    #d_step =  sys.argv[4]
    #delta_step = sys.argv[5]
    
    Start_Config = array([0.0,0.0])
    Goal_Config = array([10.0,1.0])
    Obstacle_C_Space = array([[1.0,1.0],[2.0,2.0],[4.0,2.0],[6.0,1.0],[6.0,-1.0],[4.0,-2.0],[2.0,-2.0],[1.0,-1.0],[1.0,1.0]])
    d_step =  0.3
    delta_step = math.pi/7
    
    #Obstacle_C_Space_plt = Obstacle_C_Space
    #Obstacle_C_Space_plt.insert(Obstacle_C_Space[0,:])
    plt.plot(Obstacle_C_Space[:,0], Obstacle_C_Space[:,1], 'k-')
    #plt.plot([Start_Config[0], Goal_Config[0]], [Start_Config[1], Goal_Config[1]], 'k-')
    
    track_path = array(Track_Path(Start_Config, Goal_Config, Obstacle_C_Space, d_step, delta_step))
    
    plt.plot(track_path[:,0], track_path[:,1], 'k-')
    print(track_path)

#We need to change tthe Damping Value
#Include the concept of on-the-line concept
def Track_Path(Start, Goal, O_Vertices, d, delta):
    track_points = [];
    N_iter = 1000
    O_Path = path.Path(O_Vertices)
    
    Slope_m = (Goal[1] - Start[1])/(Goal[0] - Start[0])
    
    Quadrant = find_quadrant(Start,Goal)
    Slope = Slope_m

    slope_check = 0
    
    track_points.append(Start)
    
    for i in range(400):
        print(i,slope_check)
        if goal_reached(Start,Goal,d):
            track_points.append(Goal)
            #print(track_points)
            return track_points

        Slope_goal = (Goal[1] - Start[1])/(Goal[0] - Start[0])

        if (line_m(Start,Goal,Slope_m) == 0) & (slope_check == 0):
            #print(track_points,Slope)
            print("Here1")
            [Start_new,Slope_new] = Update_pos_path(Start,Slope,d,delta,O_Path,Quadrant)
            print(track_points)
                
            if Slope != Slope_new:
                Hit = Start

        elif (line_m(Start,Goal,Slope_m) != 0) & ((slope_check != 0) | (slope_check == 0)):
            print("Here2")
            [Return,Start_new, Slope_new] = check_update_neg_path_collides(Start,Slope,d,delta,O_Path,Quadrant)
            
            if Return==True:
                [Start_new,Slope_new] = Update_pos_path(Start,Slope,d,delta,O_Path,Quadrant)    
            
        #Exit Case
        elif (line_m(Start,Goal,Slope_m) == 0) & (slope_check != 0):
            print("Here3")
            if distance(Hit,Goal) > distance(Start,Goal):
                [Start_new,Slope_new] = Update_pos_path(Start,Slope_m,d,delta,O_Path,Quadrant)
                
                if Slope_m != Slope_new:
                    Hit = Start
            else:
                [Return,Start_new, Slope_new] = check_update_neg_path_collides(Start,Slope,d,delta,O_Path,Quadrant)
                
                if Return==True:
                    [Start_new,Slope_new] = Update_pos_path(Start,Slope,d,delta,O_Path,Quadrant)
            print("Here4")
                                     
        slope_check_new = Slope_Check(Slope_new, Slope_m)
        #print("Here",Start_new,line_m(Start_new,Goal,Slope_m))
        if (line_m(Start,Goal,Slope_m) < 0) & (line_m(Start_new,Goal,Slope_m) > 0):
            #print("I am here1")
            print("Here5")
            Start_new = array(line_intersection([array([0,0]),Goal],[Start,Start_new]))
        elif (line_m(Start,Goal,Slope_m) > 0) & (line_m(Start_new,Goal,Slope_m) < 0):
            print("Here6")
            #print("I am here2",Start, Start_new,Goal)
            Start_new = array(line_intersection([array([0,0]),Goal],[Start,Start_new]))
            
        
        if Start[0] != Start_new[0]:
            Quadrant = find_quadrant(Start,Start_new)
        slope_check = slope_check_new
        Slope = Slope_new
        Start = Start_new
        print(Start,Slope,slope_check)
        #print(track_points)
        track_points.append(Start)
        
        #print(Start,track_points)

    return track_points

def Update_pos_path(Start,Slope,d,delta,O_Path,Quadrant):
    Slope_delta_1 = (Slope + tan(delta))/(1-(Slope*tan(delta)))
    Slope_delta_2 = (Slope + tan(2*delta))/(1-(Slope*tan(2*delta)))
    Start_new_0 = [Start[0] + cos(math.atan(Slope))*d,Start[1] + sin(math.atan(Slope))*d]
    Start_new_1 = [Start[0] + cos(math.atan(Slope_delta_1))*d,Start[1] + sin(math.atan(Slope_delta_1))*d]
    Start_new_2 = [Start[0] + cos(math.atan(Slope_delta_2))*d,Start[1] + sin(math.atan(Slope_delta_2))*d]
    
    print("\n Update the path")

    
    Collision_0 = O_Path.contains_points([Start_new_0])[0]
    Collision_1 = O_Path.contains_points([Start_new_1])[0]
    Collision_2 = O_Path.contains_points([Start_new_2])[0]
    
    if Collision_0 == True & Collision_1 == True & Collision_2 == True:
        Start_new = Start_new_2
        Slope_new = Slope_delta_2
        alpha = 0.7
    elif Collision_0 == True & Collision_1 == True:
        Start_new = Start_new_1
        Slope_new = Slope_delta_1
        alpha = 0.7
    elif Collision_0 == True:
        Slope_new = Slope_delta_1
        Start_new = Start_new_0
        alpha = 0.7
    elif Collision_0 == False:
        Slope_new = Slope
        Start_new = Start_new_0
        alpha = 0.7
        
    iter = 0
    d1 = d
    while O_Path.contains_points([Start_new])[0] == True:
        Start_new = []
        if iter < 3:
            iter = iter + 1    
            Start_new.append(Start[0] + cos(math.atan(Slope_new))*d1)
            Start_new.append(Start[1] + sin(math.atan(Slope_new))*d1)
            d1 = d - 0.3*d
        else:
            Start_new = Start
            break
        
    print("\n Update the path1")
    return [array(Start_new),Slope_new]

def check_update_neg_path_collides(Start,Slope,d,delta,O_Path,Quadrant):
    print("\n Check_negative")
    Start_new = []
    #radian_delta = math.atan(Slope)-delta
    Slope_delta = (Slope - tan(delta))/(1+(Slope*tan(delta)))
    print("Quadrant",Quadrant)
    if (Quadrant == 2) & (math.atan(Slope)-delta < -math.pi/2):
         Start_new.append(Start[0] - cos(math.atan(Slope_delta))*d)
         Start_new.append(Start[1] - sin(math.atan(Slope_delta))*d)
    else:
        Start_new.append(Start[0] + cos(math.atan(Slope_delta))*d)
        Start_new.append(Start[1] + sin(math.atan(Slope_delta))*d)
    
    if O_Path.contains_points([Start_new])[0] == True:
        Return = True
        Slope_new = Slope
        Start_new = Start
    else:
        Return = False
        Slope_new = Slope_delta
        Start_new = array(Start_new)
    
    print("\n Check_negative1",Return)
    return [Return, Start_new, Slope_new]
    
def Slope_Check(Slope, Slope_m):    
    if Slope > Slope_m:
        slope_check = 1
    elif Slope == Slope_m:
        slope_check = 0
    else:
        slope_check = -1
    
    return slope_check


def line_m(point,Goal,Slope):
    Return = (point[1] - (Goal[1] + Slope * (point[0] - Goal[0])))
    
    if (Return < 1e-8) & (Return > -1e-8):
        Return = 0
    
    return Return

def line_intersection(line1, line2):
    xdiff = (line1[0][0] - line1[1][0], line2[0][0] - line2[1][0])
    ydiff = (line1[0][1] - line1[1][1], line2[0][1] - line2[1][1])

    div = det([xdiff, ydiff])
    if div == 0:
       raise Exception('lines do not intersect')

    d = (det(line1), det(line2))
    x = det([d, xdiff]) / div
    y = det([d, ydiff]) / div
    return [x,y]

def det(L):
    [a,b] = L
    return a[0] * b[1] - a[1] * b[0]

def goal_reached(Start, Goal, d):
    if distance(Start,Goal) <= d:
        return True
    else:
        return False

def distance(X,Y):
    return math.sqrt( pow(X[0] - Y[0],2) + pow(X[1] - Y[1],2) )


def find_quadrant(a,b):
    if (a[0]<b[0]) & (a[1]<b[1]):
        Quadrant = 1
    elif (a[0]<b[0]) & (a[1]>b[1]):
        Quadrant = 2
    elif (a[0]>b[0]) & (a[1]>b[1]):
        Quadrant = 3
    elif (a[0]>b[0]) & (a[1]<b[1]):
        Quadrant = 4
    
    return Quadrant
    
main()