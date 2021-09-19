import numpy as np
import matplotlib.pyplot as plt
from collections import deque

#TRACK_NAME = 'reinvent_base'
TRACK_NAME = 'reInvent2019_track'

# Load the center, inner, outer waypoints
waypoints = np.load("%s.npy" % TRACK_NAME)

def getRaceLine(r,centerLine):
    centerLine_tmp = centerLine.copy() 
    l=len(centerLine)
    centerLine_tmp[l-r:l,:]=centerLine[0:r,:]
    centerLine_tmp[0:l-r,:]=centerLine[r:l,:]
    center_sum= (centerLine+centerLine_tmp)/2
    return center_sum



def plot_points(ax, points):
    ax.scatter(points[:-1,0], points[:-1,1], s=1)
    #for i,p in enumerate(points):
    #    ax.annotate(i, (p[0], p[1]))

# Plot np_reinvent2018wp118
fig, ax = plt.subplots(figsize=(12,9))
plot_points(ax, waypoints[:-1,0:2])
plot_points(ax, waypoints[:-1,2:4])
plot_points(ax, waypoints[:-1,4:6])
center_org=waypoints[:-1,0:2].copy() 
center_tmp = center_org.copy() 

r=12 # for 2018
r=28
l=len(center_org)
print(l)

center_tmp[l-r:l,:]=center_org[0:r,:]
center_tmp[0:l-r,:]=center_org[r:l,:]
print(waypoints[:-1,0:2])
print(center_tmp)

center_sum= (center_org+center_tmp)/2
plot_points(ax, center_sum)

ax.axis('equal')
plt.savefig('reinvent_base.png')
