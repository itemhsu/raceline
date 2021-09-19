import numpy as np
import matplotlib.pyplot as plt
from collections import deque

TRACK_NAME = 'reinvent_base'
#TRACK_NAME = 'reInvent2019_wide'

# Load the center, inner, outer waypoints
waypoints = np.load("%s.npy" % TRACK_NAME)

def getRaceLine(r,centerLine):
    center_sum = centerLine.copy() 
    wpSize=len(centerLine)
    #centerLine_tmp[l-r:l,:]=centerLine[0:r,:]
    #centerLine_tmp[0:l-r,:]=centerLine[r:l,:]
    for p in range(wpSize):
        p_before=(p+wpSize-int(r/2))%wpSize
        p_after=(p+wpSize+int(r/2))%wpSize
        center_sum[p,:]= (centerLine[p_before,:]+centerLine[p_after,:])/2
    
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

r=12
rCurv=[*range(46,64)]
inLan=(waypoints[:-1,0:2].copy()+waypoints[:-1,2:4].copy())/2
outLan=(waypoints[:-1,0:2].copy()+waypoints[:-1,4:6].copy())/2
center_sum=getRaceLine(r,waypoints[:-1,0:2].copy())
in_center_sum=getRaceLine(8,inLan)
out_center_sum=getRaceLine(8,outLan)
in_center_sum[46:64,:]=inLan[46:64,:].copy()
out_center_sum[0:46,:]=outLan[0:46,:].copy()
out_center_sum[64:,:]=outLan[64:,:].copy()
plot_points(ax, center_sum)
plot_points(ax, in_center_sum)
plot_points(ax, out_center_sum)

ax.axis('equal')
plt.savefig('reinvent_base.png')
