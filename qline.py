WP=-1

def dist_2_points(x1, x2, y1, y2):
    return abs(abs(x1-x2)**2 + abs(y1-y2)**2)**0.5

def reward_function(params):
    '''
    Example of rewarding the agent to stay inside two borders
    and penalizing getting too close to the objects in front
    '''
    global WP
    
    RANGE=6
    waypoints = params['waypoints']
    x = params['x']
    y = params['y']
    p = params['closest_waypoints'][0]
    wpSize=len(waypoints)
    
    p_before=(p+wpSize-RANGE)%wpSize
    p_after=(p+wpSize+RANGE)%wpSize
    
    p_before_x,p_before_y=waypoints[p_before]
    p_after_x,p_after_y=waypoints[p_after]
    
    qline_x=(p_before_x+p_after_x)/2
    qline_y=(p_before_y+p_after_y)/2
    
    # Initialize reward with a small number but not zero
    # because zero means off-track or crashed
    reward = 1e-3

    # Distance to the next object
    distance = dist_2_points(x1=qline_x, x2=x,y1=qline_y, y2=y)

    if p==WP:
        reward = 1e-3
    else:
        WP=p
        if distance < 0.05: 
            reward = 1.0
        elif distance < 0.1:
            reward = 0.8
        elif distance < 0.15:
            reward = 0.5

    return reward
