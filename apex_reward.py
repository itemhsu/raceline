def reward_function(params):
    l=[*range(,),*range(,),*range(,)]
    r=[*range(,),*range(,)]
    hi=[*range(,)]

    # Read input parameters
    all_wheels_on_track = params['all_wheels_on_track']
    closest_waypoints = params['closest_waypoints']
    is_left_of_center = params['is_left_of_center']
    speed = params['speed']
    p=closest_waypoints[0]
    
    # Give a very low reward by default
    reward = 1e-3

    if p in l and is_left_of_center and not all_wheels_on_track :
        reward = 1.0
    elif p in r and not is_left_of_center and not all_wheels_on_track :
        reward = 1.0
        
    if p in hi and speed >3 :
        reward += 1.0

    # Always return a float value
    return float(reward)