def sqrt(n):
    ans = n ** 0.5
    return ans

def factorial(n):
    k = 1
    for i in range(1, n+1):
        k = i * k

    return k 

def degrees(x):
    pi = 3.14159265359
    y=x/pi*180.0
    return y

def radians(x):
    pi = 3.14159265359
    y=x/180.0*pi
    return y

def sin(x):
    pi = 3.14159265359
    #n = 180 / int(d) # 180 degrees = pi radians
    #x = pi / n # Converting degrees to radians
    while x > pi:
        x=x-2*pi
    while x < -pi:
        x=x+2*pi    
    ans = x - ( x ** 3 / factorial(3) ) + ( x ** 5 / factorial(5) ) - ( x ** 7 / factorial(7) ) + ( x ** 9 / factorial(9) )
    return ans 

def cos(x):
    pi = 3.14159265359
    while x > pi:
        x=x-2*pi
    while x < -pi:
        x=x+2*pi 
    ans = 1 - ( x ** 2 / factorial(2) ) + ( x ** 4 / factorial(4) ) - ( x ** 6 / factorial(6) ) + ( x ** 8 / factorial(8) )
    return ans 

def tan(x): 
    ans = sin(x) / sqrt(1 - sin(x) ** 2)
    return ans 

def arctan_taylor(x, terms=9):
    """
    Compute arctan for small x via Taylor polynomials.

    Uses a fixed number of terms. The default of 9 should give good results for
    abs(x) < 0.1. Results will become poorer as abs(x) increases, becoming
    unusable as abs(x) approaches 1.0 (the radius of convergence of the
    series).
    """
    # Uses Horner's method for evaluation.
    t = 0.0
    for n in range(2*terms-1, 0, -2):
        t = 1.0/n - x*x*t
    return x * t

def arctan_taylor_with_reduction(x, terms=9, threshold=0.1):
    """
    Compute arctan via argument reduction and Taylor series.

    Applies reduction steps until x is below `threshold`,
    then uses Taylor series.
    """
    reductions = 0
    while abs(x) > threshold:
        x = x / (1 + sqrt(1 + x*x))
        reductions += 1

    return arctan_taylor(x, terms=terms) * 2**reductions

def atan2(y,x):
    pi = 3.14159265359
    z=0
    if x > 0 :
        z=arctan_taylor_with_reduction(y/x)
    elif x<0 and y>=0:
        z=arctan_taylor_with_reduction(y/x)+pi
    elif x<0 and y<0:
        z=arctan_taylor_with_reduction(y/x)-pi
    elif x==0 and y>0:
        z=pi/2    
    elif x==0 and y<0:
        z=-pi/2
    else:
        z=0
    return z



class Reward:
    def __init__(self, verbose=False):
        self.first_racingpoint_index = None
        self.verbose = verbose
        self._x = None
        self._y = None
        self._speed = None
        
        

    def reward_function(self, params):


        ################## HELPER FUNCTIONS ###################

        def dist_2_points(x1, x2, y1, y2):
            return abs(abs(x1-x2)**2 + abs(y1-y2)**2)**0.5

        def closest_2_racing_points_index(racing_coords, car_coords):

            # Calculate all distances to racing points
            distances = []
            for i in range(len(racing_coords)):
                distance = dist_2_points(x1=racing_coords[i][0], x2=car_coords[0],
                                         y1=racing_coords[i][1], y2=car_coords[1])
                distances.append(distance)

            # Get index of the closest racing point
            closest_index = distances.index(min(distances))

            # Get index of the second closest racing point
            distances_no_closest = distances.copy()
            distances_no_closest[closest_index] = 999
            second_closest_index = distances_no_closest.index(
                min(distances_no_closest))

            return [closest_index, second_closest_index]

        def dist_to_racing_line(closest_coords, second_closest_coords, car_coords):
            
            # Calculate the distances between 2 closest racing points
            a = abs(dist_2_points(x1=closest_coords[0],
                                  x2=second_closest_coords[0],
                                  y1=closest_coords[1],
                                  y2=second_closest_coords[1]))

            # Distances between car and closest and second closest racing point
            b = abs(dist_2_points(x1=car_coords[0],
                                  x2=closest_coords[0],
                                  y1=car_coords[1],
                                  y2=closest_coords[1]))
            c = abs(dist_2_points(x1=car_coords[0],
                                  x2=second_closest_coords[0],
                                  y1=car_coords[1],
                                  y2=second_closest_coords[1]))

            # Calculate distance between car and racing line (goes through 2 closest racing points)
            # try-except in case a=0 (rare bug in DeepRacer)
            try:
                distance = abs(-(a**4) + 2*(a**2)*(b**2) + 2*(a**2)*(c**2) -
                               (b**4) + 2*(b**2)*(c**2) - (c**4))**0.5 / (2*a)
            except:
                distance = b

            return distance

        # Calculate which one of the closest racing points is the next one and which one the previous one
        def next_prev_racing_point(closest_coords, second_closest_coords, car_coords, heading):

            # Virtually set the car more into the heading direction
            heading_vector = [cos(radians(
                heading)), sin(radians(heading))]
            new_car_coords = [car_coords[0]+heading_vector[0],
                              car_coords[1]+heading_vector[1]]

            # Calculate distance from new car coords to 2 closest racing points
            distance_closest_coords_new = dist_2_points(x1=new_car_coords[0],
                                                        x2=closest_coords[0],
                                                        y1=new_car_coords[1],
                                                        y2=closest_coords[1])
            distance_second_closest_coords_new = dist_2_points(x1=new_car_coords[0],
                                                               x2=second_closest_coords[0],
                                                               y1=new_car_coords[1],
                                                               y2=second_closest_coords[1])

            if distance_closest_coords_new <= distance_second_closest_coords_new:
                next_point_coords = closest_coords
                prev_point_coords = second_closest_coords
            else:
                next_point_coords = second_closest_coords
                prev_point_coords = closest_coords

            return [next_point_coords, prev_point_coords]

        def racing_direction_diff(closest_coords, second_closest_coords, car_coords, heading):

            # Calculate the direction of the center line based on the closest waypoints
            next_point, prev_point = next_prev_racing_point(closest_coords,
                                                            second_closest_coords,
                                                            car_coords,
                                                            heading)

            # Calculate the direction in radius, arctan2(dy, dx), the result is (-pi, pi) in radians
            track_direction = atan2(
                next_point[1] - prev_point[1], next_point[0] - prev_point[0])

            # Convert to degree
            track_direction = degrees(track_direction)

            # Calculate the difference between the track direction and the heading direction of the car
            direction_diff = abs(track_direction - heading)
            if direction_diff > 180:
                direction_diff = 360 - direction_diff

            return direction_diff

        # Gives back indexes that lie between start and end index of a cyclical list 
        # (start index is included, end index is not)
        def indexes_cyclical(start, end, array_len):

            if end < start:
                end += array_len

            return [index % array_len for index in range(start, end)]

        # Calculate how long car would take for entire lap, if it continued like it did until now
        def projected_time(first_index, closest_index, step_count, times_list):

            # Calculate how much time has passed since start
            current_actual_time = (step_count-1) / 15

            # Calculate which indexes were already passed
            indexes_traveled = indexes_cyclical(first_index, closest_index, len(times_list))

            # Calculate how much time should have passed if car would have followed optimals
            current_expected_time = sum([times_list[i] for i in indexes_traveled])

            # Calculate how long one entire lap takes if car follows optimals
            total_expected_time = sum(times_list)

            # Calculate how long car would take for entire lap, if it continued like it did until now
            try:
                projected_time_ans = (current_actual_time/current_expected_time) * total_expected_time
            except:
                projected_time_ans = 9999

            return projected_time_ans

        #################### RACING LINE ######################

        # Optimal racing line for the Spain track
        # Each row: [x,y,speed,timeFromPreviousPoint]
        racing_track = [[0.31104, 2.83077, 1.94357],
[0.32414, 2.68038, 2.12813],
[0.34543, 2.53082, 2.36228],
[0.3733, 2.38233, 2.00229],
[0.4071, 2.23508, 1.72364],
[0.44978, 2.09051, 1.55854],
[0.50383, 1.95098, 1.45322],
[0.57068, 1.81924, 1.3823],
[0.65072, 1.69787, 1.35427],
[0.74357, 1.58903, 1.34103],
[0.84808, 1.49401, 1.34103],
[0.96297, 1.4137, 1.35405],
[1.0868, 1.34841, 1.38287],
[1.21814, 1.29817, 1.43986],
[1.35553, 1.2625, 1.50596],
[1.49767, 1.24086, 1.58553],
[1.64334, 1.23254, 1.69214],
[1.79132, 1.23651, 1.8091],
[1.94049, 1.25165, 1.9574],
[2.08953, 1.27651, 1.76552],
[2.23853, 1.29226, 1.61025],
[2.38633, 1.29631, 1.51251],
[2.53169, 1.28683, 1.47229],
[2.67336, 1.26274, 1.47229],
[2.81028, 1.22393, 1.47402],
[2.94174, 1.17098, 1.52205],
[3.06743, 1.10507, 1.63422],
[3.18761, 1.02804, 1.84269],
[3.3031, 0.94225, 2.23531],
[3.4152, 0.85046, 3.1885],
[3.52557, 0.7557, 2.94368],
[3.63872, 0.65878, 2.18937],
[3.75415, 0.56537, 1.88155],
[3.87338, 0.47846, 1.69813],
[3.99682, 0.40069, 1.59584],
[4.12332, 0.33476, 1.53928],
[4.24963, 0.2827, 1.50562],
[4.3721, 0.24486, 1.49287],
[4.49039, 0.21965, 1.49287],
[4.60842, 0.20518, 1.51407],
[4.73178, 0.20077, 1.54031],
[4.86435, 0.20774, 1.57986],
[5.00455, 0.22781, 1.62794],
[5.14724, 0.26138, 1.68547],
[5.28852, 0.30756, 1.75118],
[5.42652, 0.36515, 1.83711],
[5.56049, 0.43283, 1.93476],
[5.69013, 0.50938, 2.04539],
[5.8153, 0.5936, 2.1894],
[5.93611, 0.68423, 2.35569],
[6.05281, 0.78017, 2.56213],
[6.16585, 0.88045, 2.81983],
[6.27572, 0.98419, 3.06035],
[6.38282, 1.09078, 3.12985],
[6.4872, 1.20004, 3.19794],
[6.58892, 1.31179, 3.26933],
[6.68802, 1.42585, 3.34194],
[6.78459, 1.54207, 3.41706],
[6.87867, 1.6603, 3.49638],
[6.97035, 1.78041, 3.57949],
[7.05971, 1.90227, 3.65611],
[7.1468, 2.02575, 3.73555],
[7.23169, 2.15076, 3.80011],
[7.31445, 2.27719, 3.78058],
[7.39504, 2.40502, 3.75671],
[7.47341, 2.53422, 3.73008],
[7.5495, 2.66476, 3.69918],
[7.62327, 2.79664, 3.66741],
[7.69465, 2.92982, 3.62625],
[7.76358, 3.06429, 3.58438],
[7.82997, 3.20002, 3.53921],
[7.89374, 3.33701, 3.48919],
[7.9548, 3.47523, 3.42826],
[8.01305, 3.61466, 3.2132],
[8.06836, 3.75528, 3.02286],
[8.12029, 3.89719, 2.86173],
[8.16837, 4.04044, 2.71998],
[8.21212, 4.18507, 2.59684],
[8.25102, 4.33108, 2.23712],
[8.28455, 4.47841, 1.89036],
[8.31076, 4.62712, 1.68362],
[8.32662, 4.7767, 1.54915],
[8.3295, 4.9258, 1.45486],
[8.3175, 5.07263, 1.395],
[8.28941, 5.21522, 1.3615],
[8.24483, 5.35169, 1.34423],
[8.18399, 5.48034, 1.34423],
[8.10747, 5.59967, 1.34561],
[8.01624, 5.70839, 1.36535],
[7.91155, 5.8055, 1.40174],
[7.79495, 5.89026, 1.45342],
[7.66815, 5.96227, 1.52127],
[7.53302, 6.02147, 1.60606],
[7.39153, 6.06821, 1.71379],
[7.24558, 6.10329, 1.84598],
[7.09685, 6.12787, 2.01119],
[6.94661, 6.14338, 2.22889],
[6.79572, 6.15142, 2.52062],
[6.64464, 6.15359, 2.49557],
[6.49359, 6.15005, 2.43894],
[6.3428, 6.14053, 2.38561],
[6.19261, 6.12478, 2.31923],
[6.04329, 6.10254, 2.07534],
[5.89516, 6.07349, 1.90975],
[5.74895, 6.03602, 1.78784],
[5.60594, 5.98888, 1.70369],
[5.46766, 5.93121, 1.66046],
[5.33555, 5.86257, 1.63623],
[5.2107, 5.78305, 1.63623],
[5.09402, 5.69302, 1.65581],
[4.98592, 5.59334, 1.69526],
[4.8865, 5.48496, 1.78349],
[4.79536, 5.36915, 1.92002],
[4.71168, 5.24718, 2.118],
[4.63438, 5.12036, 2.42769],
[4.5621, 4.98995, 2.93145],
[4.49333, 4.85714, 2.41862],
[4.42748, 4.72486, 1.96875],
[4.3568, 4.59639, 1.72707],
[4.27913, 4.47373, 1.58386],
[4.19286, 4.35871, 1.49626],
[4.09694, 4.25299, 1.44541],
[3.99088, 4.15808, 1.41799],
[3.8747, 4.07539, 1.41799],
[3.74891, 4.00629, 1.4216],
[3.61476, 3.9518, 1.43635],
[3.47408, 3.91276, 1.47334],
[3.32913, 3.8893, 1.52574],
[3.18214, 3.88086, 1.59439],
[3.03484, 3.88636, 1.6901],
[2.88834, 3.90427, 1.82251],
[2.74318, 3.93284, 1.98323],
[2.59958, 3.9704, 2.20254],
[2.4575, 4.01527, 2.50779],
[2.31676, 4.0658, 2.55991],
[2.1769, 4.12041, 2.13511],
[2.03545, 4.16963, 1.86435],
[1.89195, 4.2111, 1.69176],
[1.74617, 4.24236, 1.56844],
[1.59842, 4.2611, 1.47598],
[1.44967, 4.26516, 1.41622],
[1.30167, 4.25268, 1.37185],
[1.15686, 4.22259, 1.34187],
[1.01787, 4.17453, 1.33159],
[0.88724, 4.10883, 1.33],
[0.76709, 4.02656, 1.33],
[0.65923, 3.92908, 1.34182],
[0.56503, 3.81813, 1.36901],
[0.48536, 3.69568, 1.40283],
[0.42077, 3.56373, 1.45199],
[0.37132, 3.42437, 1.51676],
[0.33657, 3.27966, 1.59183],
[0.31576, 3.13147, 1.68543],
[0.30776, 2.98142, 1.81042]]
        
################## INPUT PARAMETERS ###################
        # episode,steps,X,Y,yaw,steer,throttle,action,reward,done,all_wheels_on_track,progress,closest_waypoint,track_len,tstamp,episode_status
        # Read all input parameters
        all_wheels_on_track = params['all_wheels_on_track']
        x = params['x']
        y = params['y']
        distance_from_center = params['distance_from_center']
        is_left_of_center = params['is_left_of_center']
        heading = params['heading']
        progress = params['progress']
        steps = params['steps']
        speed = params['speed']
        steering_angle = params['steering_angle']
        track_width = params['track_width']
        waypoints = params['waypoints']
        closest_waypoints = params['closest_waypoints']
        is_offtrack = params['is_offtrack']

        ############### OPTIMAL X,Y,SPEED,TIME ################

        # Get closest indexes for racing line (and distances to all points on racing line)
        closest_index, second_closest_index = closest_2_racing_points_index(
            racing_track, [x, y])

        # Get optimal [x, y, speed, time] for closest and second closest index
        optimals = racing_track[closest_index]
        optimals_second = racing_track[second_closest_index]


      
        ################ REWARD AND PUNISHMENT ################
        ## Define the default reward ##
        reward = 0.001

        ## Reward if car goes close to optimal racing line ##
        DISTANCE_MULTIPLE = 1
        dist = dist_to_racing_line(optimals[0:2], optimals_second[0:2], [x, y])
        distance_reward = max(1e-3, 1 - (dist/(track_width*0.5))**2)
        

        ## Reward if speed is close to optimal speed ##
        SPEED_DIFF_NO_REWARD = 1
        SPEED_MULTIPLE = 2

        speed_diff = abs(optimals[2]-speed)
        if speed_diff <= SPEED_DIFF_NO_REWARD:
            speed_reward = (1 - (speed_diff/SPEED_DIFF_NO_REWARD))
        else:
            speed_reward = 0.0001

        ## combine
        reward += distance_reward * DISTANCE_MULTIPLE
        reward += speed_reward * SPEED_MULTIPLE


        # Always return a float value
        
        return float(reward)


reward_object = Reward() # add parameter verbose=True to get noisy output for testing


def reward_function(params):
    return reward_object.reward_function(params)
