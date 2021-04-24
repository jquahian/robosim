import numpy as np
import ik

'''
calcuations for movement on a linear course from waypoint A to waypoint B
we set a predetermined number of steps to take between the two waypoints
step size: can increase or decrease this depending on resolution needed
'''

step_size = 10

# waypoints, cartesian, position in mm
waypoints = [(78.0, 0, 100.0), (100.0, 20.0, 130.0)]

# interpolate between the waypoints
# create a set of equally spaced, descrete points from interpolation
def create_vector(point_a, point_b):
    interp_point_list = []
    vector = np.subtract(point_b, point_a)
    vector_length = np.sqrt(np.sum(np.square(vector)))
    unit_vector = vector / vector_length
    
    # starting at the inital point, get a series of points between A and B at equal intervals
    for i in range(1, step_size):    
        interval = i * vector_length / step_size
        
        # convert to int otherwise we get results that are rounded up??
        # might be a display thing??
        interval = int(interval)
        
        # get a series of (more or less) equally spaced points
        interp_point = np.add(point_a, np.multiply(interval, unit_vector))
        
        # generate a list of these points
        interp_point_list.append(interp_point)
        
    return interp_point_list

def convert_ik():
    interpolated_points = create_vector(waypoints[0], waypoints[1])
    for i in range(len(interpolated_points)):
        print(f'command {i}')
        ik.solve_ik(
            interpolated_points[i][0], 
            interpolated_points[i][1], 
            interpolated_points[i][2])

convert_ik()
