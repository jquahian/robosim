import numpy as np
import time


def generate_linear_path(waypoints, intermediate_point_dist):
    """function takes a list of waypoints and generates a list of points to form a linear path between them.
    The order of the waypoints matters.  intermediate points will be generated between waypoint n and n+1.
    Function will take any number of waypoints.
    
    Args:
        waypoints (array): array of cartesian points where each point is given as an array [x, y, z] (mm)
        intermediate_point_dist (float): distance to space the intermediate points between waypoints (mm)
    Returns:
        array of arrays: each item in the array is a cartesian point in the form [x, y, z] (mm)
    """
    
    start_time = time.time()
    linear_coordinates = []

    for i in range(len(waypoints)):
        if i < len(waypoints) - 1:

            point_1 = np.array(waypoints[i])
            point_2 = np.array(waypoints[i + 1])
            vector_p1_p2 = np.subtract(point_2, point_1)

            print(f'vector: {vector_p1_p2}')

            segment_length = np.linalg.norm(vector_p1_p2)

            print(f'segment is {segment_length} mm \n')

            # determine number of intermediate points spaced some distance between the waypoints
            num_points = int(segment_length // intermediate_point_dist)

            # generate the intermediate points
            for j in range(num_points):
                linear_point = np.multiply(vector_p1_p2, j/num_points)
                linear_point = np.add(linear_point, point_1)

                linear_coordinates.append(linear_point)

    elapsed_time = round(start_time - time.time(), 7)

    linear_coordinates.append(np.array(waypoints[-1]))

    print(f'linear solve: {len(linear_coordinates)} points generated between {len(waypoints)} waypoints in {elapsed_time} s')

    return linear_coordinates

# # example useage and for testing
# point_a = [393.67, -227.585, 428.274]
# point_b = [480., 40., 400.]
# point_c = [393.67, 0, 428.274]

# waypoints = [point_a, point_b, point_c]

# generate_linear_path(waypoints, 10)
