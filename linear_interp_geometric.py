from controller import zero_arm
import robot_config as config
import math
import numpy as np
import fk


def linear_move():
    # where theta is j1
    calculate_j5_pos(0, -41)

# this gets us back to where j5 is

def calculate_j5_pos(j1_theta, j5_theta):
    j5_j6_length = config.j5_j6[2]

    # convert initial rotation on xy to a right angle triangle
    # by extending the resulting j5_j6_length.
    # determine length of side opposite to j1_theta
    # tan_theta = opp/adj
    # xy_opp = tan_theta * adj
    
    xy_opp = math.tan(math.radians(j1_theta)) * j5_j6_length
    
    xy_projected_j5_j6_length = math.sqrt(math.pow(xy_opp, 2) + math.pow(j5_j6_length, 2))
    
    delta_j5_j6_length = xy_projected_j5_j6_length - j5_j6_length
    
    j1_theta_coangle = 90.0 - j1_theta
    
    # determine displacement of x

    x_displacement = math.sin(math.radians(j1_theta_coangle)) * delta_j5_j6_length
    
    #determine displacement of y
    
    y_displacement = math.cos(math.radians(j1_theta_coangle)) * delta_j5_j6_length
    
    # determine displacement of z
    
    z_displacement = math.sin(math.radians(j5_theta)) * j5_j6_length
    
    displacement_vector = np.array([x_displacement, y_displacement, z_displacement])
    
    current_pos = fk.solve_fk(25, 31, 120, 0, -20, 0)
    
    j5_current_pos = np.subtract(current_pos, displacement_vector)

    print(displacement_vector)
    print(current_pos)
    print(j5_current_pos)

    
linear_move()
