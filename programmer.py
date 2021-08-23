import numpy as np
import solve_orientation as so
import limit_check as lim
import fk
import robot_config as config
import linear_interp as lin
import time
import generate_file
import controller

'''
1. unifies path planning to generate raw joint angles
2. checks the validity of the requested position and the resulting angles
'''

def safety_checks(targets):
    orientation_rot = targets['orientation']
    target_point = targets['point']
    
    if lim.work_envelope_check(target_point) == False:
        print('Target point is outside of work envelope')
        
    orientation_rotations, total_rotations = so.solve_orientation(orientation_rot, target_point)
    
    pos = fk.solve_fk(orientation_rotations[0], orientation_rotations[1], orientation_rotations[2], 0, 0, 0)
    
    if lim.work_envelope_check(pos) == False:
        print('Position resulting from orientation is outside the work envelope')
        
    # need a check to see if joint 5's position is within it's own position envelope
        
    j1_rot = total_rotations[0]
    j2_rot = total_rotations[1]
    j3_rot = total_rotations[2]
    j4_rot = total_rotations[3]
    j5_rot = total_rotations[4]
    
    if j1_rot < config.j1_angle_min_lim or j1_rot > config.j1_angle_max_lim:
        j1_valid = False
    else:
        j1_valid = True
        
    if j2_rot < config.j2_angle_min_lim or j2_rot > config.j2_angle_max_lim:
        j2_valid = False
    else:
        j2_valid = True

    if j3_rot < config.j3_angle_min_lim or j3_rot > config.j3_angle_max_lim:
        j3_valid = False
    else:
        j3_valid = True

    if j4_rot < config.j4_angle_min_lim or j4_rot > config.j4_angle_max_lim:
        j4_valid = False
    else:
        j4_valid = True
        
    if j5_rot < config.j5_angle_min_lim or j5_rot > config.j5_angle_max_lim:
        j5_valid = False
    else:
        j5_valid = True
        
    joint_validity_check = [j1_valid, j2_valid, j3_valid, j4_valid, j5_valid]
    
    if False in joint_validity_check:
        for i in range(len(joint_validity_check)):
            if joint_validity_check[i] == False:
                joint_num = i + 1
                print(
                    f'Joint {joint_num} rotation, {total_rotations[i]}, is out of range for this solve')
                return False
    else:
        print(f'Position, orientation, and joint angles valid! \nAngles: {total_rotations} \n')
                
        return total_rotations
    

def program_path(targets, move_type, interp_orient, realtime):
    angle_list = []
    start_time = time.time()
        
    for target in targets:
        
        valid_solve = safety_checks(target)
        
        if valid_solve == False:
            break
    
    try:  
        if move_type == 'linear':
            points = lin.create_vector(targets, interp_orient)
        else:
            points = targets

        for i in range(len(points)):
            _, solved_angles = so.solve_orientation(points[i]['orientation'], points[i]['point'])
            angle_list.append(solved_angles)
        
        time_elapsed = round((time.time() - start_time), 3)

        print(f'solve completed in: {time_elapsed}s')

        if realtime == False:
            print('offline programming enabled: generating solve file')
            generate_file.write_out(angle_list)
        else:
            print('realtime programming enabled: streaming solve')
            return angle_list    
    except:
        print('something went wrong with the solve')

# test
# incoming points are in a dict with 'orientation' and 'point' as keys and xyz array as value pairs
point_a = {
        "orientation": np.array([90., 0., 0.]),
        "point": np.array([277.1173366807718, -215.72995774121685, 243.45753120996508]),
}

point_b = {
    "orientation": np.array([90., 0., 0.]),
    "point": np.array([277.1173366807718, -215.72995774121685, 343.45753120996505]),
}

# point_c = {
#     "orientation": np.array([0., 0., 0.]),
#     "point": np.array([350.014, -49.482, 240.]),
# }

# point_d = {
#     "orientation": np.array([0., 90., 0.]),
#     "point": np.array([350.014, 50.518, 240.]),
# }

targets = [point_a, point_b]


'''
arguments:

1) targets as an array of points (see above example)
2) move type:
    a) if 'none', robot will just move from point to point in the most direct way possible
    b) if 'linear', robot will generate intermediate points that are on a linear trajectory between point and point + 1.
3) blend orientation (TRUE/FALSE):
    a) True: will blend the requested orientation from point to point + 1
    b) if false: will take the orientation of the first point and apply it to the entire range of motion
4) realtime (TRUE/FALSE):
    determines whether to save the generated points as a file for later playback or to stream the data directly to the arm
'''

# # uncomment for testing
# program_path(targets, 'linear', interp_orient=False, realtime=False)
