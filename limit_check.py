import math
import robot_config as config
import numpy as np


def work_envelope_check(target_point):
    center = config.position_offset
    
    radius = np.add(config.j2_j3, config.j3_j4)
    radius = np.add(radius,  config.j4_j5,)
    radius = np.add(radius, config.j5_j6)
    
    # decrease by 10 for margin of safety
    radius = radius[-1] - 10 
    
    try:
        check_valid = math.sqrt((math.pow(target_point[0], 2))
                    - (math.pow(target_point[1], 2))
                    - (math.pow(target_point[2], 2) - math.pow(center[2], 2)))
            
        check_valid = round((check_valid / radius), 3)
            
        if check_valid <= 1.00:
            is_valid = True
        else:
            is_valid = False

        # print(center[2], radius, check_valid, is_valid)
        
        return is_valid
    except:
        print('work_envelope: target point is invalid')

# tests  
# target_point = np.array([380.726, -266.95, 144.875])
# work_envelope_check(target_point)
