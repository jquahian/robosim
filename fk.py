import numpy as np
import rotation_matrices as rot_mat
import math
import robot_config as config


def solve_fk(j1_theta, j2_theta, j3_theta, j4_theta, j5_theta, j6_theta):
    """[returns the position of joint 6 based on the given angles of joints 1 - 5]

    Args:
        angles (j1, j2, j3, j4, j5, j6 angles): [angles of joints 1 - 6, as floats in degrees]

    Returns:
        [x-coord, y-coord, z-coord]: [cartesian coordinates of joint 6 as floats in mm]
    """
    # solve from j5 down

    # rotation of j5 about j5 on global y
    pos = np.matmul(config.j5_j6, rot_mat.y_rot_mat(math.radians(j5_theta)))

    # rotation of j5 about j4 on global z
    pos = np.matmul(pos, rot_mat.z_rot_mat(math.radians(j4_theta)))

    # apply j5 offset -- length of j4 to j5
    pos = np.add(pos, config.j4_j5)

    # apply length of j3 to j4 and rotate about j3
    pos = np.add(pos, config.j3_j4)

    pos = np.matmul(pos, rot_mat.y_rot_mat(math.radians(j3_theta)))

    # apply length of j2 to j3 and rotate about j2
    pos = np.add(pos, config.j2_j3)

    pos = np.matmul(pos, rot_mat.y_rot_mat(math.radians(j2_theta)))

    # apply length of base to j2 and rotate about j1
    pos = np.add(pos, config.position_offset)

    pos = np.matmul(pos, rot_mat.z_rot_mat(math.radians(j1_theta)))

    # correct for sign on x-coord based on robot orientation
    pos[0] = -pos[0]

    print(f'position: {pos} with j6 rotated {j6_theta} degrees')
    
    return pos

## test
## correct answer: [386.67815519 238.82363824 430.02169076]
#solve_fk(30, 33.5, 45, 15, 20, 0)
