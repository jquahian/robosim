import math
import numpy as np
import rotation_matrices as rot
import robot_config as config

'''
given angles for j1 - j5, determines where the arm is in space given as x/y/z

1. treat each joint as a distinct vector
2. apply rotations individually
3. add all vectors
'''

def solve_fk(j1_theta, j2_theta, j3_theta, j4_theta, j5_theta, j6_theta):

	# calculate position by finding the location by joints 2 + 3
	j2_j3_vec = np.matmul(config.j2_j3, rot.y_rot_mat(math.radians(j2_theta)))
	j3_j5_vec = np.matmul(
		config.j3_j4 + config.j4_j5, rot.y_rot_mat(math.radians(j2_theta + j3_theta)))

	# calculate orientation of the sphereical wrist
	j5_rot_vec_y = np.matmul(config.basis_vectors, rot.y_rot_mat(math.radians(j5_theta)))
	j4_rot_vec_z = np.matmul(j5_rot_vec_y, rot.z_rot_mat(math.radians(j4_theta)))

	# apply the rotation of the arm to the wrist
	wrist_arm_rot_vec_y = np.matmul(
		j4_rot_vec_z, rot.y_rot_mat(math.radians(j2_theta + j3_theta)))
	j5_j6_vec = np.matmul(config.j5_j6, wrist_arm_rot_vec_y)

	total_pos = np.add(j5_j6_vec, j2_j3_vec)
	total_pos = np.add(total_pos, j3_j5_vec)

	# find final postion by rotating about j1
	final_pos = np.matmul(total_pos, rot.z_rot_mat(math.radians(j1_theta)))
	final_pos = np.add(final_pos, config.position_offset)

	# flip the x coord to positive to match the orientation of the robot
	correction_matrix = np.array([[-1., 0., 0.],
								[0., 1., 0.],
								[0., 0., 1.]])

	final_pos = np.matmul(final_pos, correction_matrix)

	return final_pos
