import numpy as np
import math
import ik
import fk
import rotation_matrices as rot
import robot_config as config


def solve_orientation(orientation_rot, target_point):
	x_theta = orientation_rot[0]
	y_theta = orientation_rot[1]
	z_theta = orientation_rot[2]
	
	orientation_vector = np.array([0., 0., 152.373])
	orientation = np.matmul(config.basis_vectors, rot.x_rot_mat(math.radians(x_theta)))
	orientation = np.matmul(orientation, rot.y_rot_mat(math.radians(y_theta)))
	orientation = np.matmul(orientation, rot.z_rot_mat(math.radians(z_theta)))
	orientation = np.matmul(orientation_vector, orientation)
	
	# this is the target for IK positioning
	j5_location = np.add(target_point, orientation)
	
	j5_x = j5_location[0]
	j5_y = j5_location[1]
	j5_z = j5_location[2]
	
	position_angles = ik.solve_ik(j5_x, j5_y, j5_z)
	j5_j6_intermediate_vector = fk.solve_fk(position_angles[0], position_angles[1], position_angles[2], 0, 0, 0)
	
	'''
	normalize the j5_j6 vector
	and the vector from the target point to joint 5's position
	to the origin so we can calculate the angle between them
	the resulting angle defines joint 5's rotation
	'''

	vec_a = np.subtract(j5_location, j5_j6_intermediate_vector)
	vec_b = np.subtract(j5_location, target_point)
	
	j5_angle = (vec_a[0] * vec_b[0] + vec_a[1] * vec_b[1] + vec_a[2] * vec_b[2])
	j5_angle = j5_angle / (math.sqrt(math.pow(vec_a[0], 2) + math.pow(vec_a[1], 2) + math.pow(vec_a[2], 2)) * math.sqrt(math.pow(vec_b[0], 2) + math.pow(vec_b[1], 2) + math.pow(vec_b[2], 2)))
	j5_angle = math.degrees(math.acos(j5_angle))
	
	'''
	are we on the target?
	check for colinearity between j5_target_vector and j5_j6_vector
	need to also check if the negative of j5_angle gets us a valid solution of j4
	spin j4 until we hit the target (or close to) i have no idea how else to solve this one
	this is horribly inefficient...
	'''

	potential_j4_angles = []
	for i in range(3610):
		i = i / 10
		check_target = fk.solve_fk(position_angles[0], position_angles[1], position_angles[2], i, -j5_angle, 0)

		co_linear_x = round(check_target[0]/target_point[0], 2)
		co_linear_y = round(check_target[1]/target_point[1], 2)
		co_linear_z = round(check_target[1]/target_point[1], 2)

		if co_linear_x >= 0.98 and co_linear_y >= 0.98 and co_linear_z >= 0.98:
			potential_j4_angles.append(i)
	
	# this code is sort of repeated below.  Refeactor these.
	j4_target_errors = []
	for angle in potential_j4_angles:
		location_check = fk.solve_fk(position_angles[0], position_angles[1], position_angles[2], angle, -j5_angle, 0)
		delta_x = abs(location_check[0] - target_point[0]) 
		delta_y = abs(location_check[1] - target_point[1]) 
		delta_z = abs(location_check[2] - target_point[2])

		total_delta = delta_x + delta_y + delta_z
		j4_target_errors.append(total_delta)

		# print(f'{angle}: error total: {total_delta}')
	
	minimal_error = min(j4_target_errors) 
	j4_angle_index = j4_target_errors.index(minimal_error)

	# need to consider the two solves that can get us to the same position
	j4_angle_option_1 = potential_j4_angles[j4_angle_index]
	# Check the opposite rotation solution
	j4_angle_option_2 = j4_angle_option_2 = j4_angle_option_1 - 360
	
	# find which of the two options requires the least amount of absolute rotation
	if abs(j4_angle_option_1) < abs(j4_angle_option_2):
		j4_angle = j4_angle_option_1
	else:
		j4_angle = j4_angle_option_2
	
	# check the opposite solution to the angle
	if j4_angle < 0:
		j4_angle_temp = j4_angle + 180
	else:
		j4_angle_temp = j4_angle - 180
	
	# final check to find smallest solution
	if abs(j4_angle_temp) < abs(j4_angle):
		j4_angle = j4_angle_temp
	else:
		j4_angle = j4_angle

	# otherwise the solution at like, 180.1, 179, 178 etc will have a crazy stupid rotation
	if abs(j4_angle - 180) <= 2:
		j4_angle = 0
	
	total_angles_a = [position_angles[0], position_angles[1], position_angles[2], j4_angle, j5_angle]
	total_angles_b = [position_angles[0], position_angles[1], position_angles[2], j4_angle, -j5_angle]
	total_angle_options = [total_angles_a, total_angles_b]

	# depedning on which j4_angle we choose, we pair with the correct j5_angle (positive or negative)
	check_j5_option_a = fk.solve_fk(total_angles_a[0], total_angles_a[1], total_angles_a[2], j4_angle, j5_angle, 0)
	check_j5_option_b = fk.solve_fk(total_angles_b[0], total_angles_b[1], total_angles_b[2], j4_angle, -j5_angle, 0)
	
	joint_5_options = [check_j5_option_a, check_j5_option_b]
	
	j5_target_errors = []
	for j5_options in joint_5_options:
		delta_x = abs(j5_options[0] - target_point[0])
		delta_y = abs(j5_options[1] - target_point[1])
		delta_z = abs(j5_options[2] - target_point[2])
		total_delta = delta_x + delta_y + delta_z

		j5_target_errors.append(total_delta)

	minimal_error = min(j5_target_errors)
	j5_angle_index = j5_target_errors.index(minimal_error)

	total_angles = total_angle_options[j5_angle_index]
	
	print(total_angles)
	return position_angles, total_angles


# tests
# target_point = np.array([338.168119, -0.0733414471,  240.000000])
# orientation_rot = np.array([0., 0., 0.])
# solve_orientation(orientation_rot, target_point)
