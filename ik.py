import math

# fixed joint lengths
# solve for the position/orientation of the spherical wrist separately
# where the tcp is the middle of the target and the length of the wrist is the radius aruond the point
# first, calculate the position of j3 around a 'sphere' to the target, then set the orientation of the wrist
j1_j2_length = 245.00
j2_j3_length = 200.00
j3_j4_length = 159.70

# has a div by zero error when entering xyz : 00z.  Needs exception for these coords
def solve_ik(x, y, z):
	# projection of arm on xy plane
	xy_projection_length = math.sqrt(pow(x, 2) + pow(y, 2))

	# distance from j1_j2 to the end effector z position
	r3 = z - j1_j2_length

	j2_effector_length = math.sqrt(pow(xy_projection_length, 2) + pow(r3, 2))

	# find rotation of joint 1
	j1_theta = round(math.degrees(math.asin(x/xy_projection_length)), 3)

	# find angle between joint length 2 and joint length 3
	phi_3 = math.acos((pow(j2_effector_length, 2) - pow(j2_j3_length, 2) -
					   pow(j3_j4_length, 2)) / (-2 * j2_j3_length * j3_j4_length))

	# find rotation of joint 3
	j3_theta = round(math.degrees(math.radians(180) - phi_3), 3)

	# angle between the j2_effector_length and horizontal x-axis
	phi_2 = math.acos((pow(j3_j4_length, 2) - pow(j2_j3_length, 2) -
					   pow(j2_effector_length, 2)) / (-2 * j2_j3_length * j2_effector_length))

	# angle between the j2_effector_length and the vertical z-axis
	phi_1 = math.asin((z - j1_j2_length)/j2_effector_length)

	# find rotation of joint 2 -- relative to the vertical axis
	j2_theta = round(math.degrees(phi_2 + phi_1 + math.radians(90)), 3)

	# need to correct which co-angle is chosen for each motor depending on motor configuration and ensure correct rotation direction
	if y < 0:
		j1_theta_motor = -(round(90 - j1_theta, 3))
	else:
		j1_theta_motor = (round(90 - j1_theta, 3))

	j2_theta_motor = round(180 - j2_theta, 3)
	j3_theta_motor = round(j3_theta, 3)

	# print(f'joint 1 target angle: {j1_theta} -- value to be passsed to motor is {j1_theta_motor}')
	# print(f'joint 2 target angle: {j2_theta} -- value to be passsed to motor is {j2_theta_motor}')
	# print(f'joint 3 target angle: {j3_theta} -- value to be passsed to motor is {j3_theta_motor}')
	
	motor_angles = [j1_theta_motor, j2_theta_motor, j3_theta_motor]

	return (motor_angles)

# # for tests
# solve_ik(78.0, 0, 100.0)
