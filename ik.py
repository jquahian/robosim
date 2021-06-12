import math
import robot_config as config
import numpy as np

'''
given point x/y/z, generates angles for j1, j2, j3 for arm positioning
'''

j1_j2_length = np.add(config.base_j1, config.j1_j2)
j1_j2_length = j1_j2_length[-1]
j2_j3_length = config.j2_j3[-1]
j3_j4_length = config.j3_j4[-1]
j4_j5_length = config.j4_j5[-1]
# j5_j6_length = config.j5_j6[-1]

j3_j5_length = j3_j4_length + j4_j5_length

# has a div by zero error when entering xyz : 00z.  Needs exception for these coords
def solve_ik(x, y, z):

	try:
		# projection of arm on xy plane
		xy_projection_length = math.sqrt(pow(x, 2) + pow(y, 2))

		# distance from j1_j2 to the end effector z position
		r3 = z - j1_j2_length

		j2_effector_length = math.sqrt(pow(xy_projection_length, 2) + pow(r3, 2))

		# find rotation of joint 1
		j1_theta = round(math.degrees(math.asin(x/xy_projection_length)), 3)

		# find angle between joint length 2 and joint length 3
		phi_3 = math.acos((pow(j2_effector_length, 2) - pow(j2_j3_length, 2) -
						pow(j3_j5_length, 2)) / (-2 * j2_j3_length * j3_j5_length))

		# find rotation of joint 3
		j3_theta = round(math.degrees(math.radians(180) - phi_3), 3)

		# angle between the j2_effector_length and horizontal x-axis
		phi_2 = math.acos((pow(j3_j5_length, 2) - pow(j2_j3_length, 2) -
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
		
		motor_angles = [j1_theta_motor, j2_theta_motor, j3_theta_motor]

		return (motor_angles)
	
	except:
		print('ik: requested position out of bounds')
