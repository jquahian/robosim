import numpy as np
import ik

'''
calcuations for movement on a linear course from waypoint A to waypoint B
we set a predetermined number of steps to take between the two waypoints
step size: can increase or decrease this depending on resolution needed
'''

# higher values mean more interpolated points
step_size_multi = 5

# interpolate between the waypoints
# create a set of equally spaced, descrete points from interpolation
# pass an array of any size
def create_vector(waypoints):
	interp_point_list = []
	
	for i in range(len(waypoints)):
		if i == len(waypoints) - 1:
			print('last waypoint reached')
		else:
			vector = np.subtract(waypoints[i + 1], waypoints[i])
			vector_length = np.sqrt(np.sum(np.square(vector)))
			unit_vector = vector / vector_length
			
			# starting at the inital point, get a series of points between two way points
			# at 1 mm increments which can be modifed by step_size_multi
			step_size = int(vector_length * step_size_multi)

			for j in range(1, int(vector_length * step_size_multi)):
				# get a series of (more or less) equally spaced points
				interp_point = np.add(waypoints[i], np.multiply(j/step_size_multi, unit_vector))
				
				# generate a list of these points
				interp_point_list.append(interp_point)
	
	return interp_point_list

def convert_ik(waypoints):
	interpolated_points = create_vector(waypoints)
	motor_angles = []
	
	for i in range(len(interpolated_points)):
		print(f'\ncommand {i}, point: {interpolated_points[i]}')
		
		motor_angles.append(ik.solve_ik(interpolated_points[i][0], 
									interpolated_points[i][1], 
									interpolated_points[i][2]))
	
	# print(motor_angles)
	return motor_angles

'''
for testing
'''

# waypoints, cartesian, position in mm
waypoints = [(175.0, 0.0, 270.0), (280.0, 0.0, 270.0)]
convert_ik(waypoints)

# create_vector(waypoints)
