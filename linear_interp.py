import numpy as np
import robot_config as config

'''
calcuations for movement on a linear course from waypoint A to waypoint B
we set a predetermined number of steps to take between the two waypoints
step size: can increase or decrease this depending on resolution needed
'''

j5_j6_length = config.j5_j6

def parse_waypoints(waypoints):
	targets = []
	orientations = []

	for point in waypoints:
		targets.append(point["point"])

	for orientation in waypoints:
		orientations.append(orientation["orientation"])

	return orientations, targets

'''
interpolate between the waypoints
create a set of equally spaced, descrete points from interpolation
pass an array of any size
'''

def create_vector(waypoints, interp_orientation):
	linear_positions = []
	
	# higher values mean more interpolated points
	step_size_multi = 0.25

	orientations, targets, = parse_waypoints(waypoints)

	if interp_orientation == True:
		interpolated_points = interp(targets, step_size_multi)
		interpolated_orientations = interp(orientations, step_size_multi)

		normalize_point_spread = len(interpolated_points) / len(interpolated_orientations)	

		interpolated_orientations = interp(orientations, step_size_multi * normalize_point_spread)

		if len(interpolated_points) != len(interpolated_orientations):
			interpolated_orientations.pop(-1)

		for i in range(len(interpolated_points)):
			new_point = dict({"orientation": interpolated_orientations[i],
							"point": interpolated_points[i]
					})

			linear_positions.append(new_point)
	else:
		interpolated_points = interp(targets, step_size_multi)
		orientation = orientations[0]

		for i in range(len(interpolated_points)):
			new_point = dict({"orientation":orientation,
							"point": interpolated_points[i]
						})
			linear_positions.append(new_point)
			
	print(len(linear_positions))
	return linear_positions

def interp(targets, step_size_multi):
	interp_point_list = []

	# generate our interpolated targets
	for i in range(len(targets)):
		if i == len(targets) - 1:
			print('last waypoint reached')
		else:
			vector = np.subtract(targets[i + 1], targets[i])
			vector_length = np.sqrt(np.sum(np.square(vector)))
			unit_vector = vector / vector_length

			# starting at the inital point, get a series of points between two way points
			# at 1 mm increments which can be modifed by step_size_multi
			step_size = int(vector_length * step_size_multi)

			for j in range(1, int(vector_length * step_size_multi)):
				# get a series of (more or less) equally spaced points
				interp_point = np.add(targets[i], np.multiply(
					j/step_size_multi, unit_vector))

				# generate a list of these points
				interp_point_list.append(interp_point)

	return interp_point_list
