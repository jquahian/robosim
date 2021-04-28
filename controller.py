import time
import odrive
import linear_interp
from odrive.enums import *

# identify boards
odrv0 = '207D37A53548'
odrv1 = '387F37573437'
odrv2 = '20873592524B'

oboard = [odrv0, odrv1, odrv2]

# connect to boards
def connect_to_boards():
	global oboard

	# find the odrives
	print('Connecting to arm')

	oboard[0] = odrive.find_any(serial_number=odrv0)
	oboard[1] = odrive.find_any(serial_number=odrv1)
	oboard[2] = odrive.find_any(serial_number=odrv2)
	is_connected = True

	print('Arm connected')

# calibrate odrives and set to closed loop control
def calibrate_all():
	oboard[0].axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE

	while oboard[0].axis0.current_state != AXIS_STATE_IDLE:
		time.sleep(0.1)

	oboard[0].axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

	time.sleep(0.5)

	oboard[0].axis1.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE

	while oboard[0].axis1.current_state != AXIS_STATE_IDLE:
		time.sleep(0.1)

	oboard[1].axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

	time.sleep(0.5)

	oboard[1].axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE

	while oboard[1].axis0.current_state != AXIS_STATE_IDLE:
		time.sleep(0.1)

	oboard[1].axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

	time.sleep(0.5)

	oboard[1].axis1.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE

	while oboard[1].axis1.current_state != AXIS_STATE_IDLE:
		time.sleep(0.1)

	oboard[1].axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

	time.sleep(0.5)

	oboard[2].axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE

	while oboard[2].axis0.current_state != AXIS_STATE_IDLE:
		time.sleep(0.1)

	oboard[2].axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

	print('Joint calibration complete!')

def linear_trajectory(waypoints):
	motor_angles = linear_interp.convert_ik(waypoints)
	print(motor_angles[0][2])

	# go to first point
	oboard[0].axis0.controller.input_pos = calculate_motor_turns(50, motor_angles[0][0])
	oboard[0].axis1.controller.input_pos = calculate_motor_turns(50, motor_angles[0][1])
	oboard[1].axis0.controller.input_pos = calculate_motor_turns(25, motor_angles[0][2])
	time.sleep(4.0)

	for i in range(1, len(motor_angles)):
		oboard[0].axis0.controller.input_post = (calculate_motor_turns(50, motor_angles[i][0]))
		oboard[0].axis1.controller.move_incremental = (calculate_motor_turns(50, motor_angles[i][1]))
		oboard[1].axis0.controller.move_incremental = (calculate_motor_turns(25, motor_angles[i][2]))

		# adjust this for smooth motion -- maybe?
		time.sleep(0.1)

	# reset to zero-- for testing
	time.sleep(2.0)
	oboard[0].axis0.controller.input_pos = 0
	oboard[0].axis1.controller.input_pos = 0
	oboard[1].axis0.controller.input_pos = 0

def calculate_motor_turns(axis_gear_ratio, input_degrees):
	# calculates the number of motor turns to get to degrees based on gear ratio
	required_turns = (input_degrees * axis_gear_ratio) / 360
	return required_turns

# waypoints, cartesian, position in mm
waypoints = [(175.0, 0.0, 270.0), (200.0, 0.0, 270.0)]

def main():
	# connect_to_boards()
	# time.sleep(2.0)
	# calibrate_all()
	linear_trajectory(waypoints)

main()
