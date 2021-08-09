import time
import odrive
import robot_config as config
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

	for board in oboard:
		board.axis0.controller.config.input_mode = INPUT_MODE_TRAP_TRAJ
		board.axis1.controller.config.input_mode = INPUT_MODE_TRAP_TRAJ

	print('Arm connected')

# calibrate odrives and set to closed loop control
# only used if pre-calibration is not setup/fails
def calibrate_all():
	global oboard
	
	for board in oboard:
		board.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE

		while board.axis0.current_state != AXIS_STATE_IDLE:
				time.sleep(0.1)

		board.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

		time.sleep(0.5)

		board.axis1.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE

		while board.axis1.current_state != AXIS_STATE_IDLE:
				time.sleep(0.1)

		board.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

def move_indv_axis(drive_num, axis_num, gear_ratio, degrees):
	global oboard

	# we get back the numbers as a string unfortunately
	degrees = float(degrees)

	if axis_num == 0:
		oboard[drive_num].axis0.controller.input_pos = return_num_turns(
			gear_ratio, degrees)
	elif axis_num == 1:
		oboard[drive_num].axis1.controller.input_pos = return_num_turns(
			gear_ratio, degrees)
	else:
		print('Fn move_indv_axis ERROR: axis does not exist!')

def set_speed(new_speed):
	global oboard

	new_speed = float(new_speed)

	for board in oboard:
		board.axis0.trap_traj.config.vel_limit = new_speed
		board.axis1.trap_traj.config.vel_limit = new_speed

def set_accel(new_accel):
	global oboard

	new_accel = float(new_accel)

	for board in oboard:
		board.axis0.trap_traj.config.accel_limit = new_accel
		board.axis1.trap_traj.config.accel_limit = new_accel

def set_decel(new_decel):
	global oboard

	new_decel = float(new_decel)

	for board in oboard:
		board.axis0.trap_traj.config.decel_limit = new_decel
		board.axis1.trap_traj.config.decel_limit = new_decel

def zero_arm():
	oboard[0].axis0.controller.input_pos = return_num_turns(50, 0)
	oboard[0].axis1.controller.input_pos = return_num_turns(50, 0)
	oboard[1].axis0.controller.input_pos = return_num_turns(25, 0)
	oboard[1].axis1.controller.input_pos = return_num_turns(25, 0)
	oboard[2].axis0.controller.input_pos = return_num_turns(25, 0)
	oboard[2].axis1.controller.input_pos = return_num_turns(25, 0)

def return_num_turns(axis_gear_ratio, input_degrees):
	# calculates the number of motor turns to get to degrees based on gear ratio
	turns = (input_degrees * axis_gear_ratio)/360
	return turns

# returns the current angle of requested joint
def return_joint_degrees(drive_num, axis_num, axis_gear_ratio):
	global oboard

	if axis_num == 0:
		joint_angle = (oboard[drive_num].axis0.controller.input_pos * 360 / axis_gear_ratio)
	elif axis_num == 1:
		joint_angle = (oboard[drive_num].axis1.controller.input_pos * 360 / axis_gear_ratio)

	return joint_angle

# returns the current velocity of the requested joint
def return_joint_velocity(drive_num, axis_num):
	global oboard

	if axis_num == 0:
		joint_vel_est = oboard[drive_num].axis0.encoder.vel_estimate
	elif axis_num == 1:
		joint_vel_est = oboard[drive_num].axis1.encoder.vel_estimate
	
	return joint_vel_est

# returns a joint number for human feedback
def return_joint_number(drive_num, axis_num):
	if drive_num == 0 and axis_num == 0:
		joint_num = 1
	elif drive_num == 0 and axis_num == 1:
		joint_num = 2
	elif drive_num == 1 and axis_num == 0:
		joint_num = 3
	elif drive_num == 1 and axis_num == 1:
		joint_num = 4
	elif drive_num == 2 and axis_num == 0:
		joint_num = 5
	elif drive_num == 2 and axis_num == 1:
		joint_num = 6

	return joint_num
