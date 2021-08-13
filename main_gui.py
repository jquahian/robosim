import PySimpleGUI as sg
import controller
import robot_config as config
import programmer
import numpy as np


sg.theme('DarkBlack')
sg.SetOptions(font='Helvetica 12', auto_size_buttons=True)
		
window_width = 1024
window_height = 600

indv_axis_col = [
	[sg.Text('J1'), sg.Slider(default_value=0, orientation='h', range=(config.j1_angle_min_lim, config.j1_angle_max_lim), resolution=0.1, key='-J1-SLIDER-', enable_events=True)],
	[sg.Text('J2'), sg.Slider(default_value=0, orientation='h', range=(config.j2_angle_min_lim, config.j2_angle_max_lim), resolution=0.1, key='-J2-SLIDER-', enable_events=True)],
	[sg.Text('J3'), sg.Slider(default_value=0, orientation='h', range=(config.j3_angle_min_lim, config.j3_angle_max_lim), resolution=0.1, key='-J3-SLIDER-', enable_events=True)],
	[sg.Text('J4'), sg.Slider(default_value=0, orientation='h', range=(config.j4_angle_min_lim, config.j4_angle_max_lim), resolution=0.1, key='-J4-SLIDER-', enable_events=True)],
	[sg.Text('J5'), sg.Slider(default_value=0, orientation='h', range=(config.j5_angle_min_lim, config.j5_angle_max_lim), resolution=0.1, key='-J5-SLIDER-', enable_events=True)],
	[sg.Text('J6'), sg.Slider(default_value=0, orientation='h', range=(config.j6_angle_min_lim, config.j6_angle_max_lim), resolution=0.1, key='-J6-SLIDER-', enable_events=True)],
]

vert_horz_col = [
    [sg.Text('Vertical Position (mm)'), sg.Button('Up', key='-VERT-UP-BTN-')],
	[sg.Text('Horizontal Positon (mm)')],
	[sg.Text('Rotational Position(mm)')],
]

layout = [
    [sg.Button('Connect'), sg.Button('Calibrate'), sg.Button('Exit')],
    [sg.Column(indv_axis_col), sg.Column(vert_horz_col)],
    # [sg.Output(key='-CURRENT-POS-READOUT-')],
]

window = sg.Window('Arm Control', layout)

def program_movement(target):
    '''
    target is expecting a dictionary with orientation/point as key
    and values are both an np.array of floats with shape ([0., 0., 0.])
    for this, we ignore the orientation completely and feed in a linear movement
    and just the target point
    '''
    
    # pass in target point, linear move type, and no orientation blending
    angles = programmer.program_path(target, 'linear', interp_orient=False, realtime=True)
    # print(angles)
    
    controller.read_move_instructions(angles)

while True:
	event, value = window.read()

	if event == 'Connect':
		controller.connect_to_boards()
		controller.set_speed(10.0)
		controller.set_accel(1.5)
		controller.set_decel(1.5)

		# should put us back to point ~(350, 0, 244)
		# i know that this point is reachable and we can move vertically and horizontally
		# with the current solve method

		controller.move_indv_axis(0, 1, config.j2_gearing, 31.0)
		controller.move_indv_axis(1, 0, config.j3_gearing, 120.0)
		controller.move_indv_axis(2, 0, config.j5_gearing, -61.0)

	if event == 'Calibrate':
		controller.calibrate_all()

	if event == '-J1-SLIDER-':
		controller.move_indv_axis(0, 0, config.j1_gearing, value[event])
	if event == '-J2-SLIDER-':
		controller.move_indv_axis(0, 1, config.j2_gearing, value[event])
	if event == '-J3-SLIDER-':
		controller.move_indv_axis(1, 0, config.j3_gearing, value[event])
	if event == '-J4-SLIDER-':
		controller.move_indv_axis(1, 1, config.j4_gearing, value[event])
	if event == '-J5-SLIDER-':
		controller.move_indv_axis(2, 0, config.j5_gearing, value[event])
	if event == '-J6-SLIDER-':
		controller.move_indv_axis(2, 1, config.j6_gearing, value[event])

	if event == '-VERT-UP-BTN-':
		print('running streaming tests!')

		current_pos = controller.return_current_pos()
		current_pos = [current_pos[0], current_pos[1], current_pos[2]]
		print(f'current pos: {current_pos}')

		# moving 10 mm vertically
		new_pos_z = current_pos[2] + 100.0

		target_pos = [current_pos[0], current_pos[1], new_pos_z]
		print(f'target pos: {target_pos}')

		point_a = {
			"orientation": np.array([0., 90., 0.]),
			"point": np.array(current_pos),
		}

		point_b = {
			"orientation": np.array([0., 90., 0.]),
			"point": np.array(target_pos),
		}

		target_move = [point_a, point_b]

		program_movement(target_move)

	if event == sg.WIN_CLOSED or event == 'Exit':
		controller.zero_arm()
		break

	# window.Element('-CURRENT-POS-READOUT-').Update()

window.close()
