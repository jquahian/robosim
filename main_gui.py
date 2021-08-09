import PySimpleGUI as sg
import controller
import robot_config as config
import fk

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
    [sg.Text('Vertical Position (mm)'), sg.Slider(default_value=0, orientation='h', range=(config.z_min, 500.0), resolution=0.1, key='-VERT-SLIDER-', enable_events=True)],
	[sg.Text('Horizontal Positon (mm)')],
	[sg.Text('Rotational Position(mm)')],
]

layout = [
    [sg.Button('Connect'), sg.Button('Calibrate'), sg.Button('Exit')],
    [sg.Column(indv_axis_col), sg.Column(vert_horz_col)],
]

def calc_current_pos():
    current_pos = fk.solve_fk(
		controller.return_joint_degrees(0, 0, config.j1_gearing),
		controller.return_joint_degrees(0, 1, config.j2_gearing),
		controller.return_joint_degrees(1, 0, config.j3_gearing),
		controller.return_joint_degrees(1, 1, config.j4_gearing),
		controller.return_joint_degrees(2, 0, config.j5_gearing),
		controller.return_joint_degrees(2, 1, config.j6_gearing),
	)
    
    # return current_pos

window = sg.Window('Arm Control', layout)

while True:
	event, value = window.read()

	if event == 'Connect':
		controller.connect_to_boards()
		controller.set_speed(10.0)
		controller.set_accel(1.5)
		controller.set_decel(1.5)
		controller.move_indv_axis(0, 1, config.j2_gearing, -(7.2 * 3.5))
		controller.move_indv_axis(1, 0, config.j3_gearing, (14.4 * 4.5))
		controller.move_indv_axis(2, 0, config.j5_gearing, 0)
		current_pos = calc_current_pos()
		print(current_pos)

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

	if event == sg.WIN_CLOSED or event == 'Exit':
		controller.zero_arm()
		break

window.close()
