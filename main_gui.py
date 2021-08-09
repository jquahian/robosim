import PySimpleGUI as sg
import controller
import robot_config as config

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

layout = [
    [sg.Button('Connect'), sg.Button('Calibrate'), sg.Button('Exit')],
    [sg.Column(indv_axis_col)],
]

window = sg.Window('Arm Control', layout)

while True:
	event, value = window.read()

	if event == 'Connect':
		controller.connect_to_boards()
		controller.set_speed(10.0)
		controller.set_accel(1.5)
		controller.set_decel(1.5)
		controller.move_indv_axis(0, 1, config.j2_gearing, 45.0)
		controller.move_indv_axis(1, 0, config.j3_gearing, 45.0)
		controller.move_indv_axis(2, 0, config.j3_gearing, 45.0)

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
