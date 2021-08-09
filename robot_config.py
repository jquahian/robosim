import numpy as np

'''
z-up

j1 - only z-axis
j2 - only y-axis
j3 - only y-axis
j4 - only z-axis
j5 - only y-axis
j6 - only z-axis

position offset: 244.5 add to z-up
'''

j1_gearing = 50
j2_gearing = 50
j3_gearing = 25
j4_gearing = 25
j5_gearing = 25
j6_gearing = 25

# link lengths as vectors
base_j1 = np.array([0., 0., 131.5])
j1_j2 = np.array([0., 0., 113.0])
j2_j3 = np.array([0., 0., 200.205])
j3_j4 = np.array([0., 0., 69.78])
j4_j5 = np.array([0., 0., 127.622])
j5_j6 = np.array([0., 0., 152.373])

position_offset = np.add(base_j1, j1_j2)

basis_vectors = np.array([[1, 0, 0],
						[0, 1, 0],
						[0, 0, 1]])

x_min = 0
x_max = 0
y_max = 0
y_min = 0
z_min = 45

# joint angle limits
j1_angle_min_lim = -90
j1_angle_max_lim = 90
j2_angle_min_lim = -20
j2_angle_max_lim = 120
j3_angle_min_lim = 0
j3_angle_max_lim = 140
j4_angle_min_lim = -180
j4_angle_max_lim = 180
j5_angle_min_lim = -90
j5_angle_max_lim = 120
j6_angle_min_lim = -360
j6_angle_max_lim = 360
