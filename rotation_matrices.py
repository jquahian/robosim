import math
import numpy as np

'''
rotation identity matrices
'''

def x_rot_mat(angle):

	x_rot = np.array([[1, 0, 0],
					[0, -math.cos(angle), -math.sin(angle)],
					[0, math.sin(angle), math.cos(angle)]])

	return x_rot

def y_rot_mat(angle):

	y_rot = np.array([[math.cos(angle), 0, math.sin(angle)],
					[0, 1, 0],
					[-math.sin(angle), 0, math.cos(angle)]])

	return y_rot

def z_rot_mat(angle):

	z_rot = np.array([[math.cos(angle), -math.sin(angle), 0],
					[math.sin(angle), math.cos(angle), 0],
					[0, 0, 1]])

	return z_rot
