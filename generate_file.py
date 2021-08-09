import os
import csv


def write_out(angles):
    with open('square_linear_no_blend_orientation.csv', mode='w', newline='') as file:
        writer = csv.writer(file, delimiter=',')
        for command in angles:
            writer.writerow(command)
        
    print('done!')

