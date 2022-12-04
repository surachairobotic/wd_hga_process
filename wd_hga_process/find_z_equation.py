import matplotlib.pyplot as plt
import csv
from scipy.optimize import curve_fit 
import numpy as np
import math

def test(x, a, b, c, d):
    k = (a*pow(x,4)) + (b*pow(x,3)) + (c*pow(x,2)) + d 
    return k

if __name__ == '__main__':
    print('555')
    
    f = open('/home/cmit/Documents/z_tuner.csv', 'r')
    reader = csv.reader(f)
    print(reader)

    robot_z = []
    red_distance = []

    first = True
    for row in reader:
        if not first:
            robot_z.append(float(row[0]))
            red_distance.append(float(row[1]))
        first=False
    f.close()

    print('robot_z')
    print(robot_z)
    print('red_distance')
    print(red_distance)
    
    param, param_cov = curve_fit(test, red_distance, robot_z)
    print("Sine function coefficients:")
    print(param)
    print("Covariance of coefficients:")
    print(param_cov)
    
    ans = []
    for x in red_distance:
        ans.append(test(x, param[0], param[1], param[2], param[3]))
    
    diff_robot_z = []
    diff_red_distance = []
    for i in range(1, len(robot_z)):
        diff_robot_z.append(robot_z[i-1]-robot_z[i])
        diff_red_distance.append(red_distance[i-1]-red_distance[i])
    
    plt.subplot(3, 1, 1)
    plt.plot(red_distance, robot_z, red_distance, ans, '.')

    plt.subplot(3, 1, 2)
    plt.plot(red_distance, ans, '.')

    plt.subplot(3, 1, 3)
    plt.plot(diff_red_distance, '.')

    plt.show()
    
    
