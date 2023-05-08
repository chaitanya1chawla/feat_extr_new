import sys
import matplotlib as mpl

if not hasattr(sys, 'argv'):
    sys.argv = []

from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import FancyArrowPatch
from mpl_toolkits.mplot3d import proj3d
import math


def print_greeting():
    print("Hello World from Python!")
    print(np.zeros(4))

def trafo_mat(mat = np.zeros([4,4]), ax1=plt.figure().add_subplot(111, projection='3d')):
    
    x_axis = np.matmul(mat[:3,:3],np.array([1,0,0]))
    y_axis = np.matmul(mat[:3,:3],np.array([0,1,0]))
    z_axis = np.matmul(mat[:3,:3],np.array([0,0,1]))

    #ax1.quiver([1,1,1], [1,1,1], [1,1,1], [1,0,0], [0,1,0], [0,0,1], arrow_length_ratio=0.1, color=['b','g','r'])
    
    ax1.quiver([mat[0,3], mat[0,3], mat[0,3]], [mat[1,3], mat[1,3], mat[1,3]], [mat[2,3], mat[2,3], mat[2,3]],
               [x_axis[0], y_axis[0], z_axis[0]],
               [x_axis[1], y_axis[1], z_axis[1]],
               [x_axis[2], y_axis[2], z_axis[2]],
               arrow_length_ratio=0.1, color=['b','g','r'])
    ax1.scatter(mat[0,3], mat[1,3], mat[2,3])

    return 
    

if __name__ == "__main__":
    print_greeting()

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlabel('X')
    ax.set_xlim(-1, 2)
    ax.set_ylabel('Y')
    ax.set_ylim(-1, 2)
    ax.set_zlabel('Z')
    ax.set_zlim(-1, 2)

    trafo_mat(np.array([[1,0,0,1.5],[0,1/2,-math.sqrt(3)/2, 1.5],[0,math.sqrt(3)/2,1/2,1.5],[0,0,0,1]]), ax)
    trafo_mat(np.array([[1,0,0,1.5],[0,0,-1, 1.5],[0,1,0,1.5],[0,0,0,1]]), ax)

    plt.show()