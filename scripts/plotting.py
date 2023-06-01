import sys
import matplotlib as mpl

if not hasattr(sys, 'argv'):
    sys.argv = []

from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation
from matplotlib.patches import FancyArrowPatch
from mpl_toolkits.mplot3d import proj3d
import math


def print_greeting():
    print("Hello World from Python!")
    print(np.zeros(4))


def plot_coord():
    # mat=np.zeros([4, 4])
    global fig
    fig = plt.figure()
    global ax
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlabel('X')
    ax.set_xlim(-0.5, 0.5)
    ax.set_ylabel('Y')
    ax.set_ylim(-0.5, 0.5)
    ax.set_zlabel('Z')
    ax.set_zlim(-0.5, 0.5)

    # trafo_mat(np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]))
    # trafo_mat(mat, ax)
    # trafo_mat(np.array([[1,0,0,1.5],[0,1/2,-math.sqrt(3)/2, 1.5],[0,math.sqrt(3)/2,1/2,1.5],[0,0,0,1]]), ax)
    # trafo_mat(np.array([[1,0,0,1.5],[0,0,-1, 1.5],[0,1,0,1.5],[0,0,0,1]]), ax)


#    return ax


def trafo_mat(mat=[]):
    # , ax1=plt.figure().add_subplot(111, projection='3d')
    print("called")
    x = [0, 0, 0]
    y = [0, 0, 0]
    z = [0, 0, 0]
    u = [1, 0, 0]
    v = [0, 1, 0]
    w = [0, 0, 1]
    q = ax.quiver(x, y, z, u, v, w, arrow_length_ratio=0.1,
                  color=['b', 'g', 'r'])

    # ax1 = plot_coord()
    def animate(k, qr, mat):
        x_axis = np.matmul(mat[k][:3, :3], np.array([1, 0, 0]))
        y_axis = np.matmul(mat[k][:3, :3], np.array([0, 1, 0]))
        z_axis = np.matmul(mat[k][:3, :3], np.array([0, 0, 1]))

        # ax1.quiver([1,1,1], [1,1,1], [1,1,1], [1,0,0], [0,1,0], [0,0,1], arrow_length_ratio=0.1, color=['b','g','r'])
        u = [x_axis[0], y_axis[0], z_axis[0]]
        v = [x_axis[1], y_axis[1], z_axis[1]]
        w = [x_axis[2], y_axis[2], z_axis[2]]
        qr.set_UVC(u, v, w)
        qr.set_offsets([mat[k][0, 3], mat[k][0, 3], mat[k][0, 3]], [mat[k][1, 3], mat[k][1, 3], mat[k][1, 3]],
                       [mat[k][2, 3], mat[k][2, 3], mat[k][2, 3]])

        return qr

    ani = FuncAnimation(fig, animate, fargs=(q, mat), interval=200)
    plt.show()



#x_axis = np.matmul(mat[k][:3, :3], np.array([1, 0, 0]))
#y_axis = np.matmul(mat[k][:3, :3], np.array([0, 1, 0]))
#z_axis = np.matmul(mat[k][:3, :3], np.array([0, 0, 1]))#

# ax1.quiver([1,1,1], [1,1,1], [1,1,1], [1,0,0], [0,1,0], [0,0,1], arrow_length_ratio=0.1, color=['b','g','r'])

#ax.quiver([mat[k][0, 3], mat[k][0, 3], mat[k][0, 3]], [mat[k][1, 3], mat[k][1, 3], mat[k][1, 3]],
#          [mat[k][2, 3], mat[k][2, 3], mat[k][2, 3]],
#          [x_axis[0], y_axis[0], z_axis[0]],
#          [x_axis[1], y_axis[1], z_axis[1]],
#          [x_axis[2], y_axis[2], z_axis[2]],
#          arrow_length_ratio=0.1, color=['b', 'g', 'r'])
#ax.scatter(mat[k][0, 3], mat[k][1, 3], mat[k][2, 3])

def plot_show():
    plt.show()


if __name__ == "__main__":
    plot_coord()
