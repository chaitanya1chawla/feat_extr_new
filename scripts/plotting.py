import sys

if not hasattr(sys, 'argv'):
    sys.argv = []

from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import numpy as np


def print_greeting():
    print("Hello World from Python!")
    print(np.zeros(4))


if __name__ == "__main__":
    print_greeting()
