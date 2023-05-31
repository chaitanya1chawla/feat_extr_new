import sys

if not hasattr(sys, 'argv'):
    sys.argv = []

import matplotlib.animation as animation
import matplotlib.pyplot as plt
import matplotlib.quiver as mquiver
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
from typing import Dict, List, Optional

plot_models_in_space_figure = None  # type: Optional[plt.Figure]
plot_models_in_space_figure_axes = None  # type: Optional[plt.Axes]


def plot_models_in_space(model_tag_positions: Dict[str, List[np.ndarray]],
                         model_tag_orientations: Dict[str, List[np.ndarray]], axis_limit: float = 10,
                         vector_scale: float = 1, model_colors: Dict[str, str] = None):
    if not sys.argv:
        sys.argv.append("python")

    global plot_models_in_space_figure, plot_models_in_space_figure_axes

    if plot_models_in_space_figure is None:
        plt.ion()
        plot_models_in_space_figure = plt.figure()  # type: plt.Figure
        plot_models_in_space_figure_axes = plot_models_in_space_figure.add_subplot(
            111, projection='3d')  # type: plt.Axes
    else:
        plot_models_in_space_figure_axes.clear()

    # Cartesian axes
    plot_models_in_space_figure_axes.quiver(-1.5, 0, 0, 1.5, 0, 0, color='#aaaaaa', linestyle='dashed')
    plot_models_in_space_figure_axes.quiver(0, -1.5, 0, 0, 1.5, 0, color='#aaaaaa', linestyle='dashed')
    plot_models_in_space_figure_axes.quiver(0, 0, -1.5, 0, 0, 1.5, color='#aaaaaa', linestyle='dashed')

    # Axis limits
    plot_models_in_space_figure_axes.set_xlim([-axis_limit, axis_limit])
    plot_models_in_space_figure_axes.set_ylim([-axis_limit, axis_limit])
    plot_models_in_space_figure_axes.set_zlim([0, axis_limit])

    for model_name in model_tag_positions:
        positions = np.array(model_tag_positions[model_name])
        orientations = vector_scale * np.array(model_tag_orientations[model_name])
        assert (positions.shape[0] == orientations.shape[0])
        assert (positions.shape[1] == orientations.shape[1] == 3)
        color = model_colors[model_name] if model_colors else 'r'

        for i in range(len(positions)):
            # Vector v at position p
            plot_models_in_space_figure_axes.quiver(positions[i, 0], positions[i, 1], positions[i, 2],
                                                    orientations[i, 0], orientations[i, 1], orientations[i, 2],
                                                    color=color)

    plot_models_in_space_figure.canvas.draw()
    plot_models_in_space_figure.canvas.flush_events()


def plot_model_in_space(model_tag_positions: List[np.ndarray], model_tag_orientations: List[np.ndarray],
                        ith_model: int, vector_scale: float = 1, axis_limit: float = 8, model_color: str = ""):
    if not sys.argv:
        sys.argv.append("python")

    global plot_models_in_space_figure, plot_models_in_space_figure_axes

    if ith_model == 0:
        plot_models_in_space_reset_canvas(axis_limit, False)

    positions = np.array(model_tag_positions)
    orientations = vector_scale * np.array(model_tag_orientations)
    assert (positions.shape[0] == orientations.shape[0])
    assert (positions.shape[1] == orientations.shape[1] == 3)
    color = model_color if model_color else 'r'

    for i in range(len(positions)):
        # Vector v at position p
        plot_models_in_space_figure_axes.quiver(positions[i, 0], positions[i, 1], positions[i, 2],
                                                orientations[i, 0], orientations[i, 1], orientations[i, 2],
                                                color=color)

    plot_models_in_space_figure.canvas.draw()
    plot_models_in_space_figure.canvas.flush_events()


def plot_models_in_space_reset_canvas(axis_limit: float = 8, with_redraw: bool = True):
    if not sys.argv:
        sys.argv.append("python")

    global plot_models_in_space_figure, plot_models_in_space_figure_axes

    if plot_models_in_space_figure is None:
        plt.ion()
        plot_models_in_space_figure = plt.figure()  # type: plt.Figure
        plot_models_in_space_figure_axes = plot_models_in_space_figure.add_subplot(
            111, projection='3d')  # type: plt.Axes
    else:
        plot_models_in_space_figure_axes.clear()

    # Cartesian axes
    plot_models_in_space_figure_axes.quiver(-1.5, 0, 0, 1.5, 0, 0, color='#aaaaaa', linestyle='dashed')
    plot_models_in_space_figure_axes.quiver(0, -1.5, 0, 0, 1.5, 0, color='#aaaaaa', linestyle='dashed')
    plot_models_in_space_figure_axes.quiver(0, 0, -1.5, 0, 0, 1.5, color='#aaaaaa', linestyle='dashed')

    # Axis limits
    plot_models_in_space_figure_axes.set_xlim([-axis_limit, axis_limit])
    plot_models_in_space_figure_axes.set_ylim([-axis_limit, axis_limit])
    plot_models_in_space_figure_axes.set_zlim([0, axis_limit])

    if with_redraw:
        plot_models_in_space_figure.canvas.draw()
        plot_models_in_space_figure.canvas.flush_events()


def plot_coordinate_frames(origin: np.ndarray, orientation: List[np.ndarray], vector_scale: float = 1,
                           axis_limit: float = 8, model_color: str = ""):
    if not sys.argv:
        sys.argv.append("python")

    global plot_models_in_space_figure, plot_models_in_space_figure_axes

    plot_models_in_space_reset_canvas(axis_limit, False)

    positions = np.zeros((3, 3))
    positions[0, :] = origin
    positions[1, :] = origin
    positions[2, :] = origin
    orientations = vector_scale * np.array(orientation)
    assert (positions.shape[0] == orientations.shape[0] == 3)
    assert (positions.shape[1] == orientations.shape[1] == 3)
    color = model_color if model_color else 'r'

    for i in range(len(positions)):
        # Vector v at position p
        plot_models_in_space_figure_axes.quiver(positions[i, 0], positions[i, 1], positions[i, 2],
                                                orientations[i, 0], orientations[i, 1], orientations[i, 2],
                                                color=color)

    plot_models_in_space_figure.canvas.draw()
    plot_models_in_space_figure.canvas.flush_events()


def plot_coordinate_frames_reset_canvas(axis_limit: float = 8, with_redraw: bool = True):
    plot_models_in_space_reset_canvas(axis_limit, with_redraw)


plot_trajectory_figure = None  # type: Optional[plt.Figure]
plot_trajectory_figure_axes = None  # type: Optional[plt.Axes]
plot_trajectory_key_stroke = None
plot_trajectory_result = ""
plot_trajectory_animation_pause = False
quit_flag = False


def plot_trajectory_process_keystroke(event):
    if event.key is None:
        return
    global plot_trajectory_key_stroke, plot_trajectory_result, plot_trajectory_animation_pause, quit_flag
    print("Pressed key: '", event.key, "'", sep="")
    plot_trajectory_key_stroke = event.key
    if plot_trajectory_key_stroke == 'y':
        plot_trajectory_result = 'y'
    elif plot_trajectory_key_stroke == 'n':
        plot_trajectory_result = 'n'
    elif plot_trajectory_key_stroke == 'p':
        plot_trajectory_animation_pause = True
    elif plot_trajectory_key_stroke == 'ctrl+q':
        quit_flag = not quit_flag
        print("quit_flag =", quit_flag)


def plot_trajectory_animation(num, data_set, line, red_dots, quivers: List[mquiver.Quiver]):
    # NOTE: there is no .set_data() for 3 dim data...
    line.set_data(data_set[0:2, :num])
    line.set_3d_properties(data_set[2, :num])
    red_dots.set_data(data_set[0:2, :num])
    red_dots.set_3d_properties(data_set[2, :num])
    for i, quiver in enumerate(quivers):
        if i < 3 * num:
            # draw / enable the quiver
            quiver.set_alpha(1)
        else:
            # clear / disable the quiver
            quiver.set_alpha(0)
    if 3 * num >= len(quivers):
        global plot_trajectory_animation_pause
        if plot_trajectory_animation_pause:
            input("Paused animation... Press anything to continue:")
            plot_trajectory_animation_pause = False
    return line


def plot_trajectory(origin: np.ndarray, orientation: np.ndarray, axis_limit: float = 8, animation_interval: int = 500,
                    quiver_axis_size: float = 1, show_full_no_animation: bool = False):
    global plot_trajectory_result
    plot_trajectory_result = ""

    # origin.shape = timeSteps x 3
    # orientation.shape = timeSteps * 3 x 3  # three vectors defining the three coordinate frame axes in 3D
    origin = np.array(origin)
    orientation = np.array(orientation)

    # print(origin.shape)
    # print(orientation.shape)
    # print(origin)
    # print(orientation)

    nr_points = origin.shape[0]
    assert (nr_points * 3 == orientation.shape[0])
    orientation *= quiver_axis_size

    if not sys.argv:
        sys.argv.append("python")

    if show_full_no_animation:
        global plot_trajectory_figure, plot_trajectory_figure_axes

        plot_trajectory_figure = plt.figure()  # type: plt.Figure
        plot_trajectory_figure_axes = plot_trajectory_figure.add_subplot(111, projection='3d')  # type: plt.Axes

        # Cartesian axes
        plot_trajectory_figure_axes.quiver(-quiver_axis_size, 0, 0, quiver_axis_size, 0, 0, color='#aaaaaa', linestyle='dashed')
        plot_trajectory_figure_axes.quiver(0, -quiver_axis_size, 0, 0, quiver_axis_size, 0, color='#aaaaaa', linestyle='dashed')
        plot_trajectory_figure_axes.quiver(0, 0, -quiver_axis_size, 0, 0, quiver_axis_size, color='#aaaaaa', linestyle='dashed')
        # noinspection PyTypeChecker
        plot_trajectory_figure_axes.set_xlim([-axis_limit, axis_limit])
        # noinspection PyTypeChecker
        plot_trajectory_figure_axes.set_ylim([-axis_limit, axis_limit])
        # noinspection PyTypeChecker
        plot_trajectory_figure_axes.set_zlim([-axis_limit, axis_limit])

        for i in range(nr_points):
            # x-axis
            plot_trajectory_figure_axes.quiver(origin[i, 0], origin[i, 1], origin[i, 2], orientation[3 * i, 0],
                                               orientation[3 * i, 1], orientation[3 * i, 2], color="b")
            # y-axis
            plot_trajectory_figure_axes.quiver(origin[i, 0], origin[i, 1], origin[i, 2], orientation[3 * i + 1, 0],
                                               orientation[3 * i + 1, 1], orientation[3 * i + 1, 2], color="g")
            # z-axis
            plot_trajectory_figure_axes.quiver(origin[i, 0], origin[i, 1], origin[i, 2], orientation[3 * i + 2, 0],
                                               orientation[3 * i + 2, 1], orientation[3 * i + 2, 2], color="r")

        # plot_trajectory_figure.canvas.draw()
        # plot_trajectory_figure.canvas.flush_events()

        print("Show no animation plot...")
        plt.show(block=True)
        print("Showed no animation plot!")

    # THE DATA POINTS
    data_set = origin.transpose()

    fig = plt.figure()
    ax = Axes3D(fig, auto_add_to_figure=False)
    fig.add_axes(ax)
    red_dots = ax.plot(data_set[0], data_set[1], data_set[2], linestyle='None', c='purple', marker='o',
                       markersize=10)[0]  # For scatter plot
    # NOTE: Can't pass empty arrays into 3d version of plot()
    line = ax.plot(data_set[0], data_set[1], data_set[2], lw=3, c='black')[0]  # For line plot
    quivers = []
    for i in range(nr_points):
        # x-axis
        quivers.append(ax.quiver(origin[i, 0], origin[i, 1], origin[i, 2], orientation[3 * i, 0], orientation[3 * i, 1],
                                 orientation[3 * i, 2], color="b"))
        # y-axis
        quivers.append(ax.quiver(origin[i, 0], origin[i, 1], origin[i, 2], orientation[3 * i + 1, 0],
                                 orientation[3 * i + 1, 1], orientation[3 * i + 1, 2], color="g"))
        # z-axis
        quivers.append(ax.quiver(origin[i, 0], origin[i, 1], origin[i, 2], orientation[3 * i + 2, 0],
                                 orientation[3 * i + 2, 1], orientation[3 * i + 2, 2], color="r"))

    # AXES PROPERTIES]
    ax.set_xlim3d([-axis_limit, axis_limit])
    ax.set_ylim3d([-axis_limit, axis_limit])
    ax.set_zlim3d([-axis_limit, axis_limit])
    ax.set_xlabel('X(t)')
    ax.set_ylabel('Y(t)')
    ax.set_zlabel('Z(t)')
    ax.set_title('Object trajectory evolution')
    # ax.axis("scaled")

    # disconnect cid with plt.gcf().canvas.mpl_disconnect(cid)
    cid = plt.gcf().canvas.mpl_connect("key_release_event", plot_trajectory_process_keystroke)

    # Creating the Animation object
    # print("Create animation")
    line_ani = animation.FuncAnimation(fig, plot_trajectory_animation, frames=nr_points + 1,
                                       fargs=(data_set, line, red_dots, quivers), interval=animation_interval,
                                       blit=False)
    # print("Created animation")
    # line_ani.save(r'Animation.mp4')

    # print("Show animation plot...")
    plt.show(block=True)
    # print("Showed animation plot!")

    plt.gcf().canvas.mpl_disconnect(cid)

    plt.close(plt.gcf())

    global quit_flag
    result = plot_trajectory_result
    plot_trajectory_result = ""
    return result, quit_flag


def plot_trajectory_reset_canvas(axis_limit: float = 8, with_redraw: bool = True):
    if not sys.argv:
        sys.argv.append("python")

    global plot_trajectory_figure, plot_trajectory_figure_axes

    if plot_trajectory_figure is None:
        plt.ion()
        plot_trajectory_figure = plt.figure()  # type: plt.Figure
        plot_trajectory_figure_axes = plot_trajectory_figure.add_subplot(111, projection='3d')  # type: plt.Axes
    else:
        plot_trajectory_figure_axes.clear()

    # Cartesian axes
    plot_trajectory_figure_axes.quiver(-1.5, 0, 0, 1.5, 0, 0, color='#aaaaaa', linestyle='dashed')
    plot_trajectory_figure_axes.quiver(0, -1.5, 0, 0, 1.5, 0, color='#aaaaaa', linestyle='dashed')
    plot_trajectory_figure_axes.quiver(0, 0, -1.5, 0, 0, 1.5, color='#aaaaaa', linestyle='dashed')

    # Axis limits
    # noinspection PyTypeChecker
    plot_trajectory_figure_axes.set_xlim([-axis_limit, axis_limit])
    # noinspection PyTypeChecker
    plot_trajectory_figure_axes.set_ylim([-axis_limit, axis_limit])
    # noinspection PyTypeChecker
    plot_trajectory_figure_axes.set_zlim([-axis_limit, axis_limit])

    if with_redraw:
        plot_trajectory_figure.canvas.draw()
        plot_trajectory_figure.canvas.flush_events()


if __name__ == "__main__":
    _res = plot_trajectory(np.array([[0, 0, 0], [2, 2, 2], [4, 4, 4]]),
                           np.array([[1, 0, 0], [0, -1, 0], [0, 0, 2],
                                     [1, 0, 0], [0, -1, 0], [0, 0, 2],
                                     [1, 0, 0], [0, -1, 0], [0, 0, 2]]), 8, 1000)
    print("Plot trajectory result: \"", _res, "\"", sep="")
