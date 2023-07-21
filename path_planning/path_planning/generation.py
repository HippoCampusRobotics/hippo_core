# Copyright (C) 2023 Thies Lennart Alff
# Copyright (C) 2023 Nathalie Bauschmann

# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License
# as published by the Free Software Foundation; either version 2
# of the License, or (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA

import numpy as np
import copy
import pyquaternion


class Generator:

    def __init__(self) -> None:
        self.position: np.ndarray
        self.heading = np.ndarray

    def lemniscate_of_bernoulli(self, n_samples, bounding_box):
        ax = bounding_box[0] / np.sqrt(2) * 0.5
        ay = bounding_box[1]
        a = np.min([ax, ay])
        t = np.linspace(0.0, 2.0 * np.pi, n_samples)

        def x_coord(t):
            return (a * np.sqrt(2) * np.cos(t)) / (np.sin(t)**2 + 1.0)

        def y_coord(t):
            return (a * np.sqrt(2) * np.cos(t) * np.sin(t) /
                    (np.sin(t)**2 + 1.0))

        x = x_coord(t)
        y = y_coord(t)
        z = np.zeros_like(x)
        self.position = np.array([x, y, z]).transpose()
        self.compute_heading(closed_loop=True)
        return self.position

    def compute_heading(self, closed_loop=True):
        self.heading = np.zeros_like(self.position)
        length = len(self.position)
        for i in range(length - 1):
            p = self.position[i, :]
            p_next = self.position[i + 1, :]
            v = p_next - p
            self.heading[i, :] = v / np.linalg.norm(v)
        if closed_loop:
            p = self.position[length - 1, :]
            p_next = self.position[0, :]
            v = p_next - p
            self.heading[length - 1, :] = v / np.linalg.norm(v)
        else:
            self.heading[length - 1, :] = self.heading[length - 2, :]

    def move(self, offset: np.ndarray):
        self.position += offset

    def scale(self, factor):
        self.position *= factor

    def swap_xy(self):
        self.position[:, [0, 1]] = self.position[:, [1, 0]]

    def rotate(self, axis, angle):
        q = pyquaternion.Quaternion(axis=axis, angle=angle)
        for i in range(self.position.shape[0]):
            self.position[i, :] = q.rotate(self.position[i, :])


if __name__ == '__main__':
    import matplotlib.pyplot as plt

    # see https://stackoverflow.com/questions/13685386/how-to-set-the-equal-aspect-ratio-for-all-axes-x-y-z
    def set_axes_equal(ax):
        x_limits = ax.get_xlim3d()
        y_limits = ax.get_ylim3d()
        z_limits = ax.get_zlim3d()

        x_range = abs(x_limits[1] - x_limits[0])
        x_middle = np.mean(x_limits)
        y_range = abs(y_limits[1] - y_limits[0])
        y_middle = np.mean(y_limits)
        z_range = abs(z_limits[1] - z_limits[0])
        z_middle = np.mean(z_limits)

        # The plot bounding box is a sphere in the sense of the infinity
        # norm, hence I call half the max range the plot radius.
        plot_radius = 0.5 * max([x_range, y_range, z_range])

        ax.set_xlim3d([x_middle - plot_radius, x_middle + plot_radius])
        ax.set_ylim3d([y_middle - plot_radius, y_middle + plot_radius])
        ax.set_zlim3d([z_middle - plot_radius, z_middle + plot_radius])

    g = Generator()
    g.lemniscate_of_bernoulli(100, [2, 1])
    g.swap_xy()
    g.rotate([1, 0, 0], np.pi / 4.0)
    g.compute_heading(True)
    heading = g.heading
    path = g.position
    print(path.shape)
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(path[:, 0], path[:, 1], path[:, 2])
    for i in range(len(heading)):
        p = path[i, :]
        v = heading[i, :] * 0.05
        ax.quiver(p[0], p[1], p[2], v[0], v[1], v[2], color='red')
    set_axes_equal(ax)
    plt.show()
