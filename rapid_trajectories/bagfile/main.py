from rosbags.rosbag2 import Reader
from rosbags.serde import deserialize_cdr
from pathlib import Path
from rosbags.typesys import get_types_from_idl, get_types_from_msg, register_types
import os
import matplotlib.pyplot as plt
import time
import numpy as np
import to_pandas
import glob
from ament_index_python.packages import get_package_share_directory
import common
import matplotlib as mpl
# import pandas as pd

os.path.expanduser('~/uuv/ros2/src')

base_path = os.path.realpath(os.path.dirname(__file__))
file_dir_name = 'perfect'
file_dir_name = 'damping_5.0'
path = os.path.join(base_path, 'files', file_dir_name)


def add_msgs(package_name):
    type_dict = {}
    path = os.path.join(get_package_share_directory(package_name), 'msg')
    files = glob.glob(path + '/*.msg')
    for file in files:
        path = Path(file)
        if not os.path.exists(file):
            print(f'Skipping message: {file}')
            continue
        msg_def = path.read_text('utf-8')
        msg_name = path.relative_to(path.parents[2]).with_suffix('')
        type_dict.update(get_types_from_msg(msg_def, str(msg_name)))
    register_types(type_dict)


add_msgs('rapid_trajectories_msgs')
add_msgs('hippo_msgs')


def get_data(topic):
    connections = [x for x in reader.connections if x.topic == topic]
    return [[deserialize_cdr(x[2], x[0].msgtype), x[1]]
            for x in reader.messages(connections=connections)]


def get_sections(dt_max):
    data = get_data('/uuv00/single_tracker/section_counter')
    counter = to_pandas.section_counter(data)
    data = get_data('/uuv00/single_tracker/trajectory_result')
    traj_result = to_pandas.trajectory_result(data)
    t_sections = to_pandas.sections(counter, traj_result, dt_max=dt_max)
    return t_sections


def get_homing_sections(t_sections):
    sections = np.zeros([len(t_sections) - 1, 2], dtype=float)
    print(t_sections.shape)
    for i in range(len(t_sections) - 1):
        sections[i, 0] = t_sections[i, 1]
        sections[i, 1] = t_sections[i + 1, 0]
    return sections


def plot_positions(t_sections):
    fig, axes = plt.subplots(2)
    cmap = plt.get_cmap('tab20')
    data = get_data('/uuv00/odometry')
    odom = to_pandas.odometry(data)
    for i in range(len(t_sections)):
        t0 = t_sections[i, 0]
        t1 = t_sections[i, 1]
        x, t = common.crop_data(odom.p[:, 0], odom.t, t0, t1)
        y, t = common.crop_data(odom.p[:, 1], odom.t, t0, t1)
        z, t = common.crop_data(odom.p[:, 2], odom.t, t0, t1)
        axes[0].plot(y, x, color=cmap(i))
        axes[1].plot(y, z, color=cmap(i))
    axes[0].invert_yaxis()
    axes[1].invert_yaxis()
    axes[0].set_aspect('equal')
    axes[1].set_aspect('equal')
    axes[0].set_xlabel('y-coordinate [m]')
    axes[0].set_ylabel('x-coordinate [m]')
    axes[0].set_xlim((0.0, 4.0))
    axes[0].set_ylim((0.0, 2.0))
    axes[1].set_xlabel('y-coordinate [m]')
    axes[1].set_ylabel('z-coordinate [m]')
    axes[1].set_xlim((0.0, 4.0))
    axes[1].set_ylim((-1.5, 0.0))


def plot_ring():
    plt.figure()
    data = get_data('/uuv00/single_tracker/trajectory_result')
    traj_result = to_pandas.trajectory_result(data)
    plt.scatter(traj_result.p_intersection[:, 1],
                traj_result.p_intersection[:, 2],
                marker='x')
    plt.scatter(traj_result.p_intersection_planned[:, 1],
                traj_result.p_intersection_planned[:, 2],
                marker='+')
    c = plt.Circle([0, 0], traj_result.r_target[0], fill=False)
    plt.gca().add_patch(c)
    plt.gca().set_aspect('equal', 'box')


with Reader(path) as reader:
    t_sections = get_sections(7.0)
    t_homing_sections = get_homing_sections(t_sections)
    plot_positions(t_sections)
    plot_ring()
    plt.show()
    # data = get_data('/uuv00/odometry')
    # print(len(data))
    # odom = to_pandas.odometry(data)
    # data = get_data('/uuv00/single_tracker/section_counter')
    # counter = to_pandas.section_counter(data)
    # data = get_data('/uuv00/single_tracker/trajectory_result')
    # traj_result = to_pandas.trajectory_result(data)
    # t_sections = to_pandas.sections(counter, traj_result, dt_max=5.5)
    # t_homing_sections = get_homing_sections(t_sections)
    # fig, axs = plt.subplots(4)
    # cmap = plt.get_cmap("tab20")
    # print(cmap)
    # for i in range(len(t_sections)):
    #     x, t = common.crop_data(odom.p[:, 0], odom.t, t_sections[i, 0],
    #                             t_sections[i, 1])
    #     y, t = common.crop_data(odom.p[:, 1], odom.t, t_sections[i, 0],
    #                             t_sections[i, 1])
    #     z, t = common.crop_data(odom.p[:, 2], odom.t, t_sections[i, 0],
    #                             t_sections[i, 1])
    #     vx, t = common.crop_data(odom.v_lin[:, 0], odom.t, t_sections[i, 0],
    #                              t_sections[i, 1])
    #     vy, t = common.crop_data(odom.v_lin[:, 1], odom.t, t_sections[i, 0],
    #                              t_sections[i, 1])
    #     vz, t = common.crop_data(odom.v_lin[:, 2], odom.t, t_sections[i, 0],
    #                              t_sections[i, 1])

    #     t0 = t_sections[i, 0]
    #     axs[0].plot(y, x, color=cmap(i))
    #     axs[1].plot(y, z, color=cmap(i))
    #     axs[2].plot(t - t0, vx, color=cmap(i))
    #     axs[3].plot(t - t0, vy, color=cmap(i))
    # axs[0].set_prop_cycle(None)
    # axs[1].set_prop_cycle(None)
    # t0 = t_sections[0, 0]
    # t1 = t_sections[len(t_sections) - 1, 1]
    # desired, t = common.crop_data(traj_result.p_desired, traj_result.t, t0, t1)
    # axs[0].invert_yaxis()
    # for i in range(len(desired)):
    #     axs[0].scatter(desired[i, 1], desired[i, 0], color=cmap(i))
    #     axs[1].scatter(desired[i, 1], desired[i, 2], color=cmap(i))
    # data = get_data('/uuv00/single_tracker/target_trajectory')
    # current_traj = to_pandas.trajectory(data)
    # plt.figure()
    # for i in range(len(t_sections)):
    #     t0 = t_sections[i, 0]
    #     t1 = t_sections[i, 1]
    #     p0, t = common.crop_data(current_traj.p0, current_traj.t, t0, t1)
    #     plt.plot(p0[:, 1], p0[:, 0])
    # plt.figure()
    # plt.scatter(traj_result.p_intersection[:, 1],
    #             traj_result.p_intersection[:, 2],
    #             marker='x')
    # c = plt.Circle([0, 0], traj_result.r_target[0], fill=False)
    # plt.gca().add_patch(c)
    # plt.gca().set_aspect('equal', 'box')
    # plt.show()
