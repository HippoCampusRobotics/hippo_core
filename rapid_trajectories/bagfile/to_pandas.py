import numpy as np
from dataclasses import dataclass
import common
from pyquaternion import Quaternion


@dataclass
class Odometry:
    t: np.ndarray
    p: np.ndarray
    q: np.ndarray
    v_lin: np.ndarray
    v_ang: np.ndarray


@dataclass
class Int64:
    t: np.ndarray
    data: np.ndarray


@dataclass
class TrajectoryState:
    t: np.ndarray


@dataclass
class TrajectoryResult:
    t: np.ndarray
    success: np.ndarray
    p_desired: np.ndarray
    p_actual: np.ndarray
    v_desired: np.ndarray
    v_actual: np.ndarray
    a_desired: np.ndarray
    a_actual: np.ndarray
    q: np.ndarray
    p_intersection: np.ndarray
    p_intersection_planned: np.ndarray
    r_target: np.ndarray
    q_target: np.ndarray
    p_target: np.ndarray


@dataclass
class Trajectory:
    t: np.ndarray
    alpha: np.ndarray
    beta: np.ndarray
    gamma: np.ndarray
    m_rb: np.ndarray
    m_added: np.ndarray
    damping: np.ndarray
    duration: np.ndarray
    t_start_abs: np.ndarray
    time_margin: np.ndarray
    p0: np.ndarray
    v0: np.ndarray
    a0: np.ndarray
    rotation: np.ndarray


@dataclass
class AttitudeTarget:
    t: np.ndarray
    rpy: np.ndarray
    thrust: np.ndarray


def stamp_to_secs(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


def quat(q):
    return [q.w, q.x, q.y, q.z]


def vec(v):
    return [v.x, v.y, v.z]


def attitude_target(msgs):
    n = len(msgs)
    t = np.zeros([n], dtype=float)
    rpy = np.zeros([n, 3], dtype=float)
    thrust = np.zeros([n], dtype=float)
    for i in range(n):
        msg, time = msgs[i]
        t[i] = stamp_to_secs(msg)
        rpy[i] = vec(msg.attitude)
        thrust[i] = msg.thrust
    return AttitudeTarget(t=t, rpy=rpy, thrust=thrust)


def odometry(msgs):
    n = len(msgs)
    p = np.zeros([n, 3], dtype=float)
    q = np.zeros([n, 4], dtype=float)
    v_lin = np.zeros([n, 3], dtype=float)
    v_ang = np.zeros([n, 3], dtype=float)
    t = np.zeros([n], dtype=float)
    for i in range(n):
        msg, time = msgs[i]
        t[i] = stamp_to_secs(msg)
        pos = msg.pose.pose.position
        quat = msg.pose.pose.orientation
        ang = msg.twist.twist.angular
        lin = msg.twist.twist.linear
        p[i, :] = [pos.x, pos.y, pos.z]
        q[i, :] = [quat.w, quat.x, quat.y, quat.z]
        v_lin[i, :] = [lin.x, lin.y, lin.z]
        v_ang[i, :] = [ang.x, ang.y, ang.z]

    return Odometry(t=t, p=p, q=q, v_lin=v_lin, v_ang=v_ang)


def section_counter(msgs):
    n = len(msgs)
    t = np.zeros([n], dtype=float)
    data = np.zeros([n], dtype=int)
    for i in range(n):
        msg, time = msgs[i]
        t[i] = stamp_to_secs(msg)
        data[i] = msg.data
    return Int64(t=t, data=data)


def trajectory_result(msgs):
    n = len(msgs)
    t = np.zeros([n], dtype=float)
    success = np.zeros([n], dtype=bool)
    p_desired = np.zeros([n, 3], dtype=float)
    p_actual = np.zeros([n, 3], dtype=float)
    v_desired = np.zeros([n, 3], dtype=float)
    v_actual = np.zeros([n, 3], dtype=float)
    a_desired = np.zeros([n, 3], dtype=float)
    a_actual = np.zeros([n, 3], dtype=float)
    r_target = np.zeros([n], dtype=float)
    p_intersection = np.zeros([n, 3], dtype=float)
    p_intersection_planned = np.zeros([n, 3], dtype=float)
    q = np.zeros([n, 4], dtype=float)
    q_target = np.zeros([n, 4], dtype=float)
    p_target = np.zeros([n, 3], dtype=float)
    for i in range(n):
        msg, time = msgs[i]
        t[i] = stamp_to_secs(msg)
        success[i] = msg.success
        p_desired[i, :] = vec(msg.state_desired.position)
        p_actual[i, :] = vec(msg.state_actual.position)
        v_desired[i, :] = vec(msg.state_desired.velocity)
        v_actual[i, :] = vec(msg.state_actual.velocity)
        a_desired[i, :] = vec(msg.state_desired.acceleration)
        a_actual[i, :] = vec(msg.state_actual.acceleration)
        q[i, :] = quat(msg.orientation)
        r_target[i] = msg.target_radius
        p_intersection[i] = vec(msg.target_intersection)
        p_intersection_planned[i] = vec(msg.target_intersection_planned)
        q_target[i, :] = quat(msg.target_orientation)
        p_target[i, :] = vec(msg.target_position)

    return TrajectoryResult(t=t,
                            success=success,
                            p_desired=p_desired,
                            p_actual=p_actual,
                            v_desired=v_desired,
                            v_actual=v_actual,
                            a_desired=a_desired,
                            a_actual=a_actual,
                            r_target=r_target,
                            p_intersection=p_intersection,
                            p_intersection_planned=p_intersection_planned,
                            q=q,
                            q_target=q_target,
                            p_target=p_target)


def trajectory(msgs):
    n = len(msgs)
    t = np.zeros([n], dtype=float)
    p0 = np.zeros([n, 3], dtype=float)
    v0 = np.zeros([n, 3], dtype=float)
    a0 = np.zeros([n, 3], dtype=float)
    alpha = np.zeros([n, 3], dtype=float)
    beta = np.zeros([n, 3], dtype=float)
    gamma = np.zeros([n, 3], dtype=float)
    m_rb = np.zeros([n], dtype=float)
    m_added = np.zeros([n], dtype=float)
    damping = np.zeros([n], dtype=float)
    t_start_abs = np.zeros([n], dtype=float)
    time_margin = np.zeros([n], dtype=float)
    duration = np.zeros([n], dtype=float)
    rotation = np.zeros([n, 4], dtype=float)
    for i in range(n):
        msg, time = msgs[i]
        t[i] = stamp_to_secs(msg)
        p0[i] = vec(msg.trajectory.p0)
        v0[i] = vec(msg.trajectory.v0)
        a0[i] = vec(msg.trajectory.a0)
        alpha[i] = vec(msg.trajectory.alpha)
        beta[i] = vec(msg.trajectory.beta)
        gamma[i] = vec(msg.trajectory.gamma)
        m_rb[i] = msg.trajectory.mass_rb
        m_added[i] = msg.trajectory.mass_added
        damping[i] = msg.trajectory.damping
        t_start_abs[i] = msg.trajectory.t_start_abs_ns
        time_margin[i] = msg.trajectory.time_margin
        duration[i] = msg.trajectory.duration
        rotation[i] = quat(msg.trajectory.rotation)

    return Trajectory(t=t,
                      alpha=alpha,
                      beta=beta,
                      gamma=gamma,
                      m_rb=m_rb,
                      m_added=m_added,
                      damping=damping,
                      duration=duration,
                      t_start_abs=t_start_abs,
                      time_margin=time_margin,
                      p0=p0,
                      v0=v0,
                      a0=a0,
                      rotation=rotation)


def sections(section_counter, trajectory_result, dt_max):
    t1 = section_counter.t.copy()
    t2 = trajectory_result.t.copy()
    while t2[0] - t1[0] < 0:
        t2 = t2[1:]
    while t2[0] - t1[0] > dt_max:
        t1 = t1[1:]

    t1, t2 = common.limit_to_shorter(t1, t2)
    t = np.concatenate([t1, t2]).reshape([-1, 2], order='F')
    return t
