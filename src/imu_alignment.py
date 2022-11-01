from copy import deepcopy

import navpy as nav
import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation as Rot

norm = np.linalg.norm
det = np.linalg.det
svd = np.linalg.svd


def check_go_stop(x, window_size, threshold_std, threshold_step):
    # stop : 0 / go : 1
    # tic : falling edge (1 -> 0)
    # tac : keep down state (0 -> 0)
    # toc : rising edge (0 -> 1)
    x_std = pd.Series(x).rolling(window=window_size).std()
    x_std[:window_size - 1] = threshold_std
    x_test = (x_std > threshold_std).values

    tic = np.zeros(len(x_test))
    tac = np.ones(len(x_test))
    toc = np.zeros(len(x_test))

    tic[1:] = (x_test[0:-1] == 1) & (x_test[1:] == 0)
    toc[1:] = (x_test[0:-1] == 0) & (x_test[1:] == 1)

    tic_idx = np.where(tic)[0]
    toc_idx = np.where(toc)[0]

    if toc_idx[0] < tic_idx[0]:
        tic_idx = np.hstack((0, tic_idx))

    if toc_idx[-1] < tic_idx[-1]:
        toc_idx = np.hstack((toc_idx, len(x)))

    for i, idx in enumerate(np.array([tic_idx, toc_idx]).T):
        if idx[1] - idx[0] > threshold_step:
            tac[idx[0]:idx[1]] = 0
    tic[1:] = (tac[0:-1] == 1) & (tac[1:] == 0)
    toc[1:] = (tac[0:-1] == 0) & (toc[1:] == 1)
    return tic, tac, toc


def z_align_matrix(a_g):
    norm = np.linalg.norm
    a_v = np.array([0, 0, 9.80665])

    u = np.cross(a_g, a_v)

    magnitude = norm(u)
    u = u / magnitude

    cos = np.inner(a_v, a_g) / np.absolute(norm(a_v) * norm(a_g))
    sin = norm(np.cross(a_g, a_v)) / np.absolute(norm(a_v) * norm(a_g))

    L_x = np.array([[0, 0, 0],
                    [0, 0, -1],
                    [0, 1, 0]])
    L_y = np.array(([0, 0, 1],
                    [0, 0, 0],
                    [-1, 0, 0]))
    L_z = np.array(([0, -1, 0],
                    [1, 0, 0],
                    [0, 0, 0]))
    L = [L_x, L_y, L_z]

    I = np.eye(3)

    uL = np.inner(u, L)

    R = I + sin * uL + (1 - cos) * (uL @ uL)

    return R


def xy_align_matrix(covMat):
    U, S, V = np.linalg.svd(covMat)

    U1 = deepcopy(U)
    U2 = deepcopy(U)
    # remove reflection
    if det(U) < 0:
        U1[-1, :] *= -1
        U2[0, :] *= -1

    R1 = np.eye(3)
    R1[0:2, 0:2] = U1
    R2 = np.eye(3)
    R2[0:2, 0:2] = U2

    R1 = Rot.from_matrix(R1)
    R2 = Rot.from_matrix(R2)
    #
    # R1.as_euler('zxy',degrees = True)
    # R2.as_euler('zxy',degrees = True)

    R1 = R1.as_matrix()
    R2 = R2.as_matrix()
    return R1, R2


def extract_gravity_vector(df, sample_rate, gravity_extract_method):
    if gravity_extract_method == 1:
        _, tac1, __ = check_go_stop(
            x=norm(df[['acc_x', 'acc_y', 'acc_z']].to_numpy(), axis=1),
            window_size=2 * sample_rate,
            threshold_std=12 / 128,
            threshold_step=2 * sample_rate
        )

        stop_area = tac1 == 0
        return np.array(
            [df.acc_x[stop_area].mean(),
             df.acc_y[stop_area].mean(),
             df.acc_z[stop_area].mean()]
        )
    else:
        raise NotImplementedError('other methods for extracting gravity vector are not currently implemented')


def align_heading(
        df,
        R_z,
        sample_rate,
        heading_alignment_data_select_method
):
    # data selection for extracting heading alignment matrix
    if heading_alignment_data_select_method == 1:
        P_rot = R_z @ df[['acc_x', 'acc_y', 'acc_z']].to_numpy().T

    elif heading_alignment_data_select_method in (2, 3):
        df['gyr_mag'] = np.linalg.norm(df[['gyr_x', 'gyr_y', 'gyr_z']].to_numpy(), axis=1)
        df['gyr_mag_std'] = df.gyr_mag.rolling(window=1 * sample_rate).std()

        condition = (df['gyr_mag_std'] <= df.gyr_mag_std.quantile(0.8)) & (df['steering'].abs() <= 2)

        twms = 5 * sample_rate  # time window min span. 5 [s] * 100[sample/s]
        df['gyr_select_flag'] = condition.rolling(twms).sum().shift(-twms).rolling(twms).max() == twms

        P_rot = R_z @ df.loc[df.gyr_select_flag, ['acc_x', 'acc_y', 'acc_z']].to_numpy().T

    else:
        raise NotImplementedError('other methods for heading alignment are not currently implemented')

    # extract candidate heading alignment matrix
    R1, R2 = xy_align_matrix(np.cov(P_rot[0:2, :]))

    # data selection for testing front/back direction of the candidate matrix
    P_rot_test = P_rot
    if heading_alignment_data_select_method in (1, 2):
        pass  # P_rot_test = P_rot

    elif heading_alignment_data_select_method == 3:
        df['speed_mean'] = df.speed.rolling(window=1 * sample_rate).mean().shift(int(-0.5 * sample_rate))
        df['dv'] = df.speed_mean - df.speed_mean.shift(1)
        df['accel_span_flag'] = (df.dv > 0) & (df.gear > 0)
        P_rot_test = df.loc[df.gyr_select_flag & df['accel_span_flag'], ['acc_x', 'acc_y', 'acc_z']].to_numpy().T

    # testing front/back direction of the candidate matrix
    # and return proper matrix
    if test_front_back(P_rot_test, R_z, R1) > 0:
        return R1 @ R_z
    else:
        return R2 @ R_z


def test_front_back(P_rot, R_z, R1):
    P_dir_test_rot = (R1 @ R_z) @ P_rot
    return P_dir_test_rot[0, :].mean()


def imu_axis_alignment(
        df,
        sample_rate,
        method_definition
):
    gravity_extract_method = method_definition.get('gravity_extract_method')
    heading_alignment_data_select_method = method_definition.get('heading_alignment_data_select_method')

    """
        gravity_extract_method : int
        1: acc only

        heading_alignment_data_select_method : int
        1: use all acc
        2: use selected acc by gyro, steering angle
        3: use selected acc by gyro, steering angle and testing with selected data
    """

    if gravity_extract_method is None:
        gravity_extract_method = 1

    if heading_alignment_data_select_method is None:
        heading_alignment_data_select_method = 1

    a_g = extract_gravity_vector(df, sample_rate, gravity_extract_method)
    R_z = z_align_matrix(a_g)
    R_fin = align_heading(df, R_z, sample_rate, heading_alignment_data_select_method)

    euler = [x - np.sign(x) * np.pi if np.abs(x) > np.pi / 2 else x for x in np.abs(np.array(nav.dcm2angle(R_fin)))]
    euler = euler[::-1]  # change for presenting order [roll, pitch, yaw]

    return R_fin, euler


if __name__ == "__main__":
    df = pd.read_pickle("data61_1_imu_ctrl_speed.pickle")
    real = [0.05, 0.04, 0.03]
    R_fin, euler = imu_axis_alignment(
        df,
        sample_rate=100,
        gravity_extract_method=1,
        heading_alignment_data_select_method=1,
    )