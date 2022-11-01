import os

from simulator_6dof_model_and_sensors.src.models import rigidbody_6DOF as rb6DOF
import pandas as pd
import numpy as np

# driving simulator coord
# //   +x points out the left side of the car (from the driver's perspective)
# //   +y points out the roof
# //   +z points out the back of the car


def sensor_data_generation_from_driving_simulator(csv_filepath, params):
    sim_result = rb6DOF.SimData()
    df = pd.read_csv(csv_filepath)

    sim_result.brake = df['FilteredBrake'].map(lambda x: float(x[:-1]))
    sim_result.steering = df['FilteredSteering'].map(lambda x: float(x[:-1]))
    sim_result.throttle = df['FilteredThrottle'].map(lambda x: float(x[:-1]))
    sim_result.kph = df['KPH']
    sim_result.gear = df['Gear']
    sim_result.rpm = df['RPM']
    sim_result.water = df['Water']

    R_l2r = np.array([[0, 0, 1],
                      [-1, 0, 0],
                      [0, -1, 0]])

    pos = (R_l2r @ df[['mPosx', 'mPosy', 'mPosz']].to_numpy().T).T
    vel = (R_l2r @ df[['mLocalVelx', 'mLocalVely', 'mLocalVelz']].to_numpy().T).T
    rot = (R_l2r @ df[['mLocalRotx', 'mLocalRoty', 'mLocalRotz']].to_numpy().T).T

    acc = (R_l2r @ df[['mLocalAccelx', 'mLocalAccely', 'mLocalAccelz']].to_numpy().T).T
    rotacc = (R_l2r @ df[['mLocalRotAccelx', 'mLocalRotAccely', 'mLocalRotAccelz']].to_numpy().T).T
    quat = -(R_l2r @ df[['quatx', 'quaty', 'quatz']].to_numpy().T).T

    # S = x[0:3]
    # V = x[3:6]
    # W = x[6:9]
    # Q = x[9:13]
    sim_result.x = np.hstack((pos, vel, rot, df['quatw'].to_numpy().reshape(-1, 1), quat))
    sim_result.x_dot = np.hstack((vel, acc, rotacc))
    sim_result.t = df.DT.cumsum()
    sim_result.euler = list(map(rb6DOF.quat2euler, sim_result.x[:, 9:13]))
    sim_result.to_numpy()

    rb6DOF.Accelerometer(sim_result, params)
    rb6DOF.Gyrometer(sim_result, params)
    rb6DOF.Magnetometer(sim_result, params)
    rb6DOF.GPS(sim_result, params)
    rb6DOF.Barometer(sim_result, params)

    # sim result abstract
    print('sim_result contains')
    for key in sim_result.__dict__:
        print(f"{key} : {sim_result.__dict__[key].shape}")

    return sim_result


if __name__ == '__main__':
    # parameters for 6dof
    params = {}

    params['time interval'] = 0.01  # [s]
    params['init_LLA'] = [54.1002387, -4.6807492, 25]  # [deg], [deg], [m] Ballabeg, Isle of Man
    params['init_time'] = '2022-08-24 10:31:21'  # '%Y-%m-%d %H:%M:%S'
    params['gravity_constant'] = 9.81  # [m/s^2]

    # parameters for acc. sensor
    params['lever_arm'] = np.array([0, 1, 0])  # [m]
    params['acc_scale_factor'] = 1
    params['acc_cross_coupling'] = np.eye(3)
    params['acc_bias'] = np.array([0, 0, 0])  # [m/s^2]
    params['acc_noise_pow'] = 120  # [ug/sqrt(Hz)]
    params['acc_saturaion_upperlimit'] = np.inf  # [m/s^2]
    params['acc_saturaion_lowerlimit'] = -np.inf  # [m/s^2]
    params['acc_installation_attitude'] = [0.05, 0.04, 0.03]  # phi, the, psi[rad] Body -> Sensor

    # parameters for gyro. sensor
    params['gyro_scale_factor'] = 1
    params['gyro_cross_coupling'] = np.eye(3)
    params['gyro_bias'] = np.array([0, 0, 0])  # [rad/s]
    params['gyro_acc_sensitive_bias'] = np.array([0, 0, 0])  # [rad/s]
    params['gyro_noise_pow'] = 3.8  # [mdps/sqrt(Hz)]
    params['gyro_saturaion_upperlimit'] = np.inf
    params['gyro_saturaion_lowerlimit'] = -np.inf
    params['gyro_installation_attitude'] = [0.05, 0.04, 0.03]  # phi, the, psi[rad] Body -> Sensor

    # parameters for mag. sensor
    params['mag_cross_axis_sensitivity'] = 0.2  # [%FS/gauss]
    params['mag_linearity'] = 0.1  # [%FS/gauss]
    params['mag_installation_attitude'] = [0.05, 0.04, 0.03]  # phi, the, psi[rad] Body -> Sensor

    # parameters for GPS
    params['GPS_horizontal_position_accuracy'] = 1.5  # [m], CEP50
    params['GPS_vertical_position_accuracy'] = 3  # [m], CEP50
    params['GPS_velocity_accuracy'] = 0.08  # [m/s], 1-sigma
    params['GPS_heading_accuracy'] = 0.3  # [deg], 1-sigma

    # parameters for barometer
    params['gas_constant'] = 8.3145  # [J / mol K]
    params['air_molar_mass'] = 28.97  # [g / mol]
    params['local_ref_temperature'] = 25  # [Celsius]
    params['local_ref_pressure'] = 1013.25  # [hPa]
    params['baro_solder_drift'] = 30  # [Pa]
    params['baro_relative_accuracy'] = 6  # [Pa]
    params['baro_short_term_drift'] = 1.5  # [Pa]
    params['baro_long_term_drift'] = 10  # [Pa]
    params['baro_absolute_accuracy'] = 30  # [Pa]
    params['baro_noise_stddev'] = 0.95  # [PaRMS]

    if "\\src" in os.getcwd():
        os.chdir(os.getcwd().replace("\\src", ""))
    elif "/src" in os.getcwd():
        os.chdir(os.getcwd().replace("/src", ""))

    sim_data = sensor_data_generation_from_driving_simulator(
        csv_filepath="data\\ExampleInternalsTelemetryOutput(61).csv",
        params=params
    )
