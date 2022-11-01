from sensor_axis_alignment.src.data.sensor_data_gen_from_driving_simulator_data \
    import sensor_data_generation_from_driving_simulator as dat_gen

import numpy as np
import pandas as pd

norm = np.linalg.norm
det = np.linalg.det
svd = np.linalg.svd


class GenTestData:
    def __init__(self):
        params = {}

        params['time interval'] = 0.01  # [s]
        params['init_LLA'] = [54.1002387, -4.6807492, 25]  # [deg], [deg], [m] Ballabeg, Isle of Man
        params['init_time'] = '2022-08-24 10:31:21'  # '%Y-%m-%d %H:%M:%S'
        params['gravity_constant'] = 9.81  # [m/s^2]

        # parameters for acc. sensor
        params['lever_arm'] = np.array([0, 0, 0])  # [m]
        params['acc_scale_factor'] = 1
        params['acc_cross_coupling'] = np.eye(3)
        params['acc_bias'] = np.array([0, 0, 0])  # [m/s^2]
        params['acc_noise_pow'] = 120  # [ug/sqrt(Hz)]
        params['acc_saturaion_upperlimit'] = np.inf  # [m/s^2]
        params['acc_saturaion_lowerlimit'] = -np.inf  # [m/s^2]
        params['acc_installation_attitude'] = [0, 0, 0]  # [0.05, 0.04, 0.03] [0.20, 0.16, 0.12]  # phi, the, psi[rad] Body -> Sensor

        # parameters for gyro. sensor
        params['gyro_scale_factor'] = 1
        params['gyro_cross_coupling'] = np.eye(3)
        params['gyro_bias'] = np.array([0, 0, 0])  # [rad/s]
        params['gyro_acc_sensitive_bias'] = np.array([0, 0, 0])  # [rad/s]
        params['gyro_noise_pow'] = 3.8  # [mdps/sqrt(Hz)]
        params['gyro_saturaion_upperlimit'] = np.inf
        params['gyro_saturaion_lowerlimit'] = -np.inf
        params['gyro_installation_attitude'] = [0, 0, 0]  # phi, the, psi[rad] Body -> Sensor

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

        self.params = params

    def set_params(self, lever_arm, imu_inst_att):
        self.params['lever_arm'] = np.array(lever_arm)
        self.params['acc_installation_attitude'] = imu_inst_att
        self.params['gyro_installation_attitude'] = imu_inst_att

    def gen_data(self, src_filename, dst_filename):
        sim_data = dat_gen(
            csv_filepath=src_filename,
            params=self.params
        )

        acc = sim_data.Ab_meas_ideal
        gyr = sim_data.W_meas_ideal
        speed = sim_data.kph

        df = pd.DataFrame({'t': sim_data.t,
                           'acc_x': acc[:, 0],
                           'acc_y': acc[:, 1],
                           'acc_z': acc[:, 2],
                           'gyr_x': gyr[:, 0],
                           'gyr_y': gyr[:, 1],
                           'gyr_z': gyr[:, 2],
                           'brake': sim_data.brake,
                           'throttle': sim_data.throttle,
                           'steering': sim_data.steering,
                           'speed': speed,
                           'gear': sim_data.gear})

        df.to_pickle(dst_filename)
        print('file saved')





if __name__ == '__main__':

    GTD = GenTestData()

    GTD.set_params(lever_arm=[0, 0, 0],
                   imu_inst_att=[0.05, 0.04, 0.03])
    GTD.gen_data(src_filename="sensor_axis_alignment/data/raw/ExampleInternalsTelemetryOutput(61).csv",
                 dst_filename="sensor_axis_alignment/data/processed/data61_1_imu_ctrl_speed.pickle")

    GTD.set_params(lever_arm=[0, 0, 0],
                   imu_inst_att=[0.20, 0.16, 0.12])
    GTD.gen_data(src_filename="sensor_axis_alignment/data/raw/ExampleInternalsTelemetryOutput(61).csv",
                 dst_filename="sensor_axis_alignment/data/processed/data61_2_imu_ctrl_speed.pickle")

    GTD.set_params(lever_arm=[0, 1, 0],
                   imu_inst_att=[0.05, 0.04, 0.03])
    GTD.gen_data(src_filename="sensor_axis_alignment/data/raw/ExampleInternalsTelemetryOutput(61).csv",
                 dst_filename="sensor_axis_alignment/data/processed/data61_3_imu_ctrl_speed.pickle")

    GTD.set_params(lever_arm=[0, 1, 0],
                   imu_inst_att=[0.20, 0.16, 0.12])
    GTD.gen_data(src_filename="sensor_axis_alignment/data/raw/ExampleInternalsTelemetryOutput(61).csv",
                 dst_filename="sensor_axis_alignment/data/processed/data61_4_imu_ctrl_speed.pickle")
