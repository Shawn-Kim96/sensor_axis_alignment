import numpy as np
import pandas as pd
import sensor_axis_alignment.src.imu_alignment as ia

def imu_alignment_test(
        filename_list,
        real_att_list,
        lever_arm_list,
        test_method_type,
        test_method_values,
):
    df_res = pd.DataFrame(columns=['roll_est', 'pitch_est', 'yaw_est',
                                   'roll_real', 'pitch_real', 'yaw_real',
                                   'roll_err', 'pitch_err', 'yaw_err',
                                   'gravity_extract_method', 'heading_alignment_data_select_method',
                                   'lever_arm'])


    for filename, real, lever_arm in zip(filename_list, real_att_list, lever_arm_list):
        df = pd.read_pickle(filename)
        method_definition = {
            'gravity_extract_method': None,
            'heading_alignment_data_select_method': None,
        }
        for method_value in test_method_values:
            method_definition[test_method_type] = method_value

            _, eul = ia.imu_axis_alignment(
                df,
                sample_rate=100,
                method_definition=method_definition
            )
            err = np.array(eul) - np.array(real)

            df_res = df_res.append({'roll_est': eul[0],
                                    'pitch_est': eul[1],
                                    'yaw_est': eul[2],
                                    'roll_real': real[0],
                                    'pitch_real': real[1],
                                    'yaw_real': real[2],
                                    'roll_err': err[0],
                                    'pitch_err': err[1],
                                    'yaw_err': err[2],
                                    'gravity_extract_method': method_definition['gravity_extract_method'],
                                    'heading_alignment_data_select_method': method_definition['heading_alignment_data_select_method'],
                                    'lever_arm': lever_arm},
                                   ignore_index=True)

    return df_res


if __name__ == "__main__":
    df_res = imu_alignment_test(
        filename_list=[
            "sensor_axis_alignment/data/processed/data61_1_imu_ctrl_speed.pickle",
            "sensor_axis_alignment/data/processed/data61_2_imu_ctrl_speed.pickle",
            "sensor_axis_alignment/data/processed/data61_3_imu_ctrl_speed.pickle",
            "sensor_axis_alignment/data/processed/data61_4_imu_ctrl_speed.pickle",
        ],
        real_att_list=[
            [0.05, 0.04, 0.03],
            [0.20, 0.16, 0.12],
            [0.05, 0.04, 0.03],
            [0.20, 0.16, 0.12]
        ],
        lever_arm_list=['no', 'no', 'yes', 'yes'],
        test_method_type='heading_alignment_data_select_method',
        test_method_values=[1, 2, 3],
    )




