'''
TODO: 


need:
time (s) : time
 gyro_x, : imu gyro x (add back offset from cartscript.ino)
 gyro_y, : imu gyro y (add back offset from cartscript.ino)
 gyro_z, : imu gyro z (add back offset from cartscript.ino)
 acce_x, : imu acc x (add back offset from cartscript.ino)
 acce_y, : imu acc y (add back offset from cartscript.ino)
 acce_z (rad/s, m^2/s): imu acc z (add back offset from cartscript.ino)
 vio_gyro_bias_x, : offset from cartscript
 vio_gyro_bias_y, : offset from cartscript
 vio_gyro_bias_z, : offset from cartscript
 vio_acce_bias_x, : offset from cartscript
 vio_acce_bias_y, : offset from cartscript
 vio_acce_bias_z (rad/s, m^2/s) : offset from cartscript
 vio_p_x, : cam x
 vio_p_y, : cam y
 vio_p_z, : 0
 vio_q_w, : cv quat0
 vio_q_x, : cv quat1
 vio_q_y, : cv quat2
 vio_q_z (m) : cv quat3
 vio_v_x, : cam velocity x
 vio_v_y, : cam velocity y
 vio_v_z(m/s) : 0
 gt_p_x, 
 gt_p_y, 
 gt_p_z, : 0
 gt_q_w, 
 gt_q_x, 
 gt_q_y, 
 gt_q_z (m)
 gv_w, 
 gv_x, 
 gv_y, 
 gv_z
 rv_w, 
 rv_x, 
 rv_y, 
 rv_z
 magnet_un_x, : 0
 magnet_un_y, : 0
 magnet_un_z, : 0
 magnet_bias_x, : 0
 magnet_bias_y, : 0
 magnet_bias_z (uT) : 0
 |---pressure (Pa) : 0

 [cam_clock / 1e3, 0,0,0, 0,0,0, v_quat[1], cv_quat[2], cv_quat[3], cv_quat[0], coords.xCoor/100, coords.yCoor/100, 0, xVel, yVel, 0]
IMU_FILE.writerow([clock, 0, gx, gz, gy, ax, az, ay]) 
xVel, yVel -- visual 
 columns_name(width)": [
            "ts_us(1)",
            "gyr_compensated_rotated_in_World(3)",
            "acc_compensated_rotated_in_World(3)",
            "qxyzw_World_Device(4)",
            "pos_World_Device(3)",
            "vel_World(3)"
        ],
        "num_rows": len(cv_data),
        "approximate_frequency_hz": 30.0,
        "t_start_us": cv_data[0][0],
        "t_end_us": cv_data[-1][0]
'''

'''
given directory with csv + np file, (ignore whether it's test, train, val).
convert to csv in same format as original.
when running model, give directory to default.yaml

issue: times are different. solution: get start time from json, then only take in rows where time >= start time
'''

#!/usr/bin/env python3

import csv, json
import numpy as np

class csvConverter (object):

    def __init__(self, csv_path, npy_path, json_path):
        self.csv_path = csv_path
        self.npy_path = npy_path

        with open (json_path) as f:
            jsonfile = json.load(f)
            self.num_rows = jsonfile['num_rows']
            self.start_time = jsonfile['t_start_us']
            self.end_time = jsonfile['t_end_us']
        
    
    def parse_npy(self):
        data = np.load(self.npy_path)
        res = []
        for row in data:
            res.append(row.tolist())
        
        return res
    
    def parse_csv(self):
        res = []

        with open (self.csv_path, 'r', newline='') as csvfile:
            reader = csv.reader(csvfile, delimiter=',')
            for row in reader:
                time, _, gx, gy, gz, ax, ay, az = row #passes in 0 to the _
                print("new")
                print(time)
                print(self.start_time)
                print(self.end_time)
                if self.start_time <= int(time) and int(time) <= self.end_time:
                    res.append(row)
        
        return res

    def run_conversion(self):
        res = self.parse_npy()
        print("+=========================================")
        res2 = self.parse_csv()
        print(res2)

def main():
    csv_path = '/home/scilab/Desktop/workspace/datatester/imu_samples_0.csv'
    npy_path = '/home/scilab/Desktop/workspace/datatester/imu0_resampled.npy'
    json_path = '/home/scilab/Desktop/workspace/datatester/imu0_resampled_description.json'

    csvConv = csvConverter(csv_path, npy_path, json_path)
    csvConv.run_conversion()

if __name__ == "__main__":
    main()




    

