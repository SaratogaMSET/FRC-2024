import pandas
import numpy as np
import math
import gtsam
import json
import os

def load(log_dict):
    data = []
    step_data = {}

    for timestamp, numpy_arrays in log_dict.items():
        for tag_id, array_list in enumerate(numpy_arrays):
            np_array = np.array(array_list)
            if isinstance(np_array, np.ndarray) and not np.any(np.isnan(np_array)):
                step_data[tag_id] = gtsam.Pose3(np_array)
        data.append(step_data)

    robot_trajectory = None
    return data, None

def divide_chunks(l, n): 
    """
    l = List
    n = size of subarray
    """
    for i in range(0, len(l), n):  
        yield l[i:i + n] 

def camera_transform(index):
    if(index == 0):
        return gtsam.Pose3(gtsam.Rot3.RzRyRx(210 * math.pi / 180, 0 , 17.5 * math.pi/180), gtsam.Point3(-18.5/2 * 0.0254, 18.5/2 * 0.0254, 7.0625 * 0.0254)).matrix()
    elif(index == 1):
        return gtsam.Pose3(gtsam.Rot3.RzRyRx(150 * math.pi / 180, 0 , 17.5 * math.pi/180), gtsam.Point3(-18.5/2 * 0.0254, -18.5/2 * 0.0254, 7.0625 * 0.0254)).matrix()
    elif(index == 2):
        return gtsam.Pose3(gtsam.Rot3.RzRyRx(0, 0 , 0), gtsam.Point3(-18.5/2 * 0.0254, -18.5/2 * 0.0254, 7.0625 * 0.0254)).matrix()
    elif(index == 3):
        return gtsam.Pose3(gtsam.Rot3.RzRyRx(0, 0 , 0), gtsam.Point3(-18.5/2 * 0.0254, -18.5/2 * 0.0254, 7.0625 * 0.0254)).matrix()
def transform_matrix(Q):
    q0 = Q.iloc[3]
    q1 = Q.iloc[4]
    q2 = Q.iloc[5]
    q3 = Q.iloc[6]
    qMag = math.sqrt(q0**2 + q1**2 + q2**2 + q3**2)
    q0 /= qMag
    q1 /= qMag
    q2 /= qMag
    q3 /= qMag

    x = Q.iloc[0]
    y = Q.iloc[1]
    z = Q.iloc[2]

    r00 = 2 * (q0 * q0 + q1 * q1) - 1
    r01 = 2 * (q1 * q2 - q0 * q3)
    r02 = 2 * (q1 * q3 + q0 * q2)
     
    r10 = 2 * (q1 * q2 + q0 * q3)
    r11 = 2 * (q0 * q0 + q2 * q2) - 1
    r12 = 2 * (q2 * q3 - q0 * q1)
     
    r20 = 2 * (q1 * q3 - q0 * q2)
    r21 = 2 * (q2 * q3 + q0 * q1)
    r22 = 2 * (q0 * q0 + q3 * q3) - 1
     
    # 4x4 rotation matrix
    rot_matrix = np.array([[r00, r01, r02, x],
                           [r10, r11, r12, y],
                           [r20, r21, r22, z],
                           [0, 0, 0, 1]])
                        
    return rot_matrix

def processLog(log_name: str):
    script_directory = os.path.dirname(os.path.abspath(__file__))
    file_path = os.path.join(script_directory, log_name)

    df = pandas.read_csv(file_path, index_col="Timestamp")
    df0 = df.filter(regex = '/Vision/Camera 0/(Number of Targets Tracked|Pipeline Result/targets/\d/(fiducial_id|best_camera_to_target/(rotation|translation).*))')
    df1 = df.filter(regex = '/Vision/Camera 1/(Number of Targets Tracked|Pipeline Result/targets/\d/(fiducial_id|best_camera_to_target/(rotation|translation).*))')
    df2 = df.filter(regex = '/Vision/Camera 2/(Number of Targets Tracked|Pipeline Result/targets/\d/(fiducial_id|best_camera_to_target/(rotation|translation).*))')
    df3 = df.filter(regex = '/Vision/Camera 3/(Number of Targets Tracked|Pipeline Result/targets/\d/(fiducial_id|best_camera_to_target/(rotation|translation).*))')

    timestamps = pandas.read_csv(file_path, usecols=["Timestamp"],index_col="Timestamp").index.tolist()

    log_dict = {}

    for timestamp in timestamps:
        log_array = []
        def process_camera(camera_df, transform):
            row = camera_df.loc[timestamp]

            for i in range(16):
                log_array.append(np.nan)
            

            # First value of row is the targetCount. 
            temp = list(divide_chunks(row, n=8)) # splits the array into a list of dataframes(? what does loc do) with 9 varaibles
            # variable order is : [squatw, quatx, quaty, quatz, transx, transy,tranz, tag_id]

            for i in temp:
                if np.isnan(i.iloc[7]):
                    continue
                tag_id = int(i.iloc[7])
                # print(transform_matrix(i))
                log_array[tag_id] = (transform @ transform_matrix(i)).tolist()

        # process_camera(df0, camera_transform(0))
        process_camera(df1, camera_transform(1))
        # process_camera(df2, camera_transform(2))
        # process_camera(df3, camera_transform(3))
        log_dict[timestamp] = log_array
    return load(log_dict)

if __name__ == "__main__":
    processLog("TestLog2.csv")