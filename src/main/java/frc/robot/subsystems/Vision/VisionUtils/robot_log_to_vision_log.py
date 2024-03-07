import pandas
import numpy as np

# df = pandas.read_csv("8.45.pm.18.2.24.csv", index_col="Timestamp")

# df = df.filter(regex = 'NT:/AdvantageKit/Vision/Camera \d/Pipeline Result/targets/\d/.+')

# df = df.filter(regex = 'NT:/AdvantageKit/Vision/Camera \d/Pipeline Result/targets/\d/(fiducial_id|best_camera_to_target/(rotation|translation).*)')

# Yield successive n-sized 
# chunks from l. 
def divide_chunks(l, n): 
    """
    l = List
    N = size of subarray
    """
      
    # looping till length l 
    for i in range(0, len(l), n):  
        yield l[i:i + n] 
 
def transform_matrix(Q):
    """
    Covert a quaternion + translation into a full three-dimensional transform matrix.
 
    Input
    :param Q: A 7 element array representing the quaternion (x,y,z,q0,q1,q2,q3) 
 
    Output
    :return: A 4x4 element matrix representing the full 3D transform matrix. 
             This transform matrix converts a point in the local reference 
             frame to a point in the global reference frame.
    """
    # Extract the values from Q

    quat = Q[3:]

    def normalize(v, tolerance=0.00001):
        mag2 = sum(n * n for n in v)
        if abs(mag2 - 1.0) > tolerance:
            mag = np.sqrt(mag2)
            v = list(n / mag for n in v)
        return v
    
    quat = normalize(quat)
    
    q0 = quat[0]
    q1 = quat[1]
    q2 = quat[2]
    q3 = quat[3]

    x = Q[0]
    y = Q[1]
    z = Q[2]

    # First row of the rotation matrix
    r00 = 2 * (q0 * q0 + q1 * q1) - 1
    r01 = 2 * (q1 * q2 - q0 * q3)
    r02 = 2 * (q1 * q3 + q0 * q2)
     
    # Second row of the rotation matrix
    r10 = 2 * (q1 * q2 + q0 * q3)
    r11 = 2 * (q0 * q0 + q2 * q2) - 1
    r12 = 2 * (q2 * q3 - q0 * q1)
     
    # Third row of the rotation matrix
    r20 = 2 * (q1 * q3 - q0 * q2)
    r21 = 2 * (q2 * q3 + q0 * q1)
    r22 = 2 * (q0 * q0 + q3 * q3) - 1
     
    # 4x4 rotation matrix
    rot_matrix = np.array([[r00, r01, r02, x],
                           [r10, r11, r12, y],
                           [r20, r21, r22, z],
                           [0, 0, 0, 1]])
                        
    return rot_matrix

def empty_4x4_array():
    return np.array([[0, 0, 0, 0],
                    [0, 0, 0, 0],
                    [0, 0, 0, 0],
                    [0, 0, 0, 0]])

def computeVisionData(log_name: str):
    """
    Convert csv log to numpy data. 

    Input
    :param log_name: string/path name to csv log file exported from Advantagescope. 

    Output
    :return: a python list, where each item is a python list with 16 numpy matricies(4x4, transformation). Each index(0-index) 
        represents the apriltag id(index 0 -> tag id 1). 
    
    """
    df = pandas.read_csv(log_name, index_col="Timestamp")

    df = df.filter(regex = 'NT:/AdvantageKit/Vision/Camera \d/Pipeline Result/targets/\d/.+')

    df = df.filter(regex = 'NT:/AdvantageKit/Vision/Camera \d/Pipeline Result/targets/\d/(fiducial_id|best_camera_to_target/(rotation|translation).*)')

    log_array = []

    for index, row in df.iterrows():
        numpy_arrays = []

        for i in range(16): numpy_arrays.append(empty_4x4_array())
        temp = list(divide_chunks(row, n=8))

        for i in temp:
            if (np.isnan(i[7]) == True): continue
            flag = int(i[7])
            numpy_arrays[flag] = transform_matrix(i)

        # np.split(temp, 8)
        log_array.append(numpy_arrays)

    # df.to_csv("output.csv")


    print(log_array) 

if __name__ == "__main__":
    computeVisionData("8.45.pm.18.2.24.csv")

