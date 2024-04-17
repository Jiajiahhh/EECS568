from filter import InEKF_imu as InEKF
from utils.system_initialization import system_initialization


import numpy as np
import re
import matplotlib.pyplot as plt
from scipy.linalg import expm
import matplotlib.pyplot as plt


def quaternion_to_yaw(quaternion):
    x, y, z, w = quaternion
    rotation_matrix = np.array([[1 - 2*(y**2 + z**2), 2*(x*y - z*w), 2*(x*z + y*w)],
                                 [2*(x*y + z*w), 1 - 2*(x**2 + z**2), 2*(y*z - x*w)],
                                 [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x**2 + y**2)]])

    yaw = np.arctan2(rotation_matrix[1,0], rotation_matrix[0,0])
    return yaw

def quaternion_to_rotation_matrix(quat):
    """
    quaternion to rotation matrix.
    """
    x, y, z, w = quat
    R = np.array([
        [1 - 2*y**2 - 2*z**2,     2*x*y - 2*z*w,       2*x*z + 2*y*w],
        [2*x*y + 2*z*w,       1 - 2*x**2 - 2*z**2,     2*y*z - 2*x*w],
        [2*x*z - 2*y*w,       2*y*z + 2*x*w,       1 - 2*x**2 - 2*y**2]
    ])
    return R

def remove_yaw_effect(point):
    x, y, yaw = point 
    theta = np.radians(yaw[0])
    R_inv = np.array([[np.cos(theta), np.sin(theta)],
                        [-np.sin(theta),  np.cos(theta)]])
    new_xy = R_inv[0] @ np.array([x, y]).reshape(-1,1)
    return new_xy

def wedge(phi):
    """
    R^3 vector to so(3) matrix
    @param  phi: R^3
    @return Phi: so(3) matrix
    """
    phi = phi.squeeze()
    Phi = np.array([[0, -phi[2], phi[1]],
                    [phi[2], 0, -phi[0]],
                    [-phi[1], phi[0], 0]])
    return Phi

def parse_true_line_imu(line):

    try:
        parts = [float(part) for part in line.split()]

        coord = parts[:2]  # x, y
        orientation = parts[2:6]  # orientation (x, y, z, w)
        angular_velocity = parts[6:9]  # angular_velocity (x, y, z)
        linear_acceleration = parts[9:]  # linear_acceleration (x, y, z)
        
        return coord, orientation, angular_velocity, linear_acceleration
    except ValueError as e:
        print(f"data error: {e}")
        return None

def read_true_imu(filename):

    coords, orientations, angular_velocities, linear_accelerations = [], [], [], []

    with open(filename, 'r') as file:
        for line in file:
            parsed = parse_true_line_imu(line) 
            if parsed:
                coord, orientation, angular_velocity, linear_acceleration = parsed
                coords.append(coord)
                orientations.append(orientation)
                angular_velocities.append(angular_velocity)
                linear_accelerations.append(linear_acceleration)
    
    return (np.array(coords), np.array(orientations), np.array(angular_velocities), np.array(linear_accelerations))

def parse_data_line2(line):

    x = y = yaw = dt = None
    points1 = []
    points2 = []
    
    pattern_main = re.compile(r"x=\s*([+-]?\d+\.?\d*e?[+-]?\d*)\s*y=\s*([+-]?\d+\.?\d*e?[+-]?\d*)\s*yaw=\s*([+-]?\d+\.?\d*e?[+-]?\d*)\s*dt=\s*([+-]?\d+\.?\d*e?[+-]?\d*)")
    main_match = pattern_main.search(line)
    if main_match:
        x, y, yaw, dt = map(float, main_match.groups())
    else:
        return None
    
    pattern_points = re.compile(r"\[([+-]?\d+\.?\d*e?[+-]?\d*),([+-]?\d+\.?\d*e?[+-]?\d*)\|([+-]?\d+\.?\d*e?[+-]?\d*),([+-]?\d+\.?\d*e?[+-]?\d*)\]")
    matches = pattern_points.findall(line)
    for match in matches:
        points1.append((float(match[0]), float(match[1])))
        points2.append((float(match[2]), float(match[3])))
    
    return x, y, yaw, dt, points1, points2

def read_data2(filename):
    main_data = [] 
    points_data1 = [] 
    points_data2 = [] 

    with open(filename, 'r') as file:
        for line in file:
            parsed = parse_data_line2(line)
            if parsed:
                x, y, yaw, dt, points1, points2 = parsed
                main_data.append([x, y, yaw, dt])
                points_data1.append(points1)
                points_data2.append(points2)
    
    return np.array(main_data), points_data1, points_data2

def calculate_yaw(R): # R to yaw
    yaw = np.arctan2(R[1, 0], R[0, 0])
    return yaw

def main():

    
    sys = system_initialization(alphas=None, beta=None)  
    inekf = InEKF.InEKF(sys)  

    filetrue = 'D:\\Desktop\\568Project\\IMU_Correction\\data\\true1.txt'
    filepath2 = 'D:\\Desktop\\568Project\\IMU_Correction\\data\\output1.txt'

    data, landmarks, points  = read_data2(filepath2) #data provides vo path and dt
    ov_path = []
    ov_path = np.copy(data)
    
    points_true, orientations, angular_velocities, linear_accelerationsread = read_true_imu(filetrue)
    
    for i in range (len(data)): # Correction initial position:
        ov_path[i][0] -= 3
        ov_path[i][1] += 1
    for i in range(len(data)): # Eliminate the impact of rotation on trajectory in VO
        data[i][0] -= 3
        data[i][1] += 1
        theat = np.radians(data[i][3])
        n = np.array([[data[i][0]],[data[i][1]],[data[i][2]]])
        data[i][0] = data[i][0] * np.cos(theat)
        data[i][1] = 1

    X = np.array([data[0][0], data[0][1], np.radians(data[0][2])])  

    estimated_states = [X.flatten().tolist()] #InEKF path
    true_states = [np.array([-3, 1])] #Rrue path
    estimated_states_world = [np.array([-data[0][0], -data[0][1]])]
    errorx_points = []
    errory_points = []

    errorx = []
    errory = []

    for i in range(len(data)): 

        R = quaternion_to_rotation_matrix(orientations[0]) # Initialize rotation matrix

        X_pred, P_pred = inekf.prediction(data, angular_velocities[i], i, R)
        Y, P = inekf.correction(data, X_pred, P_pred, linear_accelerationsread[i], i)
        
        estimated_states.append(Y.flatten().tolist())
        true_states.append(ov_path[i][:2])
        estimated_states_world.append([-X.flatten()[0], -X.flatten()[1]])

        errorx.append(points_true[i][0] - X[0])
        errory.append(points_true[i][1] - X[1])
        errorx_points.append(points_true[i][0] - data[i][0])
        errory_points.append(points_true[i][1] - data[i][0])

    estimated_states = np.array(estimated_states)
    true_states = np.array(true_states)
    estimated_states_world = np.array(estimated_states_world)
    true_state = np.array(points_true)

    plt.plot(true_states[:, 0], true_states[:, 1], 'r-',label='VO Path')
    plt.plot(estimated_states[:, 0] , estimated_states[:, 1], 'g-',label='InEKF Estimated Path')
    plt.plot(true_state[:, 0], true_state[:, 1], 'b-', label='True')
    plt.axis('equal')
    plt.xlabel('X position')
    plt.ylabel('Y position')
    plt.legend()
    plt.show()

    # plt.figure(figsize=(10, 6))
    # plt.plot(estimated_states[:, 0], estimated_states[:, 1], 'b--', label='EKF Estimated Path')
    # plt.xlabel('Time Step')
    # plt.ylabel('Errorx')
    # plt.title('Error Over Time')
    # plt.legend()
    # plt.show()

    # plt.figure(figsize=(10, 6))
    # plt.plot(true_state[:, 0], true_state[:, 1], 'b--', label='True')
    # plt.plot(estimated_states_world[:, 0], estimated_states_world[:, 1], 'b--', label='EKF Estimated Path World')
    # plt.xlabel('Time Step')
    # plt.ylabel('estimated_states_world')
    # plt.title('EKF Estimated Path in world')
    # plt.legend()
    # plt.show()

    # Plotting the variance of positional errors
    # plt.figure(figsize=(10, 6))
    # plt.plot(range(len(errorx)), errorx, 'r-', label='errorx')
    # plt.plot(range(len(errorx)), errorx_points, 'g-', label='errorx_points')
    # plt.xlabel('Time Step')
    # plt.ylabel('Errorx')
    # plt.title('Error Over Time')
    # plt.legend()
    # plt.show()

    # # Plotting the variance of positional errors
    # plt.figure(figsize=(10, 6))
    # # plt.plot(range(len(errory)), errory, 'r-', label='errory')
    # plt.plot(errory[:], 'r-',label='errory')
    # plt.plot(range(len(errory_points)), errory_points, 'g-', label='errory_points')
    # plt.xlabel('Time Step')
    # plt.ylabel('Errory')
    # plt.title('Error Over Time')
    # plt.legend()
    # plt.show()

    # init_poses = np.array([[p.x, p.y, p.theta] for p in poses])
    # opt_poses = pg.isam_2d(poses, edges)
    # print(dir(gtsam))
    # print(hasattr(gtsam, 'ISAM2'))

    # # plt.plot(init_poses[:,0], init_poses[:,1])
    # plt.plot(opt_poses[:,0], opt_poses[:,1])
    # plt.title('isam2 2D')
    # plt.legend(['Unoptimized Trajectory', 'Optimized Trajectory'])
    # plt.axis('equal')
    # plt.show()


if __name__ == '__main__':
    main()