import os
import numpy as np
import pickle
import yaml
from natsort import natsorted
from scipy.spatial.transform import Rotation as R, Slerp
from geometry_msgs.msg import Transform, Pose

def invert_transform(T):
    R = T[:3, :3]
    t = T[:3, 3]
    R_inv = R.T
    t_inv = -R_inv @ t

    T_inv = np.eye(4)
    T_inv[:3, :3] = R_inv
    T_inv[:3, 3] = t_inv
    return T_inv

def load_pose_sequence(folder):
    files = natsorted([f for f in os.listdir(folder) if f.endswith(".txt")])
    poses = []
    for f in files:
        mat = np.loadtxt(os.path.join(folder, f)).reshape(4, 4)
        poses.append(mat)
    return np.stack(poses)  # (N, 4, 4)

def get_peg_trajectory_in_hole_frame(peg_poses_cam, hole_poses_cam):
    # 평균 hole pose 계산
    hole_pose_cam_avg = average_pose(hole_poses_cam)
    cma_pose_hole_avg = np.linalg.inv(hole_pose_cam_avg) 

    # peg의 hole 기준 상대 pose 계산
    peg_poses_hole = []
    for peg_pose in peg_poses_cam:
        # hole_pose_inv @ peg_pose
        peg_poses_hole.append(cma_pose_hole_avg @ peg_pose)
    peg_poses_hole = np.stack(peg_poses_hole)  # (N, 4, 4)
    peg_poses_hole = moving_average_se3(peg_poses_hole, 10)  # (N, 4, 4)
    return peg_poses_hole

def pose2homo(pose):
    """
    geometry_msgs.Pose -> 4x4 homogeneous matrix
    (Pose는 position, orientation 필드를 사용함)
    """
    from scipy.spatial.transform import Rotation as R
    import numpy as np
    
    homo = np.eye(4)
    # Pose는 'position' 필드를 사용
    homo[0, 3] = pose.position.x
    homo[1, 3] = pose.position.y
    homo[2, 3] = pose.position.z
    
    # Pose는 'orientation' 필드를 사용
    quat = [
        pose.orientation.x, 
        pose.orientation.y, 
        pose.orientation.z, 
        pose.orientation.w
    ]
    
    # Scipy 버전 호환성 처리 (이전에 구현한 방식)
    rot_obj = R.from_quat(quat)
    if hasattr(rot_obj, 'as_matrix'):
        homo[:3, :3] = rot_obj.as_matrix()
    else:
        homo[:3, :3] = rot_obj.as_dcm()
        
    return homo

def homo2pose(homo: np.ndarray):
    """
    4x4 homogeneous matrix -> geometry_msgs.Pose
    """
    pose = Pose()
    
    # 1. Position (위치) 추출
    pose.position.x = homo[0, 3]
    pose.position.y = homo[1, 3]
    pose.position.z = homo[2, 3]
    
    # 2. Rotation Matrix -> Quaternion (회전) 변환
    # Scipy의 as_quat()은 [x, y, z, w] 순서로 반환하여 ROS와 호환됨
    rot_matrix = homo[:3, :3]
    quat = R.from_matrix(rot_matrix).as_quat()
    
    pose.orientation.x = quat[0]
    pose.orientation.y = quat[1]
    pose.orientation.z = quat[2]
    pose.orientation.w = quat[3]
    
    return pose

def transform2homo(transform: Transform):
    '''
    geometry_msgs.Transform -> 4x4 homogeneous matrix
    '''
    # 1. 4x4 단위 행렬 생성
    homo = np.eye(4)
    
    # 2. Translation (위치) 데이터 채우기
    homo[0, 3] = transform.translation.x
    homo[1, 3] = transform.translation.y
    homo[2, 3] = transform.translation.z
    
    # 3. Rotation (쿼터니언) 데이터 채우기 [x, y, z, w]
    quat = [
        transform.rotation.x,
        transform.rotation.y,
        transform.rotation.z,
        transform.rotation.w
    ]
    
    # 4. Scipy를 이용해 쿼터니언을 3x3 회전 행렬로 변환하여 삽입
    homo[:3, :3] = R.from_quat(quat).as_matrix()
    
    return homo

def homo2transform(homo: np.ndarray):
    '''
    4x4 homogeneous matrix -> geometry_msgs.Transform
    '''
    transform = Transform()
    
    # 1. Translation 데이터 추출 및 입력
    transform.translation.x = homo[0, 3]
    transform.translation.y = homo[1, 3]
    transform.translation.z = homo[2, 3]
    
    # 2. 3x3 회전 행렬에서 쿼터니언 추출
    # Scipy의 as_quat()은 [x, y, z, w] 순서의 numpy array를 반환합니다.
    quat = R.from_matrix(homo[:3, :3]).as_quat()
    
    # 3. Rotation 데이터 입력
    transform.rotation.x = quat[0]
    transform.rotation.y = quat[1]
    transform.rotation.z = quat[2]
    transform.rotation.w = quat[3]
    
    return transform

def sample_vis_indices(traj, frame_vis_gap, sample_axis=''):
    pos_traj = traj[:, :3, 3]  # (N, 3)
    if frame_vis_gap != -1 and frame_vis_gap > 0:
        # --- 일정 X 또는 Y 간격마다 peg 좌표계 표시 ---
        # x, y 중 더 넓은 구간 판단
        x_vals = pos_traj[:, 0]
        y_vals = pos_traj[:, 1]
        x_min, x_max = x_vals.min(), x_vals.max()
        y_min, y_max = y_vals.min(), y_vals.max()
        if (x_max - x_min) > (y_max - y_min) or (sample_axis == 'x'):
            # X 간격마다 표시
            interval = (x_max - x_min) / frame_vis_gap
            _ticks = np.arange(x_min, x_max + interval, interval)
            _vals = x_vals
        elif (y_max - y_min) >= (x_max - x_min) or (sample_axis == 'y'):
            interval = (y_max-y_min) / frame_vis_gap
            _ticks = np.arange(y_min, y_max + interval, interval)
            _vals = y_vals

        indices = []
        for _t in _ticks:
            idx = np.argmin(np.abs(_vals - _t))
            if idx not in indices:
                indices.append(idx)
        if 0 not in indices: indices.insert(0, 0)
        if (len(traj)-1) not in indices: indices.append(len(traj)-1)
    return sorted(indices)

def readpkl(pkl_file):
    with open(pkl_file, 'rb') as f:
        data = pickle.load(f)
    return data

def load_yaml(file_path):
    with open(file_path, 'r') as f:
        data = yaml.safe_load(f)
    return data

def rotation_matrix_from_vectors(a, b):
    """
    Returns rotation matrix R (3x3) that rotates vector a to vector b.
    """
    a = a / np.linalg.norm(a)
    b = b / np.linalg.norm(b)

    cross = np.cross(a, b)
    dot = np.dot(a, b)

    # --- Case 1: A and B are almost same direction ---
    if np.isclose(dot, 1.0):
        return np.eye(3)

    # --- Case 2: A and B are opposite direction ---
    if np.isclose(dot, -1.0):
        # any orthogonal vector works
        orth = np.array([1, 0, 0])
        if np.allclose(a, orth):
            orth = np.array([0, 1, 0])
        v = np.cross(a, orth)
        v = v / np.linalg.norm(v)
        # 180-degree rotation
        return -np.eye(3) + 2 * np.outer(v, v)

    # --- General case ---
    v = cross
    s = np.linalg.norm(v)
    k_mat = np.array([[    0, -v[2],  v[1]],
                      [ v[2],     0, -v[0]],
                      [-v[1],  v[0],     0]])

    R = np.eye(3) + k_mat + k_mat @ k_mat * ((1 - dot) / (s**2))
    return R

def average_pose(poses):
    positions = poses[:, :3, 3]
    mean_pos = positions.mean(axis=0)

    Rs = poses[:, :3, :3]
    R_mean = np.zeros((3, 3))
    for R in Rs:
        R_mean += R
    U, _, Vt = np.linalg.svd(R_mean)
    R_avg = U @ Vt

    T_avg = np.eye(4)
    T_avg[:3, :3] = R_avg
    T_avg[:3, 3] = mean_pos
    return T_avg

def moving_average_se3(traj, window=5):
    """
    traj: (N, 4, 4) array of SE(3) poses
    window: half window size (i ± window)
    """
    N = len(traj)
    traj_smooth = np.zeros_like(traj)
    traj_smooth[:, 3, 3] = 1.0

    for i in range(N):
        # window 범위
        start = max(0, i - window)
        end = min(N, i + window + 1)
        sub_T = traj[start:end]

        # translation 평균
        mean_t = sub_T[:, :3, 3].mean(axis=0)

        # rotation 평균 (scipy >= 1.11)
        mean_R = R.from_matrix(sub_T[:, :3, :3]).mean().as_matrix()

        traj_smooth[i, :3, :3] = mean_R
        traj_smooth[i, :3, 3] = mean_t

    return traj_smooth