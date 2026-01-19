# from my_lab.lib import *

import numpy as np
from scipy.spatial.transform import Rotation as R, Slerp
from geometry_msgs.msg import Transform

from .path import get_file_paths
from .utils import invert_transform, load_pose_sequence, get_peg_trajectory_in_hole_frame, \
    transform2homo, homo2transform, sample_vis_indices, readpkl

class PegInHoleDemo:
    def __init__(self, bag_name, hole_part_name, peg_part_name, hole_pose_env_homo=np.eye(4), start_idx=0, approach_stride=1, use_refined=False):
        self.start_idx = start_idx
        self.approach_stride = approach_stride
        self.retarget_poses_hole_homo = []
        self.retarget_poses_env_homo = []
        self.sampled_demo_idxs = []

        self.PATH = get_file_paths(bag_name, hole_part_name, peg_part_name)
        self.hole_traj_folder = self.PATH['HOLE_DEMO_FOLDER']
        self.peg_traj_folder  = self.PATH['PEG_DEMO_FOLDER']

        # self.hole_pose_env_transform = hole_pose_env_transform
        self.hole_pose_env_homo = hole_pose_env_homo # transform2homo(self.hole_pose_env_transform) # (4, 4)
        self.peg_poses_hole_homo = None
        self.peg_poses_env_homo = None
        self.length = 0

        self.load_demo_trajectory(use_refined)

    def load_peg_poses_hole_homo(self, use_refined):
        if use_refined: 
            peg_poses_hole_homo = load_pose_sequence(self.PATH['REFINED_DEMO_FOLDER'])
        else:
            hole_poses_cam_homo = load_pose_sequence(self.hole_traj_folder) 
            peg_poses_cam_homo  = load_pose_sequence(self.peg_traj_folder)          
            peg_poses_hole_homo = get_peg_trajectory_in_hole_frame(peg_poses_cam_homo, hole_poses_cam_homo)
        return np.stack(peg_poses_hole_homo)

    def load_demo_trajectory(self, use_refined):
        # Get peg poses in hole frame
        self.peg_poses_hole_homo = self.load_peg_poses_hole_homo(use_refined) # (N, 4, 4)

        # Get peg poses in env frame
        self.peg_poses_env_homo = self.hole_pose_env_homo @ self.peg_poses_hole_homo

        self.length = len(self.peg_poses_hole_homo)
        self.final_state_idx = self.length - 1
        if use_refined:
            self.key_state_idx = readpkl(self.PATH['KEY_STATE_IDX_PKL'])[self.PATH['BAG_NAME']]
        else:
            K = 170
            self.key_state_idx = self.length - K

    def retarget_traj_position_to_new_t(self, traj, new_t, retarget_start=True, z_sensitivity=0.5):
        """
        traj: (N, 4, 4) numpy array
        new_t: (3,) [x, y, z] target
        z_sensitivity: 오프셋 크기에 따른 가중치 변화 민감도
        """
        N = len(traj)
        retargeted_traj = traj.copy()
        
        # 1. 오프셋 계산
        current_t = traj[0, :3, 3] if retarget_start else traj[-1, :3, 3]
        offset = new_t - current_t
        
        # 2. 기본 선형 가중치 (1.0 to 0.0)
        w = np.linspace(1, 0, N) if retarget_start else np.linspace(0, 1, N)
        
        # 3. Z축 전용 가중치 계산
        # z_offset 크기에 비례하여 가중치 지수(exponent) 결정
        # 오프셋이 클수록 가중치가 더 강하게(quadratic 방향으로) 작용하도록 설정
        z_offset_mag = abs(offset[2])
        
        # exponent가 1이면 선형, 1보다 크면 곡선(w^k)
        # 예: z_offset이 0.2m이고 sensitivity가 5라면 exponent는 2가 됨
        exponent = 1.0 + (z_offset_mag * z_sensitivity)
        w_z = w ** exponent
        
        # 4. 적용
        retargeted_traj[:, 0, 3] += w * offset[0] # X
        retargeted_traj[:, 1, 3] += w * offset[1] # Y
        retargeted_traj[:, 2, 3] += w * offset[2] # Z (Adaptive)
        
        return retargeted_traj

    def retarget_traj_orientation_to_new_R(self, traj, new_R, retarget_start=True):
        """
        traj: (N, 4, 4) numpy array
        new_R: (3, 3) target rotation matrix
        """
        N = len(traj)
        retargeted_traj = traj.copy()
        
        # 1. 현재 회전과 목표 회전 추출
        curr_R_mat = traj[0, :3, :3] if retarget_start else traj[-1, :3, :3]
        
        # 2. 회전 오프셋(Delta Rotation) 계산: R_delta * R_curr = R_new
        # R_delta = R_new * R_curr^T
        delta_R = R.from_matrix(new_R) * R.from_matrix(curr_R_mat).inv()
        
        # 3. Identity(변화 없음)에서 delta_R까지의 Slerp 생성
        key_times = [0, 1]
        key_rots = R.from_quat([[0, 0, 0, 1], delta_R.as_quat()]) # [Identity, Delta]
        slerp = Slerp(key_times, key_rots)
        
        # 4. 가중치에 따른 회전 적용
        # retarget_start=True이면 t=1(start)에서 t=0(end)으로 감쇄
        weights = np.linspace(1, 0, N) if retarget_start else np.linspace(0, 1, N)
        interpolated_deltas = slerp(weights)
        
        for i in range(N):
            # 기존 회전에 보간된 델타 회전 적용
            current_frame_R = R.from_matrix(traj[i, :3, :3])
            new_frame_R = interpolated_deltas[i] * current_frame_R
            retargeted_traj[i, :3, :3] = new_frame_R.as_matrix()
            
        return retargeted_traj

    def represent_trajectory_wrt_frame(self, traj_base_homo, frame_base_homo):
        N = traj_base_homo.shape[0]
        assert traj_base_homo.shape == (N, 4, 4)  # N: number of poses
        assert frame_base_homo.shape == (4, 4)
        
        frame_to_base_homo = invert_transform(frame_base_homo)
        traj_frame_homo = np.zeros_like(traj_base_homo)

        for i in range(len(traj_base_homo)):
            traj_frame_homo[i] = frame_to_base_homo @ traj_base_homo[i]
        return traj_frame_homo

    def retarget_trajectory_to_new_pose(self, traj_base_homo, new_pose_base_homo, retarget_start=True):
        """
        Pose trajectory를 새로운 start or end point를 새로운 pose로 retargeting
        * 기준 프레임이 동일해야 함
        """
        # 1. 새로운 시작점의 위치(point)와 회전(R) 추출
        new_t = new_pose_base_homo[:3, 3].copy()
        new_R = new_pose_base_homo[:3, :3].copy()

        # 2. 전체 궤적의 위치를 새로운 시작점으로 이동 (Position Retargeting)
        # 기존 데모 궤적(self.peg_poses_env_homo)의 시작점을 new_point에 맞춤
        traj_pos_retargeted = self.retarget_traj_position_to_new_t(
            traj_base_homo.copy(), 
            new_t, 
            retarget_start=retarget_start
        )

        # 3. Approach 구간의 회전을 새로운 시작 자세에 맞춤 (Orientation Retargeting)
        # key_state_idx 이전까지만 현재 Peg의 회전값을 반영하여 정렬
        traj_ori_retargeted = self.retarget_traj_orientation_to_new_R(
            traj_pos_retargeted.copy(), 
            new_R, 
            retarget_start=retarget_start
        )

        # 4. 구간별 데이터 병합
        new_traj_base_homo = traj_pos_retargeted.copy()
        new_traj_base_homo[:self.key_state_idx] = traj_ori_retargeted.copy()

        return new_traj_base_homo

    # def generate_retargeted_trajectory(self, new_x, new_y, new_yaw):
    #     '''
    #     new_x, new_y, new_yaw -> wrt hole frame
    #     '''
    #     self.retarget_trajectory_to_new_start(new_x, new_y, new_yaw, target_hole_frame=self.hole_pose_env_homo)

    # def sample_demo_idxs_for_visualization(self, traj_homo, num_samples, ):
    #     return sorted()

    # def sample_demo_idxs_for_visualization(self, num_samples):
    #     for retargeted_traj in self.retarget_poses_env_homo:
    #         self.sampled_demo_idxs.append(sorted(sample_vis_indices(retargeted_traj, num_samples, sample_axis='y')))