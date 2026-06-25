#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import os
import copy
import argparse
import rospy
import sys
import select
import termios
import tty
import actionlib
import moveit_commander  # 추가
import tf2_ros          # 추가
import tf2_geometry_msgs  # 이 줄이 있어야 PoseStamped 변환이 지원됩니다.
from std_msgs.msg import String, Bool 
from controller_manager_msgs.srv import SwitchController, SwitchControllerRequest, ListControllers
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal, FollowJointTrajectoryResult
from geometry_msgs.msg import PoseStamped, TransformStamped # 추가
import tf.transformations as tft

from cv_bridge import CvBridge
from sensor_msgs.msg import Image, Joy
import numpy as np
import cv2

from scipy.spatial.transform import Rotation as R

MOVEIT = 'a'
CARTESIAN_IMPEDANCE = 's'
CARTESIAN_POSE = 'd'
TELEOP = 't'
TELEOP_IMPEDANCE = 'i'
SELECT_HOLE = 'h'
SELECT_PEG = 'p'
OPEN_GRIPPER = 'o'
CLOSE_GRIPPER = 'c'
GRASPING = 'g'
REGISTER_FRAMES = 'f'
SELECT_HOLE_POSE = 'j'

SET_VIEWPOINT = '0'
MOVE_TO_HOME = '1'
MOVE_TO_VIEWPOINT = '2'
ZOOM_TO_OBJECT = '3'
MOVE_TO_PRE_GRASP = '4'
MOVE_TO_GRASP = '5'
GENERATE_TRAJECTORY = '6'
APPROACH = '7'
INSERTION = '8'

PEG_LIST = [9, 11, 12, 17]
PEG_NAME_EXTRA = ["part11-2"]

MODE_SPECS = [
    (MOVEIT, 'MOVEIT', 'MoveIt Mode', True),
    (CARTESIAN_IMPEDANCE, 'CARTESIAN_IMPEDANCE', 'Cartesian Impedance Mode', True),
    (CARTESIAN_POSE, 'CARTESIAN_POSE', 'Cartesian Pose Mode', True),
    (TELEOP, 'TELEOP', 'Teleop Mode', True),
    (TELEOP_IMPEDANCE, 'TELEOP_IMPEDANCE', 'Teleop Impedance Mode', True),
    (SELECT_HOLE, 'SELECT_HOLE', 'Select Hole as Target Object', True),
    (SELECT_PEG, 'SELECT_PEG', 'Select Peg as Target Object', True),
    (OPEN_GRIPPER, 'OPEN_GRIPPER', 'Open Gripper', False),
    (CLOSE_GRIPPER, 'CLOSE_GRIPPER', 'Close Gripper', False),
    (GRASPING, 'GRASPING', 'Grasping', False),
    (REGISTER_FRAMES, 'REGISTER_FRAMES', 'Register Frames', True),
    (SELECT_HOLE_POSE, 'SELECT_HOLE_POSE', 'Select Hole Pose (current_x.json)', True),
    (SET_VIEWPOINT, 'SET_VIEWPOINT', 'Set Viewpoint Pose', True),
    (MOVE_TO_HOME, 'MOVE_TO_HOME', 'Move to Home', True),
    (MOVE_TO_VIEWPOINT, 'MOVE_TO_VIEWPOINT', 'Move to Viewpoint', True),
    (ZOOM_TO_OBJECT, 'ZOOM_TO_OBJECT', 'Zoom to Object', True),
    (MOVE_TO_PRE_GRASP, 'MOVE_TO_PRE_GRASP', 'Move to Pre-Grasp', True),
    (MOVE_TO_GRASP, 'MOVE_TO_GRASP', 'Move to Grasp', True),
    (GENERATE_TRAJECTORY, 'GENERATE_TRAJECTORY', 'Generate Trajectory', True),
    (APPROACH, 'APPROACH', 'Approach', True),
    (INSERTION, 'INSERTION', 'Insertion', True),
]

MODE_DICT = {key: name for key, name, _, _ in MODE_SPECS}
MODE_GUIDE_ITEMS = [(key, guide) for key, _, guide, show_in_guide in MODE_SPECS if show_in_guide]



grasp_pose_dict = { # wrt peg object frame
    "part7": { # wrt object_part7
        # - Translation: [-0.026, -0.037, 0.000]
        # - Rotation: in Quaternion [1.000, -0.001, 0.021, 0.008]
        'position': {
            'x': -0.026,
            'y': -0.037,
            'z': 0.000
        },
        'orientation': {
            'x': 1.000,
            'y': -0.001,
            'z': 0.021,
            'w': 0.008
        }
    },
    "part9": { # wrt object_part9
        # - Translation: [-0.003, -0.011, 0.015]
        # - Rotation: in Quaternion [1.000, 0.007, 0.030, 0.003]
        'position': {
            'x': -0.003,
            'y': -0.011,
            'z': 0.015
        },
        'orientation': {
            'x': 1.000,
            'y': 0.007,
            'z': 0.030,
            'w': 0.003
        }
    },
    "part11": { # wrt object_part11
        'position': {
            'x': 0.006,
            'y': -0.002,
            'z': 0.008
        },
        'orientation': {
            'x': 1.000,
            'y': -0.004,
            'z': 0.003,
            'w': -0.010
        }
    },
    "part12": { # wrt object_part12
        # - Translation: [-0.005, -0.001, 0.017]
        # - Rotation: in Quaternion [-0.704, 0.710, 0.010, -0.000]
        'position': {
            'x': -0.005,
            'y': -0.001,
            'z': 0.017
        },
        'orientation': {
            'x': -0.704,
            'y': 0.710,
            'z': 0.010,
            'w': -0.000
        }
    }
}

dummy_camera_pose_dict = {
    "position": {
        "x": 0.05,
        "y": 0.0,
        "z": 0.05
    },
    "orientation": {
        "x": 0.0,
        "y": 0.0,
        "z": 0.0,
        "w": 1.0
    }
}

measured_goal_tcp_pose_dict = { # wrt panda_link0
    "position": {
        "x": 0.606,
        "y": -0.078,
        "z": 0.043
    },
    "orientation": {
        "x": 0.940,
        "y": -0.341,
        "z": -0.007,
        "w": -0.015
    }
}

pre_goal_tcp_pose_dict = { # wrt base frame (panda_link0)
    "part17": {
        "position": {
            "x": 0.494,
            "y": 0.033,
            "z": 0.070
        },
        "orientation": {
            "x": 0.945,
            "y": -0.327,
            "z": -0.024,
            "w": 0.009
        }
    }
}

goal_pose_path = {
    "part9": "/root/share/catkin_ws/src/demo_traj/data/refined_final_peg_pose/20260201_214303_seq1_part9_part9_refined_final_pose.txt",
    "part11": "/root/share/catkin_ws/src/demo_traj/data/refined_final_peg_pose/test4_20251030_151337_part11_refined_final_pose.txt",
    "part11-2": "/root/share/catkin_ws/src/demo_traj/data/refined_final_peg_pose/20260201_214449_seq3_part11_2_part11_refined_final_pose.txt",
    "part17": "/root/share/catkin_ws/src/demo_traj/data/refined_final_peg_pose/20260201_214643_seq6_part17_part17_refined_final_pose.txt",
    # "part11": "/root/share/catkin_ws/src/franka_interface/scripts/fixed_pose/part11-2/grasp_tcp.json"
    # "part11": "/root/share/catkin_ws/src/franka_interface/scripts/fixed_pose/part11/grasp_tcp.json"
}

POSE_FALLBACK_PEG = {
    # "part11-2": "part11",
    # "part12": "part11",
}

PRE_GRASP_OFFSET_Z = 0.07  # 7cm
PRE_GOAL_OFFSET_Z = 0.03  # 5cm
PRE_GOAL_OFFSET_Z_BY_PART = {
    "part11": 0.03,
    "part11-2": 0.04,
    "part9": 0.04,
    "part17": 0.03,
}
GRIPPER_MAX_WIDTH = 0.0396 * 2

# Pre-goal position noise configuration
# mode: 'none', 'none', 'xy', 'z', 'custom'
PRE_GOAL_NOISE_MODE = 'xy'
PRE_GOAL_NOISE_RANGE_M = {
    'x': (-0.0025, 0.0025),
    'y': (-0.0025, 0.0025),
    'z': (-0.0025, 0.0025),
}
# PRE_GOAL_NOISE_RANGE_M = {
#     'x': (-0.005, 0.005),
#     'y': (-0.005, 0.005),
#     'z': (-0.005, 0.005),
# }
PRE_GOAL_NOISE_STEP_M = 0.0002

# Pre-goal orientation noise configuration (RPY in degrees)
# mode: 'none', 'rpy', 'roll', 'pitch', 'yaw', 'custom'
PRE_GOAL_ROT_NOISE_MODE = 'rpy'
PRE_GOAL_RPY_NOISE_RANGE_DEG = {
    'r': (-2, 2),
    'p': (-2, 2),
    'y': (-2, 2),
}
PRE_GOAL_ROT_NOISE_STEP_DEG = 0.1

PRE_GOAL_TARGET_FROM_VIDEO = 'from_video'
PRE_GOAL_TARGET_FROM_MANUAL = 'from_manual'
ARM_JOINT_STATES_TOPIC = '/franka_state_controller/joint_states'
VIEWPOINT_DIR = 'viewpoint'
CURRENT_POSE_FILENAME = 'current.json'
CAMERA_FRAME = 'camera_link'
PANDA_HAND_FRAME = 'panda_hand'

def get_transformed_pose(base_pose, relative_pose):
    """
    base_pose를 기준으로 relative_pose만큼 떨어진 위치와 회전을 계산합니다.
    """
    # 1. 위치 및 회전 데이터 추출 (리스트 형태)
    p_base = [base_pose['position']['x'], base_pose['position']['y'], base_pose['position']['z']]
    q_base = [base_pose['orientation']['x'], base_pose['orientation']['y'], 
              base_pose['orientation']['z'], base_pose['orientation']['w']]
    
    p_rel = [relative_pose['position']['x'], relative_pose['position']['y'], relative_pose['position']['z']]
    q_rel = [relative_pose['orientation']['x'], relative_pose['orientation']['y'], 
             relative_pose['orientation']['z'], relative_pose['orientation']['w']]

    # 2. Scipy Rotation 객체 생성
    rot_base = R.from_quat(q_base)
    rot_rel = R.from_quat(q_rel)

    # 3. 신규 위치 계산: Base 회전만큼 상대 위치를 회전시킨 후 합산
    # P_new = P_base + (R_base * P_rel)
    p_new = p_base + rot_base.apply(p_rel)

    # 4. 신규 회전 계산: 두 쿼터니언 곱셈 (순서: Base * Relative)
    # Q_new = Q_base * Q_rel
    q_new = (rot_base * rot_rel).as_quat()

    # 5. 딕셔너리 형태로 반환
    return {
        "position": {"x": p_new[0], "y": p_new[1], "z": p_new[2]},
        "orientation": {"x": q_new[0], "y": q_new[1], "z": q_new[2], "w": q_new[3]}
    }

class ControllerSwitcher:
    def __init__(self, selected_object=None, tf_live_update='off', pre_goal_target=PRE_GOAL_TARGET_FROM_VIDEO):
        rospy.init_node('controller_switcher_node')
        self.path = os.path.dirname(__file__)

        self._init_frame_paths()
        self._init_selection(selected_object, tf_live_update, pre_goal_target)
        self._init_target_area_ratio()
        self._init_noise_config()
        self._init_ros_interfaces()
        self._init_motion_interfaces()
        self._init_controllers()
        self._init_grasp_state()
        self._init_home_pose()
        self._init_fixed_pose_state()
        self._init_runtime_state()

        initial_mode = MOVEIT
        self.mode = initial_mode
        self.mode_pub.publish(initial_mode)

        self.print_guide(initial_mode)

    def _init_frame_paths(self):

        self.base_frame = "panda_link0"
        self.tcp_frame = "panda_hand_tcp"
        self.fixed_pose_root = os.path.join(self.path, "fixed_pose")
        self.hole_name = "part1"
        self.hole_frame = "object_" + self.hole_name
        self.hole_pose_file = os.path.join(self.fixed_pose_root, self.hole_name, CURRENT_POSE_FILENAME)
        self.viewpoint_file = os.path.join(self.fixed_pose_root, VIEWPOINT_DIR, CURRENT_POSE_FILENAME)

    def _init_selection(self, selected_object, tf_live_update, pre_goal_target):
        self.selected_object = self.resolve_initial_selected_object(selected_object)
        self.update_selected_peg(self.selected_object)
        self.tf_live_update_enabled = str(tf_live_update).strip().lower() == 'on'
        rospy.loginfo(f"TF live update: {'ON' if self.tf_live_update_enabled else 'OFF'}")
        self.pre_goal_target = str(pre_goal_target).strip().lower()
        if self.pre_goal_target not in [PRE_GOAL_TARGET_FROM_VIDEO, PRE_GOAL_TARGET_FROM_MANUAL]:
            rospy.logwarn(
                f"Invalid pre_goal_target '{pre_goal_target}'. "
                f"Fallback to '{PRE_GOAL_TARGET_FROM_VIDEO}'."
            )
            self.pre_goal_target = PRE_GOAL_TARGET_FROM_VIDEO
        rospy.loginfo(f"Pre-goal target source: {self.pre_goal_target}")

    def _init_target_area_ratio(self):
        self.target_area_ratio = {
            'part1': 0.18,
            'part7': 0.05,
            'part9': 0.05,
            'part11': 0.03,
            'part11-2': 0.03,
            'part12': 0.03,
            'part17': 0.01,
        }

    def _init_noise_config(self):
        self.pre_goal_noise_mode = str(rospy.get_param('~pre_goal_noise_mode', PRE_GOAL_NOISE_MODE)).lower()
        self.pre_goal_noise_range_m = {
            'x': (
                self._param_float('~pre_goal_noise_x_min_m', PRE_GOAL_NOISE_RANGE_M['x'][0]),
                self._param_float('~pre_goal_noise_x_max_m', PRE_GOAL_NOISE_RANGE_M['x'][1])
            ),
            'y': (
                self._param_float('~pre_goal_noise_y_min_m', PRE_GOAL_NOISE_RANGE_M['y'][0]),
                self._param_float('~pre_goal_noise_y_max_m', PRE_GOAL_NOISE_RANGE_M['y'][1])
            ),
            'z': (
                self._param_float('~pre_goal_noise_z_min_m', PRE_GOAL_NOISE_RANGE_M['z'][0]),
                self._param_float('~pre_goal_noise_z_max_m', PRE_GOAL_NOISE_RANGE_M['z'][1])
            ),
        }
        self.pre_goal_noise_step_m = max(
            0.0,
            self._param_float('~pre_goal_noise_step_m', PRE_GOAL_NOISE_STEP_M)
        )
        self.pre_goal_rot_noise_mode = str(rospy.get_param('~pre_goal_rot_noise_mode', PRE_GOAL_ROT_NOISE_MODE)).lower()
        self.pre_goal_rpy_noise_range_deg = {
            'r': (
                self._param_float('~pre_goal_noise_roll_min_deg', PRE_GOAL_RPY_NOISE_RANGE_DEG['r'][0]),
                self._param_float('~pre_goal_noise_roll_max_deg', PRE_GOAL_RPY_NOISE_RANGE_DEG['r'][1])
            ),
            'p': (
                self._param_float('~pre_goal_noise_pitch_min_deg', PRE_GOAL_RPY_NOISE_RANGE_DEG['p'][0]),
                self._param_float('~pre_goal_noise_pitch_max_deg', PRE_GOAL_RPY_NOISE_RANGE_DEG['p'][1])
            ),
            'y': (
                self._param_float('~pre_goal_noise_yaw_min_deg', PRE_GOAL_RPY_NOISE_RANGE_DEG['y'][0]),
                self._param_float('~pre_goal_noise_yaw_max_deg', PRE_GOAL_RPY_NOISE_RANGE_DEG['y'][1])
            ),
        }
        self.pre_goal_rot_noise_step_deg = max(
            0.0,
            self._param_float('~pre_goal_rot_noise_step_deg', PRE_GOAL_ROT_NOISE_STEP_DEG)
        )

    def _init_ros_interfaces(self):
        self.switch_srv = rospy.ServiceProxy('/controller_manager/switch_controller', SwitchController)
        self.list_srv = rospy.ServiceProxy('/controller_manager/list_controllers', ListControllers)
        self.mode_sub = rospy.Subscriber('/set_controller_mode', String, self.mode_callback)
        self.gripper_joint_sub = rospy.Subscriber('/franka_gripper/joint_states', JointState, self.gripper_joint_callback)
        self.arm_joint_sub = rospy.Subscriber('/franka_state_controller/joint_states', JointState, self.arm_joint_callback)
        self.pose_sub = None
        self.pose_received = False

        self.mode_pub = rospy.Publisher('/current_mode', String, queue_size=1, latch=True)
        self.sam2_pub = rospy.Publisher('/sam2_target_object', String, queue_size=1, latch=False)
        self.fp_pub = rospy.Publisher('/fp_target_object', String, queue_size=1, latch=False)
        self.target_frame_pub = rospy.Publisher('/change_target_frame', String, queue_size=1, latch=True)
        self.joy_pub = rospy.Publisher('/spacenav/joy', Joy, queue_size=1)
        self.policy_reset_pub = rospy.Publisher('/policy/reset', Bool, queue_size=1)
        self.policy_enable_pub = rospy.Publisher('/policy/enable', Bool, queue_size=1)
        self.policy_status_sub = rospy.Subscriber('/policy/status', String, self.policy_status_callback)
        self.policy_status = 'UNKNOWN'
        self.insertion_active = False

    def _init_motion_interfaces(self):
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.arm = moveit_commander.MoveGroupCommander("panda_arm")
        self.arm.set_end_effector_link("panda_hand_tcp")

    def _init_controllers(self):
        self.pos_controller = "position_joint_trajectory_controller"
        self.imp_controller = "cartesian_impedance_example_controller"
        self.pose_controller = "cartesian_pose_controller"
        self.all_controllers = [self.pos_controller, self.imp_controller, self.pose_controller]

        self.trajectory_client = actionlib.SimpleActionClient(
            f'/{self.pos_controller}/follow_joint_trajectory', 
            FollowJointTrajectoryAction
        )

    def _init_grasp_state(self):
        self.is_gripper_closed = False
        self.is_grasped = False
        self.grasp_threshold = 0.02  # 2cm 이내일 때 잡은 것으로 간주
        self.latest_gripper_width = None
        self.last_gripper_state_stamp = rospy.Time(0)
        self.gripper_state_timeout = 0.5
        self.latest_arm_joint_positions = {}
        self.last_arm_joint_state_stamp = rospy.Time(0)

        self.grasp_status_pub = rospy.Publisher('/is_grasped', Bool, queue_size=1, latch=True)
        self.grasp_check_timer = rospy.Timer(rospy.Duration(0.1), self.check_grasp_status)

    def _init_home_pose(self):
        self.home_pose = {
            'panda_joint1': 0.0, 'panda_joint2': -0.785, 'panda_joint3': 0.0,
            'panda_joint4': -2.356, 'panda_joint5': 0.0, 'panda_joint6': 1.571, 'panda_joint7': 0.785
        }

    def _init_fixed_pose_state(self):
        self.goal_pose_path = goal_pose_path
        self.goal_pose_data = {}
        self.static_br = tf2_ros.StaticTransformBroadcaster()
        self.fixed_poses = {}
        self.load_and_publish_all_saved_poses()

    def _init_runtime_state(self):
        self.settings = termios.tcgetattr(sys.stdin)

    # ---------------------------
    # Selection And Configuration
    # ---------------------------

    def resolve_initial_selected_object(self, selected_object):
        candidate = selected_object
        if candidate is None:
            candidate = rospy.get_param('~selected_object', 'part11')

        candidate = str(candidate).strip()
        if not candidate:
            return 'part11'

        if candidate.isdigit():
            candidate = f"part{candidate}"
        elif candidate == '11-2':
            candidate = 'part11-2'

        is_valid = (
            candidate == self.hole_name or
            candidate in PEG_NAME_EXTRA or
            os.path.isdir(os.path.join(self.fixed_pose_root, candidate))
        )
        if not is_valid:
            rospy.logwarn(f"Invalid initial selected_object '{candidate}'. Fallback to 'part11'.")
            return 'part11'

        return candidate

    def update_selected_peg(self, peg_name):
        self.peg_name = peg_name
        self.peg_frame = "object_" + peg_name
        self.peg_frame_filtered = "memory_" + peg_name
        self.grasp_tcp_frame = f"grasp_tcp_{peg_name}"
        self.pre_grasp_tcp_frame = f"pre_grasp_tcp_{peg_name}"
        self.pre_goal_frame = f"pre_goal_pose_{peg_name}"
        self.pre_goal_tcp_frame = f"pre_goal_tcp_{peg_name}"
        self.insert_center_frame = f"insert_center_{peg_name}"
        self.measured_goal_tcp_frame = f"measured_goal_tcp_pose_{peg_name}"
        self.measured_pre_goal_tcp_frame = f"measured_pre_goal_tcp_pose_{peg_name}"
        self.peg_pose_root = os.path.join(self.fixed_pose_root, peg_name)
        self.peg_pose_file = os.path.join(self.fixed_pose_root, peg_name, "current.json")
        self.grasp_tcp_pose_file = os.path.join(self.fixed_pose_root, peg_name, "grasp_tcp.json")
        self.pre_grasp_tcp_pose_file = os.path.join(self.fixed_pose_root, peg_name, "pre_grasp_tcp.json")
        self.insert_center_pose_file = os.path.join(self.fixed_pose_root, peg_name, "insert_center.json")

        # parameter server에 선택된 peg 이름 저장
        rospy.set_param('/selected_peg', peg_name)
        self.target_pose_frame = self.insert_center_frame
        rospy.set_param('/target_pose_frame', self.target_pose_frame)

        if hasattr(self, 'target_frame_pub'):
            frame_msg = String()
            frame_msg.data = self.target_pose_frame
            self.target_frame_pub.publish(frame_msg)

    def _get_pose_fallback_name(self, obj_name):
        return POSE_FALLBACK_PEG.get(obj_name)

    def _param_float(self, name, default):
        return float(str(rospy.get_param(name, default)))

    # -----------------
    # Pose And TF State
    # -----------------

    def _sample_with_step(self, lo, hi, step):
        lo = float(lo)
        hi = float(hi)
        if lo > hi:
            lo, hi = hi, lo

        if step <= 0.0:
            return float(np.random.uniform(lo, hi))

        start_idx = int(np.ceil(lo / step))
        end_idx = int(np.floor(hi / step))
        if start_idx <= end_idx:
            return float(np.random.randint(start_idx, end_idx + 1) * step)

        sampled = float(np.random.uniform(lo, hi))
        snapped = round(sampled / step) * step
        return float(min(max(snapped, lo), hi))

    def get_pre_goal_offset_z(self, peg_name):
        return PRE_GOAL_OFFSET_Z_BY_PART.get(peg_name, PRE_GOAL_OFFSET_Z)

    def load_and_publish_all_saved_poses(self):
        """저장 폴더 내의 모든 _fixed_pose.json 파일을 찾아 TF로 등록"""

        self.update_fixed_pose(
            pose_data=dummy_camera_pose_dict,
            parent_frame=PANDA_HAND_FRAME,
            child_frame=CAMERA_FRAME
        )

        if os.path.exists(self.hole_pose_file):
            with open(self.hole_pose_file, 'r') as f:
                hole_pose_dict = json.load(f)
            self.update_fixed_pose(
                pose_data=hole_pose_dict,
                parent_frame=self.base_frame,
                child_frame=self.hole_frame
            )
            rospy.loginfo(f"Loaded hole fixed pose from {self.hole_pose_file}")

        self.update_goal_related_frames(with_noise=True)

        # Measured goal pose 등록
        # self.update_fixed_pose(
        #     pose_data=measured_goal_tcp_pose_dict,
        #     parent_frame=self.base_frame,
        #     child_frame=self.measured_goal_tcp_frame
        # )

        # measured_pre_goal_tcp_pose_dict = copy.deepcopy(measured_goal_tcp_pose_dict)
        # measured_pre_goal_tcp_pose_dict['position']['z'] += 0.05
        # self.update_fixed_pose(
        #     pose_data=measured_pre_goal_tcp_pose_dict,
        #     parent_frame=self.base_frame,
        #     child_frame=self.measured_pre_goal_tcp_frame
        # )

        self.publish_all_static_tfs()

    def add_position_noise(self, pose_dict):
        noisy_pose = copy.deepcopy(pose_dict)
        axes = {'x': False, 'y': False, 'z': False}

        mode = (self.pre_goal_noise_mode or PRE_GOAL_NOISE_MODE).lower()
        if mode == 'xy':
            axes['x'] = True
            axes['y'] = True
        elif mode == 'z':
            axes['z'] = True
        elif mode == 'custom':
            axes = {'x': True, 'y': True, 'z': True}
        elif mode == 'none':
            pass
        else:
            axes = {'x': True, 'y': True, 'z': True}

        noise = {}
        for axis, enabled in axes.items():
            min_v, max_v = self.pre_goal_noise_range_m[axis]
            lo = min(min_v, max_v)
            hi = max(min_v, max_v)
            delta = self._sample_with_step(lo, hi, self.pre_goal_noise_step_m) if enabled else 0.0
            noisy_pose['position'][axis] += delta
            noise[axis] = delta

        rospy.loginfo(
            f"Applied shared goal/pre-goal position noise [m]: dx={noise['x']:.4f}, dy={noise['y']:.4f}, dz={noise['z']:.4f} "
            f"(mode={mode}, x=[{self.pre_goal_noise_range_m['x'][0]:.4f},{self.pre_goal_noise_range_m['x'][1]:.4f}], "
            f"y=[{self.pre_goal_noise_range_m['y'][0]:.4f},{self.pre_goal_noise_range_m['y'][1]:.4f}], "
            f"z=[{self.pre_goal_noise_range_m['z'][0]:.4f},{self.pre_goal_noise_range_m['z'][1]:.4f}], "
            f"step={self.pre_goal_noise_step_m:.4f})"
        )
        return noisy_pose

    def add_orientation_noise(self, pose_dict):
        noisy_pose = copy.deepcopy(pose_dict)
        axes = {'r': False, 'p': False, 'y': False}

        mode = (self.pre_goal_rot_noise_mode or PRE_GOAL_ROT_NOISE_MODE).lower()
        if mode == 'rpy':
            axes = {'r': True, 'p': True, 'y': True}
        elif mode == 'roll':
            axes['r'] = True
        elif mode == 'pitch':
            axes['p'] = True
        elif mode == 'yaw':
            axes['y'] = True
        elif mode == 'custom':
            axes = {'r': True, 'p': True, 'y': True}
        elif mode == 'none':
            pass
        else:
            axes = {'r': True, 'p': True, 'y': True}

        noise_deg = {}
        for axis, enabled in axes.items():
            min_v, max_v = self.pre_goal_rpy_noise_range_deg[axis]
            lo = min(min_v, max_v)
            hi = max(min_v, max_v)
            delta_deg = self._sample_with_step(lo, hi, self.pre_goal_rot_noise_step_deg) if enabled else 0.0
            noise_deg[axis] = delta_deg

        q = [
            noisy_pose['orientation']['x'],
            noisy_pose['orientation']['y'],
            noisy_pose['orientation']['z'],
            noisy_pose['orientation']['w']
        ]
        rot_current = R.from_quat(q)
        rot_noise = R.from_euler(
            'xyz',
            [noise_deg['r'], noise_deg['p'], noise_deg['y']],
            degrees=True
        )

        # parent frame(hole_frame) 축 기준으로 회전 노이즈 적용
        q_noisy = (rot_noise * rot_current).as_quat()
        noisy_pose['orientation']['x'] = q_noisy[0]
        noisy_pose['orientation']['y'] = q_noisy[1]
        noisy_pose['orientation']['z'] = q_noisy[2]
        noisy_pose['orientation']['w'] = q_noisy[3]

        rospy.loginfo(
            f"Applied shared goal/pre-goal RPY noise [deg]: dr={noise_deg['r']:.3f}, dp={noise_deg['p']:.3f}, dy={noise_deg['y']:.3f} "
            f"(mode={mode}, r=[{self.pre_goal_rpy_noise_range_deg['r'][0]:.3f},{self.pre_goal_rpy_noise_range_deg['r'][1]:.3f}], "
            f"p=[{self.pre_goal_rpy_noise_range_deg['p'][0]:.3f},{self.pre_goal_rpy_noise_range_deg['p'][1]:.3f}], "
            f"y=[{self.pre_goal_rpy_noise_range_deg['y'][0]:.3f},{self.pre_goal_rpy_noise_range_deg['y'][1]:.3f}], "
            f"step={self.pre_goal_rot_noise_step_deg:.3f})"
        )
        return noisy_pose

    def update_goal_related_frames(self, with_noise=True):
        goal_key = self.peg_name
        if goal_key not in self.goal_pose_path:
            fallback_name = self._get_pose_fallback_name(self.peg_name)
            if fallback_name in self.goal_pose_path:
                rospy.logwarn(f"Using goal pose fallback for {self.peg_name}: {fallback_name}")
                goal_key = fallback_name
            else:
                return

        goal_path = self.goal_pose_path[goal_key]
        if not os.path.exists(goal_path):
            fallback_name = self._get_pose_fallback_name(self.peg_name)
            fallback_path = self.goal_pose_path[fallback_name] if fallback_name in self.goal_pose_path else ""
            if fallback_name is not None and os.path.exists(fallback_path):
                rospy.logwarn(f"Goal pose file missing for {self.peg_name}. Using {fallback_name}: {fallback_path}")
                goal_key = fallback_name
                goal_path = fallback_path
            else:
                rospy.logwarn(f"Goal pose file not found for {self.peg_name}: {goal_path}")
                return

        self.goal_pose_data[self.peg_name] = self.load_goal_matrix(goal_path)
        if not self.goal_pose_data[self.peg_name]:
            rospy.logwarn(f"Failed to load goal pose for {self.peg_name}")
            return

        goal_pose_dict = copy.deepcopy(self.goal_pose_data[self.peg_name])
        if with_noise:
            # 소켓 pose 추정 오차를 모사하기 위해 goal/pre-goal에 동일한 오차를 공유한다.
            goal_pose_dict = self.add_position_noise(goal_pose_dict)
            goal_pose_dict = self.add_orientation_noise(goal_pose_dict)

        self.update_fixed_pose(
            pose_data=goal_pose_dict,
            parent_frame=self.hole_frame,
            child_frame=f"goal_pose_{self.peg_name}"
        )

        pre_goal_pose_dict = copy.deepcopy(goal_pose_dict)
        pre_goal_offset_z = self.get_pre_goal_offset_z(self.peg_name)
        pre_goal_pose_dict['position']['z'] += pre_goal_offset_z

        self.update_fixed_pose(
            pose_data=pre_goal_pose_dict,
            parent_frame=self.hole_frame,
            child_frame=self.pre_goal_frame
        )

        pre_goal_grasp_pose = None
        if self.tf_live_update_enabled and self.is_grasped and self.peg_name == 'part11':
            pre_goal_grasp_pose = self.get_live_grasp_pose_from_tf(self.peg_frame_filtered)
            if pre_goal_grasp_pose is not None:
                rospy.loginfo(
                    f"Using live grasp TF for {self.pre_goal_tcp_frame} from {self.peg_frame_filtered} -> {self.tcp_frame}"
                )

        if pre_goal_grasp_pose is None:
            pre_goal_grasp_pose = self.get_grasp_pose(self.peg_name)

        if pre_goal_grasp_pose is not None:
            self.update_fixed_pose(
                pose_data=pre_goal_grasp_pose,
                parent_frame=self.pre_goal_frame,
                child_frame=self.pre_goal_tcp_frame
            )
        else:
            rospy.logwarn(f"Skipped {self.pre_goal_tcp_frame}: grasp pose is unavailable for {self.peg_name}")

    def list_hole_pose_candidates(self):
        """part1 폴더에서 선택 가능한 current*.json 목록을 반환"""
        hole_dir = os.path.join(self.fixed_pose_root, self.hole_name)
        if not os.path.isdir(hole_dir):
            return []

        candidates = []
        for filename in os.listdir(hole_dir):
            if not filename.endswith('.json'):
                continue
            if not filename.startswith('current'):
                continue
            candidates.append(filename)

        def sort_key(name):
            if name == 'current.json':
                return (0, 0, name)
            stem = name[:-5]  # remove .json
            if stem.startswith('current_'):
                suffix = stem.split('current_', 1)[1]
                if suffix.isdigit():
                    return (1, int(suffix), name)
            return (2, 0, name)

        candidates.sort(key=sort_key)
        return candidates

    def select_hole_pose_menu(self):
        """Hole 기준 포즈 파일(current*.json)을 메뉴에서 선택"""
        candidates = self.list_hole_pose_candidates()
        if not candidates:
            rospy.logwarn(f"No current*.json files found in {os.path.join(self.fixed_pose_root, self.hole_name)}")
            return

        print("\nSelect hole pose file:")
        for idx, name in enumerate(candidates, start=1):
            print(f"  {idx}. {name}")

        user_input = input("Choose index or filename: ").strip()
        if not user_input:
            rospy.logwarn("No input provided. Hole pose selection cancelled.")
            return

        selected_name = None
        if user_input.isdigit():
            selected_idx = int(user_input) - 1
            if 0 <= selected_idx < len(candidates):
                selected_name = candidates[selected_idx]
        elif user_input in candidates:
            selected_name = user_input

        if selected_name is None:
            rospy.logwarn("Invalid selection. Please choose a valid index or filename.")
            return

        selected_path = os.path.join(self.fixed_pose_root, self.hole_name, selected_name)
        try:
            with open(selected_path, 'r') as f:
                hole_pose_dict = json.load(f)
        except Exception as e:
            rospy.logerr(f"Failed to load selected hole pose file {selected_path}: {e}")
            return

        self.hole_pose_file = selected_path

        rospy.loginfo(
            f"Pre-goal noise config: mode={self.pre_goal_noise_mode}, "
            f"x=[{self.pre_goal_noise_range_m['x'][0]:.4f},{self.pre_goal_noise_range_m['x'][1]:.4f}], "
            f"y=[{self.pre_goal_noise_range_m['y'][0]:.4f},{self.pre_goal_noise_range_m['y'][1]:.4f}], "
            f"z=[{self.pre_goal_noise_range_m['z'][0]:.4f},{self.pre_goal_noise_range_m['z'][1]:.4f}], "
            f"rot_mode={self.pre_goal_rot_noise_mode}, "
            f"r=[{self.pre_goal_rpy_noise_range_deg['r'][0]:.3f},{self.pre_goal_rpy_noise_range_deg['r'][1]:.3f}], "
            f"p=[{self.pre_goal_rpy_noise_range_deg['p'][0]:.3f},{self.pre_goal_rpy_noise_range_deg['p'][1]:.3f}], "
            f"yaw=[{self.pre_goal_rpy_noise_range_deg['y'][0]:.3f},{self.pre_goal_rpy_noise_range_deg['y'][1]:.3f}]"
        )

        self.update_fixed_pose(
            pose_data=hole_pose_dict,
            parent_frame=self.base_frame,
            child_frame=self.hole_frame
        )
        self.update_goal_related_frames(with_noise=True)
        self.publish_all_static_tfs()
        rospy.loginfo(f"Switched hole pose file to: {self.hole_pose_file}")

    def load_goal_matrix(self, goal_pose_path):
        """txt(4x4) 또는 json(position/orientation) goal pose를 로드"""
        try:
            ext = os.path.splitext(goal_pose_path)[1].lower()

            if ext == '.json':
                with open(goal_pose_path, 'r') as f:
                    data = json.load(f)

                if 'position' in data and 'orientation' in data:
                    rospy.loginfo("Successfully loaded goal pose from json file.")
                    return data

                rospy.logerr("Invalid goal json format. Expected position/orientation keys.")
                return False

            # default: 텍스트 파일에서 4x4 행렬 로드 (공백 또는 탭 구분)
            matrix = np.loadtxt(goal_pose_path)
            if matrix.shape != (4, 4):
                rospy.logerr("Matrix shape is not 4x4")
                return False

            trans = matrix[:3, 3]
            q = tft.quaternion_from_matrix(matrix)

            goal_pose_data = {
                "position": {
                    "x": trans[0],
                    "y": trans[1],
                    "z": trans[2]
                },
                "orientation": {
                    "x": q[0],
                    "y": q[1],
                    "z": q[2],
                    "w": q[3]
                }
            }

            rospy.loginfo("Successfully loaded goal matrix from txt file.")
            return goal_pose_data
        except Exception as e:
            rospy.logerr(f"Failed to load goal matrix: {e}")
            return False

    def _save_json(self, data, file_path):
        os.makedirs(os.path.dirname(file_path), exist_ok=True)
        with open(file_path, 'w') as f:
            json.dump(data, f, indent=4)

    def _load_json(self, file_path):
        with open(file_path, 'r') as f:
            return json.load(f)

    def _load_pose_json(self, file_path):
        data = self._load_json(file_path)
        if 'position' not in data or 'orientation' not in data:
            raise ValueError(f"Invalid pose format in {file_path}")
        return data

    def _pose_msg_to_dict(self, pose):
        return {
            "position": {
                "x": pose.position.x,
                "y": pose.position.y,
                "z": pose.position.z
            },
            "orientation": {
                "x": pose.orientation.x,
                "y": pose.orientation.y,
                "z": pose.orientation.z,
                "w": pose.orientation.w
            }
        }

    def _pose_dict_to_pose_stamped(self, frame_id, pose_dict):
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = frame_id
        pose_stamped.header.stamp = rospy.Time.now()
        pose_stamped.pose.position.x = pose_dict['position']['x']
        pose_stamped.pose.position.y = pose_dict['position']['y']
        pose_stamped.pose.position.z = pose_dict['position']['z']
        pose_stamped.pose.orientation.x = pose_dict['orientation']['x']
        pose_stamped.pose.orientation.y = pose_dict['orientation']['y']
        pose_stamped.pose.orientation.z = pose_dict['orientation']['z']
        pose_stamped.pose.orientation.w = pose_dict['orientation']['w']
        return pose_stamped

    def _pose_dict_to_transform(self, parent_frame, child_frame, pose_data):
        transform = TransformStamped()
        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = parent_frame
        transform.child_frame_id = child_frame
        transform.transform.translation.x = pose_data['position']['x']
        transform.transform.translation.y = pose_data['position']['y']
        transform.transform.translation.z = pose_data['position']['z']
        transform.transform.rotation.x = pose_data['orientation']['x']
        transform.transform.rotation.y = pose_data['orientation']['y']
        transform.transform.rotation.z = pose_data['orientation']['z']
        transform.transform.rotation.w = pose_data['orientation']['w']
        return transform

    def _build_pose_stamped(self, frame_id, pose_dict):
        return self._pose_dict_to_pose_stamped(frame_id, pose_dict)

    def _get_current_arm_joint_positions(self, timeout=2.0):
        joint_state = rospy.wait_for_message(ARM_JOINT_STATES_TOPIC, JointState, timeout=timeout)
        return dict(zip(joint_state.name, joint_state.position))

    def _compute_joint_move_duration(self, target_joints, current_joints, velocity_limit=0.5, min_duration=2.0):
        max_movement = max(
            abs(target_joints[joint] - current_joints[joint])
            for joint in target_joints
            if joint in current_joints
        )
        return max(max_movement / velocity_limit, min_duration)

    def _send_joint_goal(self, target_joints, move_duration):
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = list(target_joints.keys())

        point = JointTrajectoryPoint()
        point.positions = [target_joints[joint] for joint in goal.trajectory.joint_names]
        point.velocities = [0.0] * len(goal.trajectory.joint_names)
        point.time_from_start = rospy.Duration.from_sec(move_duration)
        goal.trajectory.points.append(point)

        self.trajectory_client.send_goal(goal)

    def _compute_and_execute_cartesian_path(
        self,
        waypoints,
        success_fraction=0.9,
        velocity_scaling_factor=None,
        acceleration_scaling_factor=None,
        failure_log='Cartesian path planning failed'
    ):
        plan, fraction = self.arm.compute_cartesian_path(waypoints, 0.01, False)
        if fraction <= success_fraction:
            rospy.logwarn(f"{failure_log} (Fraction: {fraction})")
            return False

        if velocity_scaling_factor is not None and acceleration_scaling_factor is not None:
            plan = self.arm.retime_trajectory(
                self.arm.get_current_state(),
                plan,
                velocity_scaling_factor=velocity_scaling_factor,
                acceleration_scaling_factor=acceleration_scaling_factor,
                algorithm="time_optimal_trajectory_generation"
            )

        self.arm.execute(plan, wait=True)
        return True

    def normalize_quaternion(self, q):
        norm = np.sqrt(q.x**2 + q.y**2 + q.z**2 + q.w**2)
        q.x /= norm
        q.y /= norm
        q.z /= norm
        q.w /= norm
        return q

    def get_current_tcp_pose(self):
        return self.arm.get_current_pose().pose

    # -----------------
    # ROS Interactions
    # -----------------

    # 2. 다른 노드에서 메시지를 보냈을 때 실행될 콜백 함수
    def mode_callback(self, msg):
        command = msg.data.strip()
        rospy.loginfo(f"Received mode command from topic: {command}")
        self.change_mode(command)

    def policy_status_callback(self, msg):
        self.policy_status = msg.data

    def start_insertion(self):
        rospy.loginfo('Starting insertion mode...')
        self.switch_controller(self.imp_controller)
        rospy.sleep(0.5)
        self.policy_reset_pub.publish(Bool(data=True))
        rospy.sleep(0.05)
        self.policy_reset_pub.publish(Bool(data=False))
        self.policy_enable_pub.publish(Bool(data=True))
        self.insertion_active = True

    def stop_insertion(self):
        if not self.insertion_active:
            return
        rospy.loginfo('Stopping insertion mode...')
        self.policy_enable_pub.publish(Bool(data=False))
        self.insertion_active = False

    def gripper_joint_callback(self, msg):
        """그리퍼 조인트 상태를 구독하여 그리퍼 열림/닫힘 상태를 업데이트"""
        try:
            idx1 = msg.name.index('panda_finger_joint1')
            idx2 = msg.name.index('panda_finger_joint2')
            gripper_width = msg.position[idx1] + msg.position[idx2]  # 두 조인트의 위치 합산
            self.latest_gripper_width = gripper_width
            self.last_gripper_state_stamp = rospy.Time.now()
            self.is_gripper_closed = (gripper_width + 0.005 < GRIPPER_MAX_WIDTH)
        except (ValueError, IndexError):
            rospy.logwarn_throttle(1.0, "finger joints not found in /franka_gripper/joint_states")

    def arm_joint_callback(self, msg):
        self.latest_arm_joint_positions = dict(zip(msg.name, msg.position))
        self.last_arm_joint_state_stamp = rospy.Time.now()

    def get_latest_arm_joint(self, joint_name, timeout_sec=0.5):
        if not self.latest_arm_joint_positions:
            return None
        if (rospy.Time.now() - self.last_arm_joint_state_stamp).to_sec() > timeout_sec:
            return None
        return self.latest_arm_joint_positions.get(joint_name)

    def _mask_callback(self, msg):
        try:
            # 마스크 이미지를 OpenCV 포맷으로 변환
            self.mask_data = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            rospy.logerr(f"Mask conversion error: {e}")

    def get_current_running_controller(self):
        try:
            list_res = self.list_srv()
            for c in list_res.controller:
                if c.state == 'running' and c.name in self.all_controllers:
                    return c.name
        except:
            pass
        return "UNKNOWN / NONE"

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        r, _, _ = select.select([sys.stdin], [], [], 0.1)
        key = sys.stdin.read(1) if r else None
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def switch_controller(self, target_controller):
        try:
            list_res = self.list_srv()
            running_controllers = [c.name for c in list_res.controller if c.state == 'running']

            if target_controller in running_controllers:
                rospy.logwarn(f"{target_controller} is ALREADY running.")

            to_stop = [c for c in running_controllers if c in self.all_controllers]
            req = SwitchControllerRequest()
            req.start_controllers = [target_controller]
            req.stop_controllers = to_stop
            req.strictness = SwitchControllerRequest.STRICT
            req.start_asap = True
            
            res = self.switch_srv(req)
            if res.ok:
                self.print_guide(self.mode)
        except Exception as e:
            rospy.logerr(f"Service call failed: {e}")

    def get_pose_from_tf(self, target_frame, parent_frame="panda_link0"):
        try:
            # parent_frame 기준으로 target_frame의 좌표를 가져옴
            trans = self.tf_buffer.lookup_transform(parent_frame, target_frame, rospy.Time(0), rospy.Duration(2.0))
            pose_dict = {
                'position': {
                    'x': trans.transform.translation.x,
                    'y': trans.transform.translation.y,
                    'z': trans.transform.translation.z,
                },
                'orientation': {
                    'x': trans.transform.rotation.x,
                    'y': trans.transform.rotation.y,
                    'z': trans.transform.rotation.z,
                    'w': trans.transform.rotation.w,
                }
            }
            return self._pose_dict_to_pose_stamped(parent_frame, pose_dict)
        except Exception as e:
            rospy.logerr(f"TF Lookup failed for {target_frame}: {e}")
            return None

    def pose_to_pose_dict(self, pose):
        return self._pose_msg_to_dict(pose)

    def set_trajectory_time(self, plan, target_duration):
        """궤적 전체 시간을 target_duration초로 재설정"""
        points = plan.joint_trajectory.points
        num_points = len(points)
        if num_points == 0: return plan
        for i in range(num_points):
            t = (i / float(num_points - 1)) * target_duration
            points[i].time_from_start = rospy.Duration(t)
            points[i].velocities = [0.0] * len(points[i].positions)
            points[i].accelerations = [0.0] * len(points[i].positions)
        return plan

    # -------------
    # Motion Prims
    # -------------

    def move_to_home(self):
        rospy.loginfo("move_to_home: Waiting for action server...")
        if not self.trajectory_client.wait_for_server(timeout=rospy.Duration(2.0)):
            rospy.logerr("move_to_home: Action server not found.")
            return

        rospy.loginfo("move_to_home: Lifting TCP by +5cm on base Z before homing...")
        if not self.move_tcp_xyz(0.0, 0.0, 0.05):
            rospy.logwarn("move_to_home: Failed to lift TCP by +5cm. Home motion aborted.")
            return

        try:
            initial_pose = self._get_current_arm_joint_positions(timeout=2.0)
        except:
            rospy.logerr("move_to_home: Could not get current joint states.")
            return

        move_duration = self._compute_joint_move_duration(self.home_pose, initial_pose, min_duration=2.0)
        self._send_joint_goal(self.home_pose, move_duration)

    def move_to_pre_grasp(self):
        rospy.loginfo(f"Moving {self.tcp_frame} to {self.pre_grasp_tcp_frame}...")
        pre_grasp_pose_stamped = self.get_pose_from_tf(self.pre_grasp_tcp_frame)
        if pre_grasp_pose_stamped:
            rospy.loginfo("Approaching Pre-Grasp Point (Cartesian Path)...")
            target_pose = pre_grasp_pose_stamped.pose
            target_pose.orientation = self.normalize_quaternion(target_pose.orientation)
            if self._compute_and_execute_cartesian_path(
                [target_pose],
                velocity_scaling_factor=0.5,
                acceleration_scaling_factor=0.2,
                failure_log='Cartesian path planning to Pre-Grasp failed'
            ):
                rospy.loginfo("Pre-Grasp position reached!")
                return True

            return False
            
    def move_to_grasp(self):
        rospy.loginfo(f"Moving {self.tcp_frame} to {self.grasp_tcp_frame}...")
        grasp_pose_stamped = self.get_pose_from_tf(self.grasp_tcp_frame)
        if grasp_pose_stamped:
            rospy.loginfo("Approaching Grasp Point (Cartesian Path)...")
            target_pose = grasp_pose_stamped.pose
            target_pose.orientation = self.normalize_quaternion(target_pose.orientation)
            if self._compute_and_execute_cartesian_path(
                [target_pose],
                velocity_scaling_factor=0.05,
                acceleration_scaling_factor=0.05
            ):
                rospy.loginfo("Grasp position reached!")
                return True
        
        return False

    def set_viewpoint_pose(self):
        """현재 로봇의 Joint State를 viewpoint.json 파일로 저장합니다."""
        rospy.loginfo("Setting current pose as viewpoint...")
        try:
            viewpoint_data = self._get_current_arm_joint_positions(timeout=1.0)
            
            # 필터링: panda_arm에 해당하는 joint만 저장 (필요 시)
            arm_joints = {k: v for k, v in viewpoint_data.items() if 'panda_joint' in k}

            self._save_json(arm_joints, self.viewpoint_file)
            
            rospy.loginfo(f"Viewpoint saved successfully to: {self.viewpoint_file}")
            print(f"\n[SAVED] {arm_joints}")
            
        except rospy.ROSException:
            rospy.logerr("Failed to get joint states. Is the robot driver running?")
        except Exception as e:
            rospy.logerr(f"Error saving viewpoint: {e}")

    def close_gripper(self):
        rospy.loginfo("Closing gripper...")
        # 그리퍼 닫기 로직을 여기에 추가합니다.
        joy = Joy()
        joy.buttons = [0, 1]
        self.joy_pub.publish(joy)

    def open_gripper(self):
        rospy.loginfo("Opening gripper...")
        # 그리퍼 열기 로직을 여기에 추가합니다.
        joy = Joy()
        joy.buttons = [1, 0]
        self.joy_pub.publish(joy)

    def check_grasp_status(self, event):
        """TCP와 목표 Grasp Pose 사이의 거리를 계산하여 잡기 성공 여부 판단"""
        now = rospy.Time.now()
        state_age_sec = (now - self.last_gripper_state_stamp).to_sec()
        has_recent_gripper_state = (
            self.latest_gripper_width is not None and
            state_age_sec <= self.gripper_state_timeout
        )

        if not has_recent_gripper_state:
            self.is_gripper_closed = False
            self.is_grasped = False
            self.grasp_status_pub.publish(Bool(self.is_grasped))
            return

        gripper_width = self.latest_gripper_width
        if gripper_width is None:
            self.is_gripper_closed = False
            self.is_grasped = False
            self.grasp_status_pub.publish(Bool(self.is_grasped))
            return

        self.is_gripper_closed = (gripper_width + 0.005 < GRIPPER_MAX_WIDTH)
        if not self.is_gripper_closed:
            self.is_grasped = False
            self.grasp_status_pub.publish(Bool(self.is_grasped))
            return

        try:
            # base_frame(panda_link0) 기준이 아닌,
            # grasp_tcp_frame 기준의 panda_hand_tcp 위치를 직접 가져오면 거리가 바로 나옵니다.
            trans = self.tf_buffer.lookup_transform(
                self.grasp_tcp_frame,
                self.tcp_frame,
                rospy.Time(0)
            )

            # 원점(0,0,0)으로부터의 거리 계산 (Euclidean Distance)
            dist = np.sqrt(trans.transform.translation.x**2 +
                           trans.transform.translation.y**2 +
                           trans.transform.translation.z**2)

            self.is_grasped = (dist < self.grasp_threshold)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            # TF를 찾을 수 없는 경우 (아직 Fix Pose가 안 되었을 때 등)
            self.is_grasped = False

        # 결과 발행
        self.grasp_status_pub.publish(Bool(self.is_grasped))

    def move_to_viewpoint(self):
        """저장된 viewpoint.json 파일을 읽어 로봇을 해당 자세로 이동시킵니다."""
        rospy.loginfo("Moving to saved viewpoint...")
        
        if not os.path.exists(self.viewpoint_file):
            rospy.logerr("Viewpoint file not found! Please set pose first (Press '0').")
            return

        try:
            # 1. 파일에서 데이터 로드
            target_viewpoint = self._load_json(self.viewpoint_file)
            print(f"\n[LOADED VIEWPOINT] {target_viewpoint}")

            # 2. 액션 서버 확인
            if not self.trajectory_client.wait_for_server(timeout=rospy.Duration(2.0)):
                rospy.logerr("Action server not found.")
                return

            # 3. 현재 위치 확인 및 이동 시간 계산
            current_pose = self._get_current_arm_joint_positions(timeout=2.0)
            move_duration = self._compute_joint_move_duration(target_viewpoint, current_pose, min_duration=3.0)
            
            rospy.loginfo(f"Sending viewpoint goal (Duration: {move_duration:.2f}s)")
            self._send_joint_goal(target_viewpoint, move_duration)
            
            # 이동 완료 대기 (필요 시)
            # self.trajectory_client.wait_for_result()
            
        except Exception as e:
            rospy.logerr(f"Error moving to viewpoint: {e}")

    def zoom_to_object(self, obj_name, target_area_ratio = 0.05):
        rospy.loginfo(f"Starting Zoom to {obj_name}...")
        self.bridge = CvBridge()
        self.mask_data = None
        sub = rospy.Subscriber(f"/sam2_mask/{obj_name}", Image, self._mask_callback)

        zoom_cfg = {
            'gain_xy': 0.00100,
            'max_xy_step_m': 0.025,
            'forward_step_m': 0.010,
            'deadzone_ratio': 1.0 / 3.0,
            'stop_px': 12,
            'q7_target': 0.0,
            'q7_deadband': np.deg2rad(2.0),
            'k_yaw': 0.18,
            'max_yaw_step': np.deg2rad(0.8),
            'max_yaw_total': np.deg2rad(20.0),
            'stable_required': 4,
            'rate_hz': 45,
        }

        stable_frames = 0
        yaw_offset = 0.0
        q_lock = None
        rate = rospy.Rate(zoom_cfg['rate_hz'])

        self.pose_pub = rospy.Publisher('/cartesian_pose_controller/tcp_target_pose', PoseStamped, queue_size=1)

        try:
            while not rospy.is_shutdown():
                if self.mask_data is None:
                    rate.sleep()
                    continue

                try:
                    tcp_tf = self.tf_buffer.lookup_transform(self.base_frame, self.tcp_frame, rospy.Time(0))
                    cam_tf = self.tf_buffer.lookup_transform(self.base_frame, CAMERA_FRAME, rospy.Time(0))

                    if q_lock is None:
                        q_lock = [
                            tcp_tf.transform.rotation.x,
                            tcp_tf.transform.rotation.y,
                            tcp_tf.transform.rotation.z,
                            tcp_tf.transform.rotation.w,
                        ]

                    mask_np = np.asarray(self.mask_data)
                    if mask_np.ndim == 3:
                        mask_np = mask_np[:, :, 0]

                    moments = cv2.moments(mask_np)
                    if moments['m00'] <= 0:
                        rate.sleep()
                        continue

                    u = int(moments['m10'] / moments['m00'])
                    v = int(moments['m01'] / moments['m00'])
                    h, w = mask_np.shape[:2]
                    err_u = (w / 2.0 - u)
                    err_v = (h / 2.0 - v)
                    area_ratio = moments['m00'] / (h * w * 255.0)

                    deadzone_u = w * zoom_cfg['deadzone_ratio'] * 0.5
                    deadzone_v = h * zoom_cfg['deadzone_ratio'] * 0.5

                    # 중심 근처에서는 XY를 멈춰 불필요한 미세 진동 방지
                    if abs(err_u) < deadzone_u:
                        err_u = 0.0
                    if abs(err_v) < deadzone_v:
                        err_v = 0.0

                    d_x_cam = np.clip(err_u * zoom_cfg['gain_xy'], -zoom_cfg['max_xy_step_m'], zoom_cfg['max_xy_step_m'])
                    d_y_cam = np.clip(err_v * zoom_cfg['gain_xy'], -zoom_cfg['max_xy_step_m'], zoom_cfg['max_xy_step_m'])

                    # XY와 Z를 동시에 제어: 면적이 부족하면 전진은 계속 수행
                    d_z_cam = zoom_cfg['forward_step_m'] if area_ratio < target_area_ratio else 0.0

                    q_cam = [
                        cam_tf.transform.rotation.x,
                        cam_tf.transform.rotation.y,
                        cam_tf.transform.rotation.z,
                        cam_tf.transform.rotation.w,
                    ]
                    cam_rot = tft.quaternion_matrix(q_cam)[:3, :3]
                    delta_cam = np.array([d_x_cam, d_y_cam, d_z_cam])
                    delta_base = np.dot(cam_rot, delta_cam)

                    q7 = self.get_latest_arm_joint('panda_joint7', timeout_sec=0.5)
                    if q7 is not None:
                        q7_err = zoom_cfg['q7_target'] - q7
                        if abs(q7_err) < zoom_cfg['q7_deadband']:
                            q7_err = 0.0
                        yaw_step = np.clip(q7_err * zoom_cfg['k_yaw'], -zoom_cfg['max_yaw_step'], zoom_cfg['max_yaw_step'])
                        yaw_offset = float(np.clip(yaw_offset + yaw_step, -zoom_cfg['max_yaw_total'], zoom_cfg['max_yaw_total']))

                    q_yaw = tft.quaternion_from_euler(0.0, 0.0, yaw_offset)
                    q_target = tft.quaternion_multiply(q_lock, q_yaw)
                    q_target = q_target / np.linalg.norm(q_target)

                    target_stamped = PoseStamped()
                    target_stamped.header.frame_id = self.base_frame
                    target_stamped.header.stamp = rospy.Time.now()
                    target_stamped.pose.position.x = tcp_tf.transform.translation.x + delta_base[0]
                    target_stamped.pose.position.y = tcp_tf.transform.translation.y + delta_base[1]
                    target_stamped.pose.position.z = tcp_tf.transform.translation.z + delta_base[2]
                    target_stamped.pose.orientation.x = q_target[0]
                    target_stamped.pose.orientation.y = q_target[1]
                    target_stamped.pose.orientation.z = q_target[2]
                    target_stamped.pose.orientation.w = q_target[3]
                    self.pose_pub.publish(target_stamped)

                    reached = (
                        abs(err_u) < zoom_cfg['stop_px'] and
                        abs(err_v) < zoom_cfg['stop_px'] and
                        area_ratio >= target_area_ratio
                    )
                    stable_frames = stable_frames + 1 if reached else 0

                    rospy.loginfo_throttle(
                        0.5,
                        f"zoom err(u,v)=({err_u:.1f},{err_v:.1f}) area={area_ratio:.4f}/{target_area_ratio:.4f} "
                        f"q7={q7 if q7 is not None else float('nan'):.3f} yaw_ofs={np.rad2deg(yaw_offset):.2f}deg"
                    )

                    if stable_frames >= zoom_cfg['stable_required']:
                        rospy.loginfo("Reached! (stable center + target area)")
                        break

                except Exception as e:
                    rospy.logerr_throttle(1.0, f"Zoom Error: {e}")

                rate.sleep()
        finally:
            sub.unregister()

    # -----------------------
    # Object Pose Operations
    # -----------------------

    def select_object(self, obj_name):
        rospy.loginfo(f"Selecting target object: {obj_name}")
        # 객체 선택 로직을 여기에 추가합니다.
        # sam2 노드 동작 요청
        self.selected_object = obj_name
        if obj_name != self.hole_name:
            self.update_selected_peg(obj_name)
        else:
            rospy.loginfo("Hole selected: keep /selected_peg unchanged to avoid grasp pose dependency.")

        msg = String()
        msg.data = obj_name
        
        # 3. 토픽 발행 (Publish)
        self.sam2_pub.publish(msg)
        self.fp_pub.publish(msg)
        
        rospy.loginfo(f"Published target object '{obj_name}' to /sam2_target_object")
        rospy.loginfo(f"Published target object '{obj_name}' to /fp_target_object")
        rospy.loginfo(f"Selected target object: {self.selected_object}")

    def request_object_pose(self, obj_name):
        rospy.loginfo(f"Requesting pose estimation for object: {obj_name}")
        topic_name = f"/object_pose/{obj_name}"
        self.pose_sub = rospy.Subscriber(topic_name, PoseStamped, self.pose_callback)
        self.pose_received = False

        # FoundationPose에 객체 포즈 추정을 요청합니다.
        msg = String()
        msg.data = obj_name
        
        # 3. 토픽 발행 (Publish)
        self.fp_pub.publish(msg)
        rospy.loginfo(f"Published target object '{obj_name}' to /fp_target_object")

        start_time = rospy.Time.now()
        while not self.pose_received:
            key = self.get_key()
            if key == 'q': # 'q' 키를 누르면 대기 중단
                rospy.loginfo("Pose request cancelled by user.")
                self.print_guide(self.mode)
                break
            if (rospy.Time.now() - start_time).to_sec() > 5:
                rospy.logwarn("Timeout while waiting for pose data.")
                break
            print("Waiting for pose data...", end='\r')
            rospy.sleep(0.1)
        # if self.pose_received:
        #     rospy.loginfo("Pose data received successfully.")
        #     self.update_grasp_pose(obj_name, self.peg_frame_filtered)
        #     self.publish_all_static_tfs()

        self.pose_sub.unregister()
        self.pose_received = False

    def pose_callback(self, msg):
        self.current_pose = msg
        self.pose_received = True
        rospy.loginfo("New pose received via callback.")

    def _filter_outlier_poses(self, poses, threshold=0.03, min_fraction=0.5):
        if len(poses) < 2:
            return poses

        positions = np.array([
            [p.pose.position.x, p.pose.position.y, p.pose.position.z]
            for p in poses
        ])
        median_pos = np.median(positions, axis=0)
        distances = np.linalg.norm(positions - median_pos, axis=1)
        filtered_indices = np.where(distances < threshold)[0]
        min_count = max(1, int(np.ceil(len(poses) * min_fraction)))

        if len(filtered_indices) < min_count:
            rospy.logwarn("Too many outliers detected. Checking sensor stability...")
            return poses

        return [poses[i] for i in filtered_indices]

    def _collect_object_poses_in_base(self, topic_name, num_samples=10, timeout_sec=10.0):
        poses_in_base = []
        start_time = rospy.Time.now()

        rospy.loginfo(f"Collecting samples and transforming to {self.base_frame}...")
        while len(poses_in_base) < num_samples:
            if (rospy.Time.now() - start_time).to_sec() > timeout_sec:
                rospy.logwarn("Timeout while collecting samples.")
                break

            try:
                pose_msg = rospy.wait_for_message(topic_name, PoseStamped, timeout=0.1)
                pose_base = self.tf_buffer.transform(pose_msg, self.base_frame, timeout=rospy.Duration(1.0))
                poses_in_base.append(pose_base)
                if len(poses_in_base) >= num_samples:
                    poses_in_base = self._filter_outlier_poses(poses_in_base)
                print(f"Collected {len(poses_in_base)}/{num_samples} samples...", end='\r')
            except (rospy.ROSException, tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                continue

        return poses_in_base

    def _load_or_collect_fixed_pose(self, file_path, topic_name, num_samples=10, timeout_sec=10.0):
        poses_in_base = self._collect_object_poses_in_base(topic_name, num_samples=num_samples, timeout_sec=timeout_sec)
        if len(poses_in_base) >= num_samples:
            avg_pose_dict = self.calculate_average_pose(poses_in_base)
            self.save_pose_to_file(avg_pose_dict, file_path)
            rospy.loginfo(f"Successfully saved base-link pose to {file_path}")
            return avg_pose_dict

        rospy.logwarn("Not enough real-time samples. Trying to load from file...")
        if os.path.exists(file_path):
            return self.load_pose_json(file_path)

        rospy.logerr("No data available to fix pose.")
        return None

    def fix_object_pose(self):
        rospy.loginfo(f"Fixing pose for object: {self.selected_object}")
        file_path = os.path.join(self.fixed_pose_root, self.selected_object, CURRENT_POSE_FILENAME)
        topic_name = f"/object_pose/{self.selected_object}"
        try:
            avg_pose_dict = self._load_or_collect_fixed_pose(file_path, topic_name)
            if avg_pose_dict is None:
                return

            self.fp_pub.publish("")

            self.update_fixed_pose(
                pose_data=avg_pose_dict,
                parent_frame=self.base_frame,
                child_frame=self.peg_frame_filtered
            )

            self.publish_all_static_tfs()
            rospy.loginfo(f"Fixed pose for {self.selected_object} established on panda_link0.")

        except Exception as e:
            rospy.logerr(f"Error in fix_object_pose: {e}")

    # --------------------
    # Grasp And Frame Data
    # --------------------

    def update_grasp_pose(self, obj_name, parent_frame):
        grasp_pose = self.get_grasp_pose(obj_name)
        if grasp_pose is None:
            rospy.logwarn(f"Grasp pose unavailable for {obj_name}")
            return

        self.update_fixed_pose(
            pose_data=grasp_pose,
            parent_frame=parent_frame,
            child_frame=self.grasp_tcp_frame
        )
        rospy.loginfo(f"Grasp pose updated for: {self.grasp_tcp_frame}")

        pre_grasp_pose = self.get_pre_grasp_pose(obj_name, grasp_pose)
        self.update_fixed_pose(
            pose_data=pre_grasp_pose,
            parent_frame=parent_frame,
            child_frame=self.pre_grasp_tcp_frame
        )
        rospy.loginfo(f"Pre-grasp pose updated for: {self.pre_grasp_tcp_frame}")

        insert_center_pose = self.get_insert_center_pose(obj_name)
        if insert_center_pose is None:
            rospy.logwarn(f"Insert center pose unavailable for {obj_name}")
            return

        self.update_fixed_pose(
            pose_data=insert_center_pose,
            parent_frame=parent_frame,
            child_frame=self.insert_center_frame
        )

    def calculate_relative_grasp_orientation(self, object_quat_base):
        """참고 코드에서 제공된 Peg 정렬 로직"""
        R_obj_base = tft.quaternion_matrix(object_quat_base)

        x_axis_obj = R_obj_base[:3, 0]
        y_axis_obj = R_obj_base[:3, 1]
        
        # Peg 정렬 로직: X축은 물체의 -Y, Y축은 물체의 -X
        x_target_w = -y_axis_obj
        y_target_w = -x_axis_obj
        z_target_w = np.cross(x_target_w, y_target_w)
        
        R_grasp_base = np.eye(4)
        R_grasp_base[:3, 0] = x_target_w / np.linalg.norm(x_target_w)
        R_grasp_base[:3, 1] = y_target_w / np.linalg.norm(y_target_w)
        R_grasp_base[:3, 2] = z_target_w / np.linalg.norm(z_target_w)
        
        # 상대 회전: R_rel = R_obj^-1 * R_grasp
        R_relative_obj = np.dot(tft.inverse_matrix(R_obj_base), R_grasp_base)
        return tft.quaternion_from_matrix(R_relative_obj)

    def calculate_average_pose(self, poses):
        """PoseStamped 리스트를 받아 평균 위치와 쿼터니언을 반환 (딕셔너리 형태)"""
        pos_list = [[p.pose.position.x, p.pose.position.y, p.pose.position.z] for p in poses]
        quat_list = [[p.pose.orientation.x, p.pose.orientation.y, p.pose.orientation.z, p.pose.orientation.w] for p in poses]
        
        avg_pos = np.mean(pos_list, axis=0)
        
        # 쿼터니언 평균 (단순 평균 후 정규화)
        avg_quat = np.mean(quat_list, axis=0)
        avg_quat /= np.linalg.norm(avg_quat)
        
        return {
            'position': {'x': avg_pos[0], 'y': avg_pos[1], 'z': avg_pos[2]},
            'orientation': {'x': avg_quat[0], 'y': avg_quat[1], 'z': avg_quat[2], 'w': avg_quat[3]}
        }

    def save_pose_to_file(self, pose_data, file_path):
        self._save_json(pose_data, file_path)

    def load_pose_json(self, file_path):
        return self._load_pose_json(file_path)

    def get_grasp_pose(self, obj_name):
        grasp_path = os.path.join(self.fixed_pose_root, obj_name, 'grasp_tcp.json')
        if os.path.exists(grasp_path):
            try:
                return self.load_pose_json(grasp_path)
            except Exception as e:
                rospy.logwarn(f"Failed to load grasp pose from {grasp_path}: {e}")

        if obj_name in grasp_pose_dict:
            rospy.logwarn(f"Using legacy grasp_pose_dict fallback for {obj_name}")
            return grasp_pose_dict[obj_name]

        return None

    def get_live_grasp_pose_from_tf(self, object_memory_frame):
        """memory object frame 기준 현재 TCP pose를 grasp pose로 반환"""
        try:
            trans = self.tf_buffer.lookup_transform(
                object_memory_frame,
                self.tcp_frame,
                rospy.Time(0),
                rospy.Duration(0.2)
            )
            return {
                'position': {
                    'x': trans.transform.translation.x,
                    'y': trans.transform.translation.y,
                    'z': trans.transform.translation.z
                },
                'orientation': {
                    'x': trans.transform.rotation.x,
                    'y': trans.transform.rotation.y,
                    'z': trans.transform.rotation.z,
                    'w': trans.transform.rotation.w
                }
            }
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn(f"Failed to read live grasp TF {object_memory_frame} -> {self.tcp_frame}: {e}")
            return None

    def get_pre_grasp_pose(self, obj_name, grasp_pose):
        pre_grasp_path = os.path.join(self.fixed_pose_root, obj_name, 'pre_grasp_tcp.json')
        if os.path.exists(pre_grasp_path):
            try:
                return self.load_pose_json(pre_grasp_path)
            except Exception as e:
                rospy.logwarn(f"Failed to load pre-grasp pose from {pre_grasp_path}: {e}")

        pre_grasp_pose = copy.deepcopy(grasp_pose)
        pre_grasp_pose['position']['z'] += PRE_GRASP_OFFSET_Z
        return pre_grasp_pose

    def get_insert_center_pose(self, obj_name):
        insert_center_path = os.path.join(self.fixed_pose_root, obj_name, 'insert_center.json')
        if os.path.exists(insert_center_path):
            try:
                return self.load_pose_json(insert_center_path)
            except Exception as e:
                rospy.logwarn(f"Failed to load insert center pose from {insert_center_path}: {e}")

        fallback_name = self._get_pose_fallback_name(obj_name)
        if fallback_name is not None:
            fallback_insert_center_path = os.path.join(self.fixed_pose_root, fallback_name, 'insert_center.json')
            if os.path.exists(fallback_insert_center_path):
                try:
                    rospy.loginfo(f"Using {fallback_name} insert_center.json for {obj_name}")
                    return self.load_pose_json(fallback_insert_center_path)
                except Exception as e:
                    rospy.logwarn(f"Failed to load insert center pose from {fallback_insert_center_path}: {e}")

        return None

    def update_fixed_pose(self, pose_data, parent_frame, child_frame):
        """딕셔너리에 TF 정보를 등록하거나 업데이트합니다."""
        self.fixed_poses[child_frame] = self._pose_dict_to_transform(parent_frame, child_frame, pose_data)
        rospy.loginfo(f"Pose updated for: {child_frame}")

    def publish_all_static_tfs(self):
        """현재까지 등록된 모든 Static TF를 한꺼번에 발행합니다."""
        if self.fixed_poses:
            # 리스트로 변환하여 한 번에 전송
            self.static_br.sendTransform(list(self.fixed_poses.values()))
            rospy.loginfo(f"Successfully broadcasted {len(self.fixed_poses)} static transforms.")

    def generate_trajectory(self):
        rospy.loginfo("Generating Trajectory...")
        # trajectory_generation_node.py의 TrajectoryGeneration 클래스를 여기에 통합하거나
        # 별도의 노드로 실행하여 궤적 생성 동작을 수행할 수 있습니다.
        # 여기서는 단순히 로그만 출력합니다.
        rospy.loginfo("Trajectory generation logic should be implemented here.")

    def _apply_manual_pre_goal_noise(self, manual_pose_dict):
        manual_pose_dict = copy.deepcopy(manual_pose_dict)
        applied_pos_noise = False
        applied_rot_noise = False

        if (self.pre_goal_noise_mode or PRE_GOAL_NOISE_MODE).lower() != 'none':
            manual_pose_dict = self.add_position_noise(manual_pose_dict)
            applied_pos_noise = True

        if (self.pre_goal_rot_noise_mode or PRE_GOAL_ROT_NOISE_MODE).lower() != 'none':
            manual_pose_dict = self.add_orientation_noise(manual_pose_dict)
            applied_rot_noise = True

        return manual_pose_dict, applied_pos_noise, applied_rot_noise

    def _resolve_approach_target_pose(self):
        requested_target = self.pre_goal_target

        if requested_target == PRE_GOAL_TARGET_FROM_MANUAL:
            manual_pose_dict = pre_goal_tcp_pose_dict.get(self.peg_name)
            if manual_pose_dict is not None:
                manual_pose_dict, applied_pos_noise, applied_rot_noise = self._apply_manual_pre_goal_noise(manual_pose_dict)
                rospy.loginfo(
                    f"Approach source=from_manual (peg={self.peg_name}, "
                    f"position_noise={'on' if applied_pos_noise else 'off'}, "
                    f"orientation_noise={'on' if applied_rot_noise else 'off'})"
                )
                return (
                    self._pose_dict_to_pose_stamped(self.base_frame, manual_pose_dict).pose,
                    f"manual pre_goal_tcp_pose_dict[{self.peg_name}]"
                )

            rospy.logwarn(
                f"Approach source=from_manual requested but key '{self.peg_name}' "
                "is missing in pre_goal_tcp_pose_dict. Falling back to from_video."
            )

        target_frame = self.pre_goal_tcp_frame
        pre_goal_tcp_pose_stamped = self.get_pose_from_tf(target_frame)
        if not pre_goal_tcp_pose_stamped:
            rospy.logwarn(f"Failed to get target pose from TF: {target_frame}")
            return None, None

        rospy.loginfo(
            f"Approach source=from_video"
            f"{' (fallback)' if requested_target == PRE_GOAL_TARGET_FROM_MANUAL else ''}"
        )
        return pre_goal_tcp_pose_stamped.pose, target_frame

    def _lift_tcp_for_approach(self):
        current_tcp_pose = self.get_current_tcp_pose()
        target_lift_z_base = 0.15
        if current_tcp_pose.position.z > target_lift_z_base:
            target_lift_z_base = 0.5 * (current_tcp_pose.position.z + target_lift_z_base)

        dz_to_lift = target_lift_z_base - current_tcp_pose.position.z
        if abs(dz_to_lift) > 0.001:
            self.move_tcp_xyz(dx=0, dy=0, dz=dz_to_lift)
        else:
            rospy.loginfo(
                f"Current TCP z ({current_tcp_pose.position.z:.3f}) is already at "
                f"target lift z ({target_lift_z_base:.3f})."
            )

    def _move_xy_then_z(self, current_tcp_pose, target_pose):
        target_pose.orientation = self.normalize_quaternion(target_pose.orientation)

        xy_pose = copy.deepcopy(current_tcp_pose)
        xy_pose.position.x = target_pose.position.x
        xy_pose.position.y = target_pose.position.y
        xy_pose.orientation = target_pose.orientation

        rospy.loginfo("Step 1/2: Moving in XY plane (keep current Z)...")
        if not self._compute_and_execute_cartesian_path(
            [xy_pose],
            velocity_scaling_factor=0.3,
            acceleration_scaling_factor=0.1,
            failure_log='Cartesian XY planning failed'
        ):
            return False

        current_after_xy = self.get_current_tcp_pose()
        z_pose = copy.deepcopy(current_after_xy)
        z_pose.position.z = target_pose.position.z
        z_pose.orientation = target_pose.orientation

        rospy.loginfo("Step 2/2: Descending along Z axis...")
        if not self._compute_and_execute_cartesian_path(
            [z_pose],
            velocity_scaling_factor=0.2,
            acceleration_scaling_factor=0.08,
            failure_log='Cartesian Z planning failed'
        ):
            return False

        return True

    def approach(self):
        # Approach를 호출할 때마다 pre-goal position noise를 재샘플링
        self.update_goal_related_frames(with_noise=True)
        self.publish_all_static_tfs()

        self._lift_tcp_for_approach()
        current_tcp_pose = self.get_current_tcp_pose()

        target_pose, target_desc = self._resolve_approach_target_pose()
        if target_pose is None:
            return False

        rospy.loginfo(f"Moving {self.tcp_frame} to {target_desc} with XY-first strategy...")
        if not self._move_xy_then_z(current_tcp_pose, target_pose):
            return False

        rospy.loginfo("Pre-Goal position reached.")
        return True

    # -----------------
    # UI And Commands
    # -----------------

    def print_guide(self, current_mode):
        print("\n" + "="*50)
        print(f" [ CURRENT ACTIVE MODE: {MODE_DICT[current_mode]} ]")
        print("="*50)
        for key, guide_text in MODE_GUIDE_ITEMS:
            print(f" Press '{key}': {guide_text}")
        print(" Press 'q': Quit and Shutdown Node")
        print("-" * 50)
        print(" (Also listening on topic: /set_controller_mode)")
        print(" >> Select Mode: ", end='', flush=True)

    def _parse_peg_name(self, peg_input):
        if peg_input.startswith("part"):
            return peg_input
        if peg_input == "11-2":
            return "part11-2"

        try:
            peg_number = int(peg_input)
        except ValueError:
            return None

        return f"part{peg_number}" if peg_number in PEG_LIST else None

    def _handle_select_peg(self):
        peg_input = input("Enter peg number/name (e.g. 11, 11-2, part12): ").strip()
        peg_name = self._parse_peg_name(peg_input)
        is_valid = peg_name in PEG_NAME_EXTRA or (
            peg_name is not None and os.path.isdir(os.path.join(self.fixed_pose_root, peg_name))
        )
        if is_valid:
            self.peg_name = peg_name
            self.select_object(self.peg_name)
            return

        rospy.logwarn("Invalid peg input. Use one of [9, 11, 11-2, 12, 17] or part name.")

    def _run_position_mode_action(self, action):
        self.switch_controller(self.pos_controller)
        rospy.sleep(0.5)
        return action()

    def _handle_zoom_to_object(self):
        target_ratio = self.target_area_ratio.get(self.selected_object, self.target_area_ratio['part11'])
        if self.selected_object not in self.target_area_ratio:
            rospy.logwarn(f"No target_area_ratio for {self.selected_object}. Using fallback: {target_ratio}")

        self.switch_controller(self.pose_controller)
        rospy.sleep(0.5)
        self.zoom_to_object(self.selected_object, target_ratio)

    def _register_current_pose(self):
        rospy.loginfo(f"Registering current averaged pose for: {self.selected_object}")
        self.request_object_pose(self.selected_object)
        self.fix_object_pose()
        saved_path = os.path.join(self.fixed_pose_root, self.selected_object, 'current.json')
        rospy.loginfo(f"Averaged pose registration complete: {saved_path}")

    def _mode_handlers(self):
        return {
            MOVEIT: lambda: self.switch_controller(self.pos_controller),
            CARTESIAN_IMPEDANCE: lambda: self.switch_controller(self.imp_controller),
            CARTESIAN_POSE: lambda: self.switch_controller(self.pose_controller),
            TELEOP: lambda: self.switch_controller(self.pose_controller),
            TELEOP_IMPEDANCE: lambda: self.switch_controller(self.imp_controller),
            OPEN_GRIPPER: self.open_gripper,
            CLOSE_GRIPPER: self.close_gripper,
            SELECT_HOLE: lambda: self.select_object(self.hole_name),
            SELECT_HOLE_POSE: self.select_hole_pose_menu,
            SELECT_PEG: self._handle_select_peg,
            SET_VIEWPOINT: self.set_viewpoint_pose,
            MOVE_TO_HOME: lambda: self._run_position_mode_action(self.move_to_home),
            MOVE_TO_VIEWPOINT: lambda: self._run_position_mode_action(self.move_to_viewpoint),
            ZOOM_TO_OBJECT: self._handle_zoom_to_object,
            MOVE_TO_PRE_GRASP: lambda: self._run_position_mode_action(self.move_to_pre_grasp),
            MOVE_TO_GRASP: lambda: self._run_position_mode_action(self.move_to_grasp),
            GENERATE_TRAJECTORY: lambda: self._run_position_mode_action(self.generate_trajectory),
            APPROACH: lambda: self._run_position_mode_action(self.approach),
            INSERTION: self.start_insertion,
            GRASPING: self.grasping,
            REGISTER_FRAMES: self._register_current_pose,
        }

    # -----------------
    # Motion Workflows
    # -----------------

    def move_tcp_xyz(self, dx, dy, dz):
        """
        현재 TCP 위치에서 지정된 거리만큼 XYZ 축으로 직선 이동합니다.
        """
        rospy.loginfo(f"Moving TCP by dx: {dx}, dy: {dy}, dz: {dz}")

        # 1. 현재 포즈 가져오기
        current_pose_stamped = self.arm.get_current_pose()
        current_pose = current_pose_stamped.pose

        # 2. 목표 포즈 계산 (현재 위치 + 증분)
        target_pose = copy.deepcopy(current_pose)
        target_pose.position.x += dx
        target_pose.position.y += dy
        target_pose.position.z += dz
        # target_pose.orientation = self.normalize_quaternion(target_pose.orientation)

        # 4. Cartesian Path 생성
        waypoints = [target_pose]
        # eef_step은 아까 성공했던 0.01~0.05 사이의 값 사용
        (plan, fraction) = self.arm.compute_cartesian_path(
            waypoints, 
            0.01,   # eef_step
            False   # avoid_collisions
        )

        # 5. 실행
        if fraction > 0.9:
            rospy.loginfo(f"TCP move plan successful (fraction: {fraction}). Executing...")
            # 부드러운 이동을 위해 속도 조절이 필요하다면 retime_trajectory 추가
            return self.arm.execute(plan, wait=True)
        else:
            rospy.logwarn(f"TCP move failed. Fraction: {fraction}")
            return False
        
    def grasping(self):
        # 1. 객체 자세 추정 요청
        self.request_object_pose(self.selected_object)
        # 2. Pre-Grasp 위치로 이동
        self.switch_controller(self.pos_controller)
        self.move_to_pre_grasp()
        # 3. 객체 자세 Refinement 및 Fix
        self.fix_object_pose()
        # 4. Grasping 시도
        self.move_to_pre_grasp()
        self.move_to_grasp()
        self.close_gripper()
        # 5. 객체 자세 재요청 및 잡힘 확인
        self.request_object_pose(self.selected_object)
        rospy.sleep(0.5)
        if self.is_grasped:
            rospy.loginfo("Grasping successful!")
            self.approach()
            self.print_guide(self.mode)
        else:
            rospy.logwarn("Grasping failed. Object not secured.")
            # self.open_gripper()
            self.move_to_home()
            self.print_guide(self.mode)

    # -----------------
    # Main Event Loop
    # -----------------

    def change_mode(self, key):
        if key is None: 
            return False

        if key != INSERTION:
            self.stop_insertion()

        if key == 'q':
            self.stop_insertion()
            return rospy.signal_shutdown("User requested shutdown.")

        handler = self._mode_handlers().get(key)
        if handler is not None:
            handler()

        self.mode = key
        return True

    def run(self):
        try:
            while not rospy.is_shutdown():
                key = self.get_key()
                if self.change_mode(key):
                    self.mode_pub.publish(self.mode)

        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Controller switcher node')
    parser.add_argument(
        '--selected_object',
        type=str,
        default='part9',
        help='Initial target object (e.g. part11, 11, 11-2, part12, part1)'
    )
    parser.add_argument(
        '--tf_live_update',
        type=str,   
        choices=['on', 'off'],
        default='off',
        help='Enable/disable live grasp TF update (default: off)'
    )
    parser.add_argument(
        '--pre_goal_target',
        type=str,
        choices=[PRE_GOAL_TARGET_FROM_VIDEO, PRE_GOAL_TARGET_FROM_MANUAL],
        default=PRE_GOAL_TARGET_FROM_MANUAL,
        help='Pre-goal target source: from_video (TF chain) or from_manual (pre_goal_tcp_pose_dict)'
    )
    args = parser.parse_args(rospy.myargv(argv=sys.argv)[1:])

    switcher = ControllerSwitcher(
        selected_object=args.selected_object,
        tf_live_update=args.tf_live_update,
        pre_goal_target=args.pre_goal_target
    )
    switcher.run()
