#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import os
import copy
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

MODE_DICT = {
    'a': 'MOVEIT',
    's': 'CARTESIAN_IMPEDANCE',
    'd': 'CARTESIAN_POSE',
    't': 'TELEOP',
    'i': 'TELEOP_IMPEDANCE',
    'h': 'SELECT_HOLE',
    'p': 'SELECT_PEG',
    'o': 'OPEN_GRIPPER',
    'c': 'CLOSE_GRIPPER',
    'g': 'GRASPING',
    'f': 'REGISTER_FRAMES',
    'j': 'SELECT_HOLE_POSE',
    '0': 'SET_VIEWPOINT',
    '1': 'MOVE_TO_HOME',
    '2': 'MOVE_TO_VIEWPOINT',
    '3': 'ZOOM_TO_OBJECT',
    '4': 'MOVE_TO_PRE_GRASP',
    '5': 'MOVE_TO_GRASP',
    '6': 'GENERATE_TRAJECTORY',
    '7': 'APPROACH',
    '8': 'INSERTION',
}



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

goal_pose_path = {
    "part11": "/root/share/catkin_ws/src/demo_traj/data/refined_final_peg_pose/test4_20251030_151337_part11_refined_final_pose.txt"
    # "part11": "/root/share/catkin_ws/src/franka_interface/scripts/fixed_pose/part11-2/grasp_tcp.json"
    # "part11": "/root/share/catkin_ws/src/franka_interface/scripts/fixed_pose/part11/grasp_tcp.json"
}

PRE_GRASP_OFFSET_Z = 0.07  # 7cm
PRE_GOAL_OFFSET_Z = 0.03  # 5cm
GRIPPER_MAX_WIDTH = 0.0396 * 2

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
    def __init__(self):
        rospy.init_node('controller_switcher_node')
        self.path = os.path.dirname(__file__)

        self.base_frame = "panda_link0"
        self.tcp_frame = "panda_hand_tcp"
        self.fixed_pose_root = os.path.join(self.path, "fixed_pose")
        self.hole_name = "part1"
        self.hole_frame = "object_" + self.hole_name
        self.hole_pose_file = os.path.join(self.fixed_pose_root, self.hole_name, "current.json")
        
        self.selected_object = "part11"
        self.update_selected_peg(self.selected_object)
        self.target_pose_frame = f"insert_center_{self.selected_object}"
        rospy.set_param('/target_pose_frame', self.target_pose_frame)

        self.target_area_ratio = {
            'part1': 0.18,
            'part7': 0.05,
            'part9': 0.05,
            'part11': 0.03
        }

        # 서비스 클라이언트 설정
        self.switch_srv = rospy.ServiceProxy('/controller_manager/switch_controller', SwitchController)
        self.list_srv = rospy.ServiceProxy('/controller_manager/list_controllers', ListControllers)

        # TF 리스너 및 MoveIt 팔 그룹 설정 (추가)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.arm = moveit_commander.MoveGroupCommander("panda_arm")
        self.arm.set_end_effector_link("panda_hand_tcp")

        # 1. 외부 노드로부터 명령을 받을 Subscriber 추가
        # /set_controller_mode 토픽으로 "1", "2", "3" 또는 컨트롤러 이름을 보내면 바뀝니다.
        self.mode_sub = rospy.Subscriber('/set_controller_mode', String, self.mode_callback)
        self.gripper_joint_sub = rospy.Subscriber('/franka_gripper/joint_states', JointState, self.gripper_joint_callback)
        self.pose_sub = None
        self.pose_received = False

        # 현재 모드 발행용 Publisher (latch=True)
        self.mode_pub = rospy.Publisher('/current_mode', String, queue_size=1, latch=True)
        self.sam2_pub = rospy.Publisher('/sam2_target_object', String, queue_size=1, latch=False)
        self.fp_pub = rospy.Publisher('/fp_target_object', String, queue_size=1, latch=False)
        self.joy_pub = rospy.Publisher('/spacenav/joy', Joy, queue_size=1)
        self.policy_reset_pub = rospy.Publisher('/policy/reset', Bool, queue_size=1)
        self.policy_enable_pub = rospy.Publisher('/policy/enable', Bool, queue_size=1)
        self.policy_status_sub = rospy.Subscriber('/policy/status', String, self.policy_status_callback)
        self.policy_status = 'UNKNOWN'
        self.insertion_active = False

        # 컨트롤러 이름 정의
        self.pos_controller = "position_joint_trajectory_controller"
        self.imp_controller = "cartesian_impedance_example_controller"
        self.pose_controller = "cartesian_pose_controller"
        self.all_controllers = [self.pos_controller, self.imp_controller, self.pose_controller]

        # 액션 클라이언트 (Home 복귀용)
        self.trajectory_client = actionlib.SimpleActionClient(
            f'/{self.pos_controller}/follow_joint_trajectory', 
            FollowJointTrajectoryAction
        )

        # Grasp 상태 관련 변수
        self.is_gripper_closed = False
        self.is_grasped = False
        self.grasp_threshold = 0.02  # 2cm 이내일 때 잡은 것으로 간주
        self.latest_gripper_width = None
        self.last_gripper_state_stamp = rospy.Time(0)
        self.gripper_state_timeout = 0.5
        
        # Grasp 상태 발행용 Publisher
        self.grasp_status_pub = rospy.Publisher('/is_grasped', Bool, queue_size=1, latch=True)
        
        # 주기적으로 상태를 체크하기 위한 타이머 (10Hz)
        self.grasp_check_timer = rospy.Timer(rospy.Duration(0.1), self.check_grasp_status)
        
        # --- 사전 정의된 START POSE (Joint Angles) ---
        self.home_pose = {
            'panda_joint1': 0.0, 'panda_joint2': -0.785, 'panda_joint3': 0.0,
            'panda_joint4': -2.356, 'panda_joint5': 0.0, 'panda_joint6': 1.571, 'panda_joint7': 0.785
        }

        # --- 사전 정의된 GOAL POSE (txt 파일 경로) ---
        self.goal_pose_path = goal_pose_path
        self.goal_pose_data = {}

        self.settings = termios.tcgetattr(sys.stdin)

        self.static_br = tf2_ros.StaticTransformBroadcaster()
        self.fixed_poses = {}
        self.load_and_publish_all_saved_poses()
        
        initial_mode = MOVEIT
        self.mode = initial_mode
        self.mode_pub.publish(initial_mode)

        self.print_guide(initial_mode)

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

    def load_and_publish_all_saved_poses(self):
        """저장 폴더 내의 모든 _fixed_pose.json 파일을 찾아 TF로 등록"""

        self.update_fixed_pose(
            pose_data=dummy_camera_pose_dict,
            parent_frame="panda_hand",
            child_frame="camera_link"
        )

        if os.path.exists(self.hole_pose_file):
            hole_pose_dict = json.load(open(self.hole_pose_file, 'r'))
            self.update_fixed_pose(
                pose_data=hole_pose_dict,
                parent_frame=self.base_frame,
                child_frame=self.hole_frame
            )
            rospy.loginfo(f"Loaded hole fixed pose from {self.hole_pose_file}")

        if self.peg_name in self.goal_pose_path:
            if os.path.exists(self.goal_pose_path[self.peg_name]):
                self.goal_pose_data[self.peg_name] = self.load_goal_matrix(self.goal_pose_path[self.peg_name])
                self.update_fixed_pose(
                    pose_data=self.goal_pose_data[self.peg_name],
                    parent_frame=self.hole_frame,
                    child_frame=f"goal_pose_{self.peg_name}"
                )

                # Pre-goal 포즈 계산 및 등록
                pre_goal_pose_dict = copy.deepcopy(self.goal_pose_data[self.peg_name])
                pre_goal_pose_dict['position']['z'] += PRE_GOAL_OFFSET_Z
                self.update_fixed_pose(
                    pose_data=pre_goal_pose_dict,
                    parent_frame=self.hole_frame,
                    child_frame=self.pre_goal_frame
                )

                # Pre-goal TCP 포즈 계산 및 등록
                pre_goal_grasp_pose = self.get_grasp_pose(self.peg_name)
                if pre_goal_grasp_pose is not None:
                    self.update_fixed_pose(
                        pose_data=pre_goal_grasp_pose,
                        parent_frame=self.pre_goal_frame,
                        child_frame=self.pre_goal_tcp_frame
                    )
                else:
                    rospy.logwarn(f"Skipped {self.pre_goal_tcp_frame}: grasp pose is unavailable for {self.peg_name}")

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
            hole_pose_dict = json.load(open(selected_path, 'r'))
        except Exception as e:
            rospy.logerr(f"Failed to load selected hole pose file {selected_path}: {e}")
            return

        self.hole_pose_file = selected_path
        self.update_fixed_pose(
            pose_data=hole_pose_dict,
            parent_frame=self.base_frame,
            child_frame=self.hole_frame
        )
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

    def normalize_quaternion(self, q):
        norm = np.sqrt(q.x**2 + q.y**2 + q.z**2 + q.w**2)
        q.x /= norm
        q.y /= norm
        q.z /= norm
        q.w /= norm
        return q

    def get_current_tcp_pose(self):
        return self.arm.get_current_pose().pose

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
            pose = PoseStamped()
            pose.header.frame_id = parent_frame
            pose.pose.position = trans.transform.translation
            pose.pose.orientation = trans.transform.rotation
            return pose
        except Exception as e:
            rospy.logerr(f"TF Lookup failed for {target_frame}: {e}")
            return None

    def pose_to_pose_dict(self, pose):
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
            joint_state = rospy.wait_for_message('/franka_state_controller/joint_states', JointState, timeout=2.0)
            initial_pose = dict(zip(joint_state.name, joint_state.position))
        except:
            rospy.logerr("move_to_home: Could not get current joint states.")
            return

        max_movement = max(abs(self.home_pose[joint] - initial_pose[joint]) for joint in self.home_pose if joint in initial_pose)
        move_duration = max(max_movement / 0.5, 2.0)

        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = list(self.home_pose.keys())
        point = JointTrajectoryPoint()
        point.positions = [self.home_pose[joint] for joint in goal.trajectory.joint_names]
        point.velocities = [0.0] * len(goal.trajectory.joint_names)
        point.time_from_start = rospy.Duration.from_sec(move_duration)
        goal.trajectory.points.append(point)

        self.trajectory_client.send_goal(goal) # send_goal_and_wait 대신 send_goal 사용 권장 (스레드 차단 방지)

    def move_to_pre_grasp(self):
        rospy.loginfo(f"Moving {self.tcp_frame} to {self.pre_grasp_tcp_frame}...")
        # 1. Pre-Grasp 위치로 이동 (일반 PTP 이동)
        pre_grasp_pose_stamped = self.get_pose_from_tf(self.pre_grasp_tcp_frame)
        if pre_grasp_pose_stamped:
            rospy.loginfo("Approaching Pre-Grasp Point (Cartesian Path)...")
            # waypoints = [pre_grasp_pose_stamped.pose]
            target_pose = pre_grasp_pose_stamped.pose
            target_pose.orientation = self.normalize_quaternion(target_pose.orientation)
            waypoints = [target_pose]
            (plan, fraction) = self.arm.compute_cartesian_path(waypoints, 0.01, False)
            if fraction > 0.9: # 경로 생성 성공률이 높을 때만 실행
                # 천천히 이동하도록 궤적 수정
                plan = self.arm.retime_trajectory(
                    self.arm.get_current_state(),
                    plan,
                    velocity_scaling_factor=0.5,
                    acceleration_scaling_factor=0.2,
                    algorithm="time_optimal_trajectory_generation"  # ROS1 MoveIt에서 흔히 사용
                )
                self.arm.execute(plan, wait=True)
                rospy.loginfo("Pre-Grasp position reached!")
            else:
                rospy.logwarn(f"Cartesian path planning to Pre-Grasp failed (Fraction: {fraction})")
                return False
            
    def move_to_grasp(self):
        rospy.loginfo(f"Moving {self.tcp_frame} to {self.grasp_tcp_frame}...")
        # 2. Grasp 위치로 직선(Cartesian) 진입
        grasp_pose_stamped = self.get_pose_from_tf(self.grasp_tcp_frame)
        if grasp_pose_stamped:
            rospy.loginfo("Approaching Grasp Point (Cartesian Path)...")
            # waypoints = [grasp_pose_stamped.pose]
            target_pose = grasp_pose_stamped.pose
            target_pose.orientation = self.normalize_quaternion(target_pose.orientation)
            waypoints = [target_pose]
            
            # 속도 제한 및 경로 계산
            (plan, fraction) = self.arm.compute_cartesian_path(waypoints, 0.01, False)
            
            if fraction > 0.9: # 경로 생성 성공률이 높을 때만 실행
                # 천천히 이동하도록 궤적 수정
                plan = self.arm.retime_trajectory(
                    self.arm.get_current_state(),
                    plan,
                    velocity_scaling_factor=0.05,
                    acceleration_scaling_factor=0.05,
                    algorithm="time_optimal_trajectory_generation"  # ROS1 MoveIt에서 흔히 사용
                )
                self.arm.execute(plan, wait=True)
                rospy.loginfo("Grasp position reached!")
                return True
            else:
                rospy.logwarn(f"Cartesian path planning failed (Fraction: {fraction})")
        
        return False

    def set_viewpoint_pose(self):
        """현재 로봇의 Joint State를 viewpoint.json 파일로 저장합니다."""
        rospy.loginfo("Setting current pose as viewpoint...")
        try:
            # 현재 Joint State 메시지 수신 (1초 대기)
            joint_state = rospy.wait_for_message('/franka_state_controller/joint_states', JointState, timeout=1.0)
            
            # Joint 이름과 위치를 딕셔너리로 결합
            viewpoint_data = dict(zip(joint_state.name, joint_state.position))
            
            # 필터링: panda_arm에 해당하는 joint만 저장 (필요 시)
            arm_joints = {k: v for k, v in viewpoint_data.items() if 'panda_joint' in k}

            # 파일 저장 경로 설정 (스크립트와 같은 위치)
            file_path = os.path.join(self.fixed_pose_root, 'viewpoint', 'current.json')
            os.makedirs(os.path.dirname(file_path), exist_ok=True)
            
            with open(file_path, 'w') as f:
                json.dump(arm_joints, f, indent=4)
            
            rospy.loginfo(f"Viewpoint saved successfully to: {file_path}")
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
        
        file_path = os.path.join(self.fixed_pose_root, 'viewpoint', 'current.json')
        
        if not os.path.exists(file_path):
            rospy.logerr("Viewpoint file not found! Please set pose first (Press '0').")
            return

        try:
            # 1. 파일에서 데이터 로드
            with open(file_path, 'r') as f:
                target_viewpoint = json.load(f)
                print(f"\n[LOADED VIEWPOINT] {target_viewpoint}")

            # 2. 액션 서버 확인
            if not self.trajectory_client.wait_for_server(timeout=rospy.Duration(2.0)):
                rospy.logerr("Action server not found.")
                return

            # 3. 현재 위치 확인 및 이동 시간 계산
            joint_state = rospy.wait_for_message('/franka_state_controller/joint_states', JointState, timeout=2.0)
            current_pose = dict(zip(joint_state.name, joint_state.position))
            
            max_diff = max(abs(target_viewpoint[j] - current_pose[j]) for j in target_viewpoint if j in current_pose)
            move_duration = max(max_diff / 0.5, 3.0)  # 최소 3초 보장

            # 4. Goal 생성 및 전송
            goal = FollowJointTrajectoryGoal()
            goal.trajectory.joint_names = list(target_viewpoint.keys())
            
            point = JointTrajectoryPoint()
            point.positions = [target_viewpoint[j] for j in goal.trajectory.joint_names]
            point.velocities = [0.0] * len(goal.trajectory.joint_names)
            point.time_from_start = rospy.Duration.from_sec(move_duration)
            
            goal.trajectory.points.append(point)
            
            rospy.loginfo(f"Sending viewpoint goal (Duration: {move_duration:.2f}s)")
            self.trajectory_client.send_goal(goal)
            
            # 이동 완료 대기 (필요 시)
            # self.trajectory_client.wait_for_result()
            
        except Exception as e:
            rospy.logerr(f"Error moving to viewpoint: {e}")

    def zoom_to_object(self, obj_name, target_area_ratio = 0.05):
        rospy.loginfo(f"Starting Zoom to {obj_name}...")
        self.bridge = CvBridge()
        self.mask_data = None
        
        sub = rospy.Subscriber(f"/sam2_mask/{obj_name}", Image, self._mask_callback)
        
        # 제어 게인 (튜닝 필요)
        gain_trans = 0.0003
        rate = rospy.Rate(30)
        
        self.pose_pub = rospy.Publisher('/cartesian_pose_controller/tcp_target_pose', PoseStamped, queue_size=1)

        while not rospy.is_shutdown():
            if self.mask_data is None:
                continue

            try:
                # 1. TF에서 현재 TCP 포즈 가져오기
                trans = self.tf_buffer.lookup_transform("panda_link0", "panda_hand_tcp", rospy.Time(0))
                
                # moments 및 오차 계산
                moments = cv2.moments(self.mask_data)
                if moments['m00'] <= 0:
                    continue

                u, v = int(moments['m10']/moments['m00']), int(moments['m01']/moments['m00'])
                h, w = self.mask_data.shape
                err_u, err_v = (w/2 - u), (h/2 - v)
                area_ratio = moments['m00'] / (h * w * 255.0)

                # 2. 카메라 좌표계 기준의 이동량(Delta) 정의
                # 카메라 좌표계 관습: Z(전진), X(우측), Y(하단)
                # 이미지 u 오차 -> 카메라 X 이동 / 이미지 v 오차 -> 카메라 Y 이동
                d_x_cam = err_u * gain_trans
                d_y_cam = err_v * gain_trans
                d_z_cam = 0.01 if area_ratio < target_area_ratio else 0.0

                # 3. 카메라 Delta를 베이스 좌표계(panda_link0)로 변환
                # 현재 TCP의 회전(Quaternion)을 행렬로 변환
                import tf.transformations as tft
                q = [trans.transform.rotation.x, trans.transform.rotation.y, 
                     trans.transform.rotation.z, trans.transform.rotation.w]
                rotation_matrix = tft.quaternion_matrix(q)[:3, :3] # 3x3 회전 행렬

                # 카메라 좌표계의 증분 벡터
                delta_cam = np.array([d_x_cam, d_y_cam, d_z_cam])
                
                # 베이스 좌표계에서의 증분 벡터 = R * delta_cam
                delta_base = np.dot(rotation_matrix, delta_cam)

                # 4. 최종 목표 포즈 생성
                target_stamped = PoseStamped()
                target_stamped.header.frame_id = "panda_link0"
                target_stamped.header.stamp = rospy.Time.now()
                
                # 현재 위치 + 베이스 기준 증분
                target_stamped.pose.position.x = trans.transform.translation.x + delta_base[0]
                target_stamped.pose.position.y = trans.transform.translation.y + delta_base[1]
                target_stamped.pose.position.z = trans.transform.translation.z + delta_base[2]
                target_stamped.pose.orientation = trans.transform.rotation # 자세 유지

                self.pose_pub.publish(target_stamped)

                if abs(err_u) < 10 and abs(err_v) < 10 and area_ratio >= target_area_ratio:
                    rospy.loginfo("Reached!")
                    break

            except Exception as e:
                rospy.logerr(f"Zoom Error: {e}")

            rate.sleep()
        sub.unregister()

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

    def fix_object_pose(self):
        rospy.loginfo(f"Fixing pose for object: {self.selected_object}")
        file_path = os.path.join(self.fixed_pose_root, self.selected_object, 'current.json')

        poses_in_base = []
        topic_name = f"/object_pose/{self.selected_object}"
        num_samples = 10
        timeout = 10  # seconds
        start_time = rospy.Time.now()
        try:
            rospy.loginfo(f"Collecting samples and transforming to panda_link0...")
            while poses_in_base.__len__() < num_samples:
                try:
                    if (rospy.Time.now() - start_time).to_sec() > timeout:
                        rospy.logwarn("Timeout while collecting samples.")
                        break
                    # 1. FP 노드로부터 카메라 기준 포즈 수신
                    msg = rospy.wait_for_message(topic_name, PoseStamped, timeout=0.1)
                    
                    # 2. 카메라 기준 포즈를 panda_link0(베이스) 기준으로 변환
                    # transform_pose는 tf_buffer를 사용하여 자동으로 좌표 변환을 수행합니다.
                    pose_base = self.tf_buffer.transform(msg, self.base_frame, timeout=rospy.Duration(1.0))
                    poses_in_base.append(pose_base)
                    # remove outliers based on distance from median
                    if len(poses_in_base) >= num_samples:
                        # 1. 위치 데이터 추출 (N, 3)
                        positions = np.array([[p.pose.position.x, p.pose.position.y, p.pose.position.z] for p in poses_in_base])
                        
                        # 2. 중앙값 계산 (이상치에 강인함)
                        median_pos = np.median(positions, axis=0)
                        
                        # 3. 각 샘플과 중앙값 사이의 거리 계산
                        distances = np.linalg.norm(positions - median_pos, axis=1)
                        
                        # 4. 임계값 설정 (예: 3cm / 상황에 따라 0.02~0.05 조절 가능)
                        threshold = 0.03 
                        
                        # 5. 거리 기반 필터링
                        filtered_indices = np.where(distances < threshold)[0]
                        
                        # 필터링 후 데이터가 너무 적으면 threshold를 조금 완화하거나 그대로 진행
                        if len(filtered_indices) < (num_samples // 2):
                            rospy.logwarn("Too many outliers detected. Checking sensor stability...")
                            # 필터링 결과가 너무 적으면 일단 수집을 더 진행하거나 
                            # 현재까지의 원본 데이터 중 가장 중앙값에 가까운 것들 위주로 재선택
                        
                        # 필터링된 포즈 리스트 업데이트 (최종 샘플 수 충족 시 루프 종료)
                        poses_in_base = [poses_in_base[i] for i in filtered_indices]
                    
                    print(f"Collected {len(poses_in_base)}/{num_samples} samples...", end='\r')
                except (rospy.ROSException, tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                    continue
            
            # print(len(poses_in_base))
            if len(poses_in_base) >= num_samples:
                # 3. 베이스 기준 포즈들 평균 계산 및 저장
                avg_pose_dict = self.calculate_average_pose(poses_in_base)
                os.makedirs(os.path.dirname(file_path), exist_ok=True)
                self.save_pose_to_file(avg_pose_dict, file_path)
                rospy.loginfo(f"Successfully saved base-link pose to {file_path}")
            else:
                rospy.logwarn("Not enough real-time samples. Trying to load from file...")
                if os.path.exists(file_path):
                    with open(file_path, 'r') as f:
                        avg_pose_dict = json.load(f)
                else:
                    rospy.logerr("No data available to fix pose.")
                    return
            # 5. FP 노드 대기 상태 전환
            self.fp_pub.publish("")
                
            # 4. Static TF 발행 (항상 베이스 기준)
            self.update_fixed_pose(
                pose_data=avg_pose_dict,
                parent_frame=self.base_frame,
                child_frame=self.peg_frame_filtered
            )

            # Pre-grasp, Grasp 자세 계산
            # if (self.selected_object == self.peg_name):
            #     self.update_grasp_pose(self.peg_name, self.peg_frame_filtered)

            self.publish_all_static_tfs()
            rospy.loginfo(f"Fixed pose for {self.selected_object} established on panda_link0.")

        except Exception as e:
            rospy.logerr(f"Error in fix_object_pose: {e}")

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
        with open(file_path, 'w') as f:
            json.dump(pose_data, f, indent=4)

    def load_pose_json(self, file_path):
        with open(file_path, 'r') as f:
            data = json.load(f)
        if 'position' not in data or 'orientation' not in data:
            raise ValueError(f"Invalid pose format in {file_path}")
        return data

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

        if obj_name == 'part11-2':
            part11_insert_center_path = os.path.join(self.fixed_pose_root, 'part11', 'insert_center.json')
            if os.path.exists(part11_insert_center_path):
                try:
                    rospy.loginfo("Using part11 insert_center.json for part11-2")
                    return self.load_pose_json(part11_insert_center_path)
                except Exception as e:
                    rospy.logwarn(f"Failed to load insert center pose from {part11_insert_center_path}: {e}")

        return None

    def update_fixed_pose(self, pose_data, parent_frame, child_frame):
        """딕셔너리에 TF 정보를 등록하거나 업데이트합니다."""
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = parent_frame
        t.child_frame_id = child_frame

        # position/orientation이 딕셔너리 형태인지 확인 후 대입
        if 'position' in pose_data:
            t.transform.translation.x = pose_data['position']['x']
            t.transform.translation.y = pose_data['position']['y']
            t.transform.translation.z = pose_data['position']['z']
            t.transform.rotation.x = pose_data['orientation']['x']
            t.transform.rotation.y = pose_data['orientation']['y']
            t.transform.rotation.z = pose_data['orientation']['z']
            t.transform.rotation.w = pose_data['orientation']['w']
        
        # 관리용 딕셔너리에 저장 (키값을 child_frame으로 해야 중복 방지)
        self.fixed_poses[child_frame] = t
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

    def approach(self):
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
        current_tcp_pose = self.get_current_tcp_pose()
        # target_frame = self.measured_pre_goal_tcp_frame
        target_frame = self.pre_goal_tcp_frame
        pre_goal_tcp_pose_stamped = self.get_pose_from_tf(target_frame)
        if not pre_goal_tcp_pose_stamped:
            rospy.logwarn(f"Failed to get target pose from TF: {target_frame}")
            return False

        rospy.loginfo(f"Moving {self.tcp_frame} to {target_frame} with XY-first strategy...")
        target_pose = pre_goal_tcp_pose_stamped.pose
        target_pose.orientation = self.normalize_quaternion(target_pose.orientation)

        # Step 1) XY 먼저 정렬 (현재 Z 유지)
        xy_pose = copy.deepcopy(current_tcp_pose)
        xy_pose.position.x = target_pose.position.x
        xy_pose.position.y = target_pose.position.y
        xy_pose.orientation = target_pose.orientation

        rospy.loginfo("Step 1/2: Moving in XY plane (keep current Z)...")
        plan_xy, fraction_xy = self.arm.compute_cartesian_path([xy_pose], 0.01, False)
        if fraction_xy <= 0.9:
            rospy.logwarn(f"Cartesian XY planning failed (Fraction: {fraction_xy})")
            return False

        plan_xy = self.arm.retime_trajectory(
            self.arm.get_current_state(),
            plan_xy,
            velocity_scaling_factor=0.3,
            acceleration_scaling_factor=0.1,
            algorithm="time_optimal_trajectory_generation"
        )
        self.arm.execute(plan_xy, wait=True)

        # Step 2) Z축으로만 하강 (XY 유지)
        current_after_xy = self.get_current_tcp_pose()
        z_pose = copy.deepcopy(current_after_xy)
        z_pose.position.z = target_pose.position.z
        z_pose.orientation = target_pose.orientation

        rospy.loginfo("Step 2/2: Descending along Z axis...")
        plan_z, fraction_z = self.arm.compute_cartesian_path([z_pose], 0.01, False)
        if fraction_z <= 0.9:
            rospy.logwarn(f"Cartesian Z planning failed (Fraction: {fraction_z})")
            return False

        plan_z = self.arm.retime_trajectory(
            self.arm.get_current_state(),
            plan_z,
            velocity_scaling_factor=0.2,
            acceleration_scaling_factor=0.08,
            algorithm="time_optimal_trajectory_generation"
        )
        self.arm.execute(plan_z, wait=True)
        rospy.loginfo("Pre-Goal position reached.")
        return True

    def print_guide(self, current_mode):
        print("\n" + "="*50)
        print(f" [ CURRENT ACTIVE MODE: {MODE_DICT[current_mode]} ]")
        print("="*50)
        print(" Press 'a': [MoveIt Mode]")
        print(" Press 's': [Cartesian Impedance Mode]")
        print(" Press 'd': [Cartesian Pose Mode]")
        print(" Press 't': [Teleop Mode]")
        print(" Press 'i': [Teleop Impedance Mode]")
        print(" Press 'h': Select Hole as Target Object")
        print(" Press 'p': Select Peg as Target Object")
        print(" Press 'f': Register Frames")
        print(" Press 'j': Select Hole Pose (current_x.json)")
        print(" Press '0': Set Viewpoint Pose")
        print(" Press '1': Move to Home")
        print(" Press '2': Move to Viewpoint")
        print(" Press '3': Zoom to Object")
        print(" Press '4': Move to Pre-Grasp")
        print(" Press '5': Move to Grasp")
        print(" Press '6': Generate Trajectory")
        print(" Press '7': Approach")
        print(" Press '8': Insertion")
        print(" Press 'q': Quit and Shutdown Node")
        print("-" * 50)
        print(" (Also listening on topic: /set_controller_mode)")
        print(" >> Select Mode: ", end='', flush=True)

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
        self.request_object_pose(self.selected_object)# ; self.fix_object_pose(self.selected_object)
        # 2. Pre-Grasp 위치로 이동
        self.switch_controller(self.pos_controller); self.move_to_pre_grasp()# ; self.request_object_pose(self.selected_object)
        # 3. 객체 자세 Refinement 및 Fix
        self.fix_object_pose()
        # 4. Grasping 시도
        self.move_to_pre_grasp(); self.move_to_grasp(); self.close_gripper()
        # 5. 객체 자세 재요청 및 잡힘 확인
        self.request_object_pose(self.selected_object); rospy.sleep(0.5)
        if self.is_grasped:
            rospy.loginfo("Grasping successful!")
            self.approach()
            self.print_guide(self.mode)
        else:
            rospy.logwarn("Grasping failed. Object not secured.")
            # self.open_gripper()
            self.move_to_home()
            self.print_guide(self.mode)

    def change_mode(self, key):
        if key is None: 
            return False

        if key != INSERTION:
            self.stop_insertion()

        if   key == MOVEIT: # MoveIt Mode
            self.switch_controller(self.pos_controller) 
        elif key == CARTESIAN_IMPEDANCE: # Cartesian Impedance Mode
            self.switch_controller(self.imp_controller) 
        elif key == CARTESIAN_POSE: # Cartesian Pose Mode
            self.switch_controller(self.pose_controller) 
        elif key == TELEOP: # Teleop Mode
            self.switch_controller(self.pose_controller)
        elif key == TELEOP_IMPEDANCE:# Teleop Impedance Mode
            self.switch_controller(self.imp_controller) 
        elif key == OPEN_GRIPPER: # Open Gripper
            self.open_gripper()
        elif key == CLOSE_GRIPPER: # Close Gripper
            self.close_gripper()
        elif key == 'q': # Quit
            self.stop_insertion()
            return rospy.signal_shutdown("User requested shutdown.") 
        elif key == SELECT_HOLE: # Select Hole
            self.select_object(self.hole_name); 
        elif key == SELECT_HOLE_POSE: # Select Hole Pose
            self.select_hole_pose_menu()
        elif key == SELECT_PEG: # Select Peg
            # get peg number from keyboard input
            peg_input = input("Enter peg number/name (e.g. 11, 11-2, part12): ").strip()
            if peg_input.startswith("part"):
                peg_name = peg_input
            elif peg_input == "11-2":
                peg_name = "part11-2"
            else:
                try:
                    peg_number = int(peg_input)
                    peg_name = f"part{peg_number}" if peg_number in PEG_LIST else None
                except ValueError:
                    peg_name = None

            is_valid = peg_name in PEG_NAME_EXTRA or (
                peg_name is not None and os.path.isdir(os.path.join(self.fixed_pose_root, peg_name))
            )
            if is_valid:
                self.peg_name = peg_name
                self.select_object(self.peg_name)
            else:
                rospy.logwarn("Invalid peg input. Use one of [9, 11, 11-2, 12, 17] or part name.")
        elif key == SET_VIEWPOINT: # Set Viewpoint Pose
            self.set_viewpoint_pose() 
        elif key == MOVE_TO_HOME: # Move to Home
            self.switch_controller(self.pos_controller); rospy.sleep(0.5); self.move_to_home()
        elif key == MOVE_TO_VIEWPOINT: # Move to Viewpoint
            self.switch_controller(self.pos_controller); rospy.sleep(0.5); self.move_to_viewpoint()
        elif key == ZOOM_TO_OBJECT: # Zoom to Object
            self.switch_controller(self.pose_controller); rospy.sleep(0.5); self.zoom_to_object(self.selected_object, self.target_area_ratio[self.selected_object]);
        elif key == MOVE_TO_PRE_GRASP: # Move to Pre-Grasp
            self.switch_controller(self.pos_controller); rospy.sleep(0.5); self.move_to_pre_grasp()
        elif key == MOVE_TO_GRASP: # Move to Grasp
            self.switch_controller(self.pos_controller); rospy.sleep(0.5); self.move_to_grasp()
        elif key == GENERATE_TRAJECTORY: # Generate Trajectory
            self.switch_controller(self.pos_controller); rospy.sleep(0.5); self.generate_trajectory()
        elif key == APPROACH: # Approach
            self.switch_controller(self.pos_controller); rospy.sleep(0.5); self.approach()
        elif key == INSERTION: # Insertion
            self.start_insertion()
        elif key == GRASPING: # Grasping
            self.grasping()
        elif key == REGISTER_FRAMES: # Register Frames
            rospy.loginfo(f"Registering current averaged pose for: {self.selected_object}")
            self.request_object_pose(self.selected_object)
            self.fix_object_pose()
            saved_path = os.path.join(self.fixed_pose_root, self.selected_object, 'current.json')
            rospy.loginfo(f"Averaged pose registration complete: {saved_path}")
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
    switcher = ControllerSwitcher()
    switcher.run()
