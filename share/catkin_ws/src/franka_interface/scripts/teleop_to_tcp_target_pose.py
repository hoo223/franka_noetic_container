#!/usr/bin/env python3
import rospy
import tf
import numpy as np
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Float64MultiArray, String

MOVEIT = 'a'
CARTESIAN_IMPEDANCE = 's'
CARTESIAN_POSE = 'd'
TELEOP = 't'
TELEOP_IMPEDANCE = 'i'
SELECT_HOLE = 'h'
SELECT_PEG = 'p'

SET_VIEWPOINT = '0'
MOVE_TO_HOME = '1'
MOVE_TO_VIEWPOINT = '2'
ZOOM_TO_OBJECT = '3'
REQUEST_OBJECT_POSE = '4'
FIX_OBJECT_POSE = '5'
MOVE_TO_GRASP = '6'
GENERATE_TRAJECTORY = '7'
APPROACH = '8'

MODE_DICT = {
    'a': 'MOVEIT',
    's': 'CARTESIAN_IMPEDANCE',
    'd': 'CARTESIAN_POSE',
    't': 'TELEOP',
    'i': 'TELEOP_IMPEDANCE',
    'h': 'SELECT_HOLE',
    'p': 'SELECT_PEG',
    '0': 'SET_VIEWPOINT',
    '1': 'MOVE_TO_HOME',
    '2': 'MOVE_TO_VIEWPOINT',
    '3': 'ZOOM_TO_OBJECT',
    '4': 'REQUEST_OBJECT_POSE',
    '5': 'FIX_OBJECT_POSE',
    '6': 'MOVE_TO_GRASP',
    '7': 'GENERATE_TRAJECTORY',
    '8': 'APPROACH',
}

class DualDeviceController:
    def __init__(self):
        rospy.init_node('dual_device_integrator')

        # --- 설정 ---
        self.LINEAR_STEP = 0.5
        self.ANGULAR_STEP = 1
        self.deadzone = 0.1
        
        self.target_pos = None
        self.target_quat = None
        self.current_mode = "UNKNOWN"
        self.last_mode = "UNKNOWN" # 모드 변경 감지용

        # 상태 저장
        self.joy_linear = np.zeros(3)
        self.space_angular = np.zeros(3)

        self.listener = tf.TransformListener()
        
        # 구독자 및 발행자 설정 (이전과 동일)
        self.joy_sub = rospy.Subscriber("/joystick_command", Float64MultiArray, self.joy_callback)
        self.space_sub = rospy.Subscriber("/spacenav/twist", Twist, self.space_callback)
        self.mode_sub = rospy.Subscriber("/current_mode", String, self.mode_callback)
        
        self.pose_pub = rospy.Publisher("/cartesian_pose_controller/tcp_target_pose", 
                                         PoseStamped, queue_size=1, tcp_nodelay=True)
        self.imp_pub = rospy.Publisher("/cartesian_impedance_example_controller/equilibrium_pose", 
                                        PoseStamped, queue_size=1, tcp_nodelay=True)

        rospy.loginfo("TF 대기 중...")
        while not self.sync_target_with_tf() and not rospy.is_shutdown():
            rospy.sleep(0.1)
        rospy.loginfo("초기 동기화 완료.")

    def mode_callback(self, msg):
        self.current_mode = msg.data

    def sync_target_with_tf(self):
        """현재 로봇의 실제 포즈를 target_pose에 반영"""
        try:
            (trans, rot) = self.listener.lookupTransform('panda_link0', 'panda_hand_tcp', rospy.Time(0))
            self.target_pos = np.array(trans)
            self.target_quat = np.array(rot)
            return True
        except:
            return False

    def joy_callback(self, msg):
        self.joy_linear[0] = -msg.data[0]
        self.joy_linear[1] = msg.data[1]
        self.joy_linear[2] = msg.data[2]
        for i in range(3):
            if abs(self.joy_linear[i]) < self.deadzone: self.joy_linear[i] = 0.0

    def space_callback(self, msg):
        self.space_angular = np.array([-msg.angular.x, -msg.angular.y, msg.angular.z])

    def get_direction(self, vec):
        norm = np.linalg.norm(vec)
        return vec / norm if norm > 0.02 else np.zeros(3)

    def run(self):
        rate = rospy.Rate(50) 
        dt = 0.005

        while not rospy.is_shutdown():
            # --- [핵심] 모드 전환 감지 (Mode Switching Edge) ---
            if self.current_mode != self.last_mode:
                rospy.loginfo(f"Mode changed: {self.last_mode} -> {self.current_mode}. Syncing target pose.")
                self.sync_target_with_tf()
                self.last_mode = self.current_mode

            # --- 조종 모드 (Impedance 또는 Pose) ---
            if self.current_mode == TELEOP or self.current_mode == TELEOP_IMPEDANCE:
                # 1. Translation / Rotation 업데이트 (기존 로직)
                lin_dir = self.get_direction(self.joy_linear)
                self.target_pos += lin_dir * self.LINEAR_STEP * dt

                ang_dir = self.get_direction(self.space_angular)
                if np.any(ang_dir):
                    angle = self.ANGULAR_STEP * dt
                    delta_quat = tf.transformations.quaternion_about_axis(angle, ang_dir)
                    new_quat = tf.transformations.quaternion_multiply(delta_quat, self.target_quat)
                    self.target_quat = new_quat / np.linalg.norm(new_quat)

                # 2. 메시지 생성 및 발행
                msg = PoseStamped()
                msg.header.stamp = rospy.Time.now()
                msg.header.frame_id = "panda_link0"
                msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = self.target_pos
                msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w = self.target_quat

                if self.current_mode == TELEOP_IMPEDANCE:
                    self.imp_pub.publish(msg)
                else:
                    self.pose_pub.publish(msg)
            else:
                self.sync_target_with_tf()

            rate.sleep()

if __name__ == '__main__':
    node = DualDeviceController()
    node.run()