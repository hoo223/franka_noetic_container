#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import copy
import tf.transformations as tft
import numpy as np
import actionlib

# 메시지 타입 임포트
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64MultiArray # [추가] 조이스틱 데이터 수신용
from franka_msgs.msg import FrankaState
from franka_gripper.msg import MoveAction, MoveGoal, GraspAction, GraspGoal

class SpaceNavTeleop:
    def __init__(self):
        rospy.init_node('spacenav_franka_teleop')

        # --- 설정 변수 ---
        self.trans_scale = 0.05  # m/s (조이스틱 이동 속도)
        self.rot_scale = 0.5     # rad/s (SpaceMouse & 조이스틱 회전 속도)
        self.deadzone = 0.1      # SpaceMouse 데드존
        self.publish_rate = 30.0
        
        # [중요] 링크 이름 설정
        self.base_frame = rospy.get_param("~link_name", "fr3_link0")

        # --- 초기 상태 변수 ---
        self.target_pose = PoseStamped()
        self.target_pose.header.frame_id = self.base_frame
        
        # SpaceNav 입력값 저장용 (기존)
        self.joy_axes = [0.0] * 6
        self.prev_buttons = [0, 0]

        # [추가] Joystick 입력값 저장용
        # 구조: [x, y, z, roll, pitch, yaw, button]
        self.joystick_cmd = [0.0] * 7 

        # 1. 초기 위치 설정 (실제 로봇 상태에서 가져오기)
        rospy.loginfo("Waiting for initial robot pose from /franka_state_controller/franka_states...")
        self.wait_for_initial_pose()
        rospy.loginfo("Initial pose acquired. Ready to control.")

        # --- Publisher & Subscriber ---
        self.pose_pub = rospy.Publisher(
            '/cartesian_impedance_example_controller/equilibrium_pose', 
            PoseStamped, 
            queue_size=1
        )
        
        # SpaceMouse 구독
        rospy.Subscriber('/spacenav/joy', Joy, self.spacenav_callback)
        
        # [추가] Joystick 구독
        rospy.Subscriber('/joystick_command', Float64MultiArray, self.joystick_callback)

        # --- Gripper Action Clients ---
        self.move_client = actionlib.SimpleActionClient('/franka_gripper/move', MoveAction)
        self.grasp_client = actionlib.SimpleActionClient('/franka_gripper/grasp', GraspAction)
        
        # rospy.loginfo("Waiting for gripper action servers...")
        # self.move_client.wait_for_server(rospy.Duration(5.0))
        # self.grasp_client.wait_for_server(rospy.Duration(5.0))

    def wait_for_initial_pose(self):
        """초기 로봇 위치 수신 및 Target Pose 초기화"""
        msg = rospy.wait_for_message("franka_state_controller/franka_states", FrankaState)
        
        initial_quaternion = tft.quaternion_from_matrix(
            np.transpose(np.reshape(msg.O_T_EE, (4, 4)))
        )
        initial_quaternion = initial_quaternion / np.linalg.norm(initial_quaternion)

        self.target_pose.pose.orientation.x = initial_quaternion[0]
        self.target_pose.pose.orientation.y = initial_quaternion[1]
        self.target_pose.pose.orientation.z = initial_quaternion[2]
        self.target_pose.pose.orientation.w = initial_quaternion[3]
        
        self.target_pose.pose.position.x = msg.O_T_EE[12]
        self.target_pose.pose.position.y = msg.O_T_EE[13]
        self.target_pose.pose.position.z = msg.O_T_EE[14]

    def spacenav_callback(self, msg):
        """SpaceMouse 콜백: Roll, Pitch 및 그리퍼 제어용"""
        self.joy_axes = msg.axes
        
        current_left = msg.buttons[0]
        current_right = msg.buttons[1]

        # 왼쪽 버튼: 열기
        if current_left == 1 and self.prev_buttons[0] == 0:
            self.gripper_command("open")
        
        # 오른쪽 버튼: 닫기
        elif current_right == 1 and self.prev_buttons[1] == 0:
            self.gripper_command("close")

        self.prev_buttons = msg.buttons

    def joystick_callback(self, msg):
        """[추가] Joystick 콜백: Translation(X,Y,Z) 및 Yaw 제어용"""
        # msg.data 구조 가정: [x, y, z, roll, pitch, yaw, button]
        if len(msg.data) >= 6:
            self.joystick_cmd = msg.data

    def gripper_command(self, cmd):
        if cmd == "open":
            goal = MoveGoal(width=0.08, speed=0.1)
            self.move_client.cancel_all_goals()
            self.grasp_client.cancel_all_goals()
            self.move_client.send_goal(goal)
            rospy.loginfo("Gripper Open")
        elif cmd == "close":
            goal = GraspGoal()
            goal.width = 0.0
            goal.epsilon.inner = 0.005
            goal.epsilon.outer = 0.1
            goal.speed = 0.1
            goal.force = 60.0
            self.move_client.cancel_all_goals()
            self.grasp_client.cancel_all_goals()
            self.grasp_client.send_goal(goal)
            rospy.loginfo("Gripper Grasp")

    def apply_deadzone(self, val):
        """SpaceMouse용 데드존 처리"""
        if abs(val) < self.deadzone:
            return 0.0
        return val

    def update_pose(self, dt):
        # ---------------------------------------------------------
        # [수정됨] Hybrid Control Logic
        # ---------------------------------------------------------
        
        # 1. Translation (X, Y, Z) -> Joystick에서 가져옴
        # Joystick Node에서 이미 좌표계 매핑 및 부호 변환이 완료되어 들어옴
        # [0]: Left Stick Up/Down (X), [1]: Left Stick Side (Y), [2]: Right Stick Up/Down (Z)
        vx = self.joystick_cmd[0] * self.trans_scale
        vy = self.joystick_cmd[1] * self.trans_scale
        vz = self.joystick_cmd[2] * self.trans_scale
        
        # 2. Rotation (Roll, Pitch, Yaw)
        # Roll(Rx), Pitch(Ry) -> SpaceMouse (기존 joy_axes 사용)
        # Yaw(Rz) -> Joystick (Right Stick Side)
        
        # SpaceMouse axes: 0,1,2(XYZ), 3(Rx), 4(Ry), 5(Rz)
        wx = -self.apply_deadzone(self.joy_axes[3]) * self.rot_scale # Roll
        wy = self.apply_deadzone(self.joy_axes[4]) * self.rot_scale  # Pitch
        
        # Joystick Yaw (Index 5)
        wz = self.joystick_cmd[5] * self.rot_scale                   # Yaw

        # 입력이 없으면 업데이트 건너뜀 (연산 절약)
        if all(v == 0 for v in [vx, vy, vz, wx, wy, wz]):
            return

        # ---------------------------------------------------------
        # 3. 위치 업데이트 (Translation)
        self.target_pose.pose.position.x += vx * dt
        self.target_pose.pose.position.y += vy * dt
        self.target_pose.pose.position.z += vz * dt

        # 4. 회전 업데이트 (Rotation - Quaternion Math)
        current_quat = [
            self.target_pose.pose.orientation.x,
            self.target_pose.pose.orientation.y,
            self.target_pose.pose.orientation.z,
            self.target_pose.pose.orientation.w
        ]

        # 오일러 각속도를 쿼터니언 델타로 변환
        d_euler = [wx * dt, wy * dt, wz * dt]
        d_quat = tft.quaternion_from_euler(d_euler[0], d_euler[1], d_euler[2])
        
        # 현재 자세에 회전 적용
        new_quat = tft.quaternion_multiply(current_quat, d_quat)
        
        # 정규화 (Normalization)
        norm = np.linalg.norm(new_quat)
        new_quat = new_quat / norm

        self.target_pose.pose.orientation.x = new_quat[0]
        self.target_pose.pose.orientation.y = new_quat[1]
        self.target_pose.pose.orientation.z = new_quat[2]
        self.target_pose.pose.orientation.w = new_quat[3]

    def run(self):
        rate = rospy.Rate(self.publish_rate)
        while not rospy.is_shutdown():
            dt = 1.0 / self.publish_rate
            self.update_pose(dt)
            
            self.target_pose.header.seq += 1
            self.target_pose.header.stamp = rospy.Time.now()
            self.pose_pub.publish(self.target_pose)
            
            rate.sleep()

if __name__ == '__main__':
    try:
        teleop = SpaceNavTeleop()
        teleop.run()
    except rospy.ROSInterruptException:
        pass